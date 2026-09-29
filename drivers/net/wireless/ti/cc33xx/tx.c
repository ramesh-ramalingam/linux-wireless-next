// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2026 Texas Instruments Incorporated - https://www.ti.com/
 */

#include "acx.h"
#include "debug.h"
#include "io.h"
#include "ps.h"
#include "tx.h"
#include "cc33xx.h"

static int cc33xx_set_default_wep_key(struct cc33xx *cc,
				      struct cc33xx_vif *wlvif, u8 id)
{
	int ret;
	bool is_ap = (wlvif->bss_type == BSS_TYPE_AP_BSS);

	if (is_ap)
		ret = cc33xx_cmd_set_default_wep_key(cc, id,
						     wlvif->ap.bcast_hlid);
	else
		ret = cc33xx_cmd_set_default_wep_key(cc, id, wlvif->sta.hlid);

	if (ret < 0)
		return ret;

	cc33xx_debug(cc->dev, DEBUG_CRYPT, "default wep key idx: %d\n", (int)id);
	return 0;
}

static int cc33xx_alloc_tx_id(struct cc33xx *cc, struct sk_buff *skb)
{
	int id;

	id = find_first_zero_bit(cc->tx_frames_map, CC33XX_NUM_TX_DESCRIPTORS);
	if (id >= CC33XX_NUM_TX_DESCRIPTORS)
		return -EBUSY;

	__set_bit(id, cc->tx_frames_map);
	cc->tx_frames[id] = skb;
	cc->tx_frames_cnt++;
	cc33xx_debug(cc->dev, DEBUG_TX, "alloc desc ID. id - %d, frames count %d\n",
		     id, cc->tx_frames_cnt);
	return id;
}

void cc33xx_free_tx_id(struct cc33xx *cc, int id)
{
	if (__test_and_clear_bit(id, cc->tx_frames_map)) {
		if (unlikely(cc->tx_frames_cnt == CC33XX_NUM_TX_DESCRIPTORS))
			clear_bit(CC33XX_FLAG_FW_TX_BUSY, &cc->flags);

		cc->tx_frames[id] = NULL;
		cc->tx_frames_cnt--;
	}
	cc33xx_debug(cc->dev, DEBUG_TX, "free desc ID. id - %d, frames count %d\n",
		     id, cc->tx_frames_cnt);
}

static void cc33xx_tx_regulate_link(struct cc33xx *cc,
				    struct cc33xx_vif *wlvif,
				    u8 hlid)
{
	bool fw_ps;
	u8 tx_pkts;

	if (WARN_ON(!test_bit(hlid, wlvif->links_map)))
		return;

	fw_ps = test_bit(hlid, &cc->ap_fw_ps_map);
	tx_pkts = cc->links[hlid].allocated_pkts;

	/* if in FW PS and there is enough data in FW we can put the link
	 * into high-level PS and clean out its TX queues.
	 * Make an exception if this is the only connected link. In this
	 * case FW-memory congestion is less of a problem.
	 * Note that a single connected STA means 2*ap_count + 1 active links,
	 * since we must account for the global and broadcast AP links
	 * for each AP. The "fw_ps" check assures us the other link is a STA
	 * connected to the AP. Otherwise the FW would not set the PSM bit.
	 */
	if (cc->active_link_count > (cc->ap_count * 2 + 1) && fw_ps &&
	    tx_pkts >= CC33XX_PS_STA_MAX_PACKETS)
		cc33xx_ps_link_start(cc, wlvif, hlid, true);
}

static u8 cc33xx_tx_get_hlid_ap(struct cc33xx_vif *wlvif, struct sk_buff *skb,
				struct ieee80211_sta *sta)
{
	struct ieee80211_hdr *hdr;

	if (sta) {
		struct cc33xx_station *wl_sta;

		wl_sta = (struct cc33xx_station *)sta->drv_priv;
		return wl_sta->hlid;
	}

	if (!test_bit(WLVIF_FLAG_AP_STARTED, &wlvif->flags))
		return CC33XX_SYSTEM_HLID;

	if (skb) {
		hdr = (struct ieee80211_hdr *)skb->data;
		if (is_multicast_ether_addr(ieee80211_get_DA(hdr)))
			return wlvif->ap.bcast_hlid;
		else
			return wlvif->ap.global_hlid;
	}

	return wlvif->ap.bcast_hlid;
}

/* SKB will be NULL when called from txq based tx. dev_hlid, global_hlid
 * shouldn't be relevant for txq cases.
 */
u8 cc33xx_tx_get_hlid(struct cc33xx *cc, struct cc33xx_vif *wlvif,
		      struct sk_buff *skb, struct ieee80211_sta *sta)
{
	struct ieee80211_tx_info *control;

	if (wlvif->bss_type == BSS_TYPE_AP_BSS)
		return cc33xx_tx_get_hlid_ap(wlvif, skb, sta);

	if (skb) {
		/* RoC packets will come through tx_pending list,
		 * SKB won't be NULL for them.
		 */
		control = IEEE80211_SKB_CB(skb);
		if (control->flags & IEEE80211_TX_CTL_TX_OFFCHAN)
			return wlvif->dev_hlid;
	}

	return wlvif->sta.hlid;
}

unsigned int cc33xx_calc_packet_alignment(struct cc33xx *cc,
					  unsigned int packet_length)
{
	if ((cc->quirks & CC33XX_QUIRK_TX_PAD_LAST_FRAME) ||
	    !(cc->quirks & CC33XX_QUIRK_TX_BLOCKSIZE_ALIGN))
		return ALIGN(packet_length, CC33XX_TX_ALIGN_TO);
	else
		return ALIGN(packet_length, CC33XX_BUS_BLOCK_SIZE);
}

static u32 cc33xx_calc_tx_blocks(u32 len, u32 spare_blks)
{
	u32 blk_size = CC33XX_TX_HW_BLOCK_SIZE;
	/* In CC33xx the packet will be stored along with its internal descriptor.
	 * the descriptor is not part of the host transaction, but should be
	 * considered as part of the allocate memory blocks in the device
	 */
	len = len + CC33XX_INTERNAL_DESC_SIZE;
	return (len + blk_size - 1) / blk_size + spare_blks;
}

static inline void cc33xx_set_tx_desc_blocks(struct cc33xx_tx_hw_descr *desc,
					     u32 blks)
{
	desc->cc33xx_mem.total_mem_blocks = blks;
}

static void cc33xx_set_tx_desc_data_len(struct cc33xx *cc,
					struct cc33xx_tx_hw_descr *desc,
					struct sk_buff *skb)
{
	desc->length = cpu_to_le16(skb->len);

	/* if only the last frame is to be padded, we unset this bit on Tx */
	if (cc->quirks & CC33XX_QUIRK_TX_PAD_LAST_FRAME)
		desc->cc33xx_mem.ctrl = CC33XX_TX_CTRL_NOT_PADDED;
	else
		desc->cc33xx_mem.ctrl = 0;

	cc33xx_debug(cc->dev, DEBUG_TX, "tx_fill_hdr: hlid: %d  len: %d life: %d mem: %d\n",
		     desc->hlid, le16_to_cpu(desc->length),
		     le16_to_cpu(desc->life_time),
		     desc->cc33xx_mem.total_mem_blocks);
}

static int cc33xx_get_spare_blocks(struct cc33xx *cc)
{
	/* If we have keys requiring extra spare, indulge them */
	if (cc->extra_spare_key_count)
		return CC33XX_TX_HW_EXTRA_BLOCK_SPARE;

	return CC33XX_TX_HW_BLOCK_SPARE;
}

int cc33xx_tx_get_queue(int queue)
{
	switch (queue) {
	case 0:
		return CONF_TX_AC_VO;
	case 1:
		return CONF_TX_AC_VI;
	case 2:
		return CONF_TX_AC_BE;
	case 3:
		return CONF_TX_AC_BK;
	default:
		return CONF_TX_AC_BE;
	}
}

static int cc33xx_tx_allocate(struct cc33xx *cc, struct sk_buff *skb,
			      u32 extra, u32 buf_offset, u8 hlid,
			      struct nab_tx_header *nab_cmd)
{
	struct cc33xx_tx_hw_descr *desc;
	u32 total_blocks;
	int id, ret = -EBUSY, ac;
	u32 spare_blocks;
	u32 total_skb_len = skb->len + extra;
	/* Add NAB command required for CC33xx architecture */
	u32 total_len = sizeof(struct nab_tx_header);

	total_skb_len += sizeof(struct cc33xx_tx_hw_descr);
	total_len += total_skb_len;

	cc33xx_debug(cc->dev, DEBUG_TX, "cc->tx_blocks_available %d\n",
		     cc->tx_blocks_available);

	if (buf_offset + total_len > cc->aggr_buf_size)
		return -EAGAIN;

	spare_blocks = cc33xx_get_spare_blocks(cc);

	/* allocate free identifier for the packet */
	id = cc33xx_alloc_tx_id(cc, skb);
	if (id < 0)
		return id;

	/* memblocks should not include nab descriptor */
	total_blocks = cc33xx_calc_tx_blocks(total_skb_len, spare_blocks);
	cc33xx_debug(cc->dev, DEBUG_TX, "total blocks %d\n", total_blocks);

	if (total_blocks <= cc->tx_blocks_available) {
		/**
		 * In CC33XX the packet starts with NAB command,
		 * only then the descriptor.
		 */
		nab_cmd->sync = cpu_to_le32(CC33XX_HOST_SYNC_PATTERN);
		nab_cmd->opcode = cpu_to_le16(CC33XX_NAB_SEND_CMD);

		/**
		 * length should include the following 4 bytes
		 * of the NAB command.
		 */
		nab_cmd->len = cpu_to_le16(total_len -
						sizeof(struct nab_header));
		nab_cmd->desc_length = cpu_to_le16(total_len -
						sizeof(struct nab_tx_header));
		nab_cmd->sd = 0;
		nab_cmd->flags = CC33XX_NAB_SEND_FLAGS;

		desc = skb_push(skb, total_skb_len - skb->len);

		cc33xx_set_tx_desc_blocks(desc, total_blocks);

		desc->id = id;

		cc33xx_debug(cc->dev, DEBUG_TX,
			     "tx allocate id %u skb 0x%p tx_memblocks %d\n",
			     id, skb, desc->cc33xx_mem.total_mem_blocks);

		cc->tx_blocks_available -= total_blocks;
		cc->tx_allocated_blocks += total_blocks;

		ac = cc33xx_tx_get_queue(skb_get_queue_mapping(skb));
		desc->ac = ac;
		cc->tx_allocated_pkts[ac]++;

		if (test_bit(hlid, cc->links_map))
			cc->links[hlid].allocated_pkts++;

		ret = 0;

		cc33xx_debug(cc->dev, DEBUG_TX,
			     "tx_allocate: size: %d, blocks: %d, id: %d\n",
			     total_len, total_blocks, id);
	} else {
		cc33xx_free_tx_id(cc, id);
	}

	return ret;
}

static void cc33xx_tx_fill_hdr(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			       struct sk_buff *skb, u32 extra,
			       struct ieee80211_tx_info *control, u8 hlid)
{
	struct cc33xx_tx_hw_descr *desc;
	int ac, rate_idx;
	u16 tx_attr = 0;
	__le16 frame_control;
	struct ieee80211_hdr *hdr;
	u8 *frame_start;
	u8 session_id;

	desc = (struct cc33xx_tx_hw_descr *)skb->data;

	frame_start = (u8 *)(desc + 1);
	hdr = (struct ieee80211_hdr *)(frame_start + extra);
	frame_control = hdr->frame_control;

	/* relocate space for security header */
	if (extra) {
		int hdrlen = ieee80211_hdrlen(frame_control);

		memmove(frame_start, hdr, hdrlen);
		skb_set_network_header(skb, skb_network_offset(skb) + extra);
	}

	if (wlvif->bss_type != BSS_TYPE_AP_BSS)
		desc->life_time = cpu_to_le16(CC33XX_TX_HW_MGMT_PKT_LIFETIME_TU);
	else
		desc->life_time = cpu_to_le16(CC33XX_TX_HW_AP_MODE_PKT_LIFETIME_TU);

	/* queue */
	ac = cc33xx_tx_get_queue(skb_get_queue_mapping(skb));
	desc->tid = skb->priority;

	session_id = cc->session_ids[hlid];

	if (cc->quirks & CC33XX_QUIRK_AP_ZERO_SESSION_ID &&
	    wlvif->bss_type == BSS_TYPE_AP_BSS)
		session_id = 0;

	/* configure the tx attributes */
	tx_attr = session_id << CC33XX_TX_HW_ATTR_OFST_SESSION_COUNTER;

	desc->hlid = hlid;
	if (wlvif->bss_type != BSS_TYPE_AP_BSS) {
		/* if the packets are data packets
		 * send them with AP rate policies (EAPOLs are an exception),
		 * otherwise use default basic rates
		 */
		if (skb->protocol == cpu_to_be16(ETH_P_PAE))
			rate_idx = wlvif->sta.basic_rate_idx;
		else if (control->flags & IEEE80211_TX_CTL_NO_CCK_RATE)
			rate_idx = wlvif->sta.p2p_rate_idx;
		else if (ieee80211_is_data(frame_control))
			rate_idx = wlvif->sta.ap_rate_idx;
		else
			rate_idx = wlvif->sta.basic_rate_idx;
	} else {
		if (hlid == wlvif->ap.global_hlid)
			rate_idx = wlvif->ap.mgmt_rate_idx;
		else if (hlid == wlvif->ap.bcast_hlid ||
			 skb->protocol == cpu_to_be16(ETH_P_PAE) ||
			 !ieee80211_is_data(frame_control))
			/* send non-data, bcast and EAPOLs using the
			 * min basic rate
			 */
			rate_idx = wlvif->ap.bcast_rate_idx;
		else
			rate_idx = wlvif->ap.ucast_rate_idx[ac];
	}

	tx_attr |= rate_idx << CC33XX_TX_HW_ATTR_OFST_RATE_POLICY;

	/* for WEP shared auth - no fw encryption is needed */
	if (ieee80211_is_auth(frame_control) &&
	    ieee80211_has_protected(frame_control))
		tx_attr |= CC33XX_TX_HW_ATTR_HOST_ENCRYPT;

	/* send EAPOL frames as voice */
	if (control->control.flags & IEEE80211_TX_CTRL_PORT_CTRL_PROTO)
		tx_attr |= CC33XX_TX_HW_ATTR_EAPOL_FRAME;

	desc->tx_attr = cpu_to_le16(tx_attr);

	cc33xx_set_tx_desc_data_len(cc, desc, skb);
}

/* caller must hold cc->mutex */
static int cc33xx_prepare_tx_frame(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				   struct sk_buff *skb, u32 buf_offset, u8 hlid)
{
	struct ieee80211_tx_info *info;
	u32 extra = 0;
	int ret = 0;
	u32 total_len;
	struct nab_tx_header nab_cmd;

	if (!skb) {
		dev_err(cc->dev, "discarding null skb\n");
		return -EINVAL;
	}

	if (hlid == CC33XX_INVALID_LINK_ID) {
		dev_err(cc->dev, "invalid hlid. dropping skb 0x%p\n", skb);
		return -EINVAL;
	}

	info = IEEE80211_SKB_CB(skb);

	if ((cc->quirks & CC33XX_QUIRK_TKIP_HEADER_SPACE) &&
	    info->control.hw_key &&
	    info->control.hw_key->cipher == WLAN_CIPHER_SUITE_TKIP)
		extra = CC33XX_EXTRA_SPACE_TKIP;

	if (info->control.hw_key) {
		bool is_wep;
		u8 idx = info->control.hw_key->hw_key_idx;
		u32 cipher = info->control.hw_key->cipher;

		is_wep = (cipher == WLAN_CIPHER_SUITE_WEP40) ||
			 (cipher == WLAN_CIPHER_SUITE_WEP104);

		if (WARN_ON(is_wep && wlvif->default_key != idx)) {
			ret = cc33xx_set_default_wep_key(cc, wlvif, idx);
			if (ret < 0)
				return ret;
			wlvif->default_key = idx;
		}
	}

	/* Add 4 bytes gap, may be filled later on by the PMAC. */
	extra += IEEE80211_HT_CTL_LEN;
	ret = cc33xx_tx_allocate(cc, skb, extra, buf_offset, hlid,
				 &nab_cmd);
	cc33xx_debug(cc->dev, DEBUG_TX, "cc33xx_tx_allocate %d\n", ret);

	if (ret < 0)
		return ret;

	cc33xx_tx_fill_hdr(cc, wlvif, skb, extra, info, hlid);

	if (wlvif->bss_type == BSS_TYPE_AP_BSS)
		cc33xx_tx_regulate_link(cc, wlvif, hlid);

	/* The length of each packet is stored in terms of
	 * words. Thus, we must pad the skb data to make sure its
	 * length is aligned.  The number of padding bytes is computed
	 * and set in cc33xx_tx_fill_hdr.
	 * In special cases, we want to align to a specific block size
	 * (eg. for wl128x with SDIO we align to 256).
	 */
	total_len = cc33xx_calc_packet_alignment(cc, skb->len);

	memcpy(cc->aggr_buf + buf_offset,
	       &nab_cmd, sizeof(struct nab_tx_header));
	memcpy(cc->aggr_buf + buf_offset + sizeof(struct nab_tx_header),
	       skb->data, skb->len);
	memset(cc->aggr_buf + buf_offset + sizeof(struct nab_tx_header)
		+ skb->len, 0, total_len - skb->len);

	return (total_len + sizeof(struct nab_tx_header));
}

u32 cc33xx_tx_enabled_rates_get(struct cc33xx *cc, u32 rate_set,
				enum nl80211_band rate_band)
{
	struct ieee80211_supported_band *band;
	u32 enabled_rates = 0;
	int bit;

	band = cc->hw->wiphy->bands[rate_band];
	for (bit = 0; bit < band->n_bitrates; bit++) {
		if (rate_set & 0x1)
			enabled_rates |= band->bitrates[bit].hw_value;
		rate_set >>= 1;
	}

	/* MCS rates indication are on bits 16 - 31 */
	rate_set >>= HW_HT_RATES_OFFSET - band->n_bitrates;

	for (bit = 0; bit < 16; bit++) {
		if (rate_set & 0x1)
			enabled_rates |= (CONF_HW_BIT_RATE_MCS_0 << bit);
		rate_set >>= 1;
	}

	return enabled_rates;
}

static inline int cc33xx_tx_get_mac80211_queue(struct cc33xx_vif *wlvif,
					       int queue)
{
	int mac_queue = wlvif->hw_queue_base;

	switch (queue) {
	case CONF_TX_AC_VO:
		return mac_queue + 0;
	case CONF_TX_AC_VI:
		return mac_queue + 1;
	case CONF_TX_AC_BE:
		return mac_queue + 2;
	case CONF_TX_AC_BK:
		return mac_queue + 3;
	default:
		return mac_queue + 2;
	}
}

static bool cc33xx_lnk_high_prio(struct cc33xx *cc, u8 hlid,
				 struct cc33xx_link *lnk)
{
	u8 thold;
	struct core_fw_status *core_fw_status = &cc->core_status->fw_info;
	unsigned long suspend_bitmap, fast_bitmap, ps_bitmap;

	suspend_bitmap = le32_to_cpu(core_fw_status->link_suspend_bitmap);
	fast_bitmap = le32_to_cpu(core_fw_status->link_fast_bitmap);
	ps_bitmap = le32_to_cpu(core_fw_status->link_ps_bitmap);

	/* suspended links are never high priority */
	if (test_bit(hlid, &suspend_bitmap))
		return false;

	/* the priority thresholds are taken from FW */
	if (test_bit(hlid, &fast_bitmap) && !test_bit(hlid, &ps_bitmap))
		thold = core_fw_status->tx_fast_link_prio_threshold;
	else
		thold = core_fw_status->tx_slow_link_prio_threshold;

	return lnk->allocated_pkts < thold;
}

static bool cc33xx_lnk_low_prio(struct cc33xx *cc, u8 hlid,
				struct cc33xx_link *lnk)
{
	u8 thold;
	struct core_fw_status *core_fw_status = &cc->core_status->fw_info;
	unsigned long suspend_bitmap, fast_bitmap, ps_bitmap;

	suspend_bitmap = le32_to_cpu(core_fw_status->link_suspend_bitmap);
	fast_bitmap = le32_to_cpu(core_fw_status->link_fast_bitmap);
	ps_bitmap = le32_to_cpu(core_fw_status->link_ps_bitmap);

	if (test_bit(hlid, &suspend_bitmap))
		thold = core_fw_status->tx_suspend_threshold;
	else if (test_bit(hlid, &fast_bitmap) && !test_bit(hlid, &ps_bitmap))
		thold = core_fw_status->tx_fast_stop_threshold;
	else
		thold = core_fw_status->tx_slow_stop_threshold;

	return lnk->allocated_pkts < thold;
}

static inline bool cc33xx_tx_is_data_present(struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)(skb->data);

	return ieee80211_is_data_present(hdr->frame_control);
}

static int cc33xx_tx_buf_write(struct cc33xx *cc, u32 buf_offset)
{
	u32 transfer_len = 0;
	u32 padding_size = 0;

	if (!buf_offset)
		return 0;

	transfer_len = __ALIGN_MASK(buf_offset, CC33XX_BUS_BLOCK_SIZE * 2 - 1);
	padding_size = transfer_len - buf_offset;
	memset(cc->aggr_buf + buf_offset, 0x33, padding_size);

	cc33xx_debug(cc->dev, DEBUG_TX, "sdio transaction length: %d\n", transfer_len);

	return cc33xx_raw_write(cc, CC33XX_NAB_DATA_ADDR, cc->aggr_buf,
				transfer_len, true);
}

/* Returns failure values only in case of failed bus ops within this function.
 * cc33xx_prepare_tx_frame retvals won't be returned in order to avoid
 * triggering recovery by higher layers when not necessary.
 * In case a FW command fails within cc33xx_prepare_tx_frame fails a recovery
 * will be queued in cc33xx_cmd_send. -EAGAIN/-EBUSY from prepare_tx_frame
 * can occur and are legitimate so don't propagate. -EINVAL will emit a WARNING
 * within prepare_tx_frame code but there's nothing we should do about those
 * as well.
 */
static int cc33xx_send_txq_pkts(struct cc33xx *cc, int qid, bool low_prio_tx)
{
	struct cc33xx_vif *wlvif = NULL;
	struct cc33xx_link *lnk = NULL;
	struct ieee80211_txq *txq = NULL;
	u8 hlid = CC33XX_INVALID_LINK_ID;
	struct sk_buff *skb = NULL;
	u32 buf_offset = 0;
	int ac = CONF_TX_AC_BE;
	int ret = 0;
	bool tx_allowed = true;

	memset(cc->aggr_buf, 0, 0x300);
	while ((txq = ieee80211_next_txq(cc->hw, qid))) {
		ac = cc33xx_tx_get_queue(txq->ac);
		wlvif = cc33xx_vif_to_data(txq->vif);
		hlid = cc33xx_tx_get_hlid(cc, wlvif, NULL, txq->sta);

		/* hlid invalid. Shouldn't happen */
		if (WARN_ON(hlid == CC33XX_INVALID_LINK_ID ||
			    !test_bit(hlid, wlvif->links_map))) {
			ieee80211_return_txq(cc->hw, txq, false);
			continue;
		}

		lnk = &cc->links[hlid];
		while (!cc33xx_is_queue_stopped(cc, wlvif, ac)) {
			tx_allowed = cc33xx_lnk_high_prio(cc, hlid, lnk);
			if (!tx_allowed && low_prio_tx)
				tx_allowed = cc33xx_lnk_low_prio(cc, hlid, lnk);

			if (!tx_allowed) {
				set_bit(CC33XX_FLAG_TX_PENDING, &cc->flags);
				break;
			}

			rcu_read_lock();
			skb = ieee80211_tx_dequeue_ni(cc->hw, txq);
			rcu_read_unlock();

			if (!skb)
				break;

try_again:
			ret = cc33xx_prepare_tx_frame(cc, wlvif, skb, buf_offset, hlid);
			if (ret == -EAGAIN) {
				/* Aggregation buffer is full.
				 * Flush buffer and try again.
				 */
				ret = cc33xx_tx_buf_write(cc, buf_offset);
				if (ret < 0) {
					ieee80211_return_txq(cc->hw, txq, false);
					return ret;
				}
				buf_offset = 0;
				goto try_again;
			} else if (ret == -EBUSY) {
				/* Firmware buffer is full.
				 * Queue last skb to pending, and stop aggregating.
				 */
				set_bit(CC33XX_FLAG_FW_TX_BUSY, &cc->flags);
				skb_queue_tail(&cc->links[hlid].tx_pending, skb);

				ieee80211_return_txq(cc->hw, txq, false);
				return cc33xx_tx_buf_write(cc, buf_offset);
			} else if (ret < 0) {
				ieee80211_free_txskb(cc->hw, skb);
			} else {
				buf_offset += ret;
			}
		}
		ieee80211_return_txq(cc->hw, txq, false);
	}

	return cc33xx_tx_buf_write(cc, buf_offset);
}

static int cc33xx_send_tx_pending_pkts(struct cc33xx *cc)
{
	struct sk_buff *skb = NULL;
	int hlid = CC33XX_INVALID_LINK_ID;
	struct cc33xx_link *lnk = NULL;
	u32 buf_offset = 0;
	struct cc33xx_vif *wlvif = NULL;
	int ret = 0;
	bool tx_allowed = true;

	for (hlid = 0; hlid < CC33XX_MAX_LINKS; hlid++) {
		if (!test_bit(hlid, cc->links_map))
			continue;

		lnk = &cc->links[hlid];

		/* First try high prio link, next consider high or low prio link. */
		for (int prio = 0; prio < 2; prio++) {
			while ((skb = skb_dequeue(&lnk->tx_pending))) {
				struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
				struct ieee80211_vif *vif = info->control.vif;

				wlvif = cc33xx_vif_to_data(vif);
				if (unlikely(!vif || !test_bit(hlid, wlvif->links_map))) {
					ieee80211_free_txskb(cc->hw, skb);
					continue;
				}

				tx_allowed = cc33xx_lnk_high_prio(cc, hlid, lnk);
				if (!tx_allowed && prio == 1)
					tx_allowed = cc33xx_lnk_low_prio(cc, hlid, lnk);

				if (!tx_allowed) {
					skb_queue_head(&cc->links[hlid].tx_pending, skb);
					break;
				}

try_again:
				ret = cc33xx_prepare_tx_frame(cc, wlvif, skb, buf_offset, hlid);
				if (ret == -EAGAIN) {
					/* Aggregation buffer is full.
					 * Flush buffer and try again.
					 */
					ret = cc33xx_tx_buf_write(cc, buf_offset);
					if (ret < 0)
						return ret;

					buf_offset = 0;
					goto try_again;
				} else if (ret == -EBUSY) {
					/* Firmware buffer is full.
					 * Queue back last skb, and stop aggregating.
					 */
					set_bit(CC33XX_FLAG_FW_TX_BUSY, &cc->flags);
					skb_queue_head(&cc->links[hlid].tx_pending, skb);
					return cc33xx_tx_buf_write(cc, buf_offset);
				} else if (ret < 0) {
					ieee80211_free_txskb(cc->hw, skb);
				} else {
					buf_offset += ret;
				}
			}
		}
	}

	return cc33xx_tx_buf_write(cc, buf_offset);
}

int cc33xx_do_tx_work(struct cc33xx *cc)
{
	int i, j, ret;

	guard(mutex)(&cc->mutex);

	if (unlikely(cc->state != CC33XX_STATE_ON ||
		     test_bit(CC33XX_FLAG_FW_TX_BUSY, &cc->flags)))
		return 0;

	clear_bit(CC33XX_FLAG_TX_PENDING, &cc->flags);

	/* process offchannel, non-bufferable or any pending skbs */
	ret = cc33xx_send_tx_pending_pkts(cc);
	if (ret < 0)
		return ret;

	/* process iTXQ packets */
	for (i = 0; i < NUM_TX_QUEUES; i++) {
		/* First try high prio link, next consider high or
		 * low prio link.
		 */
		for (j = 0; j < 2; j++) {
			if (test_bit(CC33XX_FLAG_FW_TX_BUSY, &cc->flags))
				return 0;

			ieee80211_txq_schedule_start(cc->hw, i);
			ret = cc33xx_send_txq_pkts(cc, i, j == 1);
			ieee80211_txq_schedule_end(cc->hw, i);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

void cc33xx_tx_work(struct work_struct *work)
{
	struct cc33xx *cc = container_of(work, struct cc33xx, tx_work);
	int ret;

	ret = cc33xx_do_tx_work(cc);
	if (ret < 0)
		cc33xx_queue_recovery_work(cc);
}

void cc33xx_tx_reset_link_queues(struct cc33xx *cc, u8 hlid)
{
	struct sk_buff *skb;
	struct ieee80211_tx_info *info;
	struct cc33xx_link *lnk = &cc->links[hlid];

	while ((skb = skb_dequeue(&lnk->tx_pending))) {
		cc33xx_debug(cc->dev, DEBUG_TX, "link freeing skb 0x%p\n", skb);
		info = IEEE80211_SKB_CB(skb);
		info->status.rates[0].idx = -1;
		info->status.rates[0].count = 0;
		ieee80211_tx_status_ni(cc->hw, skb);
	}
}

/* caller must *NOT* hold cc->mutex and TX must be stopped */
void cc33xx_tx_reset_wlvif(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	int i;

	/* TX failure */
	for_each_set_bit(i, wlvif->links_map, CC33XX_MAX_LINKS) {
		if (wlvif->bss_type == BSS_TYPE_AP_BSS &&
		    i != wlvif->ap.bcast_hlid &&
		    i != wlvif->ap.global_hlid) {
			/* this calls cc33xx_clear_link */
			cc33xx_free_sta(cc, wlvif, i);
		} else {
			u8 hlid = i;

			cc33xx_clear_link(cc, wlvif, &hlid);
		}
	}
}

int cc33xx_tx_total_queue_count(struct cc33xx *cc)
{
	int i, count = 0;

	for (i = 0; i < CC33XX_MAX_LINKS; i++)
		count += skb_queue_len(&cc->links[i].tx_pending);

	return count;
}

/* caller must hold cc->mutex and TX must be stopped */
void cc33xx_tx_reset(struct cc33xx *cc)
{
	int i;
	struct sk_buff *skb;
	struct ieee80211_tx_info *info;

	/* only reset the queues if something bad happened */
	if (cc33xx_tx_total_queue_count(cc) != 0) {
		for (i = 0; i < CC33XX_MAX_LINKS; i++)
			cc33xx_tx_reset_link_queues(cc, i);
	}

	for (i = 0; i < CC33XX_NUM_TX_DESCRIPTORS; i++) {
		if (!cc->tx_frames[i])
			continue;

		skb = cc->tx_frames[i];
		cc33xx_free_tx_id(cc, i);
		cc33xx_debug(cc->dev, DEBUG_TX, "freeing skb 0x%p\n", skb);

		/* Remove private headers before passing the skb to
		 * mac80211
		 */
		info = IEEE80211_SKB_CB(skb);
		skb_pull(skb, sizeof(struct cc33xx_tx_hw_descr));
		if ((cc->quirks & CC33XX_QUIRK_TKIP_HEADER_SPACE) &&
		    info->control.hw_key &&
		    info->control.hw_key->cipher ==
		    WLAN_CIPHER_SUITE_TKIP) {
			int hdrlen = ieee80211_get_hdrlen_from_skb(skb);

			memmove(skb->data + CC33XX_EXTRA_SPACE_TKIP,
				skb->data, hdrlen);
			skb_pull(skb, CC33XX_EXTRA_SPACE_TKIP);
		}

		info->status.rates[0].idx = -1;
		info->status.rates[0].count = 0;

		ieee80211_tx_status_ni(cc->hw, skb);
	}
}

#define CC33XX_TX_FLUSH_TIMEOUT 500000

/* caller must *NOT* hold cc->mutex */
void cc33xx_tx_flush(struct cc33xx *cc)
{
	unsigned long timeout, start_time;
	int i;

	start_time = jiffies;
	timeout = start_time + usecs_to_jiffies(CC33XX_TX_FLUSH_TIMEOUT);

	/* only one flush should be in progress, for consistent queue state */
	lockdep_assert_wiphy(cc->hw->wiphy);

	mutex_lock(&cc->mutex);
	if (cc->tx_frames_cnt == 0 && cc33xx_tx_total_queue_count(cc) == 0) {
		mutex_unlock(&cc->mutex);
		return;
	}

	cc33xx_stop_queues(cc, CC33XX_QUEUE_STOP_REASON_FLUSH);

	while (!time_after(jiffies, timeout)) {
		cc33xx_debug(cc->dev, DEBUG_MAC80211, "flushing tx buffer: %d %d\n",
			     cc->tx_frames_cnt,
			     cc33xx_tx_total_queue_count(cc));

		/* force Tx and give the driver some time to flush data */
		mutex_unlock(&cc->mutex);
		if (cc33xx_tx_total_queue_count(cc))
			cc33xx_tx_work(&cc->tx_work);
		msleep(20);
		mutex_lock(&cc->mutex);

		if (cc->tx_frames_cnt == 0 && cc33xx_tx_total_queue_count(cc) == 0) {
			cc33xx_debug(cc->dev, DEBUG_MAC80211, "tx flush took %d ms\n",
				     jiffies_to_msecs(jiffies - start_time));
			goto out_wake;
		}
	}

	dev_warn(cc->dev, "Unable to flush all TX buffers, timed out (timeout %d ms\n",
		 CC33XX_TX_FLUSH_TIMEOUT / 1000);

	/* forcibly flush all Tx buffers on our queues */
	for (i = 0; i < CC33XX_MAX_LINKS; i++)
		cc33xx_tx_reset_link_queues(cc, i);

out_wake:
	cc33xx_wake_queues(cc, CC33XX_QUEUE_STOP_REASON_FLUSH);
	mutex_unlock(&cc->mutex);
}

u32 cc33xx_tx_min_rate_get(u32 rate_set)
{
	if (WARN_ON(!rate_set))
		return 0;

	return BIT(__ffs(rate_set));
}

void cc33xx_stop_queue_locked(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			      u8 queue, enum cc33xx_queue_stop_reason reason)
{
	int hwq = cc33xx_tx_get_mac80211_queue(wlvif, queue);
	bool stopped = !!cc->queue_stop_reasons[hwq];

	/* queue should not be stopped for this reason */
	WARN_ON_ONCE(test_and_set_bit(reason, &cc->queue_stop_reasons[hwq]));

	if (stopped)
		return;

	ieee80211_stop_queue(cc->hw, hwq);
}

void cc33xx_stop_queues(struct cc33xx *cc,
			enum cc33xx_queue_stop_reason reason)
{
	int i;

	guard(spinlock_irqsave)(&cc->cc_lock);

	set_bit(CC33XX_FLAG_TX_STOPPED, &cc->flags);

	/* mark all possible queues as stopped */
	for (i = 0; i < CC33XX_NUM_MAC_ADDRESSES * NUM_TX_QUEUES; i++) {
		WARN_ON_ONCE(test_and_set_bit(reason,
					      &cc->queue_stop_reasons[i]));
	}

	/* use the global version to make sure all vifs in mac80211 we don't
	 * know are stopped.
	 */
	ieee80211_stop_queues(cc->hw);
}

void cc33xx_wake_queues(struct cc33xx *cc,
			enum cc33xx_queue_stop_reason reason)
{
	int i;

	guard(spinlock_irqsave)(&cc->cc_lock);

	/* mark all possible queues as awake */
	for (i = 0; i < CC33XX_NUM_MAC_ADDRESSES * NUM_TX_QUEUES; i++) {
		WARN_ON_ONCE(!test_and_clear_bit(reason,
						 &cc->queue_stop_reasons[i]));
	}

	/* use the global version to make sure all vifs in mac80211 we don't
	 * know are woken up.
	 */
	ieee80211_wake_queues(cc->hw);

	clear_bit(CC33XX_FLAG_TX_STOPPED, &cc->flags);

	if (!test_bit(CC33XX_FLAG_FW_TX_BUSY, &cc->flags) &&
	    test_bit(CC33XX_FLAG_TX_PENDING, &cc->flags))
		ieee80211_queue_work(cc->hw, &cc->tx_work);
}

bool cc33xx_is_queue_stopped_by_reason(struct cc33xx *cc,
				       struct cc33xx_vif *wlvif, u8 queue,
				       enum cc33xx_queue_stop_reason reason)
{
	bool stopped;

	guard(spinlock_irqsave)(&cc->cc_lock);

	stopped = cc33xx_is_queue_stopped_by_reason_locked(cc, wlvif, queue,
							   reason);
	return stopped;
}

bool cc33xx_is_queue_stopped_by_reason_locked(struct cc33xx *cc,
					      struct cc33xx_vif *wlvif, u8 queue,
				       enum cc33xx_queue_stop_reason reason)
{
	int hwq = cc33xx_tx_get_mac80211_queue(wlvif, queue);

	assert_spin_locked(&cc->cc_lock);
	return test_bit(reason, &cc->queue_stop_reasons[hwq]);
}

bool cc33xx_is_queue_stopped(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			     u8 queue)
{
	int hwq = cc33xx_tx_get_mac80211_queue(wlvif, queue);
	bool stopped;

	guard(spinlock_irqsave)(&cc->cc_lock);
	stopped = !!cc->queue_stop_reasons[hwq];

	return stopped;
}

bool cc33xx_is_queue_stopped_locked(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				    u8 queue)
{
	int hwq = cc33xx_tx_get_mac80211_queue(wlvif, queue);

	assert_spin_locked(&cc->cc_lock);
	return !!cc->queue_stop_reasons[hwq];
}

static int cc33xx_tx_complete_packet(struct cc33xx *cc, u8 tx_stat_byte,
				     struct core_fw_status *core_fw_status)
{
	struct ieee80211_tx_info *info;
	struct sk_buff *skb;
	int id = tx_stat_byte & CC33XX_TX_STATUS_DESC_ID_MASK;
	bool tx_success;
	struct cc33xx_tx_hw_descr *tx_desc;
	u16 desc_session_idx;

	/* check for id legality */
	if (unlikely(id >= CC33XX_NUM_TX_DESCRIPTORS ||
		     !cc->tx_frames[id])) {
		dev_warn(cc->dev, "illegal id in tx completion: %d\n", id);

		print_hex_dump(KERN_DEBUG, "fw_info local:",
			       DUMP_PREFIX_OFFSET, 16, 4, (u8 *)(core_fw_status),
			       sizeof(struct core_fw_status), false);

		cc33xx_queue_recovery_work(cc);
		return -EINVAL;
	}

	/* a zero bit indicates Tx success */
	tx_success = !(tx_stat_byte & BIT(CC33XX_TX_STATUS_STAT_BIT_IDX));

	skb = cc->tx_frames[id];
	info = IEEE80211_SKB_CB(skb);
	tx_desc = (struct cc33xx_tx_hw_descr *)skb->data;

	/* update the TX status info */
	if (tx_success && !(info->flags & IEEE80211_TX_CTL_NO_ACK))
		info->flags |= IEEE80211_TX_STAT_ACK;

	info->status.rates[0].count = 1; /* no data about retries */
	info->status.ack_signal = -1;

	if (!tx_success)
		cc->stats.retry_count++;

	/* remove private header from packet */
	skb_pull(skb, sizeof(struct cc33xx_tx_hw_descr));

	/* remove TKIP header space if present */
	if ((cc->quirks & CC33XX_QUIRK_TKIP_HEADER_SPACE) &&
	    info->control.hw_key &&
	    info->control.hw_key->cipher == WLAN_CIPHER_SUITE_TKIP) {
		int hdrlen = ieee80211_get_hdrlen_from_skb(skb);

		memmove(skb->data + CC33XX_EXTRA_SPACE_TKIP, skb->data, hdrlen);
		skb_pull(skb, CC33XX_EXTRA_SPACE_TKIP);
	}

	cc33xx_debug(cc->dev, DEBUG_TX,
		     "tx status id %u skb 0x%p success %d, tx_memblocks %d\n",
		     id, skb, tx_success, tx_desc->cc33xx_mem.total_mem_blocks);

	/**
	 * in order to update the memory management
	 * we should have total_blocks, ac, and hlid
	 */
	cc->tx_blocks_available += tx_desc->cc33xx_mem.total_mem_blocks;
	cc->tx_allocated_blocks -= tx_desc->cc33xx_mem.total_mem_blocks;
	/* per queue */

	/* prevent wrap-around in freed-packets counter */
	cc->tx_allocated_pkts[tx_desc->ac]--;

	/* per link */
	desc_session_idx = le16_get_bits(tx_desc->tx_attr, CC33XX_TX_HW_ATTR_SESSION_COUNTER);

	if (cc->session_ids[tx_desc->hlid] == desc_session_idx)
		cc->links[tx_desc->hlid].allocated_pkts--;

	cc33xx_free_tx_id(cc, id);

	/* new mem blocks are available now */
	clear_bit(CC33XX_FLAG_FW_TX_BUSY, &cc->flags);

	/* return the packet to the stack */
	skb_queue_tail(&cc->deferred_tx_queue, skb);
	queue_work(cc->freezable_wq, &cc->netstack_work);

	return 0;
}

void cc33xx_tx_immediate_complete(struct cc33xx *cc)
{
	u8 tx_result_queue_index;
	struct core_fw_status core_fw_status;
	u8 i;
	int ret = -EINVAL, tx_busy;

	claim_core_status_lock(cc);
	memcpy(&core_fw_status, &cc->core_status->fw_info,
	       sizeof(struct core_fw_status));

	tx_result_queue_index = cc->core_status->fw_info.tx_result_queue_index;
	/* Lock guarantees we shadow tx_result_queue_index NOT during
	 * an active transaction. Subsequent references to fw_info can be done
	 * without locking as long we do not pass this index.
	 */
	release_core_status_lock(cc);

	cc33xx_debug(cc->dev, DEBUG_TX, "last released desc = %d, current idx = %d\n",
		     cc->last_fw_rls_idx, tx_result_queue_index);

	/* nothing to do here */
	if (cc->last_fw_rls_idx == tx_result_queue_index)
		return;

	/* freed Tx descriptors */

	if (tx_result_queue_index >= CC33XX_TX_RESULT_QUEUE_SIZE) {
		dev_err(cc->dev, "invalid desc release index %d\n",
			tx_result_queue_index);
		WARN_ON(1);
		return;
	}

	cc33xx_debug(cc->dev, DEBUG_TX, "TX result queue! priv last fw idx %d, current result index %d\n",
		     cc->last_fw_rls_idx, tx_result_queue_index);

	tx_busy = test_bit(CC33XX_FLAG_FW_TX_BUSY, &cc->flags);

	for (i = cc->last_fw_rls_idx; i != tx_result_queue_index;
	     i = (i + 1) % CC33XX_TX_RESULT_QUEUE_SIZE) {
		ret = cc33xx_tx_complete_packet(cc, core_fw_status.tx_result_queue[i],
						&core_fw_status);
	}

	cc->last_fw_rls_idx = tx_result_queue_index;

	/* if new mem blocks available and tx was busy or pending,
	 * queue the tx work.
	 */
	if (!ret && !test_bit(CC33XX_FLAG_TX_STOPPED, &cc->flags) &&
	    (tx_busy || test_bit(CC33XX_FLAG_TX_PENDING, &cc->flags)))
		ieee80211_queue_work(cc->hw, &cc->tx_work);
}
