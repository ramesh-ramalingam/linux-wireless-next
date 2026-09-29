// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2026 Texas Instruments Incorporated - https://www.ti.com/
 */

#include "acx.h"
#include "event.h"
#include "io.h"
#include "tx.h"

#define CC33XX_REBOOT_TIMEOUT_MSEC		100

static void init_cmd_header(struct cc33xx_cmd_header *header,
			    size_t cmd_len, u16 id)
{
	header->nab_header.len = cpu_to_le16(cmd_len);
	WARN_ON(le16_to_cpu(header->nab_header.len) != cmd_len);

	header->nab_header.sync_pattern = cpu_to_le32(CC33XX_HOST_SYNC_PATTERN);
	header->nab_header.opcode = cpu_to_le16(id);
}

int cc33xx_set_max_buffer_size(struct cc33xx *cc, enum buffer_size max_buffer_size)
{
	switch (max_buffer_size) {
	case INI_MAX_BUFFER_SIZE:
		/* INI FILE PAYLOAD SIZE + INI CMD PARAM + INT */
		cc->max_cmd_size = CC33XX_INI_CMD_MAX_SIZE;
		cc->max_cmd_size += sizeof(struct cc33xx_cmd_ini_params_download);
		cc->max_cmd_size += sizeof(u32);
		break;

	case CMD_MAX_BUFFER_SIZE:
		cc->max_cmd_size = CC33XX_CMD_MAX_SIZE;
		break;

	default:
		dev_warn(cc->dev, "max_buffer_size invalid, not changing buffer size\n");
		break;
	}

	return 0;
}

static int send_buffer(struct cc33xx *cc, int cmd_box_addr,
		       void *buf, size_t len)
{
	size_t max_cmd_size_align;

	memcpy(cc->cmd_buf, buf, len);

	memset(cc->cmd_buf + len, 0, (CC33XX_CMD_BUFFER_SIZE) - len);

	max_cmd_size_align = __ALIGN_MASK(cc->max_cmd_size,
					  CC33XX_BUS_BLOCK_SIZE * 2 - 1);

	return cc33xx_raw_write(cc, cmd_box_addr, cc->cmd_buf,
			    max_cmd_size_align, true);
}

/* send command to firmware
 *
 * @cc: cc struct
 * @id: command id
 * @buf: buffer containing the command, must work with dma
 * @len: length of the buffer
 * return the cmd status code on success.
 */
static int __cc33xx_cmd_send(struct cc33xx *cc, u16 id, void *buf,
			     size_t len, size_t res_len, bool sync)
{
	struct cc33xx_cmd_header *cmd;
	unsigned long timeout;
	int ret;

	if (id >= CMD_LAST_SUPPORTED_COMMAND) {
		cc33xx_debug(cc->dev, DEBUG_CMD, "command ID: %d, blocked\n", id);
		return CMD_STATUS_SUCCESS;
	}

	if (WARN_ON(len < sizeof(*cmd))		||
	    WARN_ON(len > cc->max_cmd_size)	||
	    WARN_ON(len % 4 != 0))
		return -EIO;

	cmd = buf;
	cmd->id = cpu_to_le16(id);
	cmd->status = 0;

	init_cmd_header(cmd, len, id);

	guard(mutex)(&cc->cmd_lock);

	init_completion(&cc->command_complete);
	ret = send_buffer(cc, CC33XX_NAB_DATA_ADDR, buf, len);

	if (ret < 0)
		return ret;

	if (unlikely(!sync))
		return CMD_STATUS_SUCCESS;

	timeout = msecs_to_jiffies(CC33XX_COMMAND_TIMEOUT);

	ret = wait_for_completion_timeout(&cc->command_complete, timeout);

	if (ret < 1) {
		cc33xx_debug(cc->dev, DEBUG_CMD, "Command T.O\n");
		return -EIO;
	}

	if (!res_len)
		return CMD_STATUS_SUCCESS; /* Response should be discarded */

	if (WARN_ON(cc->result_length > res_len)) {
		dev_err(cc->dev, "Error, insufficient response buffer\n");
		return -EINVAL;
	}

	switch (id) {
	case CMD_INTERROGATE:
	case CMD_DEBUG_READ:
	case CMD_TEST_MODE:
	case CMD_BM_READ_DEVICE_INFO:
		memcpy(buf + sizeof(struct nab_header), cc->command_result,
		       cc->result_length);
		break;
	default:
		memcpy(buf, cc->command_result, cc->result_length);
		break;
	}

	return CMD_STATUS_SUCCESS;
}

/* send command to fw and return cmd status on success
 * valid_rets contains a bitmap of allowed error codes
 */
static int cc33xx_cmd_send_failsafe(struct cc33xx *cc, u16 id, void *buf,
				    size_t len, size_t res_len,
				    unsigned long valid_rets)
{
	int ret = __cc33xx_cmd_send(cc, id, buf, len, res_len, true);

	if (ret < 0)
		goto fail;

	/* success is always a valid status */
	valid_rets |= BIT(CMD_STATUS_SUCCESS);

	if (ret >= MAX_COMMAND_STATUS || !test_bit(ret, &valid_rets)) {
		dev_err(cc->dev, "command execute failure %d\n", ret);
		ret = -EIO;
	}

	return ret;
fail:
	cc33xx_queue_recovery_work(cc);
	return ret;
}

/* wrapper for cc33xx_cmd_send that accept only CMD_STATUS_SUCCESS
 * return 0 on success.
 */
int cc33xx_cmd_send(struct cc33xx *cc, u16 id, void *buf,
		    size_t len, size_t res_len)
{
	int ret;

	if ((enum cc33xx_cmd)id >= CMD_LAST_SUPPORTED_COMMAND)
		return 0;

	/* Support id */
	switch ((enum cc33xx_cmd)id) {
	case CMD_EMPTY:
	case CMD_START_DHCP_MGMT_SEQ:
	case CMD_STOP_DHCP_MGMT_SEQ:
	case CMD_START_SECURITY_MGMT_SEQ:
	case CMD_STOP_SECURITY_MGMT_SEQ:
	case CMD_START_ARP_MGMT_SEQ:
	case CMD_STOP_ARP_MGMT_SEQ:
	case CMD_START_DNS_MGMT_SEQ:
	case CMD_STOP_DNS_MGMT_SEQ:
	case CMD_SEND_DEAUTH_DISASSOC:
	case CMD_SCHED_STATE_EVENT:
		return 0;
	default:
		break;
	}

	ret = cc33xx_cmd_send_failsafe(cc, id, buf, len, res_len, 0);
	if (ret < 0)
		return ret;
	return 0;
}

static int cc33xx_count_role_set_bits(unsigned long role_map)
{
	int count = 0;
	/* if device bit is set ( BIT_2 = ROLE_DEVICE)
	 * since role device is not counted
	 * remove it from map
	 */
	role_map &= ~BIT(2);

	while (role_map != 0) {
		count += role_map & 1;
		role_map >>= 1;
	}

	return count;
}

int cc33xx_cmd_role_enable(struct cc33xx *cc, const u8 *addr,
			   u8 role_type, u8 *role_id)
{
	int ret;
	unsigned long role_count;
	union cc33xx_cmd_role_enable_command *buf __free(kfree) = NULL;

	role_count = *cc->roles_map;
	ret = cc33xx_count_role_set_bits(role_count);

	/* do not enable more than 2 roles at once, exception is device role */
	if (ret >= 2 && role_type != CC33XX_ROLE_DEVICE) {
		dev_err(cc->dev, "failed to initiate cmd role enable\n");
		return -EBUSY;
	}

	if (WARN_ON(*role_id != CC33XX_INVALID_ROLE_ID))
		return -EBUSY;

	buf = kzalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;

	memcpy(buf->cmd.mac_address, addr, ETH_ALEN);
	buf->cmd.role_type = role_type;

	ret = cc33xx_cmd_send(cc, CMD_ROLE_ENABLE, &buf->cmd, sizeof(buf->cmd),
			      sizeof(buf->resp));
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd role enable\n");
		return ret;
	}
	__set_bit(buf->resp.role_id, cc->roles_map);
	*role_id = buf->resp.role_id;

	return ret;
}

int cc33xx_cmd_role_disable(struct cc33xx *cc, u8 *role_id)
{
	struct cc33xx_cmd_role_disable *cmd __free(kfree) = NULL;
	int ret;

	if (WARN_ON(*role_id == CC33XX_INVALID_ROLE_ID))
		return -ENOENT;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = *role_id;

	ret = cc33xx_cmd_send(cc, CMD_ROLE_DISABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd role disable\n");
		return ret;
	}

	__clear_bit(*role_id, cc->roles_map);
	*role_id = CC33XX_INVALID_ROLE_ID;

	return ret;
}

int cc33xx_set_link(struct cc33xx *cc, struct cc33xx_vif *wlvif, u8 link)
{
	guard(mutex)(&cc->mutex);

	/* these bits are used by op_tx */
	scoped_guard(spinlock_irqsave, &cc->cc_lock) {
		__set_bit(link, cc->links_map);
		__set_bit(link, wlvif->links_map);
	}
	cc->links[link].wlvif = wlvif;

	/* Take saved value for total freed packets from wlvif, in case this is
	 * recovery/resume
	 */
	if (wlvif->bss_type != BSS_TYPE_AP_BSS)
		cc->links[link].total_freed_pkts = wlvif->total_freed_pkts;

	cc->active_link_count++;

	return 0;
}

void cc33xx_clear_link(struct cc33xx *cc, struct cc33xx_vif *wlvif, u8 *hlid)
{
	if (*hlid == CC33XX_INVALID_LINK_ID)
		return;

	guard(mutex)(&cc->mutex);

	/* these bits are used by op_tx */
	scoped_guard(spinlock_irqsave, &cc->cc_lock) {
		__clear_bit(*hlid, cc->links_map);
		__clear_bit(*hlid, wlvif->links_map);
	}

	cc->links[*hlid].prev_freed_pkts = 0;
	cc->links[*hlid].ba_bitmap = 0;
	eth_zero_addr(cc->links[*hlid].addr);

	/* At this point op_tx() will not add more packets to the queues. We
	 * can purge them.
	 */
	cc33xx_tx_reset_link_queues(cc, *hlid);
	cc->links[*hlid].wlvif = NULL;

	if (wlvif->bss_type == BSS_TYPE_AP_BSS &&
	    *hlid == wlvif->ap.bcast_hlid) {
		u32 sqn_padding = CC33XX_TX_SQN_POST_RECOVERY_PADDING;
		/* save the total freed packets in the wlvif, in case this is
		 * recovery or suspend
		 */
		wlvif->total_freed_pkts = cc->links[*hlid].total_freed_pkts;

		/* increment the initial seq number on recovery to account for
		 * transmitted packets that we haven't yet got in the FW status
		 */
		if (test_bit(CC33XX_FLAG_RECOVERY_IN_PROGRESS, &cc->flags))
			wlvif->total_freed_pkts += sqn_padding;
	}

	cc->links[*hlid].total_freed_pkts = 0;

	*hlid = CC33XX_INVALID_LINK_ID;
	cc->active_link_count--;
	WARN_ON_ONCE(cc->active_link_count < 0);
}

static u8 cc33xx_get_native_channel_type(u8 nl_channel_type)
{
	switch (nl_channel_type) {
	case NL80211_CHAN_NO_HT:
		return CC33XX_CHAN_NO_HT;
	case NL80211_CHAN_HT20:
		return CC33XX_CHAN_HT20;
	case NL80211_CHAN_HT40MINUS:
		return CC33XX_CHAN_HT40MINUS;
	case NL80211_CHAN_HT40PLUS:
		return CC33XX_CHAN_HT40PLUS;
	default:
		WARN_ON(1);
		return CC33XX_CHAN_NO_HT;
	}
}

static int cc33xx_cmd_role_start_dev(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				     enum nl80211_band band, int channel)
{
	int ret;
	union cc33xx_cmd_role_start_command *buf __free(kfree) = NULL;

	buf = kzalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;

	buf->cmd.role_id = wlvif->dev_role_id;
	buf->cmd.role_type = CC33XX_ROLE_DEVICE;
	if (band == NL80211_BAND_5GHZ)
		buf->cmd.band = CC33XX_BAND_5GHZ;
	buf->cmd.channel = channel;
	buf->cmd.channel_type = cc33xx_get_native_channel_type(wlvif->channel_type);

	ret = cc33xx_cmd_send(cc, CMD_ROLE_START, &buf->cmd, sizeof(buf->cmd),
			      sizeof(buf->resp));
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd role device start\n");
		return ret;
	}

	scoped_guard(mutex, &cc->mutex) {
		wlvif->dev_hlid = buf->resp.sta.hlid;

		cc->links[wlvif->dev_hlid].allocated_pkts = 0;
		cc->session_ids[wlvif->dev_hlid] = buf->resp.sta.session;
	}

	cc33xx_debug(cc->dev, DEBUG_CMD, "role start: roleid=%d, hlid=%d, session=%d\n",
		     wlvif->dev_role_id, buf->resp.sta.hlid,
		     buf->resp.sta.session);
	cc33xx_set_link(cc, wlvif, wlvif->dev_hlid);

	return 0;
}

int cc33xx_cmd_role_stop_transceiver(struct cc33xx *cc)
{
	struct cc33xx_cmd_role_stop *cmd __free(kfree) = NULL;
	int ret;

	if (unlikely(cc->state != CC33XX_STATE_ON || !cc->plt))
		return -EINVAL;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = cc->plt_role_id;

	ret = cc33xx_cmd_send(cc, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "transceiver - failed to initiate cmd role stop\n");

	return ret;
}

int cc33xx_cmd_plt_disable(struct cc33xx *cc)
{
	struct cc33xx_cmd_PLT_disable *cmd __free(kfree) = NULL;
	int ret;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	ret = cc33xx_cmd_send(cc, CMD_PLT_DISABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "transceiver: failed to disable Transceiver mode\n");

	return ret;
}

static int cc333xx_cmd_role_stop_dev(struct cc33xx *cc,
				     struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_role_stop *cmd __free(kfree) = NULL;
	int ret;

	if (WARN_ON(wlvif->dev_hlid == CC33XX_INVALID_LINK_ID))
		return -EINVAL;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = wlvif->dev_role_id;

	ret = cc33xx_cmd_send(cc, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd role stop\n");
		return ret;
	}

	cc33xx_clear_link(cc, wlvif, &wlvif->dev_hlid);

	return ret;
}

int cc33xx_cmd_plt_enable(struct cc33xx *cc)
{
	struct cc33xx_cmd_PLT_enable *cmd __free(kfree) = NULL;
	s32 ret;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	ret = cc33xx_cmd_send(cc, CMD_PLT_ENABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "Failed to send CMD_PLT_ENABLE\n");

	return ret;
}

int cc33xx_cmd_role_start_transceiver(struct cc33xx *cc, u8 role_id)
{
	struct cc33xx_cmd_role_start *cmd __free(kfree) = NULL;
	s32 ret;
	u8 role_type = ROLE_TRANSCEIVER;

	/* Default values */
	u8 band = NL80211_BAND_2GHZ;
	u8 channel = 6;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_type = role_type;
	cmd->role_id = role_id;
	cmd->channel = channel;
	cmd->band = band;

	ret = cc33xx_cmd_send(cc, CMD_ROLE_START, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "failed to initiate cmd role start PLT\n");

	return ret;
}

int cc33xx_cmd_role_start_sta(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);
	union cc33xx_cmd_role_start_command *buf __free(kfree) = NULL;

	u32 supported_rates;
	int ret;

	buf = kzalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;

	buf->cmd.role_id = wlvif->role_id;
	buf->cmd.role_type = CC33XX_ROLE_STA;
	buf->cmd.channel = wlvif->channel;
	if (wlvif->band == NL80211_BAND_5GHZ) {
		buf->cmd.band = CC33XX_BAND_5GHZ;
		buf->cmd.sta.basic_rate_set = cpu_to_le32(wlvif->basic_rate_set
							& ~CONF_TX_CCK_RATES);
	} else {
		buf->cmd.sta.basic_rate_set = cpu_to_le32(wlvif->basic_rate_set);
	}
	buf->cmd.sta.beacon_interval = cpu_to_le16(wlvif->beacon_int);
	buf->cmd.sta.ssid_type = CC33XX_SSID_TYPE_ANY;
	buf->cmd.sta.ssid_len = wlvif->ssid_len;
	memcpy(buf->cmd.sta.ssid, wlvif->ssid, wlvif->ssid_len);
	memcpy(buf->cmd.sta.bssid, vif->bss_conf.bssid, ETH_ALEN);

	supported_rates = CONF_TX_ENABLED_RATES | CONF_TX_MCS_RATES | wlvif->rate_set;
	if (wlvif->band == NL80211_BAND_5GHZ)
		supported_rates &= ~CONF_TX_CCK_RATES;

	if (wlvif->p2p)
		supported_rates &= ~CONF_TX_CCK_RATES;

	buf->cmd.sta.local_rates = cpu_to_le32(supported_rates);

	buf->cmd.channel_type = cc33xx_get_native_channel_type(wlvif->channel_type);

	/* We don't have the correct remote rates in this stage.  The
	 * rates will be reconfigured later, after association, if the
	 * firmware supports ACX_PEER_CAP.  Otherwise, there's nothing
	 * we can do, so use all supported_rates here.
	 */
	buf->cmd.sta.remote_rates = cpu_to_le32(supported_rates);

	ret = cc33xx_cmd_send(cc, CMD_ROLE_START, &buf->cmd, sizeof(buf->cmd),
			      sizeof(buf->resp));
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd role start sta\n");
		return ret;
	}

	scoped_guard(mutex, &cc->mutex) {
		wlvif->sta.role_chan_type = wlvif->channel_type;

		wlvif->sta.hlid = buf->resp.sta.hlid;
		cc->links[wlvif->sta.hlid].allocated_pkts = 0;
		cc->session_ids[wlvif->sta.hlid] = buf->resp.sta.session;
	}

	cc33xx_debug(cc->dev, DEBUG_CMD, "role start: roleid=%d, hlid=%d, session=%d basic_rate_set: 0x%x, remote_rates: 0x%x\n",
		     wlvif->role_id,
		     buf->resp.sta.hlid, buf->resp.sta.session,
		     wlvif->basic_rate_set, wlvif->rate_set);
	cc33xx_set_link(cc, wlvif, wlvif->sta.hlid);

	return 0;
}

int cc33xx_cmd_role_stop_sta(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_role_stop *cmd __free(kfree) = NULL;
	int ret;

	if (WARN_ON(wlvif->sta.hlid == CC33XX_INVALID_LINK_ID))
		return -EINVAL;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = wlvif->role_id;

	ret = cc33xx_cmd_send(cc, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd role stop sta\n");
		return ret;
	}

	cc33xx_clear_link(cc, wlvif, &wlvif->sta.hlid);

	return ret;
}

int cc33xx_cmd_role_start_ap(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	union cc33xx_cmd_role_start_command *buf __free(kfree) = NULL;
	struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);
	struct ieee80211_bss_conf *bss_conf = &vif->bss_conf;
	u32 supported_rates;
	int ret;

	/* If MESH --> ssid_len is always 0 */
	if (!ieee80211_vif_is_mesh(vif)) {
		/* trying to use hidden SSID with an old hostapd version */
		if (wlvif->ssid_len == 0 && !bss_conf->hidden_ssid) {
			dev_err(cc->dev, "got a null SSID from beacon/bss\n");
			return -EINVAL;
		}
	}

	buf = kzalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;

	buf->cmd.role_id = wlvif->role_id;
	buf->cmd.role_type = CC33XX_ROLE_AP;
	buf->cmd.ap.basic_rate_set = cpu_to_le32(wlvif->basic_rate_set);
	buf->cmd.ap.beacon_interval = cpu_to_le16(wlvif->beacon_int);
	buf->cmd.ap.dtim_interval = bss_conf->dtim_period;
	buf->cmd.ap.wmm = wlvif->wmm_enabled;
	buf->cmd.channel = wlvif->channel;
	buf->cmd.channel_type = cc33xx_get_native_channel_type(wlvif->channel_type);

	supported_rates = CONF_TX_ENABLED_RATES | CONF_TX_MCS_RATES;
	if (wlvif->p2p)
		supported_rates &= ~CONF_TX_CCK_RATES;

	buf->cmd.ap.local_rates = cpu_to_le32(supported_rates);

	switch (wlvif->band) {
	case NL80211_BAND_2GHZ:
		buf->cmd.band = CC33XX_BAND_2_4GHZ;
		break;
	case NL80211_BAND_5GHZ:
		buf->cmd.band = CC33XX_BAND_5GHZ;
		break;
	default:
		dev_warn(cc->dev, "ap start - unknown band: %d\n", (int)wlvif->band);
		buf->cmd.band = CC33XX_BAND_2_4GHZ;
		break;
	}

	ret = cc33xx_cmd_send(cc, CMD_ROLE_START, buf, sizeof(buf->cmd),
			      sizeof(buf->resp));
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd role start ap\n");
		return ret;
	}

	scoped_guard(mutex, &cc->mutex) {
		wlvif->ap.global_hlid = buf->resp.ap.global_hlid;
		wlvif->ap.bcast_hlid  = buf->resp.ap.broadcast_hlid;
		cc->session_ids[wlvif->ap.global_hlid] = buf->resp.ap.global_session_id;
		cc->session_ids[wlvif->ap.bcast_hlid] = buf->resp.ap.bcast_session_id;
	}

	cc33xx_debug(cc->dev, DEBUG_CMD, "role start: roleid=%d, global_hlid=%d, broadcast_hlid=%d, global_session_id=%d, bcast_session_id=%d, basic_rate_set: 0x%x, remote_rates: 0x%x\n",
		     wlvif->role_id,
		     buf->resp.ap.global_hlid,
		     buf->resp.ap.broadcast_hlid,
		     buf->resp.ap.global_session_id,
		     buf->resp.ap.bcast_session_id,
		     wlvif->basic_rate_set, wlvif->rate_set);

	cc33xx_set_link(cc, wlvif, wlvif->ap.global_hlid);
	cc33xx_set_link(cc, wlvif, wlvif->ap.bcast_hlid);

	return 0;
}

int cc33xx_cmd_role_stop_ap(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_role_stop *cmd __free(kfree) = NULL;
	int ret;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = wlvif->role_id;

	ret = cc33xx_cmd_send(cc, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd role stop ap\n");
		return ret;
	}

	cc33xx_clear_link(cc, wlvif, &wlvif->ap.bcast_hlid);
	cc33xx_clear_link(cc, wlvif, &wlvif->ap.global_hlid);

	return ret;
}

/* send test command to firmware
 *
 * @cc: cc struct
 * @buf: buffer containing the command, with all headers, must work with dma
 * @len: length of the buffer
 * @answer: is answer needed
 */
int cc33xx_cmd_test(struct cc33xx *cc, void *buf, size_t buf_len, u8 answer)
{
	int ret;
	size_t res_len = 0;

	if (answer)
		res_len = buf_len;

	ret = cc33xx_cmd_send(cc, CMD_TEST_MODE, buf, buf_len, res_len);

	if (ret < 0)
		dev_warn(cc->dev, "TEST command failed\n");

	return ret;
}

/* read acx from firmware
 *
 * @cc: cc struct
 * @id: acx id
 * @buf: buffer for the response, including all headers, must work with dma
 * @len: length of buf
 */
int cc33xx_cmd_interrogate(struct cc33xx *cc, u16 id, void *buf,
			   size_t cmd_len, size_t res_len)
{
	struct acx_header *acx = buf;
	int ret;

	acx->id = cpu_to_le16(id);

	/* response payload length, does not include any headers */
	acx->len = cpu_to_le16(res_len - sizeof(*acx));

	ret = cc33xx_cmd_send(cc, CMD_INTERROGATE, acx, cmd_len, res_len);
	if (ret < 0)
		dev_err(cc->dev, "INTERROGATE command failed\n");

	return ret;
}

/* read debug acx from firmware
 *
 * @cc: cc struct
 * @id: acx id
 * @buf: buffer for the response, including all headers, must work with dma
 * @len: length of buf
 */
int cc33xx_cmd_debug_inter(struct cc33xx *cc, u16 id, void *buf,
			   size_t cmd_len, size_t res_len)
{
	struct acx_header *acx = buf;
	int ret;

	acx->id = cpu_to_le16(id);

	/* response payload length, does not include any headers */
	acx->len = cpu_to_le16(res_len - sizeof(*acx));

	ret = cc33xx_cmd_send(cc, CMD_DEBUG_READ, acx, cmd_len, res_len);
	if (ret < 0)
		dev_err(cc->dev, "CMD_DEBUG_READ command failed\n");

	return ret;
}

/* write acx value to firmware
 *
 * @cc: cc struct
 * @id: acx id
 * @buf: buffer containing acx, including all headers, must work with dma
 * @len: length of buf
 * @valid_rets: bitmap of valid cmd status codes (i.e. return values).
 * return the cmd status on success.
 */
int cc33xx_cmd_configure_failsafe(struct cc33xx *cc, u16 id, void *buf,
				  size_t len, unsigned long valid_rets)
{
	struct acx_header *acx = buf;
	int ret;

	if (WARN_ON_ONCE(len < sizeof(*acx)))
		return -EIO;

	acx->id = cpu_to_le16(id);

	/* payload length, does not include any headers */
	acx->len = cpu_to_le16(len - sizeof(*acx));

	ret = cc33xx_cmd_send_failsafe(cc, CMD_CONFIGURE, acx, len, 0,
				       valid_rets);
	if (ret < 0)
		dev_warn(cc->dev, "CONFIGURE command NOK\n");

	return ret;
}

/* wrapper for cc33xx_cmd_configure that accepts only success status.
 * return 0 on success
 */
int cc33xx_cmd_configure(struct cc33xx *cc, u16 id, void *buf, size_t len)
{
	int ret = cc33xx_cmd_configure_failsafe(cc, id, buf, len, 0);

	if (ret < 0)
		return ret;
	return 0;
}

/* write acx value to firmware
 *
 * @cc: cc struct
 * @id: acx id
 * @buf: buffer containing debug, including all headers, must work with dma
 * @len: length of buf
 * @valid_rets: bitmap of valid cmd status codes (i.e. return values).
 * return the cmd status on success.
 */
static int cc33xx_cmd_debug_failsafe(struct cc33xx *cc, u16 id, void *buf,
				     size_t len, unsigned long valid_rets)
{
	struct debug_header *acx = buf;
	int ret;

	if (WARN_ON_ONCE(len < sizeof(*acx)))
		return -EIO;

	acx->id = cpu_to_le16(id);

	/* payload length, does not include any headers */
	acx->len = cpu_to_le16(len - sizeof(*acx));

	ret = cc33xx_cmd_send_failsafe(cc, CMD_DEBUG, acx, len, 0,
				       valid_rets);
	if (ret < 0)
		dev_warn(cc->dev, "CONFIGURE command NOK\n");

	return ret;
}

/* wrapper for cc33xx_cmd_debug that accepts only success status.
 * return 0 on success
 */
int cc33xx_cmd_debug(struct cc33xx *cc, u16 id, void *buf, size_t len)
{
	int ret = cc33xx_cmd_debug_failsafe(cc, id, buf, len, 0);

	if (ret < 0)
		return ret;
	return 0;
}

int cc33xx_cmd_ps_mode(struct cc33xx *cc, struct cc33xx_vif *wlvif,
		       u8 ps_mode, u16 auto_ps_timeout)
{
	struct cc33xx_cmd_ps_params *ps_params __free(kfree) = NULL;
	int ret = 0;

	ps_params = kzalloc_obj(*ps_params);
	if (!ps_params)
		return -ENOMEM;

	ps_params->role_id = wlvif->role_id;
	ps_params->ps_mode = ps_mode;
	ps_params->auto_ps_timeout = cpu_to_le16(auto_ps_timeout);

	ret = cc33xx_cmd_send(cc, CMD_SET_PS_MODE, ps_params,
			      sizeof(*ps_params), 0);
	if (ret < 0)
		dev_err(cc->dev, "cmd set_ps_mode failed\n");

	return ret;
}

int cc33xx_cmd_set_default_wep_key(struct cc33xx *cc, u8 id, u8 hlid)
{
	struct cc33xx_cmd_set_keys *cmd __free(kfree) = NULL;
	int ret = 0;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->hlid = hlid;
	cmd->key_id = id;
	cmd->lid_key_type = WEP_DEFAULT_LID_TYPE;
	cmd->key_action = cpu_to_le16(KEY_SET_ID);
	cmd->key_type = KEY_WEP;

	ret = cc33xx_cmd_send(cc, CMD_SET_KEYS, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_warn(cc->dev, "cmd set_default_wep_key failed: %d\n", ret);

	return ret;
}

int cc33xx_cmd_set_sta_key(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			   u16 action, u8 id, u8 key_type, u8 key_size,
			   const u8 *key, const u8 *addr, u32 tx_seq_32,
			   u16 tx_seq_16)
{
	struct cc33xx_cmd_set_keys *cmd __free(kfree) = NULL;
	int ret = 0;

	/* hlid might have already been deleted */
	if (wlvif->sta.hlid == CC33XX_INVALID_LINK_ID)
		return 0;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->hlid = wlvif->sta.hlid;

	if (key_type == KEY_WEP)
		cmd->lid_key_type = WEP_DEFAULT_LID_TYPE;
	else if (is_broadcast_ether_addr(addr))
		cmd->lid_key_type = BROADCAST_LID_TYPE;
	else
		cmd->lid_key_type = UNICAST_LID_TYPE;

	cmd->key_action = cpu_to_le16(action);
	cmd->key_size = key_size;
	cmd->key_type = key_type;

	cmd->ac_seq_num16[0] = cpu_to_le16(tx_seq_16);
	cmd->ac_seq_num32[0] = cpu_to_le32(tx_seq_32);

	cmd->key_id = id;

	if (key_type == KEY_TKIP) {
		/* We get the key in the following form:
		 * TKIP (16 bytes) - TX MIC (8 bytes) - RX MIC (8 bytes)
		 * but the target is expecting:
		 * TKIP - RX MIC - TX MIC
		 */
		memcpy(cmd->key, key, 16);
		memcpy(cmd->key + 16, key + 24, 8);
		memcpy(cmd->key + 24, key + 16, 8);

	} else {
		memcpy(cmd->key, key, key_size);
	}

	ret = cc33xx_cmd_send(cc, CMD_SET_KEYS, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_warn(cc->dev, "could not set keys\n");

	return ret;
}

int cc33xx_cmd_set_ap_key(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			  u16 action, u8 id, u8 key_type, u8 key_size,
			  const u8 *key, u8 hlid, u32 tx_seq_32, u16 tx_seq_16)
{
	struct cc33xx_cmd_set_keys *cmd __free(kfree) = NULL;
	int ret = 0;
	u8 lid_type;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	if (hlid == wlvif->ap.bcast_hlid) {
		if (key_type == KEY_WEP)
			lid_type = WEP_DEFAULT_LID_TYPE;
		else
			lid_type = BROADCAST_LID_TYPE;
	} else {
		lid_type = UNICAST_LID_TYPE;
	}

	cmd->lid_key_type = lid_type;
	cmd->hlid = hlid;
	cmd->key_action = cpu_to_le16(action);
	cmd->key_size = key_size;
	cmd->key_type = key_type;
	cmd->key_id = id;
	cmd->ac_seq_num16[0] = cpu_to_le16(tx_seq_16);
	cmd->ac_seq_num32[0] = cpu_to_le32(tx_seq_32);

	if (key_type == KEY_TKIP) {
		/* We get the key in the following form:
		 * TKIP (16 bytes) - TX MIC (8 bytes) - RX MIC (8 bytes)
		 * but the target is expecting:
		 * TKIP - RX MIC - TX MIC
		 */
		memcpy(cmd->key, key, 16);
		memcpy(cmd->key + 16, key + 24, 8);
		memcpy(cmd->key + 24, key + 16, 8);
	} else {
		memcpy(cmd->key, key, key_size);
	}

	cc33xx_dump(DEBUG_CRYPT, "TARGET AP KEY: ", cmd, sizeof(*cmd));

	ret = cc33xx_cmd_send(cc, CMD_SET_KEYS, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_warn(cc->dev, "could not set ap keys\n");

	return ret;
}

int cc33xx_cmd_set_peer_state(struct cc33xx *cc, u8 hlid)
{
	struct cc33xx_cmd_set_peer_state *cmd __free(kfree) = NULL;
	int ret = 0;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->hlid = hlid;
	cmd->state = CMD_STA_STATE_CONNECTED;

	ret = cc33xx_cmd_send(cc, CMD_SET_LINK_CONNECTION_STATE,
			      cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "failed to send set peer state command\n");

	return ret;
}

int cc33xx_cmd_add_peer(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			struct ieee80211_sta *sta, u8 *hlid, u8 is_connected)
{
	union cc33xx_cmd_add_peer_command *buf __free(kfree) = NULL;
	int i, ret;
	u32 sta_rates;

	buf = kzalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;

	buf->cmd.is_connected = is_connected;
	buf->cmd.role_id = wlvif->role_id;
	buf->cmd.role_type = CC33XX_ROLE_AP;
	buf->cmd.link_type = 1;

	memcpy(buf->cmd.addr, sta->addr, ETH_ALEN);
	buf->cmd.bss_index = CC33XX_AP_BSS_INDEX;
	buf->cmd.aid = cpu_to_le16(sta->aid);
	buf->cmd.sp_len = sta->max_sp;
	buf->cmd.wmm = sta->wme ? 1 : 0;

	for (i = 0; i < IEEE80211_NUM_ACS; i++) {
		if (sta->wme && (sta->uapsd_queues & BIT(i))) {
			buf->cmd.psd_type[IEEE80211_NUM_ACS - 1 - i] =
					CC33XX_PSD_UPSD_TRIGGER;
		} else {
			buf->cmd.psd_type[IEEE80211_NUM_ACS - 1 - i] =
					CC33XX_PSD_LEGACY;
		}
	}

	sta_rates = sta->deflink.supp_rates[wlvif->band];
	if (sta->deflink.ht_cap.ht_supported) {
		sta_rates |=
			(sta->deflink.ht_cap.mcs.rx_mask[0] << HW_HT_RATES_OFFSET) |
			(sta->deflink.ht_cap.mcs.rx_mask[1] << HW_MIMO_RATES_OFFSET);
	}

	buf->cmd.supported_rates =
		cpu_to_le32(cc33xx_tx_enabled_rates_get(cc, sta_rates,
							wlvif->band));

	if (!buf->cmd.supported_rates)
		buf->cmd.supported_rates = cpu_to_le32(wlvif->basic_rate_set);

	if (sta->deflink.ht_cap.ht_supported) {
		buf->cmd.ht_capabilities = cpu_to_le32(sta->deflink.ht_cap.cap);
		buf->cmd.ht_capabilities |= cpu_to_le32(CC33XX_HT_CAP_HT_OPERATION);
		buf->cmd.ampdu_params = sta->deflink.ht_cap.ampdu_factor |
				    sta->deflink.ht_cap.ampdu_density;
	}

	buf->cmd.has_he = sta->deflink.he_cap.has_he;
	buf->cmd.mfp = sta->mfp;
	ret = cc33xx_cmd_send(cc, CMD_ADD_PEER, buf, sizeof(buf->cmd),
			      sizeof(buf->resp));
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd add peer\n");
		return ret;
	}

	if (hlid) {
		if (le16_to_cpu(buf->resp.header.status) == CMD_STATUS_SUCCESS) {
			*hlid = buf->resp.hlid;

			scoped_guard(mutex, &cc->mutex) {
				cc->links[*hlid].allocated_pkts = 0;
				cc->session_ids[*hlid] = buf->resp.session_id;
			}
		} else {
			ret = -EMLINK;
		}
	}

	return ret;
}

int cc33xx_cmd_remove_peer(struct cc33xx *cc,
			   struct cc33xx_vif *wlvif, u8 hlid)
{
	struct cc33xx_cmd_remove_peer *cmd __free(kfree) = NULL;
	int ret;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->hlid = hlid;

	cmd->role_id = wlvif->role_id;

	init_completion(&cc->peer_remove_complete);

	ret = cc33xx_cmd_send(cc, CMD_REMOVE_PEER, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		dev_err(cc->dev, "failed to initiate cmd remove peer\n");
		return ret;
	}

	ret = wait_for_completion_timeout(&cc->peer_remove_complete,
					  msecs_to_jiffies(CC33XX_EVENT_TIMEOUT));

	/* If there is a timeout due to event not received, queue recovery */
	if (!ret) {
		ret = -ETIMEDOUT;
		cc33xx_queue_recovery_work(cc);
	}

	return ret;
}

static int cc33xx_cmd_roc(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			  u8 role_id, enum nl80211_band band, u8 channel)
{
	struct cc33xx_cmd_roc *cmd __free(kfree) = NULL;
	int ret = 0;

	if (WARN_ON(role_id == CC33XX_INVALID_ROLE_ID))
		return -EINVAL;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = role_id;
	cmd->channel = channel;
	switch (band) {
	case NL80211_BAND_2GHZ:
		cmd->band = CC33XX_BAND_2_4GHZ;
		break;
	case NL80211_BAND_5GHZ:
		cmd->band = CC33XX_BAND_5GHZ;
		break;
	default:
		dev_err(cc->dev, "roc - unknown band: %d\n", (int)wlvif->band);
		return -EINVAL;
	}

	ret = cc33xx_cmd_send(cc, CMD_REMAIN_ON_CHANNEL, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "failed to send ROC command\n");

	return ret;
}

static int cc33xx_cmd_croc(struct cc33xx *cc, u8 role_id)
{
	struct cc33xx_cmd_croc *cmd __free(kfree) = NULL;
	int ret = 0;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = role_id;

	ret = cc33xx_cmd_send(cc, CMD_CANCEL_REMAIN_ON_CHANNEL, cmd,
			      sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "failed to send ROC command\n");

	return ret;
}

int cc33xx_roc(struct cc33xx *cc, struct cc33xx_vif *wlvif, u8 role_id,
	       enum nl80211_band band, u8 channel)
{
	int ret = 0;

	if (WARN_ON(test_bit(role_id, cc->roc_map)))
		return 0;

	ret = cc33xx_cmd_roc(cc, wlvif, role_id, band, channel);
	if (!ret)
		__set_bit(role_id, cc->roc_map);

	return ret;
}

int cc33xx_croc(struct cc33xx *cc, u8 role_id)
{
	int ret = 0;

	if (WARN_ON(!test_bit(role_id, cc->roc_map)))
		return 0;

	ret = cc33xx_cmd_croc(cc, role_id);
	if (!ret)
		__clear_bit(role_id, cc->roc_map);

	return ret;
}

int cc33xx_cmd_stop_channel_switch(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_stop_channel_switch *cmd __free(kfree) = NULL;
	int ret;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = wlvif->role_id;

	ret = cc33xx_cmd_send(cc, CMD_STOP_CHANNEL_SWITCH,
			      cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "failed to stop channel switch command\n");

	return ret;
}

/* start dev role and roc on its channel */
int cc33xx_start_dev(struct cc33xx *cc, struct cc33xx_vif *wlvif,
		     enum nl80211_band band, int channel)
{
	int ret;

	if (WARN_ON(wlvif->bss_type != BSS_TYPE_STA_BSS))
		return -EINVAL;

	/* the dev role is already started for p2p mgmt interfaces */

	if (!cc33xx_is_p2p_mgmt(wlvif)) {
		ret = cc33xx_cmd_role_enable(cc,
					     cc33xx_wlvif_to_vif(wlvif)->addr,
					     CC33XX_ROLE_DEVICE,
					     &wlvif->dev_role_id);
		if (ret < 0)
			return ret;
	}

	ret = cc33xx_cmd_role_start_dev(cc, wlvif, band, channel);
	if (ret < 0)
		goto out_disable;

	ret = cc33xx_roc(cc, wlvif, wlvif->dev_role_id, band, channel);
	if (ret < 0)
		goto out_stop;

	return 0;

out_stop:
	cc333xx_cmd_role_stop_dev(cc, wlvif);
out_disable:
	if (!cc33xx_is_p2p_mgmt(wlvif))
		cc33xx_cmd_role_disable(cc, &wlvif->dev_role_id);

	return ret;
}

/* croc dev hlid, and stop the role. */
int cc33xx_stop_dev(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	int ret;

	if (WARN_ON(wlvif->bss_type != BSS_TYPE_STA_BSS))
		return -EINVAL;

	/* flush all pending packets */
	ret = cc33xx_do_tx_work(cc);
	if (ret < 0)
		return ret;

	if (test_bit(wlvif->dev_role_id, cc->roc_map)) {
		ret = cc33xx_croc(cc, wlvif->dev_role_id);
		if (ret < 0)
			return ret;
	}

	ret = cc333xx_cmd_role_stop_dev(cc, wlvif);
	if (ret < 0)
		return ret;

	if (!cc33xx_is_p2p_mgmt(wlvif)) {
		ret = cc33xx_cmd_role_disable(cc, &wlvif->dev_role_id);
		if (ret < 0)
			return ret;
	}

	return ret;
}

int cc33xx_cmd_channel_switch(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			      struct ieee80211_channel_switch *ch_switch)
{
	struct cc33xx_cmd_channel_switch *cmd __free(kfree) = NULL;
	u32 supported_rates;
	int ret;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = wlvif->role_id;
	cmd->channel = ch_switch->chandef.chan->hw_value;
	cmd->switch_time = ch_switch->count;
	cmd->stop_tx = ch_switch->block_tx;

	switch (ch_switch->chandef.chan->band) {
	case NL80211_BAND_2GHZ:
		cmd->band = CC33XX_BAND_2_4GHZ;
		break;
	case NL80211_BAND_5GHZ:
		cmd->band = CC33XX_BAND_5GHZ;
		break;
	default:
		dev_err(cc->dev, "invalid channel switch band: %d\n",
			ch_switch->chandef.chan->band);
		return -EINVAL;
	}

	supported_rates = CONF_TX_ENABLED_RATES | CONF_TX_MCS_RATES;
	supported_rates |= wlvif->rate_set;
	if (wlvif->p2p)
		supported_rates &= ~CONF_TX_CCK_RATES;
	cmd->local_supported_rates = cpu_to_le32(supported_rates);
	cmd->channel_type = wlvif->channel_type;

	ret = cc33xx_cmd_send(cc, CMD_CHANNEL_SWITCH, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "failed to send channel switch command\n");

	return ret;
}

int cc33xx_cmd_set_bd_addr(struct cc33xx *cc, u8 *bd_addr)
{
	struct cc33xx_cmd_set_bd_addr *cmd __free(kfree) = NULL;
	int ret = 0;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	memcpy(cmd->bd_addr, bd_addr, sizeof(cmd->bd_addr));

	ret = cc33xx_cmd_send(cc, CMD_SET_BD_ADDR, cmd, sizeof(*cmd), 0);
	if (ret < 0)
		dev_err(cc->dev, "failed to set BD address\n");

	return ret;
}

int cc33xx_cmd_get_device_info(struct cc33xx *cc, u8 *info_buffer, size_t buffer_len)
{
	struct cc33xx_cmd_get_device_info *cmd __free(kfree) = NULL;
	int ret = 0;

	cmd = kzalloc_obj(*cmd);
	if (!cmd)
		return -ENOMEM;

	ret = cc33xx_cmd_send(cc, CMD_BM_READ_DEVICE_INFO, cmd,
			      sizeof(*cmd), sizeof(*cmd));
	if (ret < 0) {
		dev_err(cc->dev, "Device info command failure\n");
	} else {
		WARN_ON(buffer_len > sizeof(cmd->device_info));
		memcpy(info_buffer, cmd->device_info, buffer_len);
	}

	return ret;
}

int cc33xx_cmd_download_container_chunk(struct cc33xx *cc, u8 *chunk,
					size_t chunk_len, bool is_last_chunk)
{
	struct cc33xx_cmd_container_download *cmd __free(kfree) = NULL;
	const size_t command_size = sizeof(*cmd) + chunk_len;
	int ret;
	bool  is_sync_transfer = !is_last_chunk;

	cmd = kzalloc(command_size, GFP_KERNEL);

	if (!cmd)
		return -ENOMEM;

	memcpy(cmd->payload, chunk, chunk_len);
	cmd->length = cpu_to_le32(chunk_len);

	if (is_last_chunk) {
		cc33xx_debug(cc->dev, DEBUG_BOOT, "Suspending IRQ while device reboots\n");
		cc33xx_disable_interrupts_nosync(cc);
	}

	ret = __cc33xx_cmd_send(cc, CMD_CONTAINER_DOWNLOAD, cmd,
				command_size, sizeof(u32), is_sync_transfer);

	if (is_last_chunk) {
		msleep(CC33XX_REBOOT_TIMEOUT_MSEC);
		cc33xx_enable_interrupts(cc);
	}

	return ret;
}
