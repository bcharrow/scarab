/*
  Copyright (C) 2013 Nathan Michael

  This file is part of mesh80211s, a simple interface to wireless network
  cards.

  mesh80211s is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  You should have received a copy of the GNU General Public License along with
  this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>

#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <netlink/attr.h>

#include <linux/nl80211.h>
#include "station.h"

#define ETH_ALEN 6

static struct station_states sta_states;

void mac_addr_n2a(char *mac_addr, unsigned char *arg)
{
  int i, l;

  l = 0;
  for (i = 0; i < ETH_ALEN ; i++) {
    if (i == 0) {
      sprintf(mac_addr+l, "%02x", arg[i]);
      l += 2;
    } else {
      sprintf(mac_addr+l, ":%02x", arg[i]);
      l += 3;
    }
  }
}

enum plink_state
  {
    LISTEN,
    OPN_SNT,
    OPN_RCVD,
    CNF_RCVD,
    ESTAB,
    HOLDING,
    BLOCKED
  };

/* Remove to avoid redefinition error on 14.04 */
/* enum plink_actions */
/*   { */
/*     PLINK_ACTION_UNDEFINED, */
/*     PLINK_ACTION_OPEN, */
/*     PLINK_ACTION_BLOCK, */
/*   }; */

int station_dump_callback(struct nl_msg *msg, void *arg)
{
  struct nlattr *tb[NL80211_ATTR_MAX + 1];
  struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
  struct nlattr *sinfo[NL80211_STA_INFO_MAX + 1];
  struct nlattr *rinfo[NL80211_RATE_INFO_MAX + 1];
  char mac_addr[20], dev[20];
  static struct nla_policy stats_policy[NL80211_STA_INFO_MAX + 1] =
    {
      [NL80211_STA_INFO_INACTIVE_TIME] = { .type = NLA_U32 },
      [NL80211_STA_INFO_RX_BYTES] = { .type = NLA_U32 },
      [NL80211_STA_INFO_TX_BYTES] = { .type = NLA_U32 },
      [NL80211_STA_INFO_RX_PACKETS] = { .type = NLA_U32 },
      [NL80211_STA_INFO_TX_PACKETS] = { .type = NLA_U32 },
      [NL80211_STA_INFO_SIGNAL] = { .type = NLA_U8 },
      [NL80211_STA_INFO_TX_BITRATE] = { .type = NLA_NESTED },
      [NL80211_STA_INFO_LLID] = { .type = NLA_U16 },
      [NL80211_STA_INFO_PLID] = { .type = NLA_U16 },
      [NL80211_STA_INFO_PLINK_STATE] = { .type = NLA_U8 },
    };

  struct nla_policy rate_policy[NL80211_RATE_INFO_MAX + 1] =
    {
      [NL80211_RATE_INFO_BITRATE] = { .type = NLA_U16 },
      [NL80211_RATE_INFO_MCS] = { .type = NLA_U8 },
      [NL80211_RATE_INFO_40_MHZ_WIDTH] = { .type = NLA_FLAG },
      [NL80211_RATE_INFO_SHORT_GI] = { .type = NLA_FLAG },
    };

  nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
            genlmsg_attrlen(gnlh, 0), NULL);

  if (!tb[NL80211_ATTR_STA_INFO])
    {
      fprintf(stderr, "sta stats missing!");
      return NL_SKIP;
    }
  if (nla_parse_nested(sinfo, NL80211_STA_INFO_MAX,
                       tb[NL80211_ATTR_STA_INFO],
                       stats_policy))
    {
      fprintf(stderr, "failed to parse nested attributes!");
      return NL_SKIP;
    }

  unsigned short ind = sta_states.count;

  mac_addr_n2a(mac_addr, nla_data(tb[NL80211_ATTR_MAC]));
  if_indextoname(nla_get_u32(tb[NL80211_ATTR_IFINDEX]), dev);

  strncpy(sta_states.stations[ind].mac, mac_addr,
          sizeof(sta_states.stations[ind].mac));
  strncpy(sta_states.stations[ind].device, dev,
          sizeof(sta_states.stations[ind].device));

  if (sinfo[NL80211_STA_INFO_INACTIVE_TIME])
    sta_states.stations[ind].inactive_time = nla_get_u32(sinfo[NL80211_STA_INFO_INACTIVE_TIME]);
  if (sinfo[NL80211_STA_INFO_RX_BYTES])
    sta_states.stations[ind].rx_bytes = nla_get_u32(sinfo[NL80211_STA_INFO_RX_BYTES]);
  if (sinfo[NL80211_STA_INFO_RX_PACKETS])
    sta_states.stations[ind].rx_packets = nla_get_u32(sinfo[NL80211_STA_INFO_RX_PACKETS]);
  if (sinfo[NL80211_STA_INFO_TX_BYTES])
    sta_states.stations[ind].tx_bytes = nla_get_u32(sinfo[NL80211_STA_INFO_TX_BYTES]);
  if (sinfo[NL80211_STA_INFO_TX_PACKETS])
    sta_states.stations[ind].tx_packets = nla_get_u32(sinfo[NL80211_STA_INFO_TX_PACKETS]);
  if (sinfo[NL80211_STA_INFO_SIGNAL])
    sta_states.stations[ind].signal = (int8_t)nla_get_u8(sinfo[NL80211_STA_INFO_SIGNAL]);

  if (sinfo[NL80211_STA_INFO_TX_BITRATE])
    {
      if (nla_parse_nested(rinfo, NL80211_RATE_INFO_MAX,
                           sinfo[NL80211_STA_INFO_TX_BITRATE], rate_policy))
        fprintf(stderr, "failed to parse nested rate attributes!");
      else
        if (rinfo[NL80211_RATE_INFO_BITRATE])
          sta_states.stations[ind].tx_bitrate = nla_get_u16(rinfo[NL80211_RATE_INFO_BITRATE]);
    }

  if (sinfo[NL80211_STA_INFO_LLID])
    sta_states.stations[ind].mesh_llid = nla_get_u16(sinfo[NL80211_STA_INFO_LLID]);
  if (sinfo[NL80211_STA_INFO_PLID])
    sta_states.stations[ind].mesh_plid = nla_get_u16(sinfo[NL80211_STA_INFO_PLID]);
  if (sinfo[NL80211_STA_INFO_PLINK_STATE])
    {
      switch (nla_get_u16(sinfo[NL80211_STA_INFO_PLINK_STATE]))
        {
        case LISTEN:
          strcpy(sta_states.stations[ind].mesh_plink, "LISTEN");
          break;
        case OPN_SNT:
          strcpy(sta_states.stations[ind].mesh_plink, "OPN_SNT");
          break;
        case OPN_RCVD:
          strcpy(sta_states.stations[ind].mesh_plink, "OPN_RCVD");
          break;
        case CNF_RCVD:
          strcpy(sta_states.stations[ind].mesh_plink, "CNF_RCVD");
          break;
        case ESTAB:
          strcpy(sta_states.stations[ind].mesh_plink, "ESTAB");
          break;
        case HOLDING:
          strcpy(sta_states.stations[ind].mesh_plink, "HOLDING");
          break;
        case BLOCKED:
          strcpy(sta_states.stations[ind].mesh_plink, "BLOCKED");
          break;
        default:
          strcpy(sta_states.stations[ind].mesh_plink, "UNKNOWN");
          break;
        }
    }

  sta_states.count++;

  return NL_SKIP;
}

inline struct nl_handle *nl_socket_alloc(void)
{
  return nl_handle_alloc();
}

inline void nl_socket_free(struct nl_sock *h)
{
  nl_handle_destroy(h);
}

inline int __genl_ctrl_alloc_cache(struct nl_sock *h, struct nl_cache **cache)
{
  struct nl_cache *tmp = genl_ctrl_alloc_cache(h);
  if (!tmp)
    return -ENOMEM;
  *cache = tmp;
  return 0;
}
#define genl_ctrl_alloc_cache __genl_ctrl_alloc_cache

int nl80211_init(struct nl80211_state *state)
{
  state->nl_sock = nl_socket_alloc();
  if (!state->nl_sock)
    {
      fprintf(stderr, "Failed to allocate netlink socket.\n");
      return -ENOMEM;
    }

  if (genl_connect(state->nl_sock))
    {
      fprintf(stderr, "Failed to connect to generic netlink.\n");
      nl_socket_free(state->nl_sock);
      return -ENOLINK;
    }

  if (genl_ctrl_alloc_cache(state->nl_sock, &state->nl_cache))
    {
      fprintf(stderr, "Failed to allocate generic netlink cache.\n");
      nl_socket_free(state->nl_sock);
      return -ENOMEM;
    }

  state->nl80211 = genl_ctrl_search_by_name(state->nl_cache, "nl80211");
  if (!state->nl80211)
    {
      fprintf(stderr, "nl80211 not found.\n");
      nl_cache_free(state->nl_cache);
      return -ENOENT;
    }

  return 0;
}

void nl80211_cleanup(struct nl80211_state *state)
{
  genl_family_put(state->nl80211);
  nl_cache_free(state->nl_cache);
  nl_socket_free(state->nl_sock);
}

int error_handler(struct sockaddr_nl *nla, struct nlmsgerr *err, void *arg)
{
  int *ret = arg;
  *ret = err->error;
  return NL_STOP;
}

int finish_handler(struct nl_msg *msg, void *arg)
{
  int *ret = arg;
  *ret = 0;
  return NL_SKIP;
}

int ack_handler(struct nl_msg *msg, void *arg)
{
  int *ret = arg;
  *ret = 0;
  return NL_STOP;
}

int station_dump(struct nl80211_state *state, const char* device)
{
  struct nl_cb *cb;
  struct nl_msg *msg;
  int devidx = 0;
  int err;

  // Check for the device index given its name
  if ((devidx = if_nametoindex(device)) == 0)
    devidx = -1;

  if (devidx < 0)
    return -errno;

  // Allocate the message for getting the station dump
  if ((msg = nlmsg_alloc()) == 0)
    {
      fprintf(stderr, "failed to allocate netlink message\n");
      return 2;
    }

  // Allocate the callback that returns with the data
  if ((cb = nl_cb_alloc(NL_CB_DEFAULT)) == 0)
    {
      fprintf(stderr, "failed to allocate netlink callbacks\n");
      nlmsg_free(msg);
      return 2;
    }

  // Generate the message
  // Note that this is really where the fact that we are doing a
  // station dump is defined. Another message definition would result
  // in different behavior. Of course, the callback below would
  // also be different
  genlmsg_put(msg, 0, 0, genl_family_get_id(state->nl80211), 0,
              NLM_F_DUMP, NL80211_CMD_GET_STATION, 0);

  // Set the device index
  nla_put_u32(msg, NL80211_ATTR_IFINDEX, devidx);

  // Set the station dump callback
  nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, station_dump_callback, NULL);

  // Fill in the message header
  if ((err = nl_send_auto_complete(state->nl_sock, msg)) < 0)
    {
      nl_cb_put(cb);
      nlmsg_free(msg);
      return err;
    }

  err = 1;

  // Set the error, finish, and ack handlers
  nl_cb_err(cb, NL_CB_CUSTOM, error_handler, &err);
  nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &err);
  nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &err);

  // Now wait for the callback to return
  sta_states.count = 0;
  while (err > 0)
    nl_recvmsgs(state->nl_sock, cb);

  // Free the callback and message
  nl_cb_put(cb);
  nlmsg_free(msg);

  return err;
}

void get_station_states(struct station_states* sstates)
{
  memcpy((void*)sstates, (void*)&sta_states, sizeof(struct station_states));
}
