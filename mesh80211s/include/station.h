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

#ifdef __cplusplus
extern "C" {
#endif

#include <netlink/msg.h>
#include <netlink/genl/family.h>

#define nl_sock nl_handle

struct nl80211_state
{
  struct nl_sock *nl_sock;
  struct nl_cache *nl_cache;
  struct genl_family *nl80211;
};

struct station_state
{
  char device[64];
  char mac[18];
  unsigned int inactive_time;
  unsigned int rx_bytes;
  unsigned int rx_packets;
  unsigned int tx_bytes;
  unsigned int tx_packets;
  int signal;
  unsigned short tx_bitrate;
  unsigned short mesh_llid;
  unsigned short mesh_plid;
  char mesh_plink[10];;
};

struct station_states
{
  unsigned short count;
  struct station_state stations[100];
};

int nl80211_init(struct nl80211_state *state);
void nl80211_cleanup(struct nl80211_state *state);
int station_dump(struct nl80211_state *state, const char* device);
void get_station_states(struct station_states* sstates);

#ifdef __cplusplus
}
#endif
