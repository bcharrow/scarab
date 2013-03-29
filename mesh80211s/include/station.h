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
