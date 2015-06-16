#!/bin/bash

error=0 ; trap "error=$((error|1))" ERR

ID=${HOSTNAME/scarab/}

cat > $target/etc/network/interfaces <<EOF
auto lo wlan0

iface lo inet loopback

iface eth0 inet static
address 192.168.131.1${ID}
gateway 192.168.131.1
netmask 255.255.255.0
post-up ip route del default dev eth0
post-up ip route add default via 192.168.131.1 dev eth0 metric 200
post-up ip route del 192.168.131.0/24 dev eth0
post-up ip route add 192.168.131.0/24 src 192.168.131.1${ID} dev eth0 proto kernel scope link metric 200
dns-nameservers 192.168.129.1

iface wlan0 inet static
address 192.168.131.${ID}
netmask 255.255.255.0
wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
post-up ip route add default via 192.168.131.1 dev wlan0 metric 300
post-up ip route del 192.168.131.0/24 dev wlan0
post-up ip route add 192.168.131.0/24 src 192.168.131.${ID} dev wlan0 proto kernel scope link metric 300
dns-nameservers 192.168.129.1

iface olsr inet static
address 192.168.130.$ID
netmask 255.255.255.0
pre-up iw phy phy0 interface add olsr type ibss 
post-up iw dev olsr ibss join scarab_mesh 5180 HT40+
post-up olsrd
post-down killall -q olsrd
post-down iw dev olsr del

# iface mesh inet static
# address 192.168.132.$ID
# netmask 255.255.255.0
# gateway 192.168.132.1
# pre-up iw dev wlan0 interface add mesh type mp mesh_id scarab_mesh
# pre-up iw dev mesh set channel 40
# # post-up iw dev mesh set mesh_param mesh_path_refresh_time 250
# # post-up iw dev mesh set mesh_param mesh_hwmp_active_path_timeout 500
# post-up iw dev mesh scan trigger
# post-down iw dev mesh del

# iface bat0 inet static
# address 192.168.130.${ID}
# netmask 255.255.255.0
# pre-up iw dev wlan0 set type ibss
# pre-up ifconfig wlan0 up
# pre-up iw dev wlan0 ibss join scarab_mesh 5180 HT40+
# pre-up batctl if add wlan0
# post-up ip route add default via 192.168.130.4 dev bat0
# post-down ifconfig wlan0 down
EOF

chmod 644 /etc/network/interfaces

exit $error
