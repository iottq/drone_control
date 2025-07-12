#!/bin/bash

# 切换树莓派热点模式，并共享有线网络 eth0
# Author: ChatGPT

WIFI_IF="wlan0"
LAN_IF="eth0"
HOTSPOT_SSID="MyPiAP"
HOTSPOT_PSK="MyStrongPassword"

MODE=$1

if [ "$EUID" -ne 0 ]; then
  echo "请使用 sudo 运行！"
  exit 1
fi

if [[ "$MODE" != "hotspot" && "$MODE" != "stop" ]]; then
  echo "用法: sudo ./wifi_mode_toggle.sh [hotspot|stop]"
  exit 1
fi

echo "模式: $MODE"

# 开启内核 IP 转发
echo 1 > /proc/sys/net/ipv4/ip_forward

if [ "$MODE" == "hotspot" ]; then
  echo "配置热点..."

  # 删除旧连接
  nmcli con delete "$HOTSPOT_SSID" >/dev/null 2>&1

  # 创建热点
  nmcli con add type wifi ifname "$WIFI_IF" con-name "$HOTSPOT_SSID" autoconnect yes ssid "$HOTSPOT_SSID"
  nmcli con modify "$HOTSPOT_SSID" 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
  nmcli con modify "$HOTSPOT_SSID" wifi-sec.key-mgmt wpa-psk
  nmcli con modify "$HOTSPOT_SSID" wifi-sec.psk "$HOTSPOT_PSK"
  nmcli con modify "$HOTSPOT_SSID" ipv4.addresses 192.168.4.1/24
  nmcli con modify "$HOTSPOT_SSID" ipv4.method shared

  nmcli con up "$HOTSPOT_SSID"

  echo "配置 NAT 转发..."
  iptables -t nat -A POSTROUTING -o "$LAN_IF" -j MASQUERADE
  iptables -A FORWARD -i "$LAN_IF" -o "$WIFI_IF" -m state --state RELATED,ESTABLISHED -j ACCEPT
  iptables -A FORWARD -i "$WIFI_IF" -o "$LAN_IF" -j ACCEPT

  echo "热点已启动: $HOTSPOT_SSID"

elif [ "$MODE" == "stop" ]; then
  echo "停止热点..."
  nmcli con down "$HOTSPOT_SSID"
  nmcli con delete "$HOTSPOT_SSID"

  echo "清理 NAT..."
  iptables -t nat -D POSTROUTING -o "$LAN_IF" -j MASQUERADE || true
  iptables -D FORWARD -i "$LAN_IF" -o "$WIFI_IF" -m state --state RELATED,ESTABLISHED -j ACCEPT || true
  iptables -D FORWARD -i "$WIFI_IF" -o "$LAN_IF" -j ACCEPT || true

  echo 0 > /proc/sys/net/ipv4/ip_forward
fi


