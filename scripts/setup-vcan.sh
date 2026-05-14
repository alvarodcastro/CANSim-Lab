#!/bin/bash
# ============================================================
#  CAN Bus Lab - Host Setup Script
#  Run this ONCE on the host before starting containers
# ============================================================

set -e

echo "[*] Loading kernel modules..."
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

echo "[*] Creating virtual CAN interface vcan0..."
if ip link show vcan0 &>/dev/null; then
    echo "[!] vcan0 already exists, bringing it up..."
    sudo ip link set vcan0 up
else
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 up
fi

echo "[+] vcan0 is up:"
ip link show vcan0

echo ""
echo "[*] To monitor raw CAN traffic on the host, run:"
echo "    candump vcan0"
echo ""
echo "[*] To inject a test frame:"
echo "    cansend vcan0 7DF#0201050000000000"
echo ""
echo "[+] Setup complete! You can now run: docker-compose up"
