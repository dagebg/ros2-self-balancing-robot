# Development Networking Setup (Raspberry Pi over Ethernet)

During development, the Raspberry Pi is connected directly to a Fedora laptop via Ethernet.  
The Fedora laptop shares its Wi‑Fi internet connection with the Pi and acts as a simple NAT gateway. [web:39][web:41][web:46][web:72]

## Topology

```text
[Internet] ⇄ [Wi‑Fi router] ⇄ [Fedora laptop]
                                   │
                                   │  eno1 (10.42.0.1/24)
                                   │
                              eth0 (10.42.0.100/24)
                                   │
                                 [Raspberry Pi]
```

- Fedora laptop:
  - Wi‑Fi interface: `wlp11s0` (internet)
  - Ethernet interface: `eno1` (to Pi)
- Raspberry Pi:
  - Ethernet interface: `eth0` (to Fedora)
  - Optional Wi‑Fi: `wlan0` (not used as default route in this setup)

---

## Raspberry Pi configuration (netplan + systemd‑resolved)

The Pi uses `systemd-networkd` via netplan and `systemd-resolved` for DNS.  
Static addressing and routing on `eth0` make the Pi always reachable at `10.42.0.100`, with the Fedora laptop `10.42.0.1` as its default gateway. [web:39][web:41][web:46][web:74]

Edit the netplan file (e.g. `/etc/netplan/50-cloud-init.yaml`):

```yaml
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: false
      dhcp6: false
      addresses:
        - 10.42.0.100/24
      routes:
        - to: default
          via: 10.42.0.1

  wifis:
    wlan0:
      optional: true
      dhcp4: false
      regulatory-domain: "DE"
      addresses:
        - 192.168.178.200/24
      # no default route from Wi‑Fi
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
      access-points:
        "FRITZ!Box 7362 SL":
          password: "<redacted>"
```

Apply and verify on the Pi:

```bash
sudo netplan apply

ip a show eth0
ip route show
resolvectl status

ping -c 3 10.42.0.1       # Fedora host
ping -c 3 8.8.8.8         # raw internet
ping -c 3 google.com      # DNS check
```

Expected on the Pi:

- `eth0` has `10.42.0.100/24`
- Route table contains:

  ```text
  default via 10.42.0.1 dev eth0
  10.42.0.0/24 dev eth0 proto kernel scope link src 10.42.0.100
  ```

- `resolvectl status` shows the configured DNS servers.

---

## Fedora laptop configuration

The Fedora laptop has a static IP on `eno1` for the Pi network, enables IPv4 forwarding, and uses a small script + systemd unit to set up NAT from `eno1` to `wlp11s0`. [web:72][web:74][web:78]

### 1. Static IP on `eno1`

Configure the `eno1` connection once via NetworkManager (replace the connection name as needed):

```bash
nmcli connection show            # find the connection for eno1

sudo nmcli connection modify "<eno1-connection-name>" \
  ipv4.addresses 10.42.0.1/24 \
  ipv4.method manual \
  connection.autoconnect yes

sudo nmcli connection down "<eno1-connection-name>" || true
sudo nmcli connection up "<eno1-connection-name>"

ip a show eno1
ip route show
```

Expected route:

```text
10.42.0.0/24 dev eno1 proto kernel scope link src 10.42.0.1
```

### 2. Enable IPv4 forwarding

Create a sysctl drop‑in so forwarding is enabled on every boot:

```bash
echo 'net.ipv4.ip_forward = 1' | sudo tee /etc/sysctl.d/99-ip-forward.conf
sudo sysctl --system
sysctl net.ipv4.ip_forward
# should print: net.ipv4.ip_forward = 1
```

This keeps the host ready to route traffic between `eno1` and `wlp11s0`. [web:72][web:78]

### 3. NAT + forwarding script

This script ensures IPv4 forwarding is on and injects the minimal iptables rules to masquerade traffic from the Pi network out via Wi‑Fi. [web:63][web:69][web:78]

Create `/usr/local/sbin/pi-nat.sh`:

```bash
sudo tee /usr/local/sbin/pi-nat.sh >/dev/null << 'EOF'
#!/usr/bin/env bash
# NAT + forwarding for Raspberry Pi on eno1 via wlp11s0

# ensure forwarding is on
sysctl -w net.ipv4.ip_forward=1

# add iptables rules only if missing
iptables -t nat -C POSTROUTING -o wlp11s0 -j MASQUERADE 2>/dev/null \
  || iptables -t nat -A POSTROUTING -o wlp11s0 -j MASQUERADE

iptables -C FORWARD -i eno1 -o wlp11s0 -j ACCEPT 2>/dev/null \
  || iptables -A FORWARD -i eno1 -o wlp11s0 -j ACCEPT

iptables -C FORWARD -i wlp11s0 -o eno1 -m state --state RELATED,ESTABLISHED 2>/dev/null \
  || iptables -A FORWARD -i wlp11s0 -o eno1 -m state --state RELATED,ESTABLISHED
EOF

sudo chmod +x /usr/local/sbin/pi-nat.sh
```

### 4. Systemd unit to apply NAT at boot

Create `/etc/systemd/system/pi-nat.service`:

```bash
sudo tee /etc/systemd/system/pi-nat.service >/dev/null << 'EOF'
[Unit]
Description=Enable NAT for Raspberry Pi on eno1 via wlp11s0
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/pi-nat.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now pi-nat.service
```

On the Fedora host you can check:

```bash
sysctl net.ipv4.ip_forward
sudo iptables -L -n -v
sudo iptables -t nat -L -n -v
```

You should see:

- `net.ipv4.ip_forward = 1`
- A `MASQUERADE` rule in `POSTROUTING` for `wlp11s0`
- `FORWARD` rules allowing traffic between `eno1` and `wlp11s0`

---

## Usage

With both machines booted and the Ethernet cable connected:

From Fedora:

```bash
ping -c 3 10.42.0.100
ssh <user>@10.42.0.100
```

From the Pi:

```bash
ping -c 3 10.42.0.1     # Fedora host
ping -c 3 8.8.8.8       # internet connectivity
ping -c 3 google.com    # DNS resolution
```

If these checks pass, the Pi has:

- Stable Ethernet SSH at `10.42.0.100`
- Internet access through the Fedora host
- Working DNS for package installation and ROS2 development