# RPI setup notes

First, install an Ubuntu Server version using the rpi_imager tool (setup wifi in settings).

On the host pc (crosscompiling kernel):

- Go to [https://wiki.linuxfoundation.org/realtime/start](https://wiki.linuxfoundation.org/realtime/start), click on "Latest Stable Version" and look for the name of the most recent stable kernel

- Run `git clone git@github.com:AndPatr/rpi_preempt_rt.git` 

- Edit `setup_rpi.bash` with the name of the kernel you want to crosscompile 

- Run `./crosscompile.bash`

- Push the newly compiled .deb packages to github 

On the PI:

- Generate an ssh key for the Pi with `ssh-keygen -t ed25519 -C "your_email@example.com"` and add it to your Github account

- Run `git clone git@github.com:AndPatr/rpi_preempt_rt.git` 
								
- Go inside rpi_preempt_rt and run `./setup_rpi.bash` &rarr; the RPI will reboot to load the new rt kernel

---
 
## Miscellaneous commands 

To modify/add a wifi after the first boot:

- Modify `/etc/netplan/50-cloud-init.yaml` and add your wifi name and password (careful with the indentation)

- Apply changes and connect with `sudo netplan apply` or `sudo netplan --debug apply`

- To check run `ip a`

To SSH to the RPI:

- Power it up and wait. After 2-4 minutes max the RPI should be connected to network. On the host pc use the command `sudo nmap -sP xxx.xxx.x.x/xx` (for instance `sudo nmap -sP 192.168.1.0/24`) to retrieve RPI's IP address (the RPI should appear with a name like "Raspberry Pi Trading")

- With the IP address you found execute `ssh ubuntu@xxx.xxx.x.xx` where xxx.xxx.x.xx is the current IP address of RPI. Finally insert the ssh password for the RPI user (e.g. "sudowudo"). username: "ap_rpi"

- Should you get 'remote host identification has changed!' run `ssh-keygen -R xxx.xxx.x.xx`
 
To check RPI termperature run `vcgencmd measure_temp`

To check disk partion and free space `df -h`

To modify keyboard layout edit `/etc/default/keyboard`

Check the kernel is enabled with `uname -a`

To make a device with ubuntu 20.04 a wifi access point (useful to run ROS on multiple machines without having to use an external wifi router)

- `nmcli d` simply shows the wifi connections
- `nmcli d wifi list` list available networks	
- `nmcli d wifi hotspot ifname <wifi_iface> ssid <ssid> password <password>` (<wifi_iface>- wifi interface name [e.g. wlp58s0],<ssid> is the wifi name, <password> is the password to be assigned )

### CAN bus

To check if can0 and can1 are initialized run `dmesg | grep spi`

To initialize can0 (can1 the same way) and set the bitrate run `sudo ip link set can0 up type can bitrate 1000000`

To read from terminal the bus run
`candump can0 -xct z -n 10`

Alternatively, you can also use ` python3 -m can.viewer -c "can0" -i "socketcan"`

To check possible message conflict on the CAN buses
`canbusload -rbt can0@1000000 can1@1000000`

To automatically bring up can0 and can1 at startup:
```
sudo touch /etc/systemd/network/80-can.network

sudo nano /etc/systemd/network/80-can.network

```

with the following content:

```
[Match]
Name=can*

[CAN]
BitRate=1000000

```

Then, run

`sudo systemctl enable systemd-networkd`

to start systemd-networkd at boot time

### Setting up the RTC (made available by the Seedstudio CAN-SPI hat) on RPI:

- Simply add `dtoverlay=i2c-rtc,rtc_name` to `/boot/firmware/usercfg.txt`

- Then run `sudo hwclock` to enable the rtc

- To read the time: `hwclock --show`
