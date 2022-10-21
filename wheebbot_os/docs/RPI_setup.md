# RPI setup notes

## To install Ubuntu 20.04 Server on RPI:

- Install RPI Imager

- Write ubuntu 20.04 on SD

- Go to network-config file

- Insert wifi name and password (IMPORTANT: do not use Imager custom option, they screw things up)
 
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

To modify keyboard layouedit `/etc/default/keyboard`

---

## To install ROS Noetic run:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 

sudo apt install curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

```
`sudo apt install ros-noetic-desktop-full` or `sudo apt install ros-noetic-ros-base` (no gui nor visualization tools)

```
sudo apt install python3-rosdep

sudo rosdep init

rosdep update
```
---

## To install ROS2 foxy on the RPI run:

`locale` &rarr; check for UTF-8
 
```
sudo apt update && sudo apt install locales 

sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8
```

`locale` &rarr; verify settings

```
sudo apt update && sudo apt install curl gnupg2 lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update

sudo apt install ros-foxy-ros-base

sudo apt install python3-colcon-common-extensions
```
`echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc` &rarr; setting domain ID (necessary for DDS); any integer between 0 and 101 works fine

---

## To install and setup arduino-cli (currently not used):

```
cd ~

curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh 

nano ~/.bashrc 
```

Add the line `export PATH="$HOME/bin:$PATH"`

```
sudo reboot

arduino-cli core update-index

arduino-cli core install arduino:samd 

arduino-cli config init 

arduino-cli sketch new MyFirstSketch 

arduino-cli board list 

arduino-cli compile --fqbn arduino:samd:nano_33_iot SketchName

arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:samd:nano_33_iot SketchName

arduino-cli lib search lib_guess

arduino-cli lib install "Adafruit BNO055"

arduino-cli lib install "Adafruit BNO08x"

arduino-cli board listall

```

---

## To install rosserial_arduino:

```
sudo apt-get install ros-noetic-rosserial-arduino

sudo apt-get install ros-noetic-rosserial

cd Arduino/libraries

rm -rf ros_lib

rosrun rosserial_arduino make_libraries.py
```

Important: if using NaNo IoT or, in general, USB native boards insert the line "#define USE_USBCON" at the begginning of the code
---

## To install ODrive tool:

```
sudo apt install python3 python3-pip

sudo pip3 install --upgrade odrive
```

---

## To enable the RPI CAN hat (by Seedstudio):

`sudo nano /boot/firmware/usercfg.txt` or `/boot/firmware/config.txt` (either of those should work)

Add `dtoverlay=seeed-can-fd-hat-v2` at the end of the file

To check is can0 and can1 are initialized run `dmesg | grep spi`

---

## CAN utilities&Co:

``` 
apt get install can-utils 

sudo apt install python3-pip

pip3 install python-can

pip install cantools
```

To initialize can0 (can1 the same way) and set the bitrate run
`sudo ip link set can0 up type can bitrate 250000`

To read from terminal the bus run
`candump can0 -xct z -n 10`

Alternatively, you can also use 
` python3 -m can.viewer -c "can0" -i "socketcan"`
(nicer output)

It is possible to use cantools to generate the .dbc file and C code for coding/packaging/unpackaging/decoding messages from a given python database file (.dbc)

To check possible message conflict on the CAN buses
`canbusload -rbt can0@1000000 can1@1000000`

---

## ODrive ROS drivers (not used, they don't interface with socketcan)

`git clone https://github.com/neomanic/odrive_ros.git` &rarr; Python based

`git clone https://github.com/johnkok/ros_odrive.git` &rarr; C++ based

---

## To run python scripts in terminal 

`python3 script_name.py`

---

## To remotely copy a file to the RPI (via ssh)

`scp file_name ubuntu@xxx.xxx.x.xx: folder_to_be_copied_in`    

---

## Building (cross-compiling) and installing kernel and PREEMPT-RT patches (not used)
```
mkdir ~/kernel
cd ~/kernel
```
```
sudo apt install git bc bison flex libssl-dev make libc6-dev libncurses5-dev
sudo apt install crossbuild-essential-arm64
sudo apt-get install gcc-arm*
sudo apt-get install libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf fakeroot
```
```
cd /home/andrea/Desktop/università/magistrale/tesi/scripts/raspberry4B_scriptsfiles/RPI_Ubuntu_20.04_server/kernels
git clone -b rpi-5.10.y https://github.com/raspberrypi/linux.git linux-5.10.y
wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.10/patch-5.10.73-rt54.patch.gz
gunzip patch-5.10.73-rt54.patch.gz
cd linux-5.10.y/
patch -p1 < ../patch-5.10.73-rt54.patch
```
```
KERNEL=kernel8
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bcm2711_defconfig
yes '' | make oldconfig 
```
```
make menuconfig 
```
Set the following options:

- Disable virtualization
  - -> Virtualization
    - -> N
- Enable CONFIG_PREEMPT_RT
  - -> General Setup
    - -> Preemption Model (Fully Preemptible Kernel (Real-Time))
         (X) Fully Preemptible Kernel (Real-Time)

- Enable CONFIG_HIGH_RES_TIMERS
  - -> General setup
    - -> Timers subsystem
         [*] High Resolution Timer Support

- Enable CONFIG_NO_HZ_FULL
  - -> General setup
    - -> Timers subsystem
      - -> Timer tick handling (Full dynticks system (tickless))
           (X) Full dynticks system (tickless)

- Set CONFIG_HZ_1000 (note: this is no longer in the General Setup menu, go back twice)
  - -> processor type and features
    - -> Timer frequency (1000 HZ)
         (X) 1000 HZ

- Set CPU_FREQ_DEFAULT_GOV_PERFORMANCE [=y]
  - ->  power management and ACPI options
    - -> CPU Frequency scaling
      - -> CPU Frequency scaling (CPU_FREQ [=y])
        - -> Default CPUFreq governor (<choice> [=y])
             (X) performance

Then save and exit.

`make -j 6 ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs`

Debian packages should have been generated.

---

## Install pre-compiled rt kernel (used alternative; easier and does not require compilation):

- Reference: [kdoren_linux_realeases](https://github.com/kdoren/linux/releases/)

- Installation istructions for Ubuntu @ [preempt_rt_install_instructions](https://github.com/kdoren/linux/wiki/Installation-of-kernel-from-deb-package-%28Ubuntu%29)


Reboot and check the kernel is enabled with:

`uname -a`

---

## Preemption test:

```
sudo apt-get install libboost-thread-dev libboost-program-options-dev

git clone git://git.kernel.org/pub/scm/linux/kernel/git/clrkwllms/rt-tests.git
cd rt-tests
make all
sudo cp ./cyclictest /usr/bin/
sudo cyclictest -l50000000 -m -S -p90 -i200 -h400 -q > output.txt

grep -v -e "^#" -e "^$" output.txt | tr " " "," | tr "\t" "," >histogram.csv # generates histogram (run on host pc, not on RPI)
sed -i '1s/^/time,core1,core2,core3,core4\n /' histogram.csv
```

---

## Setting up the RTC (made available by the Seedstudio CAN-SPI hat) on RPI:

- Simply add `dtoverlay=i2c-rtc,rtc_name` to `/boot/firmware/usercfg.txt`

- Then run `sudo hwclock` to enable the rtc

- To read the time: `hwclock --show`

---
## To automatically bring up can0 and can1 at startup:

Create a file

`sudo nano /etc/systemd/network/80-can.network`

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

---
## To make a device with ubuntu 20.04 a wifi access point (useful to run ROS on multiple machines without having to use an external wifi router)

- `nmcli d` simply shows the wifi connections
- `nmcli d wifi list` list available networks	
- `nmcli d wifi hotspot ifname <wifi_iface> ssid <ssid> password <password>` (<wifi_iface>- wifi interface name [e.g. wlp58s0],<ssid> is the wifi name, <password> is the password to be assigned )
