##in QGC
set parameters:
Serial2_protocol=1
Serial2_baud=921600/5600
log_backend_type=3 which is dataflash.

sudo apt-get update    #Update the list of packages in the software center
sudo apt-get upgrade
sudo pip install pyserial
sudo apt-get install python-lxml
sudo apt-get install screen python-wxgtk2.8 python-matplotlib python-opencv python-pip python-numpy python-dev libxml2-dev libxslt-dev
sudo pip install future
sudo apt-get install git
sudo pip install pymavlink
sudo pip install mavproxy

sudo raspi-config -> Serial (For non Jessie) -> Disable serial login shell -> Enable serial interface

##Optional
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
paste: network = { ssid=" fill "
				   psk="password" 
                   #priority=number_i_e_10 (in case of multiple networks)
				   #scan_ssid=1 (in case of hidden ssid)
				   
## (Optional) Increase virtual/swap memory to 1024
sudo nano /etc/dphys-swapfile
change conf_swapsize to 1024
Reboot the service by:
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start

##Optional - Activate X11 Tunneling 
sudo nano /etc/ssh/sshd_config
set - X11Forwarding: yes
	  X11displayoffset: 10
	  X11uselocalhost: yes

##optional - install samba for easy folder sharing management 
sudo apt-get install samba samba-common-bin
Create a sharded folder: mkdir drone
Then, edit the samba config file:
sudo nano /etc/samba/smb.conf
set: wins support = yes
	 netbios name = "name" //name: raspberrypi
	 workgroup = "name" //name:workgroup
Paste the following at the end:
[pidrone]
comment=Raspberry pi drone folder
path=home/pi/drone
browsable=yes
writeable=yes
only guest=no
create mask=0777
directory mask=0777
public=no

##optional: continued from previous step::
setup samba password:
sudo smbpasswd -a pi

##check serial port:
ls /dev/tty*
check for ttyAMA0 and ttyS0 and Serial0

##speed up UART to enable fast connection with the autopilot
sudo nano /boot/config.txt
enable_uart=1
init_uart_clock=64000000
sudo reboot

#test 
sudo stty -F /dev/ttyAMA0 1500000
sudo stty -F /dev/ttyS0 1500000
sudo stty -F /dev/Serial0 1500000

## To Test
sudo -s
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --aircraft MyCopter
NOTE: change ttyAMA0 to Serial0, for jessie.
      change ttyAMA0 to ttyS0 for stretch
	  change 57600 to 921600. keep 57600 if serial2_baud=57600.

##Few test commands:
  Mode STABILIZE
  param show ARMING_CHECK
  param set ARMING_CHECK 0
  arm throttle
  mode loiter

## To enable uart serial connection 
cat /boot/config.txt
set enable_uart=1
--------------------------------------------------
##After proper connection:
Configure MAVProxy to always run:
To setup MAVProxy to start whenever the RPi is 
restarted open a terminal window and edit the
/etc/rc.local file, adding the following lines
just before the final “exit 0” line:
(
date
echo $PATH
PATH=$PATH:/bin:/sbin:/usr/bin:/usr/local/bin
export PATH
cd /home/pi
screen -d -m -s /bin/bash mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
) > /tmp/rc.log 2>&1
exit 0

Whenever the RPi connects to the Pixhawk, 
three files will be created in the 
/home/pi/MyCopter/logs/YYYY-MM-DD directory:

    mav.parm : a text file with all the parameter values from the Pixhawk
    flight.tlog : a telemetry log including the vehicles altitude, 
	attitude, etc which can be opened using the mission planner 
	(and a number of other tools)
    flight.tlog.raw : all the data in the .tlog mentioned above 
	plus any other serial data received from the Pixhawk which 
	might include non-MAVLink formatted messages like startup 
	strings or debug output
	
	to connect to MAVProxy application that has been automatically started:
	sudo screen -x
	
	------------------------------------------------------
	
	##Drone Kit setup:
	
	sudo apt-get install python-pip python-dev python-numpy python-opencv python-serial python-pyparsing python-wxgtk2.8 libxml2-dev libxslt-dev
 	sudo pip install droneapi
	echo "module load droneapi.module.api" >> ~/.mavinit.scr
	sudo apt-get install python-pip python-dev
	sudo pip install dronekit
	sudo pip install dronekit-sitl
	
	##SITL 
	sudo pip install dronekit-sitl -UI
	dronekit-sitl copter
	SITL will then start and wait for TCP connections on 127.0.0.1:5760
	
	##Dronekit start
	MANUAL> module load droneapi.module.api
	or
	MANUAL> api start vehicle_state.py
	
--------------------------------------------------
###References::

## For Dronekit Reference:
https://github.com/dronekit/dronekit-python
http://python.dronekit.io/guide/index.html

## For MAVlink Reference:
https://github.com/mavlink/mavlink
https://mavlink.io/en/
http://ardupilot.org/dev/docs/mavlink-commands.html
http://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html

## For MAVproxy Reference:
https://github.com/ArduPilot/MAVProxy

------------------------------
##To run script
python file.py --connect /dev/tts0
----------------------------------
## Dronekit Python Autostart on Pi and Pixhawk
Make a sh file containing:
sleep 60
cd /path/to/folder
python file.py --connect /dev/ttyS0
**can use /usr/bin/python instead of python, above.
------------------------------------------------------
## To counter GPS HDOP error and GPS speed error:
set param GPS_HDOP_GOOD 500
set Geofence off
set ARMING_CHECK 1998 / 2003.
set EK2_GPS_CHECK: 27.
**Alternative method is to manually edit the GPS_Info() class.
-----------------------------------------------------------
## For countering GPS Vert Error and other appoximations:
auto_Switch = blend
blend_mask = 7
ek2_check_scale = 200
hdop = 800
WPNAV_SPEED_UP
-----------------------------------------------------------
