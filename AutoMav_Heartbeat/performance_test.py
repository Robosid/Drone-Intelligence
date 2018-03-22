''' Copyright [2018] [Siddhant Mahapatra]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
'''

# Created by Siddhant Mahapatra (aka Robosid). for AutoMav of Project Heartbeat. 

 '''This performance test logs the interval between messages being 
sent by Dronekit-Python and an acknowledgment being received 
from the autopilot. It provides a running report of the maximum, 
minimum, and most recent interval for 30 seconds.
'''

# Last modified by : Robosid
# Last modifed on : 03 / 22 / 2018

from __future__ import print_function
from dronekit import connect
from pymavlink import mavutil
import time
import sys
from datetime import datetime

'''
#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Generates max, min and current interval between message sent and ack recieved. Will start and connect to SITL if no connection string specified.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string=args.connect

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

'''
#-- Connect to the vehicle
#print('Connecting...')
#vehicle = connect('udp:127.0.0.1:14551')
print(">>>> Connecting with the UAV <<<")
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='/dev/ttyS0')
args = parser.parse_args()

connection_string = args.connect


print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(args.connect, baud=921600, wait_ready=True)    #- wait_ready flag hold the program untill all the parameters are been read (=, not .)

#global vehicle


def cur_usec():
    """Return current time in usecs"""
    # t = time.time()
    dt = datetime.now()
    t = dt.minute * 60 + dt.second + dt.microsecond / (1e6)
    return t

class MeasureTime(object):
    def __init__(self):
        self.prevtime = cur_usec()
        self.previnterval = 0
        self.numcount = 0
        self.reset()

    def reset(self):
        self.maxinterval = 0
        self.mininterval = 10000
        
    def log(self):
        #print "Interval", self.previnterval
        #print "MaxInterval", self.maxinterval
        #print "MinInterval", self.mininterval
        sys.stdout.write('MaxInterval: %s\tMinInterval: %s\tInterval: %s\r' % (self.maxinterval,self.mininterval, self.previnterval) )
        sys.stdout.flush()


    def update(self):
        now = cur_usec()
        self.numcount = self.numcount + 1
        self.previnterval = now - self.prevtime
        self.prevtime = now
        if self.numcount>1: #ignore first value where self.prevtime not reliable.
            self.maxinterval = max(self.previnterval, self.maxinterval)
            self.mininterval = min(self.mininterval, self.previnterval)
            self.log()


acktime = MeasureTime()


#Create COMMAND_ACK message listener.
@vehicle.on_message('COMMAND_ACK')
def listener(self, name, message):
    acktime.update()
    send_testpackets()


def send_testpackets():
    #Send message using `command_long_encode` (returns an ACK)
    msg = vehicle.message_factory.command_long_encode(
                                                    1, 1,    # target system, target component
                                                    #mavutil.mavlink.MAV_CMD_DO_SET_RELAY, #command
                                                    mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
                                                    0, #confirmation
                                                    0, 0, 0, 0, #params 1-4
                                                    0,
                                                    0,
                                                    0
                                                    )

    vehicle.send_mavlink(msg)

#Start logging by sending a test packet
send_testpackets()

print("Logging for 30 seconds")
for x in range(1,30):
    time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()

if not args.connect:
    # Shut down simulator if it was started.
    sitl.stop()
