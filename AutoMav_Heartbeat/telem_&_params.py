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

# This script will introduce multiple things:
# > Listener test
# > Read and handle telemetry from the UAV
# > Read and change parameters

# Last modified by : Robosid
# Last modifed on : 03 / 14 / 2018

from dronekit import connect, VehicleMode
import time

#--- Start the Software In The Loop (SITL)
#import dronekit_sitl
#
#sitl = dronekit_sitl.start_default()   #(sitl.start)
#connection_string = sitl.connection_string()

#--- Now that I have started the SITL and I have the connection string (basically the ip and udp port)...
# SITL is not tested yet............kept for later.

print(">>>> Connecting with the UAV <<<")
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='/dev/ttyS0')
args = parser.parse_args()

connection_string = args.connect


print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(args.connect, baud=921600, wait_ready=True)    #- wait_ready flag hold the program untill all the parameters are been read (=, not .)
	
#-- Read information from the autopilot:
#- Version and attributes
vehicle.wait_ready('autopilot_version')I
print('Autopilot version: %s'%vehicle.version)

#- Does the firmware support the companion pc to set the attitude?
print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

#- Read the actual position
print('Position: %s'% vehicle.location.global_relative_frame)

#- Read the actual attitude roll, pitch, yaw
print('Attitude: %s'% vehicle.attitude)

#- Read the actual velocity (m/s)
print('Velocity: %s'%vehicle.velocity) #- North, east, down

#- When did I receive the last heartbeat
print('Last Heartbeat: %s'%vehicle.last_heartbeat)

#- Is the vehicle good to Arm?
print('Is the vehicle armable: %s'%vehicle.is_armable)

#- Which is the total ground speed?   Note: this is settable
print('Groundspeed: %s'% vehicle.groundspeed) #(%)

#- What is the actual flight mode?    Note: this is settable
print('Mode: %s'% vehicle.mode.name)

#- Is the vehicle armed               Note: this is settable
print('Armed: %s'%vehicle.armed)

#- Is thestate estimation filter ok?
print('EKF Ok: %s'%vehicle.ekf_ok)



#----- Adding a listener
#-- dronekit updates the variables as soon as it receives an update from the UAV
#-- I can define a callback function for predefined messages or define one for
#-- any mavlink message 

def attitude_callback(self, attr_name, value):
    print(vehicle.attitude)


print("")
print("Adding an attitude listener")
vehicle.add_attribute_listener('attitude', attitude_callback) #-- message type, callback function
time.sleep(5)

#--- Now print the attitude from the callback for 5 seconds, then I remove the callback
vehicle.remove_attribute_listener('attitude', attitude_callback) #(.remove)


#--- I can create a callback even with decorators, check the documentation out for more details


#---- PARAMETERS
print("Maximum Throttle: %d"%vehicle.parameters['THR_MIN']) 

#-- I can read and write the parameters
vehicle.parameters['THR_MIN'] = 50
time.sleep(1)
print("Maximum Throttle: %d"%vehicle.parameters['THR_MIN'])



#--- Now I close the simulation
vehicle.close()
sitl.stop()

print("done")




















































 

