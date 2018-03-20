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

# Script for Takeoff to x metres, travel to waypoints defined by user, calculate distance to waypoint with approximation, and then auto RTL.

# Last modified by : Robosid
# Last modifed on : 03 / 20 / 2018



from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

#-- Connect to the vehicle
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='/dev/ttyS0')
args = parser.parse_args()

connection_string = args.connect


print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(args.connect, baud=115200, wait_ready=True)

#-- Define the function for takeoff
def arm_and_takeoff(tgt_altitude):
    print("Arming motors")
    
    while not vehicle.is_armable:
        time.sleep(1)
        
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed: time.sleep(1)
    
    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)
    
    #-- wait to reach the target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        
        if altitude >= tgt_altitude -1:
            print("Altitude reached")
            break
            
        time.sleep(1)
        
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
        
#------ MAIN PROGRAM ----
arm_and_takeoff(10)

#-- set the default speed
vehicle.airspeed = 7

def location_callback(self, attr_name, value):
    print(vehicle.location.global_relative_frame)
#    print(vehicle.location.global_frame)

wps = []
k = []
time.sleep(2)
while True:

  wp_flag = input("DO you want to add a waypoint? Press 'n' if you don't.") 
  
  if (wp_flag == 'n' or 'N'):
    break
  # Positions in the form of Location in global frame, with altitude relative to the home location.
  wp_lon = float(input("Please enter the Longitude of the next waypoint: ")) # in the form of vehicle.location.global_relative_frame.lon
  k.append(wp_lon)
  wp_lat = float(input("Please enter the Latitude of the next waypoint: ")) # in the form of vehicle.location.global_relative_frame.lat
  k.append(wp_lat)
  wp_alt = float(input("Please enter the Altitude of the next waypoint: ")) # in the form of vehicle.location.global_relative_frame.alt
  k.append(wp_alt)
  wps.append(k)
  k = []
  time.sleep(1)


count = 0
for i in wps:
  count = count + 1
  print ("go to wp", count)
  wp = LocationGlobalRelative(i[0], i[1], i[2])
  ## simple_goto(location, airspeed=None, groundspeed=None) //Syntax
  vehicle.simple_goto(wp)

  ## TO get an idea about what 'vehicle.location.global_relative_frame' returns so that we can keep a 
  ## check for position/waypoint reach, before it starts out on its next waypoint. Precise or proximity check.
  '''
  print("Adding a location listener, for testing")
  vehicle.add_attribute_listener('location.global_relative_frame', location_callback) #-- message type, callback function
  time.sleep(30)
  '''

  #--- Here is where I can add more action later
  #time.sleep(30) # For safety sake. Not needed if the above time sleep works, or if I can incorporate Location check till waypoint reach.
  
  distancetopoint = get_distance_metres(vehicle.location.global_frame, wp)
  while not (distancetopoint < 1 and (vehicle.location.global_relative_frame.alt > wp.alt - 1  and vehicle.location.global_relative_frame.alt < wp.alt + 1 )) :
    distancetopoint = get_distance_metres(vehicle.location.global_frame, wp)
  
#--- Coming back
print("Coming back")
vehicle.mode = VehicleMode("RTL")

time.sleep(20)

#-- Close connection
vehicle.close()



