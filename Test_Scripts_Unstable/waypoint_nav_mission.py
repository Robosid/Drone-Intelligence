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

"""

Waypoint Navigation using Missions, without the involvement of GCS.
NOTE: In TEST phase. HIGHLY UNSTABLE. USE WITH CAUTION.

"""


# Last modified by : Robosid
# Last modifed on : 03 / 19 / 2018

import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    #vehicle.flush() 
    cmds.upload()

    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
        vehicle,            #--- vehicle object
        wp_Last_Latitude,   #--- [deg]  Target Latitude
        wp_Last_Longitude,  #--- [deg]  Target Longitude
        wp_Last_Altitude):  #--- [m]    Target Altitude
    
    """
    
    Upload the mission with the last WP as given and outputs the ID to be set
    
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Save the vehicle commands to a list
    #missionlist=[]
    #for cmd in cmds:
    #    missionlist.append(cmd)

    # Modify the mission as needed. For example, here I change the
    wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when I call upload())
    cmds.clear()
    vehicle.commands.clear()

    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    
    return (cmds.count)

def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True

def get_number_wp(vehicle):
    """
    Downloads the mission and returns the wp list and number of WP 
    
    Input: 
        vehicle
        
    Return:
        n_wp, wpList
    """

    cmds = vehicle.commands
    cmds.download()
    cmds.wait_valid() # wait until download is complete.
    #cmds.wait_ready() # wait until download is complete.

    n_WP        = 0
    for wp in cmds:
        n_WP += 1 
        
    return n_WP

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
#print('Connecting...')
#vehicle = connect('udp:127.0.0.1:14551')
#vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)
print(">>>> Connecting with the UAV <<<")
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect', default='/dev/ttyS0')
args = parser.parse_args()

connection_string = args.connect


print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(args.connect, baud=921600, wait_ready=True)    #- wait_ready flag hold the program untill all the parameters are been read (=, not .)

#--------------------------------------------------
#-------------- INITIALIZE  
#--------------------------------------------------      
#-- Setup the commanded flying speed
gnd_speed = 2 # [m/s]
mode      = 'GROUND'
clear_mission(vehicle)
#--------------------------------------------------
#-------------- MAIN FUNCTION  
#--------------------------------------------------    
while True:
    
  if mode == 'GROUND':

    time.sleep(2)
    while True:

    	# DO NOT add fake/home waypoint
    	wp_flag = input("DO you want to push a waypoint onto the mission_stack? Press 'n' if you don't.") 
    	
    	if (wp_flag == 'n' or 'N'):
    		break
      
      # Positions in the form of Location in global frame, with altitude relative to the home location.
      wp_lon = float(input("Please enter the Longitude of the next waypoint: ")) # in the form of vehicle.location.global_relative_frame.lon
      wp_lat = float(input("Please enter the Latitude of the next waypoint: ")) # in the form of vehicle.location.global_relative_frame.lat
      wp_alt = float(input("Please enter the Altitude of the next waypoint: ")) # in the form of vehicle.location.global_relative_frame.alt
      wp = LocationGlobalRelative(wp_lat,wp_lon,wp_alt)
      add_last_waypoint_to_mission(vehicle, wp.lat, 
                                     wp.lon, 
                                     wp.alt)
'''      add_last_waypoint_to_mission(vehicle, wp.lat, 
                                     wp.lon, 
                                     wp.alt)
'''
      time.sleep(1)
    n_WP = get_number_wp(vehicle)
    if n_WP > 0:
      print ("A valid mission has been uploaded: takeoff!")
      mode = 'TAKEOFF'
          
  elif mode == 'TAKEOFF':
     
      #-- Add a fake waypoint at the end of the mission
      add_last_waypoint_to_mission(vehicle, vehicle.location.global_relative_frame.lat, 
                                     vehicle.location.global_relative_frame.lon, 
                                     vehicle.location.global_relative_frame.alt)
      print("Home waypoint added to the mission")
      time.sleep(1)
      #-- Takeoff
      arm_and_takeoff(10)
      
      #-- Change the UAV mode to AUTO
      print("Changing to AUTO")
      ChangeMode(vehicle,"AUTO")
      
      #-- Change mode, set the ground speed
      vehicle.groundspeed = gnd_speed
      mode = 'MISSION'
      print ("Switch mode to MISSION")
      
  elif mode == 'MISSION':
      #-- Here I just monitor the mission status. Once the mission is completed I go back
      #-- vehicle.commands.cout is the total number of waypoints
      #-- vehicle.commands.next is the waypoint the vehicle is going to
      #-- once next == cout, I just go home
      
      print ("Current WP: %d of %d "%(vehicle.commands.next, vehicle.commands.count))
      if vehicle.commands.next == vehicle.commands.count:
          print ("Final waypoint reached: go back home")
          #-- First I clear the flight mission
          clear_mission(vehicle)
          print ("Mission deleted")
          
          #-- I go back home
          ChangeMode(vehicle,"RTL")
          mode = "BACK"
          
  elif mode == "BACK":
      if vehicle.location.global_relative_frame.alt < 1:
          print ("Switch to GROUND mode, waiting for new missions")
          mode = 'GROUND'
  
  
  
  
  time.sleep(0.5)
