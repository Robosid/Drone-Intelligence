--> Follow circular trajectory while keeping the forward speed constant and ajusting 
its heading to be tangent to the trajectory.
--> Offtrack error needs to be controlled by adjusting the lateral speed, Vy.
--> Lateral speed will be used to keep the drone on the track.
--> Provide WP with GCS, and the script will create a simple circular path around the waypoint
--> The drone will be on Velocity mode. Set the forward speed, Vx, as constant
and the heading as tangent to the trajectory and Vy will be used to keep the drone on the 
trajectory.
--> In Mission state, implement feedback control loop, where heading is set as 90 deg from Bearing, Vx as constant ground speed and Vy is proportional to offtrack error (Thus, its the distance between vehicle position and circumference).
--> Position will be check wrt radius.
--> After n turns, return.
