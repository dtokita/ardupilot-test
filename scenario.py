from pymavlink import mavutil
import time
import json
from dronekit import connect, VehicleMode, LocationGlobalRelative, GPSInfo, Command

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
  msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,       # time_boot_ms (not used)
    0, 0,    # target system, target component
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
    0b0000111111000111, # type_mask (only speeds enabled)
    0, 0, 0, # x, y, z positions (not used)
    velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
    0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

  vehicle.send_mavlink(msg)

def send_gps_message(master, vehicle, location, velocity, gps0):
  
  global timestamp

  master.mav.gps_input_send(
    timestamp,
    0,
    8|16|32,
    0,
    0,
    3,
    int(float(location.lat) * 10**7),
    int(float(location.lon) * 10**7),
    int(location.alt),
    0.01,
    0.01,
    int(velocity[0]),
    int(velocity[1]),
    int(velocity[2]),
    0,
    0,
    0, 
    20 
  )

  timestamp += 200000

  print("Lat: ", location.lat, ", Lon: ", location.lon, ", Alt: ", location.alt)
  print("EPH: ", gps0.eph, ", EPV: ", gps0.epv, timestamp)

timestamp = 0

# Establish MAVLink Connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()

# Establish Vehicle Connection
vehicle = connect('127.0.0.1:14550', wait_ready=True)
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

while vehicle.home_location == None:
  time.sleep(0.2)
  send_gps_message(master, vehicle, LocationGlobalRelative(21.29792, -157.8171200, 0), vehicle.velocity, vehicle.gps_0)

while not vehicle.is_armable:
  time.sleep(0.2)
  send_gps_message(master, vehicle, LocationGlobalRelative(21.29792, -157.8171200, 0), vehicle.velocity, vehicle.gps_0)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

vehicle.simple_takeoff(100)

while True:
  time.sleep(0.2)

  current_location = vehicle.location.global_relative_frame
  current_velocity = vehicle.velocity
  current_gps0 = vehicle.gps_0

  send_gps_message(master, vehicle, current_location, current_velocity, current_gps0)
 
  if vehicle.location.global_relative_frame.alt >= 95:
    break

while True:
  time.sleep(0.2)

  current_location = vehicle.location.global_relative_frame
  current_velocity = vehicle.velocity
  current_gps0 = vehicle.gps_0

  send_gps_message(master, vehicle, current_location, current_velocity, current_gps0)
#  send_ned_velocity(-0.1, 0, 0)
