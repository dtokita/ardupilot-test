from pymavlink import mavutil
import time
import json
from dronekit import connect, VehicleMode, LocationGlobalRelative

def send_gps_message(master, vehicle, latitude, longitude):
#  print("Global Location: %s" % vehicle.location.global_relative_frame)
#  print("Local Location: %s" % vehicle.location.local_frame) 
#  print("GPS: %s" % vehicle.gps_0)

  master.mav.gps_input_send(
    0,		# Timestamp (micros since boot or UNIX epoch)
    0,		# GPS ID
    8|16|32,	# Flags for ignore fields
    0,		# GPS time (millis from start of GPS week)
    0,		# GPS week number
    3,		# 0-1: no fix, 2: 2D Fix, 3: 3D Fix, 4: 3D with DiffGPS, 5: 3D with RTK
    latitude,		# Latitude (WGS84), in degrees * 1E7
    longitude,		# Longitude (WGS84), in degrees * 1E7
    0,	  	# Altitude (AMSL, not WGS84), in m (positive for up)
    1,		# GPS HDOP horizontal dilution of position in m
    1,		# GPS VDOP vertical dilution of position in m
    0,		# GPS velocity in m/s in NORTH direction in earth-fixed NED frame
    0,		# GPS velocity in m/s in EAST direction in earth-fixed NED frame
    0,		# GPS velocity in m/s in DOWN direction in earth-fixed NED frame
    0,		# GPS speed accuracy in m/s
    0,		# GPS horizontal accuracy in m
    0,		# GPS vertical accuracy in m
    7		# Number of satilites visible
  )

  return 0

# Establish connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()

# Establish drone controls
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# UH Manoa coordinates
latitude = 212979200
longitude = -1578171200

armed = False

# Send GPS position
while True:
  time.sleep(0.1)

  # Slowly change the position (this can obviously be changed to our algorithm later)
  if armed:
    latitude += 10
    longitude += 10

  send_gps_message(master, vehicle, latitude, longitude)

  if vehicle.gps_0.fix_type > 2:
  
    while not vehicle.is_armable:
      send_gps_message(master, vehicle, latitude, longitude)
      time.sleep(0.1)

    if not armed:
      print("ARMING")
      vehicle.mode = VehicleMode("GUIDED")
      vehicle.armed = True
      armed = True

      vehicle.simple_takeoff(100)

      while vehicle.location.global_relative_frame.alt <= 95:
        send_gps_message(master, vehicle, latitude, longitude)
        time.sleep(0.1)

