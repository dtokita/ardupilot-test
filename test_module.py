from pymavlink import mavutil
import time
import json
from dronekit import connect, VehicleMode, LocationGlobalRelative, GPSInfo

def send_gps_message(master, vehicle, location, velocity, gps0):
#  print("Global Location: %s" % vehicle.location.global_relative_frame)
#  print("Local Location: %s" % vehicle.location.local_frame) 
#  print("GPS: %s" % vehicle.gps_0)

  alt = int(location.alt)
  lat = int(float(location.lat) * 10**7)
  lon = int(float(location.lon) * 10**7)

  master.mav.gps_input_send(
    0,		# Timestamp (micros since boot or UNIX epoch)
    14,		# GPS ID
    8|16|32,	# Flags for ignore fields
    0,		# GPS time (millis from start of GPS week)
    0,		# GPS week number
    3,		# 0-1: no fix, 2: 2D Fix, 3: 3D Fix, 4: 3D with DiffGPS, 5: 3D with RTK
    lat,		# Latitude (WGS84), in degrees * 1E7
    lon,		# Longitude (WGS84), in degrees * 1E7
    alt,	  	# Altitude (AMSL, not WGS84), in m (positive for up)
    1,		# GPS HDOP horizontal dilution of position in m
    1,		# GPS VDOP vertical dilution of position in m
    int(velocity[0]),		# GPS velocity in m/s in NORTH direction in earth-fixed NED frame
    int(velocity[1]),		# GPS velocity in m/s in EAST direction in earth-fixed NED frame
    int(velocity[2]),		# GPS velocity in m/s in DOWN direction in earth-fixed NED frame
    0,		# GPS speed accuracy in m/s
    0,		# GPS horizontal accuracy in m
    0,		# GPS vertical accuracy in m
    7		# Number of satilites visible
  )

  print(lat, lon, alt, velocity, gps0.fix_type, gps0.eph, gps0.epv, gps0.satellites_visible)

  return 0

# Establish connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()

# Establish drone controls
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# UH Manoa coordinates
# 1 degree == 10000000 == 111132 m
# Therefore 100 == 1.1132 m
# Therefore 90 == 1 m
uh_manoa = LocationGlobalRelative(21.29792, -157.8171200, 0)

armed = False
moving = False

# Send GPS position
while True:
  time.sleep(0.05)

  current_location = vehicle.location.global_relative_frame
  velocity = vehicle.velocity
  gps0 = vehicle.gps_0

  # Slowly change the position (this can obviously be changed to our algorithm later)
  if armed:

    if not moving:
      vehicle.simple_goto(LocationGlobalRelative(21.2, -157.81712, 0))
      moving = True

    send_gps_message(master, vehicle, current_location, velocity, gps0)
    continue

  send_gps_message(master, vehicle, uh_manoa, velocity, gps0)
  
  if vehicle.gps_0.fix_type > 2:
  
    while not vehicle.is_armable:
      send_gps_message(master, vehicle, uh_manoa, velocity, gps0)
      time.sleep(0.1)

    if not armed:
      print("ARMING")
      vehicle.mode = VehicleMode("GUIDED")
      vehicle.armed = True
      armed = True

      vehicle.simple_takeoff(100)

      while vehicle.location.global_relative_frame.alt <= 95:
        send_gps_message(master, vehicle, uh_manoa, velocity, gps0)
        time.sleep(0.1)

