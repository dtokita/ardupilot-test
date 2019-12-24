from pymavlink import mavutil
import time
import json
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, GPSInfo, Command

def get_location_metres(original_location, dNorth, dEast):

    earth_radius=6378137.0 #Radius of "spherical" earth

    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)

    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)

    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)

    else:
        raise Exception("Invalid Location object passed")

    return targetlocation

def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon

    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(aLocation1, aLocation2):

    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795

    if bearing < 0:
        bearing += 360.00

    return bearing

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
    0.015,
    0.015,
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
  print("EPH: ", gps0.eph, ", EPV: ", gps0.epv, ", Vel: ", velocity)

timestamp = 0

# Establish MAVLink Connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()

# Establish Vehicle Connection
vehicle = connect('127.0.0.1:14550', wait_ready=True)
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

vehicle.parameters['FS_EKF_THRESH'] = 0

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

target_location = get_location_metres(vehicle.location.global_relative_frame, 1000, 0)
distance_to_target = get_distance_metres(vehicle.location.global_relative_frame, target_location)
speed = 10 
freq = 5 

print("Distance to target (m): ", distance_to_target)

inc_metres = float((speed)/ freq)

print("Increment Metres: ", inc_metres)

delta_location = get_location_metres(LocationGlobalRelative(0, 0, 0), inc_metres, 0)

print("Delta Location: ", delta_location)

vehicle.simple_goto(target_location, airspeed=speed)

while True:
  time.sleep(float(1/freq))

  current_location = vehicle.location.global_relative_frame

  spoof_location = LocationGlobalRelative(current_location.lat + delta_location.lat,
                                          current_location.lon + delta_location.lon,
                                          current_location.alt + delta_location.alt)

  current_velocity = vehicle.velocity

  current_gps0 = vehicle.gps_0

  send_gps_message(master, vehicle, spoof_location, current_velocity, current_gps0)

