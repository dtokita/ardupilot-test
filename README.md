# Setup Instructions

Follow Ross' set up guide https://sites.google.com/a/hawaii.edu/uh-uas-projects/research/2019-spring-x96-projects/sdr-attacks-on-sensors/project-notes/rosss-notes/sitl-and-mavproxy 

To confirm that setup was proper run the SITL simulator, cd into /ardupilot/ArduCopter directory and run "sim\_vehicle.py -w --console --map". This should launch MAVProxy (white window), a console (black window), and a map with the drone.

You can then run "pip3 install -r requirements.txt" on the requirements.txt provided in this repo, this will install the python packages necessary to run the script.

Add the line "GPS_TYPE 14.000" to the end of the file /ardupilot/Tools/autotest/default_params/copter.parm. This is necessary to do before start up because the GPS type is initialized and read only at start of SITL (cannot change once simulation is started)

# Running the Script

Close the SITL simulator. Have two terminals open, one to the SITL (in the /ardupilot/ArduCopter directory) and the other to contents of this repo. First run "python3 test\_module.py". The switch to the SITL simulation terminal and run "sim\_vehicle.py -w --console --map".

Let the SITL build and launch. The test\_module script will connect to the SITL simulation and send the GPS coordinates of UH Manoa to the simulation, you map should show the drone at UH Manoa. After the SITL initializes it's sensors, it will attempt to arm the drone (this can take upwards of 10 to 15 seconds). 

Once armed, you can watch the map and the drone will slowly move up and to the right. Note: we have not yet set a flight path, therefore we know that the GPS reading is getting placed into the SITL correctly.

Try to create flight paths by right clicking on the map and selecting "Fly To", you will see the SITL trying to correct the drone despite the drone continuing up and to the right.

We need to implement the correct algorithm instead of the static movement up and to the right
