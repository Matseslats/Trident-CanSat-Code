#!/usr/bin/env python3

"""kalman.py: Description of what foobar does."""

# Import packages
import sys
import os
import time
from queue import Queue
from threading import Thread
import math
import numpy as np
import colorama
from colorama import Fore
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import microstacknode.hardware.gps.l80gps
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
import PID
from adafruit_servokit import ServoKit
from picamera import PiCamera
import subprocess
import shlex
import signal
import socket

__author__ = "Mats Haugerud"
__copyright__ = "Copyright 2022, CanSat Trident"
__credits__ = ["", ""]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Mats Haugerud"
__email__ = "cansattrident@cmail.com"
__status__ = "Initial testing"

# ********************** CONFIG **********************
allow_shutdown = True
print_data = False
use_gps = False
use_PID = False
enable_outputs = False # Allow for physical movement

# ********************** Resources: **********************
# GPS:
# stty -F /dev/serial0 sane
# cat /dev/serial0
# https://github.com/microstack-IoT/python3-microstacknode/blob/master/microstacknode/hardware/gps/l80gps.py
# https://python3-microstacknode.readthedocs.io/en/latest/installation.html
#
# https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm
# https://docs.novatel.com/OEM7/Content/Logs/GPVTG.htm
# https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
# https://docs.novatel.com/OEM7/Content/Logs/GPGSA.htm
# https://docs.novatel.com/OEM7/Content/Logs/GPGLL.htm
#
# https://cdn.sparkfun.com/assets/parts/1/2/2/8/0/PMTK_Packet_User_Manual.pdf
# https://no.mouser.com/datasheet/2/1052/QWSC_S_A0004134860_1-2576267.pdf
#
# IMU:
# https://pypi.org/project/mpu6050-raspberrypi/
#
# Kalman:
# https://github.com/sugbuv/EKF_IMU_GPS
# https://stackoverflow.com/questions/12578499/how-to-install-boost-on-ubuntu
# https://askubuntu.com/questions/860207/how-to-install-eigen-3-3-in-ubuntu-14-04
# https://ahrs.readthedocs.io/en/latest/filters/ekf.html
# https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py
# https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
#
# PWM expander:
# sudo pip3 install adafruit-circuitpython-servokit
# https://www.aranacorp.com/en/using-a-pca9685-module-with-raspberry-pi/

# ********************** Initialize socket for communication **********************
HOST = "10.0.0.2"      # Symbolic name meaning all available interfaces
PORT = 5000                     # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# ********************** Initialize IMU **********************
sensor = mpu6050(0x68)
ax = ay = az = 0
gx = gy = gz = 0

# ********************** Init Extended Kalman Filter **********************
pitch = roll = yaw = 0
last_update = time.time()
global attitude
ekf = EKF()
# For use with mag: (Example at Munich, Germany)
# ekf = EKF(gyr=gyr_data, acc=acc_data, mag=mag_data, magnetic_ref=[17.06, 0.78, 34.39])

# ********************** Initialize GPS **********************
gps = microstacknode.hardware.gps.l80gps.L80GPS()  # creates a GPS object
setup_gps_complete = False
year = month = day = hh = mm = ss = lat = long = vel = alt = sats = 0
kill_threads = False
global g_long, g_lat

# ********************** Camera setup **********************
camera = PiCamera()
camera.resolution = (3280, 2464)
camera.framerate = 10
time.sleep(2) # Warmup
# Low Quality Stream
vlcCommand="cvlc -vvv  stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst:8080}' :demux=h264"

# ********************** Initialize PIDs **********************
PID_frequency = 60

# Orientation PIDs
PID_pitch = PID.PID(P=0.2, I=0.0, D=0.0)    # Setup a new PID controller
PID_pitch.SetPoint = 0                      # Set target of PID controller
PID_pitch.setSampleTime(1/PID_frequency)    # Set how often PID should update uts output

PID_yaw = PID.PID(P=0.2, I=0.0, D=0.0)      # Setup a new PID controller
PID_yaw.SetPoint = 0                        # Set target of PID controller
PID_yaw.setSampleTime(1/PID_frequency)      # Set how often PID should update uts output

PID_roll = PID.PID(P=0.2, I=0.0, D=0.0)     # Setup a new PID controller
PID_roll.SetPoint = 0                       # Set target of PID controller
PID_roll.setSampleTime(1/PID_frequency)     # Set how often PID should update uts output

PID_thrust = PID.PID(P=0.2, I=0.0, D=0.0)   # Setup a new PID controller
PID_thrust.SetPoint = 0                     # Set target of PID controller
PID_thrust.setSampleTime(1/PID_frequency)   # Set how often PID should update uts output

# Translation PIDs
PID_GPS_pitch = PID.PID(P=0.2, I=0.0, D=0.0)    # Setup a new PID controller
PID_GPS_pitch.SetPoint = 0                      # Set target of PID controller
PID_GPS_pitch.setSampleTime(1/PID_frequency)    # Set how often PID should update uts output

PID_GPS_roll = PID.PID(P=0.2, I=0.0, D=0.0)     # Setup a new PID controller
PID_GPS_roll.SetPoint = 0                       # Set target of PID controller
PID_GPS_roll.setSampleTime(1/PID_frequency)     # Set how often PID should update uts output

# Output values
g_lat = g_long = g_alt = g_pitch = g_roll = g_yaw = 0

# ********************** Initialize PWM expander **********************
nbPCAServo=16 
#Parameters
pwm_params = {
    "MIN_IMP_ESC": 500, # Mimimum pulse width
    "MAX_IMP_ESC": 2500, # Maximum pulse width
    "MIN_ANG_ESC": 0,
    "MAX_ANG_ESC": 180,
    "M1_PIN": 1,
    "M2_PIN": 2,
    "M3_PIN": 3,

    "MIN_IMP_SERVO": 500,
    "MAX_IMP_SERVO": 2500,
    "MIN_ANG_SERVO": 0,
    "MAX_ANG_SERVO": 180,
    "SERVO_PIN": 0
}

# Init enviroment
temp = humid = pres = 1.11
#Objects
if enable_outputs:
    pca = ServoKit(channels=16)
    pca.servo[pwm_params["SERVO_PIN"]].set_pulse_width_range(pwm_params["MIN_IMP_SERVO"] , pwm_params["MAX_IMP_SERVO"])
    pca.servo[pwm_params["M1_PIN"]].set_pulse_width_range(pwm_params["MIN_IMP_ESC"] , pwm_params["MAX_IMP_ESC"])
    pca.servo[pwm_params["M2_PIN"]].set_pulse_width_range(pwm_params["MIN_IMP_ESC"] , pwm_params["MAX_IMP_ESC"])
    pca.servo[pwm_params["M3_PIN"]].set_pulse_width_range(pwm_params["MIN_IMP_ESC"] , pwm_params["MAX_IMP_ESC"])

def dd_to_dms(longitude, latitude):
    """Converts long and lat in decimal to long and lat in degrees, minutes and seconds"""
    # math.modf() splits whole number and decimal into tuple
    # eg 53.3478 becomes (0.3478, 53)
    split_degx = math.modf(longitude)
    
    # the whole number [index 1] is the degrees
    degrees_x = int(split_degx[1])

    # multiply the decimal part by 60: 0.3478 * 60 = 20.868
    # split the whole number part of the total as the minutes: 20
    # abs() absoulte value - no negative
    minutes_x = abs(int(math.modf(split_degx[0] * 60)[1]))

    # multiply the decimal part of the split above by 60 to get the seconds
    # 0.868 x 60 = 52.08, round excess decimal places to 2 places
    # abs() absoulte value - no negative
    seconds_x = abs(round(math.modf(split_degx[0] * 60)[0] * 60,2))

    # repeat for latitude
    split_degy = math.modf(latitude)
    degrees_y = int(split_degy[1])
    minutes_y = abs(int(math.modf(split_degy[0] * 60)[1]))
    seconds_y = abs(round(math.modf(split_degy[0] * 60)[0] * 60,2))

    # account for E/W & N/S
    if degrees_x < 0:
        EorW = "W"
    else:
        EorW = "E"

    if degrees_y < 0:
        NorS = "S"
    else:
        NorS = "N"

    latitude  = b'%d/1,%d/1,%d/1000' % (degrees_y, minutes_y, seconds_y*1000)
    longitude = b'%d/1,%d/1,%d/1000' % (degrees_x, minutes_x, seconds_x*1000)

    return latitude, longitude, NorS, EorW

def camera_loop(in_q):
    """
    Starts a low resoultuon video stream at http://ip:8080
    Repeatedly takes images while streaming the video.
    """
    global camera
    print("Starting camera")
    camera.vflip = True
    camera.hflip = True
    cvlc = subprocess.Popen(shlex.split(vlcCommand), stdin=subprocess.PIPE, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    camera.start_recording(cvlc.stdin, format="h264", splitter_port=2, resize=(320, 240))
    while True:
        if kill_threads:
            print("Killing Camera Loop")
            camera.stop_recording(splitter_port=2)
            # cvlc.kill() # Exit VLC
            os.killpg(os.getpgid(cvlc.pid), signal.SIGKILL) # Kill all subprocess in group cvlc is in
            cvlc.wait() # Wait for termination
            return False
        else:
            # Attach EXIF data
            camera.exif_tags['EXIF.UserComment'] = b'By CanSat Trident for the European CanSat competition'
            camera.exif_tags['GPS.GPSAltitude'] = str(1)
            lat, long, NorS, EorW = dd_to_dms(16.103847, 69.974401)
            camera.exif_tags['GPS.GPSLongitude'] = long
            camera.exif_tags['GPS.LongitudeRef'] = EorW
            camera.exif_tags['GPS.GPSLatitude'] = lat
            camera.exif_tags['GPS.LatitudeRef'] = NorS
            camera.exif_tags['IFD0.Copyright'] = 'Copyright (c) 2022 CanSat Trident'
            camera.exif_tags['GPS.GPSDestBearing'] = 74041/49085 # The bearing to the destination point (in degrees).
            camera.exif_tags['GPS.GPSDestBearingRef'] = 'T'
            camera.exif_tags['GPS.GPSImgDirection'] = 74041/49085 # The direction of the image when it was captured (in degrees).
            camera.exif_tags['GPS.GPSImgDirectionRef'] = 'T' # True North
            camera.exif_tags['GPS.GPSDateStamp'] = time.time() # Probably wrong format
            camera.capture('foo2.jpg', use_video_port=True)
            # print("Captured")

def distance_coords(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    """
    Takes two sets of coordinates and returns the distance between them in meters

    From: https://stackoverflow.com/a/28372037

    Takes:
    - lon1: Longitude of first coordinate
    - lat1: latitude of first coordinate
    - lon2: longitude of second coordinate
    - lat2: latitude of second coordinate

    Returns:
    - Distance between coordinates in meters
    """
    R = 6378.137                                # radius of earth in Km
    dLat = (lat2-lat1)*math.pi/180
    dLon = (lon2-lon1)*math.pi/180
    a = math.sin((dLat/2))^2 + math.cos(lat1*math.pi/180)*math.cos(lat2*math.pi/180)*(math.sin(dLon/2))^2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return (d * 1000)                            # distance in meters

def update_pwm(servo: int, m1: int, m2: int, m3: int) -> None:
    """Update PWM signals on expander"""
    # Is the cansat allowed to move
    if enable_outputs:
        pca.servo[pwm_params["SERVO_PIN"]].angle = servo
        pca.servo[pwm_params["M1_PIN"]].angle = m1
        pca.servo[pwm_params["M2_PIN"]].angle = m2
        pca.servo[pwm_params["M3_PIN"]].angle = m3

def pid_loop(in_q):
    """
    Combines estimated state with destination

    PID for:

    Inner loop:
    - Pitch (Forward/Backward, y)
    - Yaw (Rotation)
    - Roll (Left/Right, x)
    - Thrust (Altitude, z)
    
    Outer Loop:
    - Pitch_GPS (Forward/Backward, y, For GPS)
    - Roll_GPS (Left/Right, x, For GPS)

    https://cdn.discordapp.com/attachments/944924059504869386/969902867001511996/unknown.png
    """
    global pitch, yaw, roll, alt, lat, long, g_long, g_lat
    global PID_thrust, PID_pitch, PID_yaw, PID_roll, PID_GPS_pitch, PID_GPS_roll, PID_frequency
    min_pwm = 0
    max_pwm = 1000
    # ************* Loop evere 1/x sec *************
    last_update = time.time
    while True:
        if last_update + 1/PID_frequency >= time.time:
            last_update = time.time
            
            # ************* Update thrust and yaw PIDs *************
            PID_thrust.update(alt)          # Set new IS value
            out_thrust = PID_thrust.output  # Get the output of the PID controller

            PID_yaw.update(yaw)             # Set new IS value
            out_yaw = PID_yaw.output        # Get the output of the PID controller

            # ************* World to body conversion (Assumes sphere earth 👀) *************
            # Lat and long to meters !!ESTIMATE!!
            lat_m = 111320*lat
            g_lat_m = 111320*g_lat
            long_m = 40075e3*math.cos(long)/math.tau
            g_long_m = 40075e3*math.cos(g_long)/math.tau
            # Distance between sat and goal
            # dist = math.sqrt((g_long_m-long_m)**2 + (g_lat_m-lat_m)**2) # Faster, but only an estimate
            dist = distance_coords(long, lat, g_long, g_lat)
            # Delta angle between north and goal x,y pos
            # d_ang = math.atan((g_long_m-long_m)/(g_lat_m-lat_m)) # Faster, but only an estimate
            d_ang = math.atan((distance_coords(long, lat, g_long, lat))/(distance_coords(long, lat, long, g_lat)))
            # Angle between pointing ang goal x,y pos
            phi = d_ang-yaw
            # Delta x and y in body refrence frame
            delta_x = math.cos(dist/phi)
            delta_y = math.sin(dist/phi)

            # ************* Outer pitch and yaw loop: *************
            PID_GPS_pitch.update(delta_x)
            out_pitch_outer = PID_GPS_pitch.output # New goal for inner pitch. Is this really correct?
            out_pitch_outer = max(min((out_pitch_outer), 5),-5) # Sanitiz. Nessesairy?
            PID_GPS_roll.update(delta_y)
            out_roll_outer = PID_GPS_roll.output
            out_roll_outer = max(min((out_roll_outer), 5),-5) # Sanitize. Nessesairy?


            # ************* Inner pitch and yaw loop: *************
            PID_pitch.SetPoint =  out_pitch_outer # Set new goal
            PID_pitch.update(pitch) # Update PID
            out_pitch = PID_pitch.update # Get new output
            PID_roll.SetPoint = out_roll_outer
            PID_roll.update(roll)
            out_roll = PID_roll.output

            # ************* Update motors and servo + Motor Mixing Algorythm *************
            target_pwm_servo = max(min( int(out_yaw), max_pwm),min_pwm) # Sanitize output
            # Motor on servo, going clockwise with m2, m3
            target_pwm_m1 = max(min( int(out_thrust + out_pitch + out_roll/2), max_pwm),min_pwm)
            target_pwm_m2 = max(min( int(out_thrust - out_pitch/2 + out_roll/2), max_pwm),min_pwm)
            target_pwm_m3 = max(min( int(out_thrust - out_pitch/2 - out_roll), max_pwm),min_pwm)
            update_pwm(servo=target_pwm_servo, m1=target_pwm_m1, m2=target_pwm_m2, m3=target_pwm_m3)
            print(target_pwm_servo, target_pwm_m1, target_pwm_m2, target_pwm_m3)


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> list:
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    From: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/

    Takes:
    - Quaternion

    Returns:
    - Roll, Pitch, Yaw
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


def calc_position(in_q):
    """Get GPS and IMU (and mag) data and combine with kalman filter"""
    global pointing, ekf, pitch, roll, yaw, attitude, last_update
    while True: # Loop this until the program is stopped
        if use_gps:
            get_gps_data(0)
        get_imu_data(0)
        # Calculate time between last and this measurment
        d_t = time.time() - last_update
        attitude = ekf.update(attitude, np.array([gx, gy, gz]), np.array([ax, ay, az]), dt=d_t)
        last_update = last_update + d_t
        # Convert to euler angles, aka pitch yaw and roll, from quaterions
        roll, pitch, yaw = euler_from_quaternion(attitude[0], attitude[1], attitude[2], attitude[3])
        # print(Fore.CYAN + "Pitch: %1.16f Yaw: %1.16f Roll: %1.16f" % (pitch, yaw, roll))

        if kill_threads:
            print("Killing GPS")
            return False

def get_imu_data(in_q):
    """Update IMU"""
    global ax, ay, az, gx, gy, gz
    accelerometer_data = sensor.get_accel_data() # Get acc data
    gyroscope_data = sensor.get_gyro_data() # Get gyro data
    ax = accelerometer_data["x"]
    ay = accelerometer_data["y"]
    az = accelerometer_data["z"]
    gx = gyroscope_data["x"]*math.pi/180
    gy = gyroscope_data["y"]*math.pi/180
    gz = gyroscope_data["z"]*math.pi/180

def get_gps_data(in_q):
    """Gets PGS data"""
    global year, month, day, hh, mm, ss, lat, long, vel, alt, sats
    dict = gps.get_gpgga() # This packet contains data of interest
    lat = dict["latitude"]
    long = dict["longitude"]
    utc_time = dict["utc"]
    alt = float(dict["altitude"])
    sats = int(dict["number_of_sv"])

    hh = math.floor(utc_time/10000) % 100
    mm = math.floor(utc_time/100) % 100
    ss = math.floor(utc_time) % 100
    # print("Got GPS data")

def send_over_socket(in_q) -> bool:
    """Prints data to socket"""
    global HOST, PORT  # socket
    global lat, long, alt, pitch, yaw, roll, temp, humid, pres, g_lat, g_long, g_alt, g_pitch, g_roll, g_yaw, sats, vel
    # print("Setting timeout")
    # print("Set timeout")
    while True: # Auto reconnect
        print("Connecting")
        try:
            with socket.socket() as s:
                s.settimeout(5) # If function takes more secs than this, exit
                s.connect((HOST, PORT))
                print("Connected")
                while True:
                    if kill_threads: # Kill comms if something was connected
                        print("Killing Comms (Connected)")
                        s.close()
                        return False
                    print("Sending data")
                    send_string = b"%16d,%02.6f,%02.6f,%05.4f,%02d,%03.4f,%1.8f,%1.8f,%1.8f,%03.4f,%03.4f,%03.4f,%02.6f,%02.6f,%5.4f,%1.8f,%1.8f,%1.8f" % \
                        ((time.time_ns()), lat, long, alt, sats, vel,\
                            pitch, yaw, roll, \
                            temp, humid, pres, \
                            g_lat, g_long, g_alt, \
                            g_pitch, g_roll, g_yaw)

                    s.sendall(send_string)
                    # s.settimeout(0.5) # Listen for 0.5 sec
                    try:
                        data = s.recv(1024).decode()
                        print(f"Received: {data!r}")
                        if data == "DATA RECIEVED":
                            """Program read data and understood it"""
                            print("Data comm OK")
                    except:
                        print("Failed to recieve data")
                    time.sleep(1/10)
        except:
            print("Oops. Couldn't connect / Lost connection to %s:%d" % (HOST, PORT))
            time.sleep(1)
            if kill_threads: # Kill comms if needed
                print("Killing Comms (Not connected)")
                s.close()
                return False
        


def print_telemetry(in_q):
    """Prints data to terminal"""
    # print("Printing data")
    global year, month, day, hh, mm, ss, lat, long, vel, alt, sats, ax, ay, az, gx, gy, gz, pitch, yaw, roll
    if kill_threads:
        print("Killing Dataprinter")
        return False
    # os.system('cls' if os.name == 'nt' else 'clear')
    print("\033[7;1H", end="")
    print(Fore.GREEN + "Axyz: %+02.16f %+02.16f %+02.16f Gxyz: %+003.16f %+003.16f %+003.16f" % (ax, ay, az, gx*180/math.pi, gy*180/math.pi, gz*180/math.pi))
    print(Fore.CYAN + "Pitch: %+03.16f Yaw: %+03.16f Roll: %+03.16f        " % (pitch*180/math.pi, yaw*180/math.pi, roll*180/math.pi))
    print(Fore.LIGHTGREEN_EX + "%02d.%02d.%02d %02d:%02d:%02d - Lat: %02.6f Long %02.6f Alt %04.2f Vel %03.3f Sats %d         " % (year, month, day, hh, mm, ss, lat, long, alt, vel, sats))
    print(Fore.WHITE + 140*' ', end="")
    time.sleep(1/100)

def print_data_loop(in_q):
    """Prints data to terminal until threads are stopped"""
    while True:
        if kill_threads:
            print("Killing Dataprinter")
            return False
        else:
            print_telemetry(1)

def setup():
    """Setup sensors"""
    global setup_gps_complete, year, month, day, hh, mm, ss, lat, long, vel, alt, sats, ax, ay, ax, gx, gy, gz, ekf, attitude, g_long, g_lat
    # As long as no data can be processed, search for first GPS signal
    get_imu_data(0)
    g_long = long
    g_lat = lat
    attitude = acc2q(np.array([ax,ay,az]))
    while setup_gps_complete == False and use_gps:
        print(colorama.Style.BRIGHT, end="") # High contrast
        print(Fore.BLUE + "Getting setup data")
        dict = gps.get_gprmc()  # get latest GPRMC data as a dictionary
        valid = dict["data_valid"]
        print(Fore.BLUE + "Got data of type %s" % valid)
        if(valid == "A"): # Is data Automomous ?
            lat = dict["latitude"]
            long = dict["longitude"]
            date = (dict["date"])
            utc_time = dict["utc"]
            vel = float(dict["speed"]) * 1.9438444924574 # Knots to mps
            year = int(date) % 100
            month = math.floor(int(date)/100) % 100
            day = math.floor(float(date)/10000) % 100

            hh = math.floor(utc_time/10000) % 100
            mm = math.floor(utc_time/100) % 100
            ss = math.floor(utc_time) % 100
            print(Fore.WHITE + "FIRST DATA: %02d.%02d.%02d %02d:%02d:%02d - Lat: %02.6f Long %02.6f Vel %03.3f" % (year, month, day, hh, mm, ss, lat, long, vel))
            gps.send_nmea_pkt('$PMTK500,1000,0,0,0.0,0.0*1A\r\n') # Set update rate to 10 Hz, not working yet
            gps.send_nmea_pkt('$PMTK220,100*2C\r\n') # Set update rate to 10 Hz, not working yet
            print("Set update rate")
            
            setup_gps_complete = True
    print(Fore.WHITE + "Setup complete!")

def loop():
    """Main loop. Runs repeatedly. Currently everything is threaded so nothing happens here"""
    # print_telemetry(1)
    


def main():
    """Run setup and loop"""
    global kill_threads
    os.system('cls' if os.name == 'nt' else 'clear')
    setup()
    print("Starting threads")

    # Start multithreading
    q = Queue()
    wifi_coms = Thread(target = send_over_socket, args =(q, ))
    pos_gatherer = Thread(target = calc_position, args =(q, ))
    data_printer = Thread(target = print_data_loop, args =(q, ))
    pid_controls = Thread(target = pid_loop, args =(q, ))
    picture_taker = Thread(target = camera_loop, args =(q, ))

    wifi_coms.start()
    pos_gatherer.start()
    picture_taker.start()

    if print_data:
        data_printer.start()

    if use_PID:
        pid_controls.start()

    def quit_program():
        global kill_threads
        kill_threads = True
        while wifi_coms.is_alive():
            """Wait for threads to finish"""
        print("Comms are closed")
        while pos_gatherer.is_alive():
            """Wait for threads to finish"""
        print("Pos gatherer is closed")
        while picture_taker.is_alive():
            """Wait for threads to finish"""
        print("Image taker is closed")
        if print_data:
            while data_printer.is_alive():
                """Wait for data printer to stop"""
        print("Dataprinter is closed")
        if use_PID:
            while pid_controls.is_alive():
                """Wait for PID to stop"""
        print("PID is closed")

        print("All threads are closed, exiting")
        GPIO.cleanup() # Cleanup on GPIO, good practice
        sys.exit(0) # Exit program
    while True:
        try:
            time.sleep(100)
        except KeyboardInterrupt:
            print("Keyboard interrupt, exiting!")
            if allow_shutdown:
                print(Fore.WHITE)
                print(15*"\n") # Don't override error messages
                print("Keyboard interrupt, exiting!")
                quit_program()
        except Exception as e:
            print(Fore.RED)
            print(15*"\n")
            print('SLEEP Caught exception %s' % e)
            print(Fore.WHITE)
            if allow_shutdown:
                kill_threads = True
                quit_program()
        except:
            print(Fore.RED)
            print(15*"\n")
            print('wat? fix')
            print(Fore.WHITE)
            if allow_shutdown:
                kill_threads = True
                quit_program()


if __name__ == '__main__': # If file is run directly, start main()
    main()
