from dronekit import connect
from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
from pymavlink import mavutil
import math
import time
import cv2
import cv2.aruco as aruco
import imutils
import numpy as np

vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
print("Connected...")
#servo
n=9
drop=900
hold=1900
#calib
cameraMatrix   = np.loadtxt('cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt('cameraDistortion.txt', delimiter=',')

#cam
horizontal_res = 640
vertical_res = 480
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) #640
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
horizontal_fov = 62.2 * (math.pi / 180 )
vertical_fov = 48.8 * (math.pi / 180)
#aruco
id_to_find = 72
marker_size = 25 #cm
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()
#dest
dest_lat=float(input("Enter dest lat: "))
dest_lon=float(input("Enter dest lon: "))
takeoff_height = 8
velocity = .5
time_to_sleep=10
z=100
def send_ned_velocity(velocity_x=0, velocity_y=0, velocity_z=-0.7, duration=15):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        if vehicle.location.global_relative_frame.alt>=.95*takeoff_height:
            print("altitude reached")
            break
        time.sleep(1)
def arm_and_takeoff(targetHeight):
    while vehicle.is_armable!=True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")
    
    vehicle.mode = VehicleMode("GUIDED")
            
    while vehicle.mode!='GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")
    vehicle.armed = True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Started...")
            
    vehicle.simple_takeoff(targetHeight)

    while True:
        print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached")

    return None

def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

    vehicle.simple_goto(targetLocation)

    while vehicle.mode.name=="GUIDED":
        currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
        print("current distance: ",currentDistance)
        if currentDistance<1.5:
            print("Reached target waypoint.")
            time.sleep(2)
            break
        time.sleep(1)
    return None

def get_distance_meters(targetLocation,currentLocation):
    dLat=targetLocation.lat - currentLocation.lat
    dLon=targetLocation.lon - currentLocation.lon

    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def send_distance_message( dist):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        10000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = laser?
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)
def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav = x_cam
    y_uav = y_cam
    return(x_uav, y_uav)
def send_land_message_v2(x_rad=0, y_rad=0, dist_m=0, x_m=0,y_m=0,z_m=0, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_FRD, # frame
        x_rad,          # X-axis angular offset, in radians
        y_rad,          # Y-axis angular offset, in radians
        dist_m,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
        x_m,          # x	float	X Position of the landing target on MAV_FRAME
        y_m,          # y	float	Y Position of the landing target on MAV_FRAME
        z_m,          # z	float	Z Position of the landing target on MAV_FRAME
        (1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        2,          # type of landing target: 2 = Fiducial marker
        1,          # position_valid boolean
    )
    vehicle.send_mavlink(msg)
def Servo(n,pwm):
    msg = vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            n,
            pwm,
            0,
            0,
            0,
            0,
            0)
    vehicle.send_mavlink(msg)  
             

#main
vehicle.parameters['LAND_SPEED'] = 30

lat_home=vehicle.location.global_relative_frame.lat
lon_home=vehicle.location.global_relative_frame.lon

wp_home=LocationGlobalRelative(lat_home,lon_home,takeoff_height)
wp_dest=LocationGlobalRelative(dest_lat,dest_lon,takeoff_height)

distancetodest=get_distance_meters(wp_dest,wp_home)
print("Distance to destination is: ",distancetodest)

arm_and_takeoff(takeoff_height)
goto(wp_dest)
#precland
while True:
    ret, frame = cap.read()
    frame = imutils.resize(frame, width=horizontal_res)
    gray_img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
    zin=int((0 if vehicle.location.global_relative_frame.alt<0 else vehicle.location.global_relative_frame.alt)*100.0)
    send_distance_message(zin)
    try:
        if ids is not None and ids[0] == id_to_find:
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            x = tvec[0]
            y = tvec[1]
            z = tvec[2]
            x_cm, y_cm = camera_to_uav(x, y)
            angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z)
            if vehicle.mode!='LAND':
                vehicle.mode = VehicleMode('LAND')
                while vehicle.mode!='LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message_v2(x_m=x_cm*0.01,y_m=y_cm*0.01, dist_m=z*0.01,z_m=z*0.01)
            else:
                send_land_message_v2(x_m=x_cm*0.01,y_m=y_cm*0.01, dist_m=z*0.01,z_m=z*0.01)
                #send_land_message(-1*y_ang,x_ang)
                pass
            print ("Marker found x = %5.0f cm  y = %5.0f cm "%(x_cm, y_cm))
            #send_land_message_v2(x_m=x_cm*0.01,y_m=y_cm*0.01, dist_m=z*0.01,z_m=z*0.01)
            print("")
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
    if zin<100:
        break
vehicle.mode = VehicleMode("GUIDED")
time.sleep(1)
Servo(n,drop)
print("*********************")
print("Arrived at destination")
print("*********************")
time.sleep(3)
Servo(n,hold)
#back
send_ned_velocity()
goto(wp_home)
while vehicle.armed==True:
    ret, frame = cap.read()
    frame = imutils.resize(frame, width=horizontal_res)
    gray_img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
    zin=int((0 if vehicle.location.global_relative_frame.alt<0 else vehicle.location.global_relative_frame.alt)*100.0)
    send_distance_message(zin)
    try:
        if ids is not None and ids[0] == id_to_find:
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            x = tvec[0]
            y = tvec[1]
            z = tvec[2]
            x_cm, y_cm = camera_to_uav(x, y)
            angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z)
            if vehicle.mode!='LAND':
                vehicle.mode = VehicleMode('LAND')
                while vehicle.mode!='LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message_v2(x_m=x_cm*0.01,y_m=y_cm*0.01, dist_m=z*0.01,z_m=z*0.01)
            else:
                send_land_message_v2(x_m=x_cm*0.01,y_m=y_cm*0.01, dist_m=z*0.01,z_m=z*0.01)
                #send_land_message(-1*y_ang,x_ang)
                pass
            print ("Marker found x = %5.0f cm  y = %5.0f cm "%(x_cm, y_cm))
            #send_land_message_v2(x_m=x_cm*0.01,y_m=y_cm*0.01, dist_m=z*0.01,z_m=z*0.01)
            print("")
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
print("*********************")
print("Arrived at starting point")
print("Ready for another delivery")
print("*********************")
cap.release()



'''@naveen"'