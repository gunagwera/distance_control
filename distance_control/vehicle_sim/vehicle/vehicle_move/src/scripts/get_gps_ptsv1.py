#!/usr/bin/env python


'''
only road map configs:
coors: last car --> first: (1, 0) --> (7, 0) 6m apart initially
coors: (-30, 0) --> by 6 | can go to -40 for more space
'''

# get gps and store data then control distance according to gps data
import rospy
import roslaunch
import message_filters
import rospkg
import PID
import os
from math import sin, cos, sqrt, atan2, radians
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
import geopy.distance
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Initialize variables
pub1 = None
pub3 = None
pub2 = None
pub4 = None
v1_d = [[]]
v2_d = [[]]
v3_d = [[]]
v4_d = [[]]
location_collect = ""
frame_time = 0.0
collect_request = True
lat_point = 0.0
latitu1 = 0.0
longitu1 = 0.0
latitude2 = 0.0
lat1 = 0.0
lat2 = 0.0
lat3 = 0.0
lat4 = 0.0
long_point = 0.0
lng1 = 0.0
lng2 = 0.0
lng3 = 0.0
lng4 = 0.0
ivd = 0.0
P =     0.03785
I =     0.001
D =     0.000289
P23 =   0.0385
I23 =   0.001
D23 =   0.0001
P34 =   0.05
I34 =   0.00001
D34 =   0.0001
T = 0                 
LV_vel = 4            
lin_accel = 1         
delta_t = 1           
u_0 = 0               
scale = 10
dec_v = 5
v2 = 0.0
v3 = 0.0
v4 = 0.0
a1 = 0.0
a2 = 0.1
a3 = 0.1
a4 = 0.1
desired_distance = 12
L = 50

d12 = 0.0
d23 = 0.0
d34 = 0.0
gotdata = False

all_gps = None


def callback2(data):
    global latitude2
    global lng2
    latitude2 = data.latitude
    lng2 = data.longitude
    global v2_d
    frame_time = data.header.stamp.secs
    v2d = [frame_time, latitude2, lng2]
    v2_d.append(v2d)
    v2_gps = open("v2_gps.csv", "a")
    v2_gps.write(("%f,%f,%f\n") % (frame_time, latitude2, lng2))
    v2_gps.close()
    print("\nSaved v2_data: \n", v2d, "\n\n")


def callback3(data):
    global lat3
    global lng3
    lat3 = data.latitude
    lng3 = data.longitude
    global v3_d
    frame_time = data.header.stamp.secs
    v3d = [frame_time, lat3, lng3]
    v3_d.append(v3d)
    v3_gps = open("v3_gps.csv", "a")
    v3_gps.write(("%f,%f,%f\n")%(frame_time, lat3, lng3))
    v3_gps.close()
    print("\nSaved v3_data: \n", v3d, "\n\n")


def callback4(data):
    global lat4
    global lng4
    lat4 = data.latitude
    lng4 = data.longitude
    global v4_d
    frame_time = data.header.stamp.secs
    v4d = [frame_time, lat4, lng4]
    v4_d.append(v4d)
    v4_gps = open("v4_gps.csv", "a")
    v4_gps.write(("%f,%f,%f\n")%(frame_time, lat4, lng4))
    v4_gps.close()
    print("\nSaved v4_data: \n", v4d, "\n\n")


def subpub_init():
    rospy.Subscriber("/vehicle1/fix", NavSatFix, callback)
    rospy.Subscriber("/vehicle2/fix", NavSatFix, callback2)
    rospy.Subscriber("/vehicle3/fix", NavSatFix, callback3)
    rospy.Subscriber("/vehicle4/fix", NavSatFix, callback4)


def callback_s(data1, data2, data3, data4):
    global frame_time, time_last
    global lat1, lng1, lat2, lng2, lat3, lng3, lat4, lng4

    frame_time = data1.header.stamp.secs
    time_last = rospy.get_time()
    print("frame_time: ", frame_time, "\n")
    print("Ros time: ", time_last, "\n")
    #get sync data
    lat1 = data1.latitude
    lng1 = data1.longitude
    lat2 = data2.latitude
    lng2 = data2.longitude
    lat3 = data3.latitude
    lng3 = data3.longitude
    lat4 = data4.latitude
    lng4 = data4.longitude

    print("about to calculate \n")
    gotdata = True
    calculate_all(gotdata)
    print("should have calculated \n")


def sync_gps_sub():
    v1_data = message_filters.Subscriber("/vehicle1/fix", NavSatFix)
    v2_data = message_filters.Subscriber("/vehicle2/fix", NavSatFix)
    v3_data = message_filters.Subscriber("/vehicle3/fix", NavSatFix)
    v4_data = message_filters.Subscriber("/vehicle4/fix", NavSatFix)

    ts = message_filters.TimeSynchronizer([v1_data, v2_data, v3_data, v4_data], 1)
    ts.registerCallback(callback_s)


def run_sim():
    global d12, d23, d34, pub1, pub2, pub3, pub4, LV_vel, lin_accel, T, delta_t, u_0, P, I, D

    rospy.init_node('platoon_controller', anonymous=True)

    rate = rospy.Rate(100)  # 10Hz

    pub1 = rospy.Publisher('/vehicle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/vehicle2/cmd_vel', Twist, queue_size=10)
    pub3 = rospy.Publisher('/vehicle3/cmd_vel', Twist, queue_size=10)
    pub4 = rospy.Publisher('/vehicle4/cmd_vel', Twist, queue_size=10)

    sync_gps_sub()
    # subpub_init() 
    #  Set initial position to have vehicles wait for at least 2 seconds
    for i in range(1,600):
        print("waiting ... \n")
        rate.sleep()
    v = u_0 + lin_accel * delta_t
    while not rospy.is_shutdown():
        # calculate velocity, v according to Newton's law
        if v < (LV_vel-1.1):
            v = u_0 + lin_accel * delta_t
            print("Moving vehicle at velocity, v = ", v, "\n\n")
            T = T + 1
        else:
            print("Achieved target velocity, after T = ", T, "(s) \nv = ", v, "\n")
        move1(v)  # LV's velocity (m/s)
        v2_vel = pid_control2(d12, P, I, D)
        move2(v2_vel)
        v3_vel = pid_control2(d23, P23, I23, D23)
        move3(v3_vel)
        v4_vel = pid_control2(d34, P34, I34, D34)
        move4(v4_vel)
        u_0 = v
        rate.sleep()


def callback(data):
    global lat1, lng1
    lat1 = data.latitude
    lng1 = data.longitude
    lat_point = lat1
    long_point = lng1
    global v3_d
    v1d = [frame_time, lat1, lng1]
    v1_d.append(v1d)
    print("\nSaved v1_data: \n", v1d, "\n\n")


def calculate_all(gotdata=False):
    global d12
    global d23
    global d34
    global frame_time
    global lat1, lng1
    global lat2
    global lat3
    global lat4
    global lng2
    global lng3
    global lng4
    global ivd
    if (gotdata):
        print("calculating distances ... \n")
        d12 = get_distance(lat1, lng1, lat2, lng2) # debug latpoint and longpoint, lat2 and lng2
        d23 = get_distance(lat2, lng2, lat3, lng3)
        d34 = get_distance(lat3, lng3, lat4, lng4)
        write_all_to_gps_file(time_last, frame_time, lat1, lng1, lat2, lng2, lat3, lng3, lat4, lng4, d12, d23, d34)
    else:
        print("patience, no data yet!\n")


# enable different constants for different vehicles
def pid_control2(current_distance, pi, ii, di):
    global desired_distance, scale, dec_v, LV_vel
    pid = PID.PID(pi, ii, di)
    pid.SetPoint = desired_distance
    ref = current_distance
    pid.update(ref)
    output = pid.output
    derr = desired_distance - output
    serr = pid.SetPoint - output
    l_er = pid.last_error

    # vehicles should not directly reverse in formation
    if derr < 0:
        derr = LV_vel / dec_v
        print("derr: ", derr)
        return derr

    returned = derr / scale
    if returned >= LV_vel:
        returned = returned / 2
    print("returned: ", returned)
    return derr / scale


def accelerate(i_v, acc, d_t):
    f_v = i_v + acc * d_t
    return f_v


def getPaths():
    # looks up gps file logging location
    global location_collect
    rospack = rospkg.RosPack()
    location_collect = "gps_data.txt"


def get_distance(lt1, lg1, lt2, lg2):
    gps1 = (lt1, lg1)
    gps2 = (lt2, lg2)

    distance = geopy.distance.distance(gps1, gps2).meters  # get distance in meters
    print("distance.distance (m) : ", distance, "\n")

    return distance


def write_all_to_gps_file(ros_time, frame_time, lt1, ln1, lt2, ln2, lt3, ln3, lt4, ln4, d12, d23, d34):
    # open file 
    global all_gps
    print("Inserting into all gps ... \n\n")
    try:
        all_gps.write(("%f, %f, %f, %f,%f, %f, %f, %f,%f, %f, %f, %f,%f\n") % (ros_time, frame_time, lt1, ln1, lt2, ln2, lt3, ln3, lt4, ln4, d12, d23, d34))
    except Exception as e:
        print("Writing to file exception %s\n" % e)
        pass


def move1(velocity=0.5):
    global pub1
    velo_msg = Twist()
    velo_msg.linear.x = velocity
    velo_msg.linear.y = 0
    velo_msg.linear.z = 0
    velo_msg.angular.x = 0
    velo_msg.angular.y = 0
    velo_msg.angular.z = 0
    pub1.publish(velo_msg)


def move2(velocity=0.5):
    global pub2
    velo_msg = Twist()
    velo_msg.linear.x = velocity
    velo_msg.linear.y = 0
    velo_msg.linear.z = 0
    velo_msg.angular.x = 0
    velo_msg.angular.y = 0
    velo_msg.angular.z = 0
    pub2.publish(velo_msg)


def move3(velocity=0.5):
    global pub3
    velo_msg = Twist()
    velo_msg.linear.x = velocity
    velo_msg.linear.y = 0
    velo_msg.linear.z = 0
    velo_msg.angular.x = 0
    velo_msg.angular.y = 0
    velo_msg.angular.z = 0
    pub3.publish(velo_msg)


def move4(velocity=0.5):
    global pub4
    velo_msg = Twist()
    velo_msg.linear.x = velocity
    velo_msg.linear.y = 0
    velo_msg.linear.z = 0
    velo_msg.angular.x = 0
    velo_msg.angular.y = 0
    velo_msg.angular.z = 0
    pub4.publish(velo_msg)


def collect_gps():
    global gps_file, all_gps
    try:
        gps_file = open(location_collect, "a")
        all_gps = open("all_gps.csv", "w")
        rospy.loginfo(gps_file)
    except IOError:
        print "Could not open gps_points_file.txt file."


if __name__ == '__main__':
    try:
        getPaths()
        print('about to collect gps ... ')
        collect_gps()
        run_sim()

    except rospy.ROSInterruptException:
        gps_file.close()
        move1(0)
        all_gps.close()
        rospy.loginfo("closed file...")
        pass
