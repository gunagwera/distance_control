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
P =     0.05
I =     0.001
D =     0.0001
P23 =   0.05
I23 =   0.001
D23 =   0.0001
P34 =   0.06
I34 =   0.00001
D34 =   0.0001

T = 0                       # time_track
LV_vel = 2 # 4                 #  1, 2, 3--> ok | 5, 10, 15 --> next try | 1, 2, 4, 10
lin_accel = 1             #  accelerate at 0.01m/s/s
delta_t = 1              #  time step
u_0 = 0                     #  initial velocity
scale = 10
dec_v = 4
v2 = 0.0
v3 = 0.0
v4 = 0.0
a1 = 0.0
a2 = 0.1
a3 = 0.1
a4 = 0.1

vehicle_length = rospy.get_param("/vehicle1/vehicle_length")
desired_distance = 12   
calculation_distance = desired_distance  ## desired_distance + vehicle_length
L = 50

v1 = 0.0
v1_o = 0.0
v2_o = 0.0
v3_o = 0.0
v4_o = 0.0

# v4 -p:0.04, i:0.0001, d = 0.00001
# euclidean IVDs - params
d1 = 0.0
d2 = 0.0
d3 = 0.0
x1 = 0.0
y1 = 0.0
x2 = 0.0
y2 = 0.0
x3 = 0.0
y3 = 0.0
x4 = 0.0
y4 = 0.0

fused_data_file = 0.0
fused_frame_time = 0.0

#  gps alone params
d12 = 0.0
d23 = 0.0
d34 = 0.0
gotdata = False
got_fused = False

R = 6373000.0
all_gps = None
fused_data_file = None
lat_last = 0.0
long_last = 0.0
time_current = 0.0
time_last = 0.0


def callbackv1(data):
    global v1_o
    v1_o = data.vector.x
    print("v1_o set to: ", v1_o)


def callbackv2(data):
    global v2_o
    v2_o = data.vector.x
    print("v2_o set to: ", v2_o)


def callbackv3(data):
    global v3_o
    v3_o = data.vector.x
    print("v3_o set to: ", v3_o)


def callbackv4(data):
    global v4_o
    v4_o = data.vector.x
    print("v4_o set to: ", v4_o)


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


def callback_fused(data1, data2, data3, data4):
    global x1, y1, x2, y2, x3, y3, x4, y4, fused_frame_time, fused_cur_time
    global v1, v2, v3, v4, v1_o, v2_o, v3_o, v4_o

    fused_frame_time = data1.header.stamp.secs
    fused_cur_time = rospy.get_time()

    v1 = data1.twist.twist.linear.x
    x1 = data1.pose.pose.position.x
    y1 = data1.pose.pose.position.y

    v2 = data2.twist.twist.linear.x
    x2 = data2.pose.pose.position.x
    y2 = data2.pose.pose.position.y

    v3 = data3.twist.twist.linear.x
    x3 = data3.pose.pose.position.x
    y3 = data3.pose.pose.position.y

    v4 = data4.twist.twist.linear.x
    x4 = data4.pose.pose.position.x
    y4 = data4.pose.pose.position.y

    print("got fused data, going maths ...")
    got_fused = True
    calculate_fused(got_fused)
    print("should have calculated fused \n")


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


def gps_to_odom(gps_data):
    msg.header.stamp = gps.header.stamp         #gps_time                   // time of gps measurement
    msg.header.frame_id = 'base_footprint'      # base_footprint          // the tracked robot frame
    msg.pose.pose.position.x = gps_data.latitude    #  gpsg_x              // x measurement GPS.
    msg.pose.pose.position.y = gps_data.longitude   #  gps_y              // y measurement GPS.
    msg.pose.pose.position.z = gps_data.altitude    #  gps_z              // z measurement GPS.
    msg.pose.pose.orientation.x = 1                 # // identity quaternion
    msg.pose.pose.orientation.y = 0                 # // identity quaternion
    msg.pose.pose.orientation.z = 0                 # // identity quaternion
    msg.pose.pose.orientation.w = 0                 # // identity quaternion
    msg.pose.covariance = {cov_x, 0, 0, 0, 0, 0,    # // covariance on gps_x
                            0, cov_y, 0, 0, 0, 0,   # // covariance on gps_y
                            0, 0, cov_z, 0, 0, 0,   # // covariance on gps_z
                            0, 0, 0, 99999, 0, 0,   # // large covariance on rot x
                            0, 0, 0, 0, 99999, 0,   # // large covariance on rot y
                            0, 0, 0, 0, 0, 99999}   # // large covariance on rot z

    return msg   ## Return the message 


def sync_fused_sub():
    v1_data = message_filters.Subscriber("/vehicle1/odometry/filtered", Odometry)
    v2_data = message_filters.Subscriber("/vehicle2/odometry/filtered", Odometry)
    v3_data = message_filters.Subscriber("/vehicle3/odometry/filtered", Odometry)
    v4_data = message_filters.Subscriber("/vehicle4/odometry/filtered", Odometry)

    ts = message_filters.TimeSynchronizer([v1_data, v2_data, v3_data, v4_data], 1)
    ts.registerCallback(callback_fused)

         # get velocity
    rospy.Subscriber("/vehicle1/fix_velocity", Vector3Stamped, callbackv1)
    rospy.Subscriber("/vehicle2/fix_velocity", Vector3Stamped, callbackv2)
    rospy.Subscriber("/vehicle3/fix_velocity", Vector3Stamped, callbackv3)
    rospy.Subscriber("/vehicle4/fix_velocity", Vector3Stamped, callbackv4)


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

    rate = rospy.Rate(100)  # 

    pub1 = rospy.Publisher('/vehicle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/vehicle2/cmd_vel', Twist, queue_size=10)
    pub3 = rospy.Publisher('/vehicle3/cmd_vel', Twist, queue_size=10)
    pub4 = rospy.Publisher('/vehicle4/cmd_vel', Twist, queue_size=10)

    
    #  init fusion callback
    sync_fused_sub()

    sync_gps_sub()
    # subpub_init() 
    #  Set initial position to have vehicles wait for at least 2 seconds
    for i in range(1,600):
        print("waiting ... \n")
        rate.sleep()
    v = u_0 + lin_accel * delta_t
    time_keeper = 0
    acc_dec = True
    dec = False
    while not rospy.is_shutdown():
        # calculate velocity, v according to Newton's law
        if acc_dec:
            if v < (LV_vel - 0.001):
                v = u_0 + lin_accel * delta_t
                print("Moving vehicle at velocity, v = ", v, "\n\n")
                T = T + 1
            else:
                print("Achieved target velocity, after T = ", T, "(s) \nv = ", v, "\n")
                acc_dec = False
        
        ##  Begin acc - dec check
        # check if target velocity reached:
        if v >= (LV_vel - 0.001):
            # move with constant velocity for about 10 seconds
            if time_keeper < 6000: 
                time_keeper = time_keeper + 1
            else:
                dec = True
        # we can start decelerating
        if dec:
            # now we should start decelerating
            if v > 0:
                v = decelerate(v, 0.1, 0.1)
                print('decelerating ...')
            # make sure car does not reverse
            if v < 0:
                v = 0

        ## end acc - dec check
        
        print('time keeper: ', time_keeper)
        print('v : ', v)

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
    # frame_time = data.header.stamp.secs
    v1d = [frame_time, lat1, lng1]
    v1_d.append(v1d)
    v1_gps = open("v1_gps.csv", "a")
    v1_gps.write(("%f,%f,%f\n")%(frame_time, latitu1, longitu1))
    v1_gps.close()
    print("\nSaved v1_data: \n", v1d, "\n\n")
 

def calculate_fused(fused=False):
    global d1, d2, d3, got_fused, x1, y1, x2, y2, x3, y3, x4, y4, fused_frame_time, fused_frame_time
    global v1, v2, v3, v4, v1_o, v2_o, v3_o, v4_o

    if (fused):
        d1 = euclidean_distance(x1, y1, x2, y2)
        d2 = euclidean_distance(x2, y2, x3, y3)
        d3 = euclidean_distance(x3, y3, x4, y4)

        print(("{(x1: %f, y1: %f) & (x2 : %f, y2: %f)} --> d1 = %f\n")%(x1, y1, x2, y2, d1))
        print(("{(x2: %f, y2: %f) & (x3 : %f, y3: %f)} --> d2 = %f\n")%(x2, y2, x3, y3, d2))
        print(("{(x3: %f, y3: %f) & (x4 : %f, y4: %f)} --> d3 = %f\n")%(x3, y3, x4, y4, d3))
        print(("v1_o: %f, v2_o: %f, v3_o: %f, v4_o: %f\n")%(v1_o, v2_o, v3_o, v4_o))
        write_fused_to_file(fused_cur_time, fused_frame_time, x1, y1, x2, y2, x3, y3, x4, y4, d1,
        d2, d3, v1, v2, v3, v4, v1_o, v2_o, v3_o, v4_o)

    else:
        print("relax, no data to work with yet")


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
    if (gotdata):
        print("calculating distances ... \n")
        d12 = get_distance(lat1, lng1, lat2, lng2) # debug latpoint and longpoint, lat2 and lng2
        d23 = get_distance(lat2, lng2, lat3, lng3)
        d34 = get_distance(lat3, lng3, lat4, lng4)
        #print(("{(lt1: %f, lg1: %f) & (lt2 : %f, lg2: %f)} --> d12 = %f\n")%(lat_point, long_point, latitude2, lng2, d12))
        print(("{(lt1: %f, lg1: %f) & (lt2 : %f, lg2: %f)} --> d12 = %f\n")%(lat1, lng1, lat2, lng2, d12))
        print(("{(lt2: %f, lg2: %f) & (lt3 : %f, lg3: %f)} --> d23 = %f\n")%(lat2, lng2, lat3, lng3, d23))
        print(("{(lt3: %f, lg3: %f) & (lt4 : %f, lg4: %f)} --> d34 = %f\n")%(lat3, lng3, lat4, lng4, d34))
        #print(("Done with distances, got d12 = %f, d23 = %f, d34 = %f\nNow Logging to file ... \n\n")%(d12, d23, d34))
        write_all_to_gps_file(time_last, frame_time, lat1, lng1, lat2, lng2, lat3, lng3, lat4, lng4, d12, d23, d34)
    else:
        print("patience, no data yet!\n")


# enable different constants for different vehicles
# rec 4 and 10, 1, 2 fine
def pid_control2(current_distance, pi, ii, di):
    global calculation_distance, scale, dec_v, LV_vel, u_0
    pid = PID.PID(pi, ii, di)
    pid.SetPoint = calculation_distance
    ref = current_distance
    pid.update(ref)
    output = pid.output
    derr = calculation_distance - output
    serr = pid.SetPoint - output
    l_er = pid.last_error
    print("current distance : ", current_distance, "\n")
    print("self error: ", pid.last_error, "\n")
    print("pid output: ", output, "\n")
    print("setpoint: ", pid.SetPoint, "\n")
    print("desired - ouput: ", derr, "\n")
    print("setpoint - output: ", serr, "\n")
    print("calculation distance: ", calculation_distance, "\n")

    if derr < 0:
        derr = LV_vel / dec_v
        print("derr: ", derr)
        return derr

    returned = derr / scale
    if returned >= LV_vel:
        returned = returned / 2
    print("returned: ", returned)
    return derr / scale


def euclidean_distance(x1, y1, x2, y2):
    euc_dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    print("Dist: ", euc_dist)
    return euc_dist


def accelerate(i_v, acc, d_t):
    f_v = i_v + acc * d_t
    return f_v


def decelerate(i_V, dec, d_t):
    f_v = i_V - dec * d_t
    return f_v


def collection_status_CB(collection_status_msg):
    global collect_request
    print('data: \n', collection_status_msg.bools)
    collect_request = collection_status_msg.bools


def getPaths():
    # looks up gps file logging location
    global location_collect
    rospack = rospkg.RosPack()
    location_collect = "gps_data.txt"


def get_distanceF(lt, lg, lt2, lg2):
    global R
    lat1 = radians(lt)
    lon1 = radians(lg)
    lat2 = radians(lt2)
    lon2 = radians(lg2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # Distance in meters
    distance = R * c * 1000
    return distance


def get_distance(lt1, lg1, lt2, lg2):

    gps1 = (lt1, lg1)
    gps2 = (lt2, lg2)

    lt1 = radians(lt1)
    lg1 = radians(lg1)
    lt2 = radians(lt2)
    lg2 = radians(lg2)

    dlng = lg2 - lg1
    dlat = lt2 - lt1

    a = sin(dlat / 2)**2 + cos(lt1) * cos(lt2) * sin(dlng / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance1 = R * c  
    print("heaverside distance(m): ", distance1, '\n') #  for comparison purposes
    distance = geopy.distance.distance(gps1, gps2).meters
    print("distance.distance (m) : ", distance, "\n")
  
    return distance


def write_all_to_gps_file(ros_time, frame_time, lt1, ln1, lt2, ln2, lt3, ln3, lt4, ln4, d12, d23, d34):
    # open file 
    global all_gps
    print("Inserting into all gps ... \n\n")
    try:
        print("wrting ... \n\n")
        #all_gps = open("all_gps.csv", "a")
        all_gps.write(("%f, %f, %f, %f,%f, %f, %f, %f,%f, %f, %f, %f,%f\n") % (ros_time, frame_time, lt1, ln1, lt2, ln2, lt3, ln3, lt4, ln4, d12, d23, d34))
        #all_gps.close()
        print("wrote to all gps \n\n")
    except Exception as e:
        print("Writing to file exception %s\n" % e)


def write_fused_to_file(rt, ft, lt1, ln1, lt2, ln2, lt3, ln3, lt4, ln4, d12,
    d23, d34, v1, v2, v3, v4, v1_o, v2_o, v3_o, v4_o):
    # open file 
    global fused_data_file
    print("Inserting into fused data file ... \n\n")
    try:
        print("wrting ... \n\n")
        fused_data_file.write(("%f, %f, %f, %f,%f, %f, %f, %f,%f, %f, %f, \
        %f,%f, %f, %f, %f, %f, %f,%f,%f,%f\n") % (rt, ft, lt1, ln1, \
         lt2, ln2, lt3, ln3, lt4, ln4, 
        d12, d23, d34, v1, v2, v3, v4, v1_o, v2_o, v3_o, v4_o))
        print("wrote to fused_data_file \n\n")
    except Exception as e:
        print("Writing to file exception %s\n" % e)


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
    global gps_file, all_gps, fused_data_file
    try:
        gps_file = open(location_collect, "a")
        all_gps = open("all_gps.csv", "w")
        fused_data_file = open("fused_data.csv", "w")
        rospy.loginfo(gps_file)
    except IOError:
        print "Could not open one of the files."


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
        fused_data_file.close()
        rospy.loginfo("closed files ...")
        pass
