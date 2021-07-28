#!/usr/bin/env python
'''This node publishes and subsribes the following topics:
        PUBLICATIONS				SUBSCRIPTIONS
	/edrone/drone_command			/edrone/gps'''


from geometry_msgs.msg import Vector3Stamped 
from sensor_msgs.msg import Imu, LaserScan
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32 ,String, Float64
from vitarana_drone.msg import MarkerData
import rospy
import time
import tf
import math
import csv

class Coordinates():
    def __init__(self, x, y, z):
	self.lat = x
	self.long = y
	self.alt = z

class Objective():
    def __init__(self, Type, sx, sy, sz, dx, dy, dz):
	self.Type = Type
	self.source = Coordinates(sx, sy, sz)
	self.destination = Coordinates(dx, dy, dz)

class Edrone():
    def __init__(self):
	rospy.init_node('position_controller')  # initializing ros node with name position_controller
	self.a1 = [18.9998102845, 72.000142461, 16.757981]
	self.x1 = [18.9999367615, 72.000142461, 16.757981]
	self.d_center = [18.9998102845+0.000013552, 72.000142461+0.000014245]
	self.r_center = [18.9999367615+0.000013552, 72.000142461+0.000014245]
	self.del_x = 0.000013552
	self.del_y = 0.000014245
	self.error = [0.0, 0.0, 0.0]
	self.prev_error = [0.0, 0.0, 0.0]
	self.sample_time = 0.016666666
	self.iterm = [0, 0, 0]     	#iterm for altitude,latitude,longitude
	self.curnt_pos = [0, 0, 0]
	self.objective = {}
	self.curnt_objective = Objective("null", 0, 0, 0, 0, 0, 0)
	self.pkg_id = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
	self.total_profit = 0
	self.current_time = 0.0
	self.prev_time = 0.0
	self.elapsed_time = 0.0
	self.start_time = time.time()	
	self.lat = 0
	self.long = 0
	self.height = 0
	self.id    = 0
	self.err_x = 0.0
	self.err_y = 0.0
	self.curnt_sensor = 0
	self.avg_speed = 0
	self.destination_reached = False
	self.height_reached = False
	self.time_h = 0
	self.time_xy = 0
	self.tmp_lat = 0.0
	self.tmp_long = 0.0
	self.task = []
	self.d_list = []
	self.curnt_task = "null"
	self.scan_cntr = 0
	self.scan = False
	self.move = False
	self.marker_found = False
	self.landing = False
	self.use_gripper = False
	self.gripper_state = False
	self.collision = False

	self.drone_cmd_pub = edrone_cmd()
	self.drone_cmd_pub.rcRoll = 1500.0
	self.drone_cmd_pub.rcThrottle = 0.0
	self.drone_cmd_pub.rcPitch = 1500.0
	self.drone_cmd_pub.rcYaw = 1500.0
	self.drone_cmd_pub.aux1 = 0.0
	self.drone_cmd_pub.aux2 = 0.0
	self.drone_cmd_pub.aux3 = 0.0 
	self.drone_cmd_pub.aux4 = 0.0
	self.ranges=[0.0, 0.0, 0.0, 0.0, 0.0]

	self.Kp = [620*0.5 ,319*1000, 319*1000]   #These constant are only to tune throttle,roll and pitch/edrone/gps_velocity
	self.Kd = [4997*5, 3690*2*10000, 3690*2*10000]
	self.Ki = [185*0.001, 0, 0]

	self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
	self.alt_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)  #error in altitude
	self.long_error_pub = rospy.Publisher('/long_error', Float32, queue_size=1) #error in longitude
	self.lat_error_pub = rospy.Publisher('/lat_error', Float32, queue_size=1)	#error in latitude
	self.set_cordinate_pub = rospy.Publisher('/zero_error', Float32, queue_size=1) #this will publish our refrence cordinate
	self.drone_cmd = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
	self.marker_found_pub = rospy.Publisher('/marker_found', String, queue_size=1)#if marker found then for next marker
	self.zm_val_pub = rospy.Publisher('/zm_val', Float32, queue_size=1)
	#rospy.Subscriber('/pid_tuning_altitude', PidTune, self.alt_set_pid)
	
	rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_cordinates)  #subscribing gps here
	rospy.Subscriber('/edrone/range_finder_top',LaserScan, self.range_finder_top)
	rospy.Subscriber('/edrone/marker_data', MarkerData, self.marker_callback )
	rospy.Subscriber('/edrone/gps_velocity', Vector3Stamped , self.velocity)
	rospy.Subscriber('/qrlatitude', Float32, self.qr_cordinates_lat)
	rospy.Subscriber('/qrlongitude', Float32, self.qr_cordinates_long)
	rospy.Subscriber('/qraltitude', Float32, self.qr_cordinates_alt)
	rospy.Subscriber('/edrone/range_finder_bottom',LaserScan,self.range_finder_bottom)

    def read_csv(self):
	f = open('/home/naman/catkin_ws/src/vitarana_drone/scripts/sequenced_manifest.csv', "w+")
	f.close()
	with open('/home/naman/catkin_ws/src/vitarana_drone/scripts/original.csv', 'r') as file:
	    reader = csv.reader(file)
	    k = 0
	    t = ""
	    sx = sy = sz = dx = dy = dz = 0
	    for row in reader:
		self.d_list.append(row)
		if (row[0] == "DELIVERY"):
		    t = row[0]
		    if (row[1][0] == 'A'):
			t_k = 0
		    elif (row[1][0] == 'B'):
			t_k = 1
		    else:
			t_k = 2
		    sx = self.a1[0] + t_k*self.del_x
		    sy = self.a1[1] + (int(row[1][1])-1)*self.del_y
		    sz = self.a1[2]
		    tmp_str = ""
		    tmp_cnt = 0
		    for i in row[2]:
			if (i == ';'):
			    if (tmp_cnt == 0):
				dx = float(tmp_str)
				tmp_str = ""
				tmp_cnt = 1
			    elif (tmp_cnt == 1):
				dy = float(tmp_str)
				tmp_str = ""
				tmp_cnt = 2
			    continue
			tmp_str = tmp_str+i
		    dz = float(tmp_str)
		    tmp_str = ""
		else:
		    t = row[0]
		    if (row[2][0] == 'X'):
			t_k = 0
		    elif (row[2][0] == 'Y'):
			t_k = 1
		    else:
			t_k = 2
		    dx = self.x1[0] + t_k*self.del_x
		    dy = self.x1[1] + (int(row[2][1])-1)*self.del_y
		    dz = self.x1[2]
		    tmp_str = ""
		    tmp_cnt = 0
		    for i in row[1]:
			if (i == ';'):
			    if (tmp_cnt == 0):
				sx = float(tmp_str)
				tmp_str = ""
				tmp_cnt = 1
			    elif (tmp_cnt == 1):
				sy = float(tmp_str)
				tmp_str = ""
				tmp_cnt = 2
			    continue
			tmp_str = tmp_str+i
		    sz = float(tmp_str)
		    tmp_str = ""
		o = Objective(t, sx, sy, sz, dx, dy , dz)
		self.objective[k] = o
		k = k+1

    def marker_callback(self , msg):
	self.id    = msg.marker_id
	self.err_x = msg.err_x_m
	self.err_y = msg.err_y_m
	if (math.isnan(self.err_x)):
	    self.marker_found = False

    def qr_cordinates_lat(self,l):          #coordinates of box scanned by qr_detect script and recieving values here.
	self.qr_cordinates[0]=l.data

    def qr_cordinates_long(self,lo):		 #coordinates of box scanned by qr_detect script and recieving values here.
	self.qr_cordinates[1]=lo.data

    def qr_cordinates_alt(self,a):		 #coordinates of box scanned by qr_detect script and recieving values here.
	self.qr_cordinates[2]=a.data

    def velocity(self, vel):            #velocity() to limit the vilocity of drone 
        self.vel_x=vel.vector.x
        self.vel_y=vel.vector.y
        self.vel_z=vel.vector.z

    def range_finder_top(self,top):            #defining range finder top
	self.ranges[0] = top.ranges[3] #left
	self.ranges[1] = top.ranges[0] #n
	self.ranges[2] = top.ranges[1] #right
	self.ranges[3] = top.ranges[2] #s

    def range_finder_bottom(self,down):   #defining range finder bottom
	self.ranges[4] = down.ranges[0]

    def gps_cordinates(self, gps): #function which recieve gps values 
	self.curnt_pos[0] = gps.altitude
	self.curnt_pos[1] = gps.latitude
	self.curnt_pos[2] = gps.longitude
	try:
	   if (self.lat == 0):
	       print("starting coordinates: ", self.curnt_pos[1], self.curnt_pos[2], self.curnt_pos[0])
	       self.lat = self.curnt_pos[1]
	       self.long = self.curnt_pos[2]
	       self.height = self.curnt_pos[0]+2
	except rospy.ServiceException as e:
	      print("position call failed")

    def lat_to_x(self, input_latitude):
    	return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
    	return -105292.0089353767 * (input_longitude - 72)

    def x_to_lat(self, input_x):
	return 19 + (input_x)/110692.0702932625

    def y_to_lon(self, input_y):
	return 72 - (input_y)/105292.0089353767

    def gripper_client(self, state):
	rospy.wait_for_service('/edrone/activate_gripper')
	try:
	   self.attach=rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
	   self.grip_rtn_srv=self.attach(state)
	   return self.grip_rtn_srv.result
	except rospy.ServiceException as e:
	      print("service call failed")
    def scheduler(self):
	ldr = 0
	dis = 10000000000
	for x in self.objective:
	    if (self.curnt_objective.Type != "DELIVERY"):
		if (self.objective[x].Type == "DELIVERY"):
		    tmp_ldr = math.sqrt((self.objective[x].destination.lat-self.d_center[0])*(self.objective[x].destination.lat-self.d_center[0]) + (self.objective[x].destination.long-self.d_center[1])*(self.objective[x].destination.long-self.d_center[1]))
		    if (tmp_ldr > ldr):
			ldr = tmp_ldr
			self.pkg_id = x
	    else:
		tmp_dis = math.sqrt((self.objective[x].source.lat-self.curnt_pos[1])*(self.objective[x].source.lat-self.curnt_pos[1]) + (self.objective[x].source.long-self.curnt_pos[2])*(self.objective[x].source.long-self.curnt_pos[2]))
		if (tmp_dis < dis):
		    dis = tmp_dis
		    self.pkg_id = x
	
	ldr = 0  
	if (self.objective[self.pkg_id].Type == "DELIVERY"):
	    for x in self.objective:
		if (self.objective[x].Type == "DELIVERY"):
		    tmp_ldr = math.sqrt((self.objective[x].destination.lat-self.d_center[0])*(self.objective[x].destination.lat-self.d_center[0]) + (self.objective[x].destination.long-self.d_center[1])*(self.objective[x].destination.long-self.d_center[1]))
		    if (tmp_ldr > ldr):
			ldr = tmp_ldr
			self.pkg_id = x
	    if (self.curnt_objective.Type == 'Delivery'):
		self.height = 50
	self.curnt_objective = self.objective[self.pkg_id]
	self.objective.pop(self.pkg_id)
	self.task.append("drop")
	if (self.curnt_objective.Type == "DELIVERY"):
	    self.task.append("scan")
	self.task.append("dest")
	self.task.append("pick")
	self.task.append("source")
	print("DELIVERING PACKAGE!")
	print("package id : ", self.pkg_id, "Type : ", self.curnt_objective.Type)
	print("package destination : ", self.curnt_objective.destination.lat, self.curnt_objective.destination.long, self.curnt_objective.destination.alt)

    def pidthr(self, h , l):    #pid for throttle
	self.current_time = time.time()
	if (l== True):
		if(abs(self.vel_z) >= 1.9 and abs(h - self.curnt_pos[0])>=0.6):
		   self.throttle =1600
		else:
		   self.throttle =1420
	elif (l== False):
		self.error[0] = h - self.curnt_pos[0]
		self.iterm[0] = (self.iterm[0] + self.error[0])*self.Ki[0]
		self.throttle = 1495.1 + ((self.error[0]*self.Kp[0]) + (self.Kd[0]*(self.error[0]-self.prev_error[0])) + self.iterm[0])

		if (self.throttle > 2000):
			self.throttle = 2000
		elif (self.throttle < 1000):
			self.throttle = 1000

		self.prev_error[0] = self.error[0] 
		self.prev_time = self.current_time
	self.drone_cmd_pub.rcThrottle = (self.throttle)
	self.alt_error_pub.publish(float(self.error[0]))
	self.drone_cmd.publish(self.drone_cmd_pub)
	if (l == True):
		return
	if (abs(self.error[0]) <= 0.5):
		if (abs(self.current_time-self.time_h) <= 2):
			self.height_reached = True
		else:
			self.height_reached = False
		self.time_h = self.current_time
	else:
		self.height_reached = False

    def pidxy(self, x , y):  #x for latitude and  y for longitude position
	if (self.curnt_pos[0]>28 and (abs(self.lat_to_x(self.x_to_lat(x)) - self.lat_to_x(self.curnt_pos[1] )) >=1.5 or abs(self.long_to_y(self.y_to_lon(y)) - self.long_to_y(self.curnt_pos[2] )) >=1.5)):
	   lowlimit=1486
	   uplimit=1514
	elif(self.curnt_pos[0]>21 and (abs(self.lat_to_x(self.x_to_lat(x)) - self.lat_to_x(self.curnt_pos[1] )) <0.2 or abs(self.long_to_y(self.y_to_lon(y)) - self.long_to_y(self.curnt_pos[2] )) <0.2)):
	    lowlimit=1498
	    uplimit=1502
	else:
	    lowlimit=1485
	    uplimit=1515
	'''if (abs(self.vel_x) < 1.95 and abs(self.vel_x) >= 0.331 or abs(self.lat_to_x(self.x_to_lat(x)) - self.lat_to_x(self.curnt_pos[1] )) <=10 and abs(self.lat_to_x(self.x_to_lat(x)) - self.lat_to_x(self.curnt_pos[1] )) >2.5):
	     self.Kp[1] = 618*1000
	     self.Kd[1] = 2991*2*10000'''
	if (self.curnt_pos[0]< 30 and (abs(self.vel_x) < 0.331 or abs(self.lat_to_x(self.x_to_lat(x)) - self.lat_to_x(self.curnt_pos[1] )) <=2.5)):
	     self.Kp[1] = 618*1000
	     self.Kd[1] = 2991*2*10000
	else:
		if (abs(self.vel_x) >= 29.5 and abs(self.lat_to_x(self.x_to_lat(x)) - self.lat_to_x(self.curnt_pos[1] )) >=65):
		   self.Kp[1] = 0
		   self.Kd[1] = 0
		else:
		     if(self.curnt_pos[0]>25):
	     	       self.Kp[1] = 319*1000
	     	       self.Kd[1] = 3690*2*10000
		     else:
			 self.Kp[1] = 618*1000
	     		 self.Kd[1] = 2991*2*10000

	self.error[1] = self.x_to_lat(x) - self.curnt_pos[1]   #pid for latitude
	self.iterm[1] = (self.iterm[1] + self.Ki[1])*self.error[1]
	self.pitch = 1500 + ((self.error[1]*self.Kp[1]) + (self.Kd[1]*(self.error[1]-self.prev_error[1])) + self.iterm[1])
	if (self.pitch < lowlimit):
	  	self.pitch = lowlimit
	elif (self.pitch > uplimit):
	       	self.pitch = uplimit
	#print(self.Kp , self.Kd , abs(self.lat_to_x(self.x_to_lat(x)) - self.lat_to_x(self.curnt_pos[1] )))
	self.prev_error[1] = self.error[1]
	self.prev_time = self.current_time
	self.drone_cmd_pub.rcPitch = self.pitch
	self.prev_time = self.current_time

	
	'''if (abs(self.vel_y) < 1.95 and abs(self.vel_y) >= 0.331 or abs(self.long_to_y(self.y_to_lon(y)) - self.long_to_y(self.curnt_pos[2] )) <=10 and abs(self.long_to_y(self.y_to_lon(y)) - self.long_to_y(self.curnt_pos[2] )) >2.5):
	     self.Kp[2] = 618*1000
	     self.Kd[2] = 2991*2*10000'''
	if (self.curnt_pos[0]< 30 and (abs(self.vel_y) < 0.331 or abs(self.long_to_y(self.y_to_lon(y)) - self.long_to_y(self.curnt_pos[2] )) <=2.5)):
	     self.Kp[2] = 618*1000
	     self.Kd[2] = 2991*2*10000
	else:
		if (abs(self.vel_y) >= 29.5 and abs(self.long_to_y(self.y_to_lon(y)) - self.long_to_y(self.curnt_pos[2] )) >=65):
		   self.Kp[2] = 0
		   self.Kd[2] = 0
		else:
	     	     if(self.curnt_pos[0]>25):
	     	       self.Kp[2] = 319*1000
	     	       self.Kd[2] = 3690*2*10000
		     else:
			 self.Kp[2] = 618*1000
	     		 self.Kd[2] = 2991*2*10000
	self.error[2] = self.y_to_lon(y) - self.curnt_pos[2]	#pid for longitude
	self.iterm[2] = (self.iterm[2] + self.Ki[2])*self.error[2]
	self.roll =  1500 + ((self.error[2]*self.Kp[2])+ (self.Kd[2]*(self.error[2]-self.prev_error[2])) + self.iterm[2])
	if (self.roll < lowlimit):
	  	self.roll = lowlimit
	elif (self.roll > uplimit):
	       	self.roll = uplimit

	self.prev_error[2] = self.error[2]
	self.prev_time = self.current_time
	self.drone_cmd_pub.rcPitch = self.pitch
	self.drone_cmd_pub.rcRoll = self.roll
	self.lat_error_pub.publish(float(self.error[1]))
	self.long_error_pub.publish(float(self.lat_to_x(self.curnt_pos[1])))
	self.drone_cmd.publish(self.drone_cmd_pub)
	error_x = x - self.lat_to_x(self.curnt_pos[1])
	error_y = y - self.long_to_y(self.curnt_pos[2])
	if (self.curnt_task == "dest"):
		if (math.sqrt(error_x*error_x + error_y*error_y) <= 5 and abs(self.vel_y) <= 1.5 and abs(self.vel_x) <= 1.5):
			self.height = self.curnt_objective.destination.alt+8
	elif (self.curnt_task == "source"):
		if (abs(error_x) < 3 and abs(error_y) < 3 and abs(self.vel_y) <= 2.2 and abs(self.vel_x) <= 2.2):
			self.height = self.curnt_objective.source.alt+1
	if (abs(error_x) <= 0.1 and abs(error_y) <= 0.045 and abs(self.vel_x) <= 0.1 and abs(self.vel_y) <= 0.1):
		if (abs(self.current_time-self.time_xy) <= 2):
			self.destination_reached = True
		else:
			self.destination_reached = False
		self.time_xy = self.current_time
	else:
		self.destination_reached = False

    def move_xy(self, dx, dy):    # this function move drone in x y palne
	x = self.lat_to_x(self.lat)
	y = self.long_to_y(self.long)
	x = x + dx
	y = y + dy
	self.lat = self.x_to_lat(x)
	self.long = self.y_to_lon(y)
	
    def detect_collision(self):  # to avoid collision
	if (self.ranges[3] < abs(self.long_to_y(self.long)-self.long_to_y(self.curnt_pos[2]))):
	    # avoid collision south
	    if (self.lat-self.curnt_pos[1] >= 0):
		self.move_xy(5, 0)
	    else:
		self.move_xy(-5, 0)
	if (self.ranges[1] < abs(self.long_to_y(self.long)-self.long_to_y(self.curnt_pos[2]))):
	    # avoid collision north
	    if (self.lat-self.curnt_pos[1] >= 0):
		self.move_xy(5, 0)
	    else:
		self.move_xy(-5, 0)
	if (self.ranges[2] < abs(self.lat_to_x(self.lat)-self.lat_to_x(self.curnt_pos[1]))):
	    # avoid collision east
	    if (self.lat-self.curnt_pos[1] >= 0):
		self.move_xy(0, 5)
	    else:
		self.move_xy(0, -5)
	if (self.ranges[0] < abs(self.lat_to_x(self.lat)-self.lat_to_x(self.curnt_pos[1]))):
	    # avoid collision east
	    if (self.lat-self.curnt_pos[1] >= 0):
		self.move_xy(0, 5)
	    else:
		self.move_xy(0, -5)

    def find_marker(self):  # To detect marker and move drone
	zm = self.curnt_pos[0]-self.curnt_objective.destination.alt
	self.zm_val_pub.publish(zm)
	if (self.marker_found == False and self.destination_reached == True and self.height_reached == True):
	    if (abs(self.curnt_pos[0]-self.curnt_objective.destination.alt) >= 9):
		self.height = self.curnt_objective.destination.alt+8
		print("go down!")
	    elif (abs(self.curnt_pos[0]-self.curnt_objective.destination.alt) <= 7):
		self.height = self.curnt_objective.destination.alt+8
	    else:
		if (self.scan_cntr == 0):
		    self.move_xy(0, 0)
		    self.scan_cntr = self.scan_cntr+1
		elif (self.scan_cntr == 1):
		    self.move_xy(0, -8)
		    self.scan_cntr = self.scan_cntr+1
		elif (self.scan_cntr == 2):
		    self.move_xy(0, 9)
		    self.scan_cntr = self.scan_cntr+1
		elif (self.scan_cntr == 3):
		    self.move_xy(0, 3)
		    self.scan_cntr = self.scan_cntr+1
		elif (self.scan_cntr == 4):
		    self.move_xy(3, -6)
		    self.scan_cntr = 0
	if (self.marker_found == True):
		if (self.destination_reached == True and self.height_reached == True):
			if (abs(self.err_x) > 0.2 or abs(self.err_y) > 0.18):
				self.move_xy(-self.err_x, self.err_y)
			else:
				self.scan = False
				self.scan_cntr = 0
	if (math.isnan(self.err_x) == False and self.marker_found == False):
	    self.move_xy(-self.err_x, self.err_y)
	    self.marker_found = True

    def pick_drop_box(self, flag):  #fuction to pick box
	if (self.destination_reached == True and self.height_reached == True):
	    self.landing = True
	if (flag == True):
	    pick = self.gripper_client(True)
	    if (pick == True):
		self.use_gripper = False
		self.height_reached = True
		self.landing = False
	    elif (pick == False and self.vel_z == 0):
		self.landing = False
		self.height_reached = False
		self.height = self.height+1
	else:
	    if (abs(self.curnt_pos[0]-self.curnt_objective.destination.alt) <= 0.5 and abs(self.vel_z) <= 0.01):
		self.gripper_client(False)
		self.use_gripper = False
		if (self.curnt_objective.Type == 'DELIVERY'):
		    self.height = self.curnt_pos[0]+10
		else:
		    self.height = self.curnt_pos[0]+1
		self.landing = False
		print("package delivered!")
		print("package id : ", self.pkg_id, "Type : ", self.curnt_objective.Type)
		print('time elapsed: ', self.elapsed_time)
		profit = 0
		if (self.curnt_objective.Type == "DELIVERY"):
		    x = self.lat_to_x(self.curnt_objective.destination.lat)-self.lat_to_x(self.d_center[0])
		    y = self.long_to_y(self.curnt_objective.destination.long)-self.long_to_y(self.d_center[1])
		    ld = math.sqrt(x*x + y*y)
		    profit = 5 + 0.1*ld
		else:
		    x = self.lat_to_x(self.curnt_objective.source.lat)-self.lat_to_x(self.r_center[0])
		    y = self.long_to_y(self.curnt_objective.source.long)-self.long_to_y(self.r_center[1])
		    ld = math.sqrt(x*x + y*y)
		    profit = 5 + 0.1*ld
		print('profit: ', profit)
		self.total_profit = self.total_profit+profit
		print('Total Profit: ', self.total_profit)
		with open('/home/naman/catkin_ws/src/vitarana_drone/scripts/sequenced_manifest.csv', 'a') as file:
		    writer = csv.writer(file)
		    writer.writerow(self.d_list[self.pkg_id])
		    
		
		    

    def main(self):  # this function calls the all necessary function
	self.elapsed_time = time.time()-self.start_time
	self.pidthr(self.height ,self.landing)
        self.pidxy(self.lat_to_x(self.lat), self.long_to_y(self.long))
	if (self.scan == True):
	    self.find_marker()
	elif (self.destination_reached == True and self.height_reached == True and self.use_gripper == False):
	    if (len(self.task) == 0):
		self.scheduler()
	    self.curnt_task = self.task.pop()
	    print(self.curnt_task)
	    if (self.curnt_task == "source"):
		self.lat = self.curnt_objective.source.lat
		self.long = self.curnt_objective.source.long
		if (self.curnt_objective.Type != "DELIVERY"):
		    self.height = 50
	    elif (self.curnt_task == "pick"):
		self.use_gripper = True
		self.gripper_state = True
		self.height_reached = False
		self.height = self.curnt_objective.source.alt+1
	    elif (self.curnt_task == "dest"):
		self.lat = self.curnt_objective.destination.lat
		self.long = self.curnt_objective.destination.long
		self.height = 50
 	    elif (self.curnt_task == "scan"):
		self.scan = True
	    elif (self.curnt_task == "drop"):
		self.use_gripper = True
		self.gripper_state = False
		self.height_reached = False
		self.avg_speed = 1 # no use of it bas temporary hai
	if (self.use_gripper == True):
	    self.pick_drop_box(self.gripper_state)

if __name__ == '__main__':
   e_drone = Edrone()
   r = rospy.Rate(60)
   e_drone.read_csv()
   while not rospy.is_shutdown():
    	e_drone.main()
        r.sleep()

