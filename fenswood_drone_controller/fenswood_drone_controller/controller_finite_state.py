"""
Very simple script-based sequencer using the old school examples
from https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber
"""
from cmath import pi
import heapq
import io
from itertools import combinations
from math import acos, atan2
from multiprocessing import Condition
import os
from typing import final

from matplotlib import pyplot as plt
import rclpy                                                    # type: ignore
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State                               # type: ignore
from sensor_msgs.msg import NavSatFix                           # type: ignore
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped                  # type: ignore

from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import Image
from sensor_msgs.msg import BatteryState
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong    # type: ignore


############ Imports for path planning
from cmath import pi, sqrt
from ctypes import sizeof
from weakref import ref
from collections import OrderedDict
import time
import numpy as np
import math
############# Imports from Image Processing
import cv2
from math import pi, tan, cos, sin, atan
from scipy import optimize
# import matplotlib.pyplot as plt

class FenswoodDroneController(Node):

    def __init__(self):

        #self.get_logger().info('Flag 1')
        super().__init__('finite_state_controller')
        self.last_status = None     # global for last received status message
        self.last_pos = None       # global for last received position message
        self.init_alt = None       # global for global altitude at start
        self.last_alt_rel = None   # global for last altitude relative to start

        self.flightHeight = 31

        self.mission_stage=0
        self.map = None

        #image processor paras
        self.br = CvBridge()
        self.image=None
        self.ready = False
        self.yellow_threadhold = 124
        self.red_threadhold = 0
        self.vertical_angel = 1.72525
        self.horizon_angel = 2
        self.al_drone = 0
        self.camera_angel = 0
        self.positions_of_points = []
        self.detect_red_num = 0
        self.r = 0
        self.center_point_of_area = [0, 0]
        self.centers_of_arc = []
        self.landing_point = [0, 0]
        self.astar_path_near_volcano_in_meters = []
        self.global_path_to_landing = []

        self.currentPath = []
        self.takeoffUAV = False
        self.em =False
        self.landing = False
        self.RTL = False
        self.abort = False

        # create service clients for long command (datastream requests)...
        self.cmd_cli = self.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command_int service not available, waiting again...')
        # ... for mode changes ...
        self.mode_cli = self.create_client(SetMode, '/vehicle_1/mavros/set_mode')
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        # ... for arming ...
        self.arm_cli = self.create_client(CommandBool, '/vehicle_1/mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
        # ... and for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, '/vehicle_1/mavros/cmd/takeoff')
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')
        # create publisher for setpoint
        self.target_pub = self.create_publisher(GeoPoseStamped, '/vehicle_1/mavros/setpoint_position/global', 10)
        self.target_v = self.create_publisher(Twist, '/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        # self.target_v = self.create_publisher(PoseStamped, '/vehicle_1/mavros/setpoint_position/pose', 10)
        # and make a placeholder for the last sent target
        self.last_target = GeoPoseStamped()

        self.angular_velocity=Twist()
        
        # initial state for finite state machine
        self.control_state = 'init'
        # timer for time spent in each state
        self.state_timer = 0

        self.d_lon = None
        self.d_lat = None
        
        self.camera_pos_pub=self.create_publisher(Float32,'/vehicle_1/gimbal_tilt_cmd',10)
        self.imageProcessed=self.create_publisher(Image,'/vehicle_1/imageProcessed',10)
        self.geoAlt=self.create_publisher(Int32,'/vehicle_1/currentAltitude',10)
        self.battery=self.create_publisher(BatteryState,'/vehicle_1/batteryState',10)
        self.flightState=self.create_publisher(String,'/vehicle_1/flightState',10)


        self.timer = None
        self.battery_voltage = 9.0 # Initialize the battery voltage level
        self.percent_charge_level = 1.0  # Initialize the percentage charge level
        self.decrement_factor = 0.99 # Used to reduce battery level each cycle
    
        
        self.camera_pos=Float32()     
        self.currentHeight = Int32()
        self.State = String()
        self.processedImage = Image()
        self.percent_charge_level = 1.0
        self.heading= None
        self.path_step=0  
        self.global_path=[]
        self.ApathWay = []
        self.final_p_global=[51.422122,-2.668699]
        self.volcano_centre = [51.4219206,-2.6687700]
        self.get_logger().info('Flag 2')
    
        

    def publish_Image(self,img):
        self.imageProcessed.publish(self.br.cv2_to_imgmsg(img, "bgr8"))

    def get_battery_state(self):
        msg = BatteryState() # Create a message of this type 
        msg.voltage = self.battery_voltage 
        msg.percentage = self.percent_charge_level
        self.battery.publish(msg) # Publish BatteryState message 
        
        # Decrement the battery state 
        self.battery_voltage = self.battery_voltage * self.decrement_factor
        self.percent_charge_level = self.percent_charge_level * self.decrement_factor

    # on receiving status message, save it to global
    def state_callback(self,msg):
        self.last_status = msg
        # self.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))

    # on receiving positon message, save it to global
    def position_callback(self,msg):
        # determine altitude relative to start
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt
            self.currentHeight.data = int(self.last_alt_rel) 
            self.geoAlt.publish(self.currentHeight)
        self.last_pos = msg
        # self.get_logger().info('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
        #                                                                 msg.longitude,
        #                                                                 self.last_alt_rel))       

    def local_position_callback(self,msg):

        #self.quaternion_x=msg.pose.orientation.x
        #self.quaternion_y=msg.pose.orientation.y
        #self.quaternion_z=msg.pose.orientation.z
        #self.quaternion_w=msg.pose.orientation.w
        #self.heading=180/pi*atan2((2 * (self.quaternion_w * self.quaternion_z + self.quaternion_x * self.quaternion_y),1 - 2 * (self.quaternion_y * self.quaternion_y + self.quaternion_z * self.quaternion_z)))

        self.get_logger().info('Local position:{},{},{}'.format(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    def heading_callback(self,msg):
        self.heading=-math.radians(msg.data)
        # self.get_logger().info('Heading:{}'.format(self.heading))

    def wait_for_new_status(self):
        """
        Wait for new state message to be received.  These are sent at
        1Hz so calling this is roughly equivalent to one second delay.
        """
        if self.last_status:
            # if had a message before, wait for higher timestamp
            last_stamp = self.last_status.header.stamp.sec
            for try_wait in range(60):
                rclpy.spin_once(self)
                if self.last_status.header.stamp.sec > last_stamp:
                    break
        else:
            # if never had a message, just wait for first one          
            for try_wait in range(60):
                if self.last_status:
                    break
                rclpy.spin_once(self)

    def hold_position(self,time):
        last_stamp=self.last_status.header.stamp.sec
        while self.last_status.header.stamp.sec<last_stamp+time:
            # self.get_logger().info('Holding time:{}s out of {}s '.format(self.last_status.header.stamp.sec-last_stamp, time))
            rclpy.spin_once(self)

    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response

    def change_mode(self,new_mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response

    def arm_request(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)

    def takeoff(self,target_alt):
        takeoff_req = CommandTOL.Request()
        #takeoff_req.yaw=90.0
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)

    def flytoWaypoints(self,waypoints,Condition = False):
        for wp in waypoints:
            done  = False
            if self.abort:
                break
            while not done and not self.em:
                self.flyto(wp[0],wp[1], self.init_alt - 50.0 + self.flightHeight) # unexplained correction factor on altitude
                self.hold_position(2)
                d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
                d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
                if (abs(d_lon) < 0.0001) & (abs(d_lat) < 0.0001):
                    self.get_logger().info('Arrived at {},{}'.format(d_lat,d_lon))
                    done = True
                elif self.state_timer > 60:
                    # timeout
                    self.get_logger().error('Failed to reach target')
            
            if not self.em and not Condition:
               self.currentPath.append(wp)
            if Condition:
                self.currentPath.pop()
                self.get_logger().info('back of currentPath:'.format(self.currentPath))
    
    def returnHome(self):
        for wp in self.currentPath:
            done  = False
            while not done:
                self.flyto(wp[0],wp[1], self.init_alt - 50.0 + self.flightHeight) # unexplained correction factor on altitude
                self.hold_position(1)
                d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
                d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
                if (abs(d_lon) < 0.00001) & (abs(d_lat) < 0.00001):
                    # self.get_logger().info('back to waypoint: {},{}'.format(d_lat,d_lon))
                    done = True
                elif self.state_timer > 60:
                    # timeout
                    self.get_logger().error('Failed to reach home')

    def flyto(self,lat,lon,alt):
        self.last_target.pose.position.latitude = lat
        self.last_target.pose.position.longitude = lon
        self.last_target.pose.position.altitude = alt
        angle = (-atan2((self.volcano_centre[0]-self.last_pos.latitude),(self.volcano_centre[1]-self.last_pos.longitude))-pi/2)*0.75
        self.last_target.pose.orientation.x=0.0
        self.last_target.pose.orientation.y=sin(angle)
        self.last_target.pose.orientation.z=0.0
        self.last_target.pose.orientation.w=cos(angle/2)
        # self.get_logger().info("x={}, w={}".format(self.last_target.pose.orientation.x,self.last_target.pose.orientation.w))
        #self.angular_velocity.angular.z=-(heading+atan2(pfinal[0]-p[0],pfinal[1]-p[1]))

        self.target_pub.publish(self.last_target)
        
        # self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(lat,lon,alt)) 

    def move_cam(self, angle):
        self.camera_pos.data=3.14*angle/180.0
        self.camera_pos_pub.publish(self.camera_pos)
        self.get_logger().info('Camera moved to {} degrees'.format(angle))

    def try_reach_destination(self):
            # wait for drone to reach desired position, or timeout after 60 attempts
        for try_arrive in range(60):
            self.wait_for_new_status()
            self.d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            self.d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            self.get_logger().info('Target error {},{}'.format(self.d_lat,self.d_lon))
            if abs(self.d_lon) < 0.0005:
                if abs(self.d_lat) < 0.0005:
                    self.get_logger().info('Close enough to target delta={},{}'.format(self.d_lat,self.d_lon))
                    break    
    def pubulish_flightState(self,state):
        self.State.data = state
        self.flightState.publish(self.State)
    
    def stageRelease(self,msg):
        state = msg.data
        if state == 'takeoff':
            self.takeoffUAV =True
            self.get_logger().info('Prepare to takeoff')
        elif state == 'landing':
            self.landing = True
            self.get_logger().info('Prepare to landing')
        elif state == 'RTL':
            self.RTL = True
            self.get_logger().info('Prepare to RTL')
        elif state == 'abort':
            self.abort = True
        elif state == 'em':
            self.em = True
            self.get_logger().info('Emergency!')
        else:
            pass

    def state_transition(self):
        if self.control_state =='init':
            self.pubulish_flightState('Initialization')
            if self.last_status.system_status==3:
                self.get_logger().info('Drone initialized')
                # send command to request regular position updates
                self.request_data_stream(33, 1000000)
                self.get_logger().info('Requested position stream')

                self.request_data_stream(32, 1000000)
                self.get_logger().info('Requested LOCAL position stream')     
                
                # change mode to GUIDED
                self.change_mode("GUIDED")
                self.get_logger().info('Request sent for GUIDED mode.')
                # move on to arming
                return('arming')
            else:
                return('init')


        elif self.control_state == 'arming':
            if not self.takeoffUAV:
                self.get_logger().info('Waiting to takeoff confirmation')
                self.pubulish_flightState('Wait to takeoff')
                self.hold_position(1)
                return('arming')
            else:
                self.pubulish_flightState('Arming(Position Estimat)')
                if self.last_status.armed:
                    self.get_logger().info('Arming successful')
                    self.timer=self.create_timer(10, self.get_battery_state)
                    # armed - grab init alt for relative working
                    if self.last_pos:
                        self.init_alt = self.last_pos.altitude
                    return('path_planning')
                elif self.state_timer > 600:
                    # timeout
                    self.get_logger().error('Failed to arm')
                    return('exit')
                else:
                    self.arm_request()
                    # self.get_logger().info('Arming request sent.')
                    return('arming')

        elif self.control_state=='path_planning':
            self.pubulish_flightState('Path_Planning_to_Volcano')
            # self.get_logger().info('AAAAAAAAAAAAAAAAAAAA') 
            drone_global_coordinate=[self.last_pos.latitude,self.last_pos.longitude] 

            if self.mission_stage==0:        
                #self.final_p_global=[51.422337798920005,-2.66750059595]
                self.final_p_global=[51.422122,-2.668699]

                final_p=self.convert_coordinate_local(self.final_p_global,drone_global_coordinate,self.heading)
                polygon_coordinates=[[51.4224669,-2.6720597],[51.4235482,-2.6673723],[51.4228490,-2.6670013],[51.4217450,-2.6714016]]
                fenswood_boundary=[[51.4234260,-2.6717208],[51.4212462,-2.6701340],[51.4224401,-2.6656878],[51.4246918,-2.6670602]]
                self.map=self.polygon_obstacles(polygon_coordinates,drone_global_coordinate,self.heading)+self.polygon_obstacles(fenswood_boundary,drone_global_coordinate,self.heading)
            
            elif self.mission_stage==1:
                self.final_p_global=[51.421895,-2.6686701]
                final_p=self.convert_coordinate_local(self.final_p_global,drone_global_coordinate,self.heading)
            
            
            self.get_logger().info('LAT {}, LONG: {}'.format(self.last_pos.latitude,self.last_pos.longitude))
           
            self.global_path+=self.path_planning_A(self.map,final_p,drone_global_coordinate,self.heading)
            self.global_path.append([51.42211380709214, -2.6679558911937806, self.init_alt - 50.0 + self.flightHeight])
            self.get_logger().info('Path planning completed')
            # self.get_logger().info('Coordinates: {}'.format(self.global_path))

            return('climbing')

        elif self.control_state == 'climbing':
            self.pubulish_flightState('Takeoff')
            # send takeoff command
            self.takeoff(31.0)
            # self.get_logger().info('Takeoff request sent.')        
            if self.last_alt_rel > 30.0:
                # self.get_logger().info('Close enough to flight altitude')
                # move drone by sending setpoint message
                
                return('on_way')
            elif self.state_timer > 500:
                # timeout
                self.get_logger().error('Failed to reach altitude')
                return('landing')
            else:
                self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
                return('climbing')

        elif self.control_state == 'on_way':
            self.pubulish_flightState('On_the_Way')
            self.ApathWay = [[51.4234178,-2.6715506],[51.42274114940929, -2.6710506500297124],[51.42353828560897, -2.667359809557059],[51.42286290831265, -2.667000516253376],[51.42232, -2.66795]]
            # self.ApathWay = [[51.4234178,-2.6715506],[51.42211380709214, -2.6679558911937806]]
            self.flytoWaypoints(self.ApathWay)
            return('rotating')
            # # self.move_cam(0)
            # self.flyto(51.42211380709214, -2.6679558911937806, self.init_alt - 50.0 + self.flightHeight) # unexplained correction factor on altitude
            # d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            # d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            # if (abs(d_lon) < 0.0001) & (abs(d_lat) < 0.0001):
            #     self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
            #     return('rotating')
            # elif self.state_timer > 60:
            #     # timeout
            #     self.get_logger().error('Failed to reach target')
            #     return('landing')
            # else:
            #     self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            #     return('on_way')

            # self.flyto(self.global_path[self.path_step][0],self.global_path[self.path_step][1],self.init_alt - 50.0 + self.flightHeight)
            # #self.adjust_heading(self.global_path[self.path_step],self.final_p_global,self.heading)
            # d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            # d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            # if (abs(d_lon) < 0.0001) & (abs(d_lat) < 0.0001):
            #     self.path_step=self.path_step+10
            #     # self.path_step=self.path_step+1
            #     self.get_logger().info('Close enough to path point {}. Distance={},{}'.format(self.path_step,d_lat,d_lon))

            #     if self.path_step>=len(self.global_path):
            #         self.flyto(self.global_path[-1][0],self.global_path[-1][1],self.init_alt - 50.0 + self.flightHeight)
            #         while not((abs(d_lon) < 0.0001) & (abs(d_lat) < 0.0001)):
            #             self.get_logger().info('Distance={},{}'.format(self.path_step,d_lat,d_lon))

            #         self.get_logger().info('Close enough to path point {}. Distance={},{}'.format(self.path_step,d_lat,d_lon))

            #         self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
            #         if self.mission_stage==0:
            #             return('img_p')
            #         elif self.mission_stage==1:
            #             self.path_step=0
            #             return('volcano_landing')
            #         else:
            #             return("landing")

            #     return('on_way')

            # elif self.state_timer > 500:
            #     # timeout
            #     self.get_logger().error('Failed to reach target')
            #     return('landing')
            # else:
            #     self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            #     return('on_way')
        elif self.control_state == 'rotating':
            self.pubulish_flightState('Rotating')    
            angle = atan2((self.volcano_centre[0]-self.last_pos.latitude),(self.volcano_centre[1]-self.last_pos.longitude))-pi/2
            # self.get_logger().info("angle:{}".format(angle))
            if abs(self.heading -angle)>pi/180*10:
                # self.get_logger().info("rotating")
                # self.get_logger().info("self.heading:{}".format(self.heading))
                # self.get_logger().info("Absolute difference:{}".format(abs(self.heading-angle)))
                self.adjust_heading(float(-0.2))
                return('rotating')
            self.get_logger().info("Finished Rotating")
            self.adjust_heading(float(0))
            self.move_cam(60)
            self.hold_position(3)
            return('img_p')

        elif self.control_state=='img_p':
            self.pubulish_flightState('Image_Processing') 
            # self.hold_position(5) #wait 2 seconds
            
            
            if(self.astar_path_near_volcano_in_meters==[]):
                self.img_start()  ## update the points in self.positions_of_points
                self.hold_position(1)
                
                return('img_p')
            else:
                if not self.landing:
                    self.get_logger().info('Waiting to landing confirmation')
                    self.pubulish_flightState('Wait to landing')
                    self.hold_position(1)
                    return('img_p')
                else:
                    # self.get_logger().info("astar_path_near_volcano_in_meters:".format(self.astar_path_near_volcano_in_meters))
                    self.get_logger().info('image processed')
                    self.global_path_to_landing = self.convert_path_global(self.astar_path_near_volcano_in_meters, [self.last_pos.latitude,self.last_pos.longitude],self.heading,[0,0],pr=0)
                    self.move_cam(90)
                    self.hold_position(1)
                    return('volcano_guiding')

        elif self.control_state=='volcano_guiding':
            self.pubulish_flightState('Volcano_Guiding') 
            self.flytoWaypoints(self.global_path_to_landing)
            self.hold_position(5)
            return('volcano_landing')

        elif self.control_state=='volcano_landing':
            self.pubulish_flightState('Volcano_Landing') 
                #    self.last_pos.altitude
            self.hold_position(1)
            self.flyto(self.global_path_to_landing[-1][0]+0.00001,self.global_path_to_landing[-1][1]-0.00001, 106.0)
            self.get_logger().info('Landing request sent. current alt:{}'.format(self.last_alt_rel))        
            if self.last_alt_rel <6.0:
                self.get_logger().info('Close enough to the ground')
                # move drone by sending setpoint message
                return('standBy')
            elif self.state_timer > 500:
                # timeout
                self.pubulish_flightState('Abort') 
                self.get_logger().error('Failed to reach altitude')
                return('abort')
            else:
                self.get_logger().info('Altitude: {}m'.format(self.last_alt_rel))
                return('volcano_landing')

        elif self.control_state == 'standBy':
            
            self.pubulish_flightState('Wait to RTL') 
            if not self.RTL:
                    self.get_logger().info('Waiting to RTL Signal')
                    self.pubulish_flightState('Wait to RTL')
                    self.hold_position(1)
                    return('standBy')
            else:
                self.pubulish_flightState('RTL') 
                # return button pressed
                # self.get_logger().info("num_of_astar_path_near_volcano_in_meters:{}".format(len(self.astar_path_near_volcano_in_meters)))
                return('ReturnToHome')

        elif self.control_state == 'ReturnToHome':
            self.pubulish_flightState('Return_To_Home') 
            # return home and land
            self.get_logger().info('Request sent for RTL mode.')
            self.global_path_to_landing.reverse()
            self.ApathWay.reverse()
            self.flytoWaypoints(self.global_path_to_landing,Condition = True)
            self.move_cam(30)
            self.hold_position(1)
            self.flytoWaypoints(self.ApathWay,Condition = True)
            self.move_cam(90)
            self.hold_position(1)
            self.change_mode("RTL")
            return('exit')
        elif self.control_state == 'abort':
            self.pubulish_flightState('Abort') 
            return('abort')
        elif self.control_state == 'exit':
            self.pubulish_flightState('Finished') 
            # nothing else to do
            return('exit')

    def run(self):
        self.get_logger().info('Flag 3')
        # set up two subscribers, one for vehicle state...
        state_sub = self.create_subscription(State, '/vehicle_1/mavros/state', self.state_callback, 10)
        # ...and the other for global position
        pos_sub = self.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10)

        # local_pos_sub=self.create_subscription(PoseStamped,'/vehicle_1/mavros/local_position/pose',self.local_position_callback,10)
        magnetic_sub=self.create_subscription(Float64,'/vehicle_1/mavros/global_position/compass_hdg',self.heading_callback,10)
        
        #image processor
        img_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10)
        
        #takeoff flag
        takeoff = self.create_subscription(String, '/vehicle_1/stageRelease', self.stageRelease, 10)
        
        for try_loop in range(600):
            if rclpy.ok():
                self.wait_for_new_status()
                if not self.em:
                    if self.abort:
                        self.control_state = 'abort'
                        self.state_transition()
                    else:
                        new_state = self.state_transition()
                        if new_state == self.control_state:
                            self.state_timer = self.state_timer + 1
                        else:
                            self.state_timer = 0
                        self.control_state = new_state
                else:
                    self.currentPath.reverse()
                    self.pubulish_flightState('EMERGENCY') 
                    self.returnHome()
                    self.change_mode("RTL")

                # self.get_logger().info('Controller state: {} for {} steps'.format(self.control_state, self.state_timer))

    class Node_Astar():


        def __init__(self,y,x,obstacle,parent,index_x,index_y):
            self.x = x
            self.y = y
            self.obstacle=obstacle
            self.parent = parent
            self.index_x=index_x
            self.index_y=index_y
            self.id=id(Node)
            self.cost=0
            self.visited=0

    def AStar(self,msg,initial_p,final_p):
        
        nodes=msg

        ref_list=[]
        path_list=[]
        current_node=nodes[initial_p[0]][initial_p[1]]
        current_node.visited=1

        while(current_node!=nodes[final_p[0]][final_p[1]]):
            for lines in range (3):
                for columns in range (3):
                    temp_y=current_node.index_y-1+lines
                    temp_x=current_node.index_x-1+columns

                    if nodes[temp_y][temp_x].obstacle!=2 and nodes[temp_y][temp_x].visited==0:
                        nodes[temp_y][temp_x].cost=((nodes[temp_y][temp_x].x-nodes[initial_p[0]][initial_p[1]].x)**2+
                                                                        (nodes[temp_y][temp_x].y-nodes[initial_p[0]][initial_p[1]].y)**2)**0.5+(abs(nodes[final_p[0]][final_p[1]].x-nodes[temp_y][temp_x].x)+abs(nodes[final_p[0]][final_p[1]].y-nodes[temp_y][temp_x].y))
                        nodes[temp_y][temp_x].parent=[current_node.index_y,current_node.index_x]
                        nodes[temp_y][temp_x].visited=1
                        ref_list.append(nodes[temp_y][temp_x])
                    
            min_cost=ref_list[0].cost
            current_node=ref_list[0]
            for i in range(len(ref_list)):
                if ref_list[i].cost<min_cost:
                    min_cost=ref_list[i].cost
                    current_node=ref_list[i]

            #print("Smallest cost node: ", current_node.index_y,current_node.index_x)

            #for i in range(len(ref_list)):         
            #    print("Remaining List, Line: ", ref_list[i].index_y," Column: ",ref_list[i].index_x, "cost: ","{:.2f}".format(ref_list[i].cost), "Parent: ",ref_list[i].parent) 

            ref_list.remove(current_node)
            #print(" ")

        #print("Arrived at final node")
        count=0
        while(current_node!=nodes[initial_p[0]][initial_p[1]]):
        
            path_list.append((current_node.index_y,current_node.index_x))
            #print(current_node.parent)
            if current_node==nodes[initial_p[0]][initial_p[1]]:
                break
            current_node=nodes[current_node.parent[0]][current_node.parent[1]]
            count=count+1
        #print("Traceback done")
        #for i in range(len(path_list)):
        #    print("Line: ", "{:.2f}".format(path_list[i][0]),"Column: ", "{:.2f}".format(path_list[i][1]))  
       
        erase=0
        current_pos=0
        
        while(True):

            #print("PATH LIST: ",path_list[i][0], ",", path_list[i][1], ",", nodes[path_list[i][0]][path_list[i][1]].obstacle)

            if erase==1:
                del path_list[current_pos+1]
            if(current_pos+2>=len(path_list)):
                break
            y1=path_list[current_pos][0]
            y2=path_list[current_pos+2][0]
            x1=path_list[current_pos][1]
            x2=path_list[current_pos+2][1]
            range_x=x2-x1
            range_y=y2-y1
            if(abs(range_x)>=abs(range_y) and range_x!=0):
                coef_a=range_y/range_x
                coef_b=(y1*x2-x1*y2)/range_x
                tries=abs(range_x)

            elif(range_x==0):
                tries=abs(range_y)

            elif(abs(range_x)<abs(range_y)):
                coef_a=range_x/range_y
                coef_b=(x1*y2-y1*x2)/range_y
                tries=abs(range_y)-1

            erase=1
            for j in range(tries):
                if(abs(range_x)>=abs(range_y) and range_x!=0):
                    x_temp=path_list[current_pos][1]+j+1
                    y_temp=round(int(coef_a*x_temp+coef_b))

                elif(range_x==0):
                    x_temp=x1
                    y_temp=y1+j+1
                elif(abs(range_x)<abs(range_y)):
                    y_temp=path_list[current_pos][0]+j+1
                    x_temp=round(int(coef_a*y_temp+coef_b))

                if nodes[y_temp][x_temp].obstacle==2:
                    erase=0
                    current_pos=current_pos+1
                    break
        

        
        return path_list

    def prepare_msg(self,msg,final_p,lines_num,columns_num):

        rounded_msg=[[round(int(i), 0) for i in nested] for nested in msg]
    

        #max_coordinates=[0,0]
        
        min_coordinates=[-round(int(lines_num/2)),-round(int(columns_num/2))]
        '''''
        min_coordinates=[0,0]
        for i in range(len(rounded_msg)):     
            if (rounded_msg[i][0]<min_coordinates[0]):
                min_coordinates[0]=rounded_msg[i][0]
            if (rounded_msg[i][1]<min_coordinates[1]):
                min_coordinates[1]=rounded_msg[i][1]      
        '''''
        #print("Min: ", min_coordinates)


        for i in range(len(rounded_msg)):
            rounded_msg[i][0]=rounded_msg[i][0]-min_coordinates[0]+1
            rounded_msg[i][1]=rounded_msg[i][1]-min_coordinates[1]+1

        adjusted_initial_p=[-min_coordinates[0]+1,-min_coordinates[1]+1]
        adjusted_final_p=[final_p[0]-min_coordinates[0]+1,final_p[1]-min_coordinates[1]+1]
        
        #Convert into list of tuples
        tpls = [tuple(x) for x in rounded_msg]
        dct = list(dict.fromkeys(tpls))
        #  Convert list of tuples to list of lists
        unique_rounded_msg = [list(x) for x in dct]
        inverted_unique_rounded_msg=[]
        for i in range(len(unique_rounded_msg)):
            inverted_unique_rounded_msg.append([unique_rounded_msg[i][1],unique_rounded_msg[i][0],unique_rounded_msg[i][2]])
        # print("Initial p adjusted:",adjusted_initial_p)
        # print("Final p adjusted:",adjusted_final_p)

        return inverted_unique_rounded_msg,adjusted_initial_p,adjusted_final_p
    
    def plot_path_plan(self,initial_p,final_p,obstacle_coordinates,path,obstacles_dict,lines_num,columns_num):

        path_dots=10*np.array(path)
        initial_dot=10*np.array(initial_p)
        final_dot=10*np.array(final_p)
        obstacles_dots=np.delete(10*np.array(obstacle_coordinates),2,1)

        #coords = [(100,70), (220, 310), (200,200)]

        img = self.Image.new("RGB", (10*lines_num,10*columns_num), color=(200,255,200))
        draw = self.ImageDraw.Draw(img)
        dotSize = 10

        count=0
        for (y,x) in obstacles_dots:         
            a=obstacle_coordinates[count][0]
            b=obstacle_coordinates[count][1]      
            if obstacles_dict.get(((a+b)*(a+b+1))/2+b)==2:
                draw.rectangle([x,y,x+dotSize,y+dotSize], fill="red")
            elif obstacles_dict.get(((a+b)*(a+b+1))/2+b)==1:
                draw.rectangle([x,y,x+dotSize-1,y+dotSize-1], fill="yellow") 
    
            count=count+1

        count=0
        
        for (y,x) in path_dots:
            a=path[count][0]
            b=path[count][1]
            if obstacles_dict.get(((a+b)*(a+b+1))/2+b)==2:
                draw.rectangle([x,y,x+dotSize-1,y+dotSize-1], fill="black")
            elif obstacles_dict.get(((a+b)*(a+b+1))/2+b)==1:
                draw.rectangle([x,y,x+dotSize-1,y+dotSize-1], fill="orange")
            else: 
                draw.rectangle([x,y,x+dotSize-1,y+dotSize-1], fill="blue")	
            count=count+1
        

        draw.rectangle([initial_dot[1],initial_dot[0],initial_dot[1]+dotSize-1,initial_dot[0]+dotSize-1], fill="purple")
        draw.rectangle([final_dot[1],final_dot[0],final_dot[1]+dotSize-1,final_dot[0]+dotSize-1], fill="green")
        img.show()  

    def create_Astar_nodes(self,lines_num,columns_num,obstacles_dict):

        nodes=[]

        for lines in range(lines_num+2):
            horizontal_list=[]

            for columns in range(columns_num+2):

                obstacle_value=obstacles_dict.get(((lines+columns)*(lines+columns+1))/2+columns,0)

                horizontal_list.append(FenswoodDroneController.Node_Astar(lines,columns,obstacle_value,None,columns,lines))
                #print(horizontal_list[columns].y, horizontal_list[columns].x,horizontal_list[columns].obstacle)
            nodes.append(horizontal_list)

        return nodes

    def convert_path_global(self,path,drone_global_coordinate,drone_heading,adjusted_initial_p,pr=1):
        global_path=[]
        drone_heading=drone_heading
        for coordinates in range (len(path)):
            y=path[coordinates][0]-adjusted_initial_p[0]
            x=path[coordinates][1]-adjusted_initial_p[1]
            global_path.append([(math.sin(drone_heading)*x+math.cos(drone_heading)*y)*0.000009000009+drone_global_coordinate[0],(math.cos(drone_heading)*x-math.sin(drone_heading)*y)*0.000009000009+drone_global_coordinate[1]])
        if(pr==0):
            self.get_logger().info("global_path:{}".format(global_path))
        return global_path

    def convert_coordinate_local(self,coordinates,drone_global_coordinate,drone_heading):
        coordinates_local=[]
        y_translated=(coordinates[0]-drone_global_coordinate[0])*111111
        x_translated=(coordinates[1]-drone_global_coordinate[1])*111111
        # self.get_logger().info('y_translated: {}, x_translated: {},drone_heading: '.format(y_translated, x_translated,drone_heading))
        coordinates_local=[round((math.sin(-1*drone_heading)*x_translated+math.cos(-1*drone_heading)*y_translated)),round(math.cos(-1*drone_heading)*x_translated-1*math.sin(-1*drone_heading)*y_translated)]
        
        return coordinates_local
        
    def polygon_obstacles(self,polygon_coordinates,drone_global_coordinate,drone_heading):
        step=1 #distance between obstacles to be obtained from line equation
        list_of_obstacles=[]
        polygon_coordinates_local=[]
        drone_heading=drone_heading
        for count in range (len(polygon_coordinates)):
            y_translated=(polygon_coordinates[count][0]-drone_global_coordinate[0])*111111
            x_translated=(polygon_coordinates[count][1]-drone_global_coordinate[1])*111111
            polygon_coordinates_local.append(((math.sin(-drone_heading)*x_translated+math.cos(-drone_heading)*y_translated),(math.cos(-drone_heading)*x_translated-math.sin(-drone_heading)*y_translated)))
            #list_of_obstacles.append((polygon_coordinates_local[count][1],polygon_coordinates_local[count][0],2))
        for count in range (len(polygon_coordinates_local)):
            x1=polygon_coordinates_local[count][1]
            y1=polygon_coordinates_local[count][0]

            if count==len(polygon_coordinates_local)-1:
                if len(polygon_coordinates_local)<3:
                    break
                index_coordinate=0
            else:
                index_coordinate=count+1

            x2=polygon_coordinates_local[index_coordinate][1]
            y2=polygon_coordinates_local[index_coordinate][0]
            #line equation
            
            a=(y1-y2)/(x1-x2)
            b=(x1*y2-x2*y1)/(x1-x2)
            
            step_x=step/((a**2+1)**0.5)
            x=x1
            y=a*x+b
            list_of_obstacles.append([x,y,2])
            for i in range (int(abs((x2-x1)/step_x))):
                x=x+abs(x2-x1)/(x2-x1)*step_x
                y=a*x+b
                for i in range(2):
                    for j in range(2):
                        list_of_obstacles.append([x+i-1,y+j-1,2])

    # print("a:",list_of_obstacles)

        return list_of_obstacles  

    def path_planning_A(self,msg,final_p,drone_global_coordinate,drone_heading):
    
        initial_time=time.time()

        #for i in range(500): #just creating a line of obstacles
        #    msg.append([460,100+i,2])
        lines_num=1200 #y dimension of grid in meters
        columns_num=1200 #x dimension of grid in meters

        (obstacle_coordinates,adjusted_initial_p,adjusted_final_p)=self.prepare_msg(msg,final_p,lines_num,columns_num)

        obstacles_dict=dict()

        #for (line,column) in ((0,columns_num+1),(lines_num+1,0)):

        #    for a in range(line+1):
        #        for b in range(column+1):
        #            obstacles_dict[((a+b)*(a+b+1))/2+b]=2
        #            obstacle_coordinates.append([a,b,2])
                    #print(a,b,2)

        for i in range(len(obstacle_coordinates)):

            a=obstacle_coordinates[i][0]
            b=obstacle_coordinates[i][1]
            if obstacles_dict.get(((a+b)*(a+b+1))/2+b) in (0,None):
                obstacles_dict[((a+b)*(a+b+1))/2+b]=obstacle_coordinates[i][2]
            elif obstacles_dict.get(((a+b)*(a+b+1))/2+b)==1 and obstacle_coordinates[i][2]==2:
                obstacles_dict[((a+b)*(a+b+1))/2+b]=obstacle_coordinates[i][2]

        '''''   #Security Margin (Work in progress)     
        for i in range(len(obstacle_coordinates)):   
            if obstacle_coordinates[i][2] in (1,2):     
                for m in range (-1,2):
                    for n in range(-1,2):        
                        a=obstacle_coordinates[i][0]+m
                        b=obstacle_coordinates[i][1]+n
                        #print(obstacles_dict.get(((a+b)*(a+b+1))/2+b))
                        if obstacles_dict.get(((a+b)*(a+b+1))/2+b) in (None,0,1):
                            obstacles_dict[((a+b)*(a+b+1))/2+b]=obstacle_coordinates[i][2]
                            print(obstacle_coordinates[i][0]+m,obstacle_coordinates[i][1]+n)
        '''''
                    
        nodes=self.create_Astar_nodes(lines_num,columns_num,obstacles_dict)
        path=self.AStar(nodes,adjusted_initial_p,adjusted_final_p)
        global_path=self.convert_path_global(path,drone_global_coordinate,drone_heading,adjusted_initial_p)


        #self.plot_path_plan(adjusted_initial_p,adjusted_final_p,obstacle_coordinates,path,obstacles_dict,lines_num,columns_num)

        reversed_global_path = list(reversed(global_path))
        elapsed_time=time.time()-initial_time
        self.get_logger().info('Elapsed time: {}s'.format(elapsed_time))

        return reversed_global_path

    def adjust_heading(self,yaw_rate):
        self.angular_velocity.angular.x=0.0
        self.angular_velocity.angular.y=0.0
        self.angular_velocity.angular.z=yaw_rate
        self.angular_velocity.linear.x=0.0
        self.angular_velocity.linear.y=0.0
        self.angular_velocity.linear.z=0.0
        self.target_v.publish(self.angular_velocity)



    ## image processor part
    def img_start(self):
        self.ready=True
        # self.get_logger().info('self.ready:{}'.format(self.ready))
        # set up subscriber for image
        
        # self.destroy_subscription(red_and_yellow_and_center_points_3d)
        
        # img = cv2.imread('51.4219364  _2.6695396 170.442555377.png')
        # red_and_yellow_and_center_points_3d = self.image_callback(msg,img)
        # self.img_return_shape()
        # return red_and_yellow_and_center_points_3d

    def image_callback(self, msg):
        if self.ready:
            
            img = self.br.imgmsg_to_cv2(msg,"bgr8")
            red_and_yellow_and_center_points_3d = []
            self.al_drone = self.last_alt_rel  # subscribe to sw
            self.move_cam(60)
            
            # self.camera_angel = pi/6  # subscribe to sw + shake of attack angel
            point_astar_path = []
            # get points of red and yellow zones in meters, also the center points of red zones for further processing and graphing
            points_r_in_meters, points_y_in_meters, center_red_points_in_meters = self.img_return_shape(img)
            
            # update center point of the area of interest
            red_centers = [x[:2] for x in center_red_points_in_meters]
            self.get_logger().info('red_centers: {}'.format(red_centers))
            if len(red_centers)>=3:
                self.img_cal_center_circle(red_centers)

                # update center points of arc for all possible landing points
                self.img_get_centers_of_arc(red_centers, self.center_point_of_area)
            
                # update the best landding points
                self.img_find_best_landing_point(points_y_in_meters,points_r_in_meters)

                #Astar path planning
                point_astar_path = self.img_get_astar_path(center_red_points_in_meters)
                self.astar_path_near_volcano_in_meters = point_astar_path
                # self.get_logger().info('point_astar_path: {}'.format(point_astar_path))
                self.get_logger().info('astar_path_near_volcano_in_meters: {}'.format(self.astar_path_near_volcano_in_meters))
                self.show_3d_map(points_r_in_meters + points_y_in_meters + center_red_points_in_meters + point_astar_path[0:len(point_astar_path)-1])
        self.ready=False
            # return points_r_in_meters + points_y_in_meters + center_red_points_in_meters + point_astar_path

    def img_get_astar_path(self, red_center_points, r=20):
        img = np.array(([[255] * 480] * 240)).astype('uint8')
        for p in red_center_points:
            img = cv2.circle(img, (int(p[1] * 2 + 240), int(p[0] * 2)), r, 0, cv2.FILLED)  # self.red_threadhold
        # img = cv2.circle(img, (240, 0), 5, 50, cv2.FILLED)  # self.red_threadhold
        # img = cv2.circle(img, (int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2)), 5, 124, cv2.FILLED)  # self.red_threadhold
        # img = np.rot90(img, 2)
        # img = np.flip(img, 1)
        # cv2.imshow("img_contours", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        start_p = [(240, 0), 700, (240, 0)]
        end_p = [(int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2)), 0, (int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2))]
        path = self.img_astar_algrithm(img, start_p, end_p)
        path.insert(0,path[0])
        path = path[::2]
        path.reverse()
        self.get_logger().info("astar path:{}".format(path))
        return path

    def img_astar_algrithm(self, maps, start, end):
        maps_size = np.array(maps)
        hight = maps_size.shape[0]  # ->y
        width = maps_size.shape[1]  # ->x

        # start  = [(60, 50), 700, (60, 50)]  # 'position','cost','parent_node'
        # end = [(630, 420), 0, (630, 420)]

        openlist = []
        closelist = [start]
        step_size = 7
        diagonal_step_size = 3
        while 1:
            s_point = closelist[-1][0]  #获取close列表最后一个点position，S点
            add = ([0, step_size], [0, -step_size], [step_size, 0], [-step_size, 0])  # 8 directions
            # , [diagonal_step_size, diagonal_step_size], [diagonal_step_size, -diagonal_step_size],
            #     [-diagonal_step_size, diagonal_step_size], [-diagonal_step_size, -diagonal_step_size]
            for i in add:
                x = s_point[0] + i[0]
                if x < 0 or x >= width:
                    continue
                y = s_point[1] + i[1]
                if y < 0 or y >= hight:
                    continue
                G = np.sqrt((x - start[0][0])**2 + (y - start[0][1])**2)  #计算cost
                H = np.sqrt((x - end[0][0])**2 + (y - end[0][1])**2)  #计算cost
                F = G + H

                addpoint = [(x, y), F, s_point]  #更新position
                count = 0
                for i in openlist:
                    if i[0] == addpoint[0]:
                        count = 1
                        break
                for i in closelist:
                    if i[0] == addpoint[0]:
                        count = 1
                        break
                if count == 0 and maps[y, x] != 0:  #新增点不在open和close列表中 #非障碍物
                    openlist.append(addpoint)
                if H < step_size / 1.4:  #当逐渐靠近终点时，搜索的步长变小
                    openlist.append([end[0], 0, t_point[2]])
                    break
                # if H < step_size:  #当逐渐靠近终点时，搜索的步长变小
                #     step_size = 1
                #     add = ([0, step_size], [0, -step_size], [step_size, 0], [-step_size, 0])

            t_point = [(240, 0), 10000, (240, 0)]
            for j in range(len(openlist)):
                if openlist[j][1] < t_point[1]:
                    t_point = openlist[j]
            for j in range(len(openlist)):
                if t_point == openlist[j]:
                    openlist.pop(j)
                    break
            closelist.append(t_point)
            if t_point[0] == end[0]:
                print("found the destionation")
                break
        road = []
        road.append(closelist[-1])
        point = road[-1]
        while 1:
            for i in closelist:
                if i[0] == point[2]:
                    point = i
                    road.append(point)
            if point == start:
                print("A-star finished")
                break
        return [(rd[0][1] / 2, (rd[0][0] - 240) / 2, 2) for rd in road]

    def img_print_path(self, maps, road):
        informap = np.array(maps)
        for i in road:
            cv2.circle(informap, i[0], 1, 200, cv2.FILLED)
        cv2.circle(informap, road[0], 5, 100, cv2.FILLED)
        cv2.circle(informap, road[-1], 5, 100, cv2.FILLED)
        informap = np.rot90(informap, 2)
        informap = np.flip(informap, 1)
        cv2.imwrite("/ros_ws/img/informap.png", informap)

    def img_find_best_landing_point(self, points_y, points_r):
        re1 = []
        re2 = []
        # self.get_logger().info('self.centers_of_arc: {}'.format(self.centers_of_arc))
        for c in self.centers_of_arc:
            cy_dmin = 100000000
            cr_dmin = 100000000
            for p in points_y:
                temp = (c[0] - p[0])**2 + (c[1] - p[1])**2
                if (cy_dmin > temp):
                    cy_dmin = temp
            for pr in points_r:
                temp = (c[0] - pr[0])**2 + (c[1] - pr[1])**2
                if (cr_dmin > temp):
                    cr_dmin = temp
            if cy_dmin > 60 and cr_dmin>60:
                dis = c[0]**2 + c[1]**2
                re1.append(c)
                re2.append(dis)
        best = map(re2.index, heapq.nsmallest(1, re2))
        self.landing_point = re1[list(best)[0]]

    def img_return_shape(self, img):
        self.detect_red_num =0
        total_area = 0
        points_r = []
        points_y = []
        center_red_points = []
        threadthold_bgr = [100, 150, 120]
        img_b = img[:, :, 0] < threadthold_bgr[0]
        img_g = img[:, :, 1] > threadthold_bgr[1]
        img_r = img[:, :, 2] > threadthold_bgr[2]
        map_color_r = (img_r & img_b) ^ (img_g & img_b)
        map_color_y = img_r & img_g & img_b
        map_color_r = np.array(map_color_r, dtype='uint8') * 255
        map_color_y = np.array(map_color_y, dtype='uint8') * 255
        contours_r = cv2.findContours(map_color_r, 1, 1)[0]
        contours_y = cv2.findContours(map_color_y, 1, 1)[0]
        for j in contours_y:
            area = cv2.contourArea(j)
            if (area > 10):
                # img_contours = cv2.drawContours(img_contours, [j], -1, (0, 124, 255), cv2.FILLED)  # self.yellow_threadhold
                if len(points_y) == 0:
                    points_y = j.sum(axis=1)
                else:
                    points_y = np.vstack((points_y, j.sum(axis=1)))
        for i in contours_r:
            area = cv2.contourArea(i)
            if (area > 10):
                total_area += area
                # img_contours = cv2.drawContours(img_contours, [i], -1, (0, 0, 255), cv2.FILLED)  # self.red_threadhold
                if len(points_r) == 0:
                    points_r = i.sum(axis=1)
                    center_red_points = self.img_cal_center_points(i)
                else:
                    points_r = np.vstack((points_r, i.sum(axis=1)))
                    center_red_points = np.vstack((center_red_points, self.img_cal_center_points(i)))
                self.detect_red_num += 1
        points_r_after = [self.img_cal_point(np.array(points), 2) for points in points_r]
        points_y_after = [self.img_cal_point(np.array(points), 1) for points in points_y]
        points_center_red_points_after = [self.img_cal_point(np.array(points), 0) for points in center_red_points]
        # cv2.imshow("img_contours", img_contours)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # cv2.imwrite("red_contours.png", img_contours)

        # np.savetxt('new.csv',np.array(map1)*255,delimiter = ',',fmt='%d')
        self.get_logger().info('Image processed with detect_red_num:{},total red area:{}'.format(self.detect_red_num,total_area))
        if(total_area>450):
            return points_r_after, points_y_after, points_center_red_points_after
        else:
            return [],[],[]

    def img_cal_center_circle(self, points):
        x = np.r_[np.array(points)[:, 0]]
        y = np.r_[np.array(points)[:, 1]]
        x_m = np.mean(x)
        y_m = np.mean(y)

        def calc_r(xc, yc):
            return np.sqrt((x - xc)**2 + (y - yc)**2)

        def f_2(c):
            ri = calc_r(*c)
            return ri - ri.mean()

        center_estimate = x_m, y_m
        center_2, _ = optimize.leastsq(f_2, center_estimate)
        self.center_point_of_area = center_2
        xc_2, yc_2 = center_2
        ri_2 = calc_r(xc_2, yc_2)
        self.r = ri_2.mean()

    def img_cal_center_points(self, points):
        red_p = [0, 0]
        for p in points:
            red_p += p
        return red_p / len(points)

    def img_cal_point(self, point, color):
        pixel_y, pixel_x = point
        camera_angel = pi/6
        vertical_angel = self.vertical_angel
        al_drone = self.al_drone
        cmv = camera_angel - vertical_angel / 2

        point_center = (tan(camera_angel) * al_drone, 0, 0)
        point_e = (tan(cmv) * al_drone, 0, 0)
        dis_eh = cos(camera_angel) * (point_center[0] - point_e[0])  # cos*dis_ef
        width_half = dis_eh * 4 / 3

        # calculate point_x_world
        pixel_x_portion = (240 - pixel_x) / 240
        theta_x = atan(pixel_x_portion * tan(vertical_angel / 2))
        point_x_world = tan(theta_x + camera_angel) * al_drone

        # calculate point_y_world
        pixel_y_portion = (480 - pixel_x) / 240
        z_y = pixel_y_portion * sin(camera_angel) * dis_eh
        point_y_max_world = al_drone / (al_drone - z_y) * width_half
        point_y_world_portion = (320 - pixel_y) / 320
        point_y_world = point_y_max_world * point_y_world_portion

        # #four corners world positions
        # p2 = []
        # cpv = camera_angel + vertical_angel/2
        # z_i = 2*sin(camera_angel)*dis_eh
        # ex_width_half = al_drone/(al_drone-z_i)*width_half
        # point_a = (tan(cmv)*al_drone,  width_half, 0)
        # point_b = (tan(cmv)*al_drone,  -width_half, 0)
        # point_c = (tan(cpv)*al_drone,  -ex_width_half, 0)
        # point_d = (tan(cpv)*al_drone,  ex_width_half, 0)
        # p2.append(point_center)
        # p2.append(point_a)
        # p2.append(point_b)
        # p2.append(point_c)
        # p2.append(point_d)
        # p2 = np.array(p2, dtype='double')
        # print(np.array(p2))
        return [point_x_world, -point_y_world, color]

    def img_cal_arc_centers(self, p1, p2, center_point_of_area):
        try:
            x0, y0 = center_point_of_area
            x1, y1 = p1
            x2, y2 = p2
            theta = acos((x1 * x2 - x1 * x0 - x2 * x0 + x0**2 + y1 * y2 - y0 * y2 - y1 * y0 + y0**2) / self.r**2) / 2
            x_1 = x0 + (x1 - x0) * cos(theta) - (y1 - y0) * sin(theta)
            y_1 = y0 + (x1 - x0) * sin(theta) + (y1 - y0) * cos(theta)
            return [x_1, y_1]
        except Exception:
            return None

    def img_get_centers_of_arc(self, red_points, center_point_of_area):
        # self.centers_of_arc = []
        # ps1 = [x for x in red_points if x[0] - center_point_of_area[0] > 0]
        # ps1.sort(key=lambda x: x[1])
        # ps2 = [x for x in red_points if x[0] - center_point_of_area[0] < 0]
        # ps2.sort(key=lambda x: x[1], reverse=True)
        # ps = ps1 + ps2
        # for i in range(len(ps)):
        #     if i == len(ps) - 1:
        #         p = self.img_cal_arc_centers(ps[-1], ps[0], center_point_of_area)
        #         self.centers_of_arc.append(p)
        #         break
        #     else:
        #         p = self.img_cal_arc_centers(ps[i], ps[i + 1], center_point_of_area)
        #         self.centers_of_arc.append(p)
        ps = list(combinations(red_points, 2))
        for i in ps:
            p1 = self.img_cal_arc_centers(i[0], i[1], center_point_of_area)
            p2 = self.img_cal_arc_centers(i[1], i[0], center_point_of_area)
            if (p1 != None and p2 != None):
                self.centers_of_arc.append(p1)
                self.centers_of_arc.append(p2)
    
    def show_3d_map(self, position_xyc):
        position_xy = np.array([x[0:2] for x in position_xyc])
        position_c = np.array([x[2] for x in position_xyc])
        c_x = np.linspace(self.center_point_of_area[1] - self.r, self.center_point_of_area[1] + self.r, 1000)
        c_y1 = np.sqrt(self.r**2 - (c_x - self.center_point_of_area[1])**2) + self.center_point_of_area[0]
        c_y2 = -np.sqrt(self.r**2 - (c_x - self.center_point_of_area[1])**2) + self.center_point_of_area[0]
        map_color = {0: 'black', 1: 'y', 2: 'r'}
        color = list(map(lambda x: map_color[x], position_c))
        plt.figure()
        plt.axes().set_aspect('equal', 'datalim')
        plt.scatter(position_xy[:, 1], position_xy[:, 0], s=10, c=color)
        plt.scatter(np.array(self.centers_of_arc)[:, 1], np.array(self.centers_of_arc)[:, 0], s=10, c='gray')
        plt.scatter(self.center_point_of_area[1], self.center_point_of_area[0], s=10, c='red', marker='o')
        plt.scatter(self.landing_point[1]-1, self.landing_point[0]-1, s=40, c='g')
        plt.scatter(0, 0, s=30, c="navy", marker="o")
        plt.plot(c_x, c_y1, linewidth=20, alpha=0.2, c='blue')
        plt.plot(c_x, c_y2, linewidth=20, alpha=0.2, c='blue')
        plt.annotate("Drone postion", xy=(0, 0), xytext=(-20, 0.1))
        plt.xlabel("Horizontal axis (y) of drone in meters")
        plt.ylabel("Vertical axis (x) of drone in meters")
        plt.title("2D image to 3D position (Altitude = 0)")
        plt.savefig('/ros_ws/img/img.png')
        im = cv2.imread('/ros_ws/img/img.png')
        self.publish_Image(im)


def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.run()


if __name__ == '__main__':
    main()