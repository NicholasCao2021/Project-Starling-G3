import rclpy
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong


class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('simple_class_controller')
        self.last_state = None     # global for last received status message
        self.last_pos = None       # global for last received position message
        self.init_alt = None       # global for global altitude at start
        self.last_alt_rel = None   # global for last altitude relative to start
        self.mode_req = None
        self.mode_cli = None

    # on receiving status message, save it to global
    def state_callback(self,msg):
        self.last_state = msg
        self.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))

    # on receiving positon message, save it to global
    def position_callback(self,msg):
        # determine altitude relative to start
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt
        self.last_pos = msg
        self.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                        msg.longitude,
                                                                        self.last_alt_rel))

    def wait_for_new_status(self):
        """
        Wait for new state message to be received.  These are sent at
        1Hz so calling this is roughly equivalent to one second delay.
        """
        if self.last_state:
            # if had a message before, wait for higher timestamp
            last_stamp = self.last_state.header.stamp.sec
            for try_wait in range(60):
                rclpy.spin_once(self)
                if self.last_state.header.stamp.sec > last_stamp:
                    break
        else:
            # if never had a message, just wait for first one          
            for try_wait in range(60):
                if self.last_state:
                    break
                rclpy.spin_once(self)

    def initialization(self):
        # set up two subscribers, one for vehicle state...
        state_sub = self.create_subscription(State, '/vehicle_1/mavros/state', self.state_callback, 10)
        # ...and the other for global position
        pos_sub = self.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10)

        # first wait for the system status to become 3 "standby"
        # see https://mavlink.io/en/messages/common.html#MAV_STATE
        for try_standby in range(60):
            self.wait_for_new_status()
            if self.last_state.system_status==3:
                self.get_logger().info('Drone ready for flight')
                break 
        # send command to request regular position updates
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(33)  # msg ID for position is 33 \
                                    # https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
        cmd_req.param2 = float(1000000)    # 1000000 micro-second interval : 1Hz rate
        cmd_cli = self.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')
        while not cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command_int service not available, waiting again...')
        future = cmd_cli.call_async(cmd_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response
        self.get_logger().info('Requested position stream')
        # Get mode para info
        self.mode_req = SetMode.Request()

    def changeModeTo(self,mode):
        # now change mode to mode
        self.mode_req.custom_mode = mode
        self.mode_cli = self.create_client(SetMode, '/vehicle_1/mavros/set_mode')
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        future = self.mode_cli.call_async(self.mode_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response
        self.get_logger().info('Request sent for {} mode.'.format(mode))

    def armDrone(self):
        # next, try to arm the drone
        arm_req = CommandBool.Request()
        arm_req.value = True
        arm_cli = self.create_client(CommandBool, '/vehicle_1/mavros/cmd/arming')
        while not arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
        # keep trying until arming detected in state message, or 60 attempts
        for try_arm in range(60):
            future = arm_cli.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future)
            # self.get_logger().info('Arming request sent.')
            self.wait_for_new_status()
            if self.last_state.armed:
                self.get_logger().info('Arming successful')
                # armed - grab init alt for relative working
                if self.last_pos:
                    self.init_alt = self.last_pos.altitude
                break
        else:
            self.get_logger().error('Failed to arm')
    
    def climbTo(self,height):
        # take off and climb to 20.0m at current location
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = height
        takeoff_cli = self.create_client(CommandTOL, '/vehicle_1/mavros/cmd/takeoff')
        while not takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')
        # only call once - seems to work OK
        future = takeoff_cli.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Takeoff request sent.')

        # wait for drone to reach desired altitude, or 60 attempts
        for try_alt in range(60):
            self.wait_for_new_status()
            self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
            if self.last_alt_rel > 19.0:
                self.get_logger().info('Close enough to flight altitude')
                break
    
    def moveTo(self,position):
        # move drone by sending setpoint message
        target_msg = GeoPoseStamped()
        target_msg.pose.position.latitude = position[0]
        target_msg.pose.position.longitude = position[1]
        target_msg.pose.position.altitude = self.init_alt - 30.0 # unexplained correction factor
        target_pub = self.create_publisher(GeoPoseStamped, '/vehicle_1/mavros/setpoint_position/global', 10)
        self.wait_for_new_status() # short delay after creating publisher ensures message not lost
        target_pub.publish(target_msg)
        self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(target_msg.pose.position.latitude,
                                                                            target_msg.pose.position.longitude,
                                                                            target_msg.pose.position.altitude)) 

        # wait for drone to reach desired position, or timeout after 60 attempts
        for try_arrive in range(60):
            self.wait_for_new_status()
            d_lon = self.last_pos.longitude - target_msg.pose.position.longitude
            d_lat = self.last_pos.latitude - target_msg.pose.position.latitude
            self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            if abs(d_lon) < 0.0001:
                if abs(d_lat) < 0.0001:
                    self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                    break

    def run(self):


        
        self.initialization()
        self.changeModeTo("GUIDED")
        self.armDrone()
        self.climbTo(20.0)
        position = [51.423,-2.671]
        self.moveTo(position)
        self.changeModeTo("RTL")


        # # return home and land
        # self.mode_req = SetMode.Request()
        # self.mode_req.custom_mode = "RTL"
        # future = self.mode_cli.call_async(self.mode_req)
        # rclpy.spin_until_future_complete(self, future)    # wait for response
        # self.get_logger().info('Request sent for RTL mode.')
        
        # now just serve out the time until process killed
        while rclpy.ok():
            rclpy.spin_once(self)


def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.run()


if __name__ == '__main__':
    main()