"""
Very simple script-based sequencer using the old school examples
from https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber
"""
import rclpy                                                    # type: ignore

# import message definitions for receiving status and position
from mavros_msgs.msg import State                               # type: ignore
from sensor_msgs.msg import NavSatFix                           # type: ignore
# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped                  # type: ignore

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong    # type: ignore

g_node = None           # global for the node handle
g_last_state = None     # global for last received status message
g_last_pos = None       # global for last received position message
g_init_alt = None       # global for global altitude at start
g_last_alt_rel = None   # global for last altitude relative to start

# on receiving status message, save it to global
def state_callback(msg):
    global g_last_state
    g_last_state = msg
    g_node.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))


# on receiving positon message, save it to global
def position_callback(msg):
    global g_last_pos, g_last_alt_rel
    # determine altitude relative to start
    if g_init_alt:
        g_last_alt_rel = msg.altitude - g_init_alt
    g_last_pos = msg
    g_node.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                    msg.longitude,
                                                                    g_last_alt_rel))


def wait_for_new_status():
    """
    Wait for new state message to be received.  These are sent at
    1Hz so calling this is roughly equivalent to one second delay.
    """
    if g_last_state:
        # if had a message before, wait for higher timestamp
        last_stamp = g_last_state.header.stamp.sec
        for try_wait in range(60):
            rclpy.spin_once(g_node)
            if g_last_state.header.stamp.sec > last_stamp:
                break
    else:
        # if never had a message, just wait for first one          
        for try_wait in range(60):
            if g_last_state:
                break
            rclpy.spin_once(g_node)


def main(args=None):
    global g_node, g_init_alt
    
    rclpy.init(args=args)

    g_node = rclpy.create_node('old_school_controller')

    # set up two subscribers, one for vehicle state...
    state_sub = g_node.create_subscription(State, '/vehicle_1/mavros/state', state_callback, 10)

    # ...and the other for global position
    pos_sub = g_node.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', position_callback, 10)

    # first wait for the system status to become 3 "standby"
    # see https://mavlink.io/en/messages/common.html#MAV_STATE
    for try_standby in range(60):
        wait_for_new_status()
        if g_last_state.system_status==3:
            g_node.get_logger().info('Drone ready for flight')
            break 

    # send command to request regular position updates
    cmd_req = CommandLong.Request()
    cmd_req.command = 511
    cmd_req.param1 = float(33)  # msg ID for position is 33 \
                                # https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
    cmd_req.param2 = float(1000000)    # 1000000 micro-second interval : 1Hz rate
    cmd_cli = g_node.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')
    while not cmd_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('command_int service not available, waiting again...')
    future = cmd_cli.call_async(cmd_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Requested position stream')

    # now change mode to GUIDED
    mode_req = SetMode.Request()
    mode_req.custom_mode = "GUIDED"
    mode_cli = g_node.create_client(SetMode, '/vehicle_1/mavros/set_mode')
    while not mode_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('set_mode service not available, waiting again...')
    future = mode_cli.call_async(mode_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Request sent for GUIDED mode.')
    
    # next, try to arm the drone
    arm_req = CommandBool.Request()
    arm_req.value = True
    arm_cli = g_node.create_client(CommandBool, '/vehicle_1/mavros/cmd/arming')
    while not arm_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('arming service not available, waiting again...')
    # keep trying until arming detected in state message, or 60 attempts
    for try_arm in range(60):
        future = arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(g_node, future)
        g_node.get_logger().info('Arming request sent.')
        wait_for_new_status()
        if g_last_state.armed:
            g_node.get_logger().info('Arming successful')
            # armed - grab init alt for relative working
            if g_last_pos:
                g_init_alt = g_last_pos.altitude
            break
    else:
        g_node.get_logger().error('Failed to arm')

    # take off and climb to 20.0m at current location
    takeoff_req = CommandTOL.Request()
    takeoff_req.altitude = 20.0
    takeoff_cli = g_node.create_client(CommandTOL, '/vehicle_1/mavros/cmd/takeoff')
    while not takeoff_cli.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().info('takeoff service not available, waiting again...')
    # only call once - seems to work OK
    future = takeoff_cli.call_async(takeoff_req)
    rclpy.spin_until_future_complete(g_node, future)
    g_node.get_logger().info('Takeoff request sent.')

    # wait for drone to reach desired altitude, or 60 attempts
    for try_alt in range(600):
        wait_for_new_status()
        g_node.get_logger().info('Climbing, altitude {}m'.format(g_last_alt_rel))
        if g_last_alt_rel > 19.0:
            g_node.get_logger().info('Close enough to flight altitude')
            break

    # move drone by sending setpoint message
    target_msg = GeoPoseStamped()
    target_msg.pose.position.latitude = 51.423
    target_msg.pose.position.longitude = -2.671
    target_msg.pose.position.altitude = g_init_alt - 30.0 # unexplained correction factor
    target_pub = g_node.create_publisher(GeoPoseStamped, '/vehicle_1/mavros/setpoint_position/global', 10)
    wait_for_new_status() # short delay after creating publisher ensures message not lost
    target_pub.publish(target_msg)
    g_node.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(target_msg.pose.position.latitude,
                                                                           target_msg.pose.position.longitude,
                                                                           target_msg.pose.position.altitude)) 

    # wait for drone to reach desired position, or timeout after 60 attempts
    for try_arrive in range(600):
        wait_for_new_status()
        d_lon = g_last_pos.longitude - target_msg.pose.position.longitude
        d_lat = g_last_pos.latitude - target_msg.pose.position.latitude
        g_node.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
        if abs(d_lon) < 0.0001:
            if abs(d_lat) < 0.0001:
                g_node.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                break

    # return home and land
    mode_req.custom_mode = "RTL"
    future = mode_cli.call_async(mode_req)
    rclpy.spin_until_future_complete(g_node, future)    # wait for response
    g_node.get_logger().info('Request sent for RTL mode.')
    
    # now just serve out the time until process killed
    while rclpy.ok():
        rclpy.spin_once(g_node)


if __name__ == '__main__':
    
    main()