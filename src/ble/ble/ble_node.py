import rclpy
from rclpy.node import Node
from bluezero import adapter
from bluezero import dbus_tools
from bluezero import device
from bluezero import peripheral
import struct
import json
from bluezero import async_tools
import threading
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import BatteryStatus
from px4_msgs.msg import VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleCommand
from rclpy.clock import Clock
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from action_geoland_interfaces.action import GeoLand
from geometry_msgs.msg import PoseStamped

SERVICE_UUID =  "1234"
JOYSTICK_CHR_UUID =  "00008000-0001-11E1-AC36-0002A5D5C51B"
BATTERY_CHR_UUID = "00080000-0001-11E1-AC36-0002A5D5C51B"
STATUS_CHR_UUID = "00800000-0001-11E1-AC36-0002A5D5C51B"
TASK_CHR_UUID = "01000000-0001-11E1-AC36-0002A5D5C51B"

# add advertisement on peripheral.py  def _create_advertisement(self)
# int_array = [int(x, 16) for x in self.address.split(':')]
# hex_array = [0x01,0x88,0x80,0x00] 
# self.advert.manufacturer_data(0x8001,hex_array + int_array)


def on_connect(ble_device: device.Device):
    print("Connected to " + str(ble_device.address))
def on_disconnect(adapter_address, device_address):
    print("Disconnected from " + device_address)



class ble_node(Node):
    def __init__(self):
        super().__init__("ble_node")
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.power_voltage_v = 0
        self.arm_state = False
        self.offboard_message = True
        self.connect_state = False
        self.background_thread = threading.Thread(target=self.create_ble)
        self.background_thread.daemon = True
        self.background_thread.start()
        self.flightCheck = False
        self.failsafe = False
        self.arm_message = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.time_alive = self.get_clock().now().nanoseconds // 1e6
        self.pose_alive = self.get_clock().now().nanoseconds // 1e6
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.state = ""
        self.version1 = 1
        self.version2 = 0
        self.version3 = 1
        self.battery_state = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.vehicle_battery_callback,
            qos_profile)
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.status_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_pose_callback,
            qos_profile)
        self.publisher_manual_control = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_pub = self.create_publisher(Bool, '/offboard_message', qos_profile)
        self._action_client = ActionClient(self, GeoLand, 'geoland')
        #self.create_ble()

    def send_goal(self, goal_msg):
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = "Rejected"
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self.state = "Accepted"
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        if(result.message == "landed"):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.state = "Landed"
        if(result.message == "Canceled"):
            self.state = "Canceled"
        self.get_logger().info('Result: {0}'.format(result.message))
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.state = feedback.current_step
        self.get_logger().info('Received feedback: {0} {1}'.format(feedback.current_distance, feedback.current_step))
    
    def timer_callback(self):
        delta_ms = self.get_clock().now().nanoseconds // 1e6 - self.time_alive
        delta_pose_ms = self.get_clock().now().nanoseconds // 1e6 - self.pose_alive
        #print(f"{delta_ms}")
        if delta_ms > 15000:
            self.nav_state = 31
            self.power_voltage_v = 0
        if delta_pose_ms > 5000:
            self.x = 0
            self.y = 0
            self.z = 0
    
    def vehicle_battery_callback(self, msg):
        self.power_voltage_v = msg.voltage_v

    def state_callback(self, msg):
        #self.get_logger().info(msg)
        self.arm_state = msg.armed
        self.connect_state = msg.connected
        self.nav_mode = msg.mode
    
        #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass
        self.time_alive = self.get_clock().now().nanoseconds // 1e6
    
    def set_mode(self, mode):
        mode_raw =int.from_bytes(mode,"big")
        match (mode_raw):
            case 2:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0)
                print("Position Mode")
            case 4:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                print("hold Mode")
            case 5:
                print("RTL Mode")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            case 14:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                print("Offboard Mode")
            case 15:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 7.0)
                print("Stablilized Mode")
            case 17:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0)
                print("Take off Mode")
            case 18:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                print("Land Mode")
            case _:
                print("Unknown Mode")

    def on_write(self,value, options):
        lastCommand = value[16:17]
        mode = value[17:18]
        if lastCommand == bytearray(b'\x02'):
                self.arm()
                return
        if lastCommand == bytearray(b'\x01'):
                self.disarm()
                return
        if lastCommand == bytearray(b'\x05'):
            msg = Bool()
            msg.data = self.offboard_message
            self.offboard_pub.publish(msg)
        if(mode != bytearray(b'\x00')):
            self.set_mode(mode)
            return
        yaw = struct.unpack('<f',value[0:4])[0]
        power = struct.unpack('<f',value[4:8])[0]
        roll = struct.unpack('<f',value[8:12])[0]
        pitch = struct.unpack('<f',value[12:16])[0]
        msg = ManualControlSetpoint()
        msg.valid = True
        msg.data_source = 2
        msg.sticks_moving = True
        msg.buttons = 0
        msg.timestamp = self.get_clock().now().nanoseconds // 100000
        msg.roll = roll
        msg.pitch = pitch
        msg.throttle = power
        msg.yaw = yaw
        self.publisher_manual_control.publish(msg)
        #self.get_logger().info(f"ble yaw: {yaw} power: {power} roll: {roll} pitch: {pitch}")

    def on_task_write(self, value, options):
        print(f"task:{value.hex()}")
        obj = json.loads(value.decode('utf-8'))
        goal = GeoLand.Goal()
        goal.latitude = float(obj['lat'])
        goal.longitude = float(obj['lon'])
        goal.altitude = float(obj['alt'])
        goal.cmd = int(obj['cmd'])
        self.send_goal(goal)
        print(obj)

    def update_value(self, characteristic):
         power = round(self.power_voltage_v, 1) * 10
         power_hex = int(power).to_bytes(4,byteorder='big')
         characteristic.set_value(power_hex)
         return characteristic.is_notifying
    def on_notify(self,notifying, characteristic):
         if notifying:
            async_tools.add_timer_seconds(2, self.update_value, characteristic)
    def on_status_notify(self,notifying, characteristic):
         if notifying:
            async_tools.add_timer_seconds(1, self.update_status_value, characteristic)
    

    def update_status_value(self,characteristic):
        state_hex = self.state.encode('utf-8')
        packed_data = struct.pack('>B B B f f f B', self.nav_state, self.flightCheck, self.arm_state, self.x, self.y, self.z, len(state_hex))
        merge_hex = packed_data + state_hex
        #self.get_logger().info('state: {0}'.format(merge_hex.hex()))
        characteristic.set_value(merge_hex)
        return characteristic.is_notifying

        #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

        # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")
    
    def target_pose_callback(self, msg):
        self.x = round(msg.pose.position.x,2)
        self.y = round(msg.pose.position.y,2)
        self.z = round(msg.pose.position.z,2)
        self.pose_alive = self.get_clock().now().nanoseconds // 1e6
        # print(f"x:{self.x} y:{self.y} z:{self.z}")

    def read_version(self):
        #version_hex = self.version.encode('utf-8')
        ver1 = int(self.version1).to_bytes(2,byteorder='big')
        ver2 = int(self.version2).to_bytes(2,byteorder='big')
        ver3 = int(self.version3).to_bytes(2,byteorder='big')
        version_hex = ver1 + ver2 + ver3
        return version_hex

    def create_ble(self):
        adapter_address = list(adapter.Adapter.available())[0].address
        ble = peripheral.Peripheral(adapter_address,local_name="ros2")
        ble.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

        ble.add_characteristic(
            srv_id=1,
            chr_id=1,
            uuid=JOYSTICK_CHR_UUID,
            value=[],
            notifying=False,
            flags=["write-without-response"],
            write_callback=self.on_write
        )

        ble.add_characteristic(
            srv_id=1,
            chr_id=2,
            uuid=BATTERY_CHR_UUID,
            value=[],
            notifying=False,
            flags=["notify"],
            #read_callback=cb_with_options,
            notify_callback=self.on_notify
        )

        ble.add_characteristic(
            srv_id=1,
            chr_id=3,
            uuid=STATUS_CHR_UUID,
            value=[],
            notifying=False,
            flags=["notify"],
            notify_callback=self.on_status_notify
        )

        ble.add_characteristic(
            srv_id=1,
            chr_id=4,
            uuid=TASK_CHR_UUID,
            value=[],
            notifying=False,
            flags=["write","read"],
            read_callback=self.read_version,
            #read_callback=cb_with_options,
            write_callback=self.on_task_write,
            #notify_callback=self.on_task_notify
        )

        ble.on_connect = on_connect
        ble.on_disconnect = on_disconnect
        ble.publish()

def main(args=None):
    rclpy.init(args=args)
    my_ble_node = ble_node()
    rclpy.spin(my_ble_node)
    my_ble_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
