from rclpy.node import Node
from rm_interfaces.msg import VisualDisplayData, VisualInputData
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rm_interfaces.srv import GetDesiredPosX

import threading

from .data_structures import DartParams, DartDisplayParams

class VisualizationNode(Node):
    def __init__(self, update_display):
        super().__init__('visualization_node')

        self.receive_debug_data_topic_name = str(self.declare_parameter('receive_debug_data_topic_name', "/debug/original_data").value)
        self.send_debug_data_topic_name = str(self.declare_parameter('send_debug_data_topic_name', "/debug/processed_data").value)
        self.serial_driver_node_name = str(
            self.declare_parameter(
                'serial_driver_node_name', 
                "serial_driver_node"
            ).value
        )

        self.subscriber = self.create_subscription(
            VisualDisplayData,
            self.receive_debug_data_topic_name,
            self.callback,
            10)
        
        self.publisher = self.create_publisher(
            VisualInputData,
            self.send_debug_data_topic_name,
            10)
        
        self.param_client = self.create_client(
            SetParameters, 
            f'/{self.serial_driver_node_name}/set_parameters')
        
        self.get_desired_pos_x_client = self.create_client(
            GetDesiredPosX,
            f'/{self.serial_driver_node_name}/get_desired_pos_x'
        )

        self.display_params = DartDisplayParams()
        # self.user_input_params = DartParams()

        self.update_display = update_display
        self.desired_pos_x_thread = threading.Thread(target=self.get_initial_desired_pos_x, daemon=True)
        self.desired_pos_x_thread.start()

    def get_initial_desired_pos_x(self):
        """在节点启动时获取desiredPosX"""
        import asyncio

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.fetch_desired_pos_x())
        finally:
            loop.close()

    async def fetch_desired_pos_x(self):
        """异步获取desiredPosX"""
        if not self.get_desired_pos_x_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'GetDesiredPosX service not available for {self.serial_driver_node_name}')
            return
        
        request = GetDesiredPosX.Request()
        try:
            future = self.get_desired_pos_x_client.call_async(request)
            response = await future
            
            if response:
                self.display_params.curr_pos_x = str(response.desired_pos_x)
                self.update_display(self.display_params)
                self.get_logger().info(f'Fetched initial desiredPosX: {response.desired_pos_x}')
        except Exception as e:
            self.get_logger().error(f'Error fetching desiredPosX: {e}')

    async def set_serial_driver_parameter(self, param_name, param_value: int):
        """设置serial_driver_node的参数"""
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Parameter service not available for {self.serial_driver_node_name}')
            return False
        
        # 创建参数设置请求
        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = param_name
        parameter.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=param_value)
        request.parameters = [parameter]
        
        try:
            future = self.param_client.call_async(request)
            # 等待响应
            response = await future
            if response and response.results[0].successful:
                self.get_logger().info(f'Successfully set parameter {param_name} to {param_value}')
                return True
            else:
                self.get_logger().error(f'Failed to set parameter {param_name}: {response.results[0].reason}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error setting parameter: {e}')
            return False

    def callback(self, msg):
        """接受到串口节点发布的话题后，触发的回调"""

        self.get_logger().info(f'Received VisualDisplayData message with {msg} dart_init values')
        if len(msg.dart_init) > 0:
            self.display_params.dart1_init = msg.dart_init[0]
        if len(msg.dart_cur) > 0:
            self.display_params.dart1_cur = msg.dart_cur[0]
        if len(msg.dart_vel) > 0:
            self.display_params.dart1_vel = msg.dart_vel[0]
        
        # 调用更新显示函数
        if self.update_display:
            self.update_display(self.display_params)
        
    def send_input_data(self, input_params: DartParams):
        """发布用户通过ui输入的数据: 发射拉力值"""

        msg = VisualInputData()

        # 处理空字符串或无效输入的情况
        try:
            dart1_power = int(input_params.dart1_power) if input_params.dart1_power else 0
        except ValueError:
            dart1_power = 0
            
        msg.dart_set = [dart1_power]
        self.publisher.publish(msg)