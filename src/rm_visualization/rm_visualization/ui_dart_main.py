from PyQt5 import QtWidgets, uic
import sys, os
from PyQt5.QtWidgets import QLineEdit, QInputDialog
from .visualization_node import VisualizationNode
import rclpy, threading
from PyQt5.QtCore import QTimer
from .data_structures import DartParams, DartDisplayParams
from .DisplayUpdater import DisplayUpdater

class DartUI:
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        ui_file_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'DartV1.0.ui')
        ui_file_path = os.path.abspath(ui_file_path)
        self.ui = uic.loadUi(ui_file_path)
        
        self.is_locked: bool = False
        self.locked_dart1_init_value: str = ""

        self.input_params = DartParams()

        # 创建线程安全的更新器
        self.display_updater = DisplayUpdater()
        self.display_updater.main_ui = self # type: ignore

        self.ros_node: VisualizationNode | None = None
        self.ros_thread: threading.Thread | None = None

        if self.ui is not None:
            self.init_ros_node()
            self.setup_connections()
            self.bind_display_fields()
        else:
            print("无法加载UI文件")

    def lock_data(self):
        """锁定/解锁 Dart1_Init 显示值"""
        self.is_locked = not self.is_locked
        if self.ui and hasattr(self.ui, 'lockButton'):
            if self.is_locked:
                # 锁定时保存当前值并更新按钮文字
                self.locked_dart1_init_value = self.ui.Dart1_Init.text()
                self.ui.lockButton.setText("取消锁定")
            else:
                # 解锁时恢复按钮文字
                self.ui.lockButton.setText("锁定")
    
    def init_ros_node(self):
        """初始化ROS2节点"""
        def spin_ros_node():
            rclpy.init()
            self.ros_node = VisualizationNode(self.display_updater.update_display)
            
            try:
                rclpy.spin(self.ros_node)
            finally:
                self.ros_node.destroy_node()
                rclpy.shutdown()

        self.ros_thread = threading.Thread(target=spin_ros_node, daemon=True)
        self.ros_thread.start()

    def send_dart_params(self):
        if self.ros_node:
            self.ros_node.send_input_data(self.input_params)
            print("数据已发送!")
        else:
            print("ROS节点未初始化!")

    def run(self):
        if self.ui:
            self.ui.show()
        try:
            sys.exit(self.app.exec_())
        finally:
            # 清理ROS资源
            if self.ros_node:
                self.ros_node.destroy_node()
            rclpy.shutdown()
    def bind_display_fields(self):
        """绑定显示参数到对应的UI控件"""
        if self.ui:
            self.display_bindings = {
                'dart1_init': self.ui.Dart1_Init,
                'dart1_cur': self.ui.Dart1_Cur,
                'dart1_vel': self.ui.Dart1_Vel,
                'curr_pos_x': self.ui.currPosX,
            }

            # 初始化UI字段为空字符串
            for _, widget in self.display_bindings.items():
                widget.setText("")
    
    def _update_ui_impl(self, display_params: DartDisplayParams):
        """实际更新UI的实现（在主线程中执行）"""
        if not display_params:
            return
            
        print(f"更新UI显示: dart1_init={display_params.dart1_init}, dart1_cur={display_params.dart1_cur}, curr_pos_x={display_params.curr_pos_x}")
        
        for attr_name, widget in self.display_bindings.items():
            if hasattr(display_params, attr_name):
                value = getattr(display_params, attr_name)
                
                # 处理锁定逻辑
                if attr_name == 'dart1_init' and self.is_locked:
                    value = self.locked_dart1_init_value

                if value is not None:
                    widget.setText(str(value))
                else:
                    widget.setText("")
    
    def setup_connections(self):
        """设置信号与槽的连接"""
        if self.ui:
            if hasattr(self.ui, 'Dart1_Set'):
                self.ui.Dart1_Set.returnPressed.connect(
                    lambda: self.set_shoot_power(1, self.ui.Dart1_Set) # type: ignore
                )

            if hasattr(self.ui, 'sendButton'):
                self.ui.sendButton.clicked.connect(self.send_dart_params)

            if hasattr(self.ui, 'lockButton'):
                self.ui.lockButton.clicked.connect(self.lock_data)

            if hasattr(self.ui, 'applyNewPosXButton'):
                self.ui.applyNewPosXButton.clicked.connect(self.apply_new_pos_x)

    def apply_new_pos_x(self):
        """处理desiredPosX参数变化"""
        if self.ui:
            new_value = self.ui.setNewPosX.text()
            if new_value and self.ros_node:
                try:
                    # 验证输入是否为数字
                    int(new_value)
                    self.input_params.desiredPosX = str(new_value)
                    
                    # 更新显示
                    self.ros_node.display_params.curr_pos_x = str(new_value)
                    self._update_ui_impl(self.ros_node.display_params)
                    
                    print(f"设置 desiredPosX: {new_value}")

                    # 立即发送到ROS节点
                    self.send_parameter_to_ros("desiredPosX", int(new_value))
                
                except ValueError:
                    print("请输入有效的数字")

    def send_parameter_to_ros(self, param_name, param_value: int):
        """发送参数到ROS节点"""
        if self.ros_node:
            # 在新线程中执行异步操作
            def set_param():
                import asyncio
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    loop.run_until_complete(
                        self.ros_node.set_serial_driver_parameter(param_name, param_value) # type: ignore
                    )
                finally:
                    loop.close()
            
            param_thread = threading.Thread(target=set_param, daemon=True)
            param_thread.start()
    def set_shoot_power(self, dart_num: int, dart_set_widget: QLineEdit):
        """设置飞镖发射拉力值"""
        prompt_text = f"这是飞镖{dart_num}"
        if self.ui and self.ros_node:
            param, ok = QInputDialog.getText(
                self.ui.gridLayoutWidget, 
                "请设置发射拉力值", 
                prompt_text, 
                QLineEdit.EchoMode.Normal, 
                ""
            )
        
            if ok and param:
                dart_set_widget.setText(param)

                # 将参数存储到输入参数结构体中
                if dart_num == 1:
                    self.input_params.dart1_power = param
            
                # 打印当前参数结构体的内容（用于验证）
                print(f"输入参数: {self.input_params}")
                print(f"显示参数: {self.ros_node.display_params}")

if __name__ == "__main__":
    dart_ui = DartUI()
    dart_ui.run()

def main():
    dart_ui = DartUI()
    dart_ui.run()