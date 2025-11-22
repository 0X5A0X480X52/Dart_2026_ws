
from PyQt5.QtCore import pyqtSignal, QObject
from .data_structures import DartDisplayParams

class DisplayUpdater(QObject):
    """用于线程安全的UI更新"""
    update_signal = pyqtSignal(object)
    
    def __init__(self):
        super().__init__()
        self.update_signal.connect(self._update_ui)
        self.current_params = None
        self.main_ui = None
        
    def update_display(self, display_params: DartDisplayParams):
        """从任何线程调用此方法来更新UI"""
        self.current_params = display_params
        self.update_signal.emit(display_params)
        
    def _update_ui(self, display_params: DartDisplayParams):
        """在主线程中实际更新UI"""
        if self.main_ui and hasattr(self.main_ui, '_update_ui_impl'):
            self.main_ui._update_ui_impl(display_params)