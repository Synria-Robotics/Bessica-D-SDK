from .beauty_logger import *
from datetime import datetime
from .beauty_logger import hex_print

# 创建统一的日志文件，使用日期时间命名
_log_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
logger = BeautyLogger(log_dir="./logs", log_name=f"bessica_d_sdk_{_log_timestamp}.log", verbose=True, min_level=LogLevel.INFO)
