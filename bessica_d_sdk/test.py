from typing import List, Dict, Tuple, Optional, Union, NamedTuple

class JointState(NamedTuple):
    """关节状态数据结构"""
    angles: List[float]  # 六个关节角度(弧度)
    gripper: float       # 夹爪角度(弧度)
    timestamp: float     # 时间戳(秒)
    # button1: bool        # 按钮1状态
    # button2: bool        # 按钮2状态

JointStateDict = Dict[str, JointState]

states = JointStateDict = {"left_arm": JointState(None, None, None),
                            "right_arm": JointState(None, None, None)}     

angles = 1
states["left_arm"] = JointState(angles,None,None)
a = 1

def main():
    a = [1,2,3]
    b = a * 3
    print(b)

if __name__ == "__main__":
    main()
