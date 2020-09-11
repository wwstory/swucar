#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from common.msg import Lane
from status import Target

from utils import read_lane_msg, set_twist

class LaneHandler:

    def load_status(self, status, lane_msg):
        status.have_data = True                 # 接收到了数据，置为True
        lanes = read_lane_msg(lane_msg)

        status.lanes = lanes
        status.lane_num = lanes.shape[0]
        
        status.time_perceive['lane'] = rospy.Time.now()   # 记录目标检测到的时间


    def process(self, twist, status):
        '''
        lanes:
            拿到的是，numpy (4, 18, 2)的数据。
            对应4条车道，每条有18个点（索引0~18是从远->近），每个点有2个数分别为坐标(x, y)。
            没有预测到的地方，(x, y)为(0, 0)。
        '''
        lanes = status.lanes        # 从状态中拿出车道数据
        # lanes = lanes[:, 18//4*3:,:]    # 只处理1/4行的点
        l_p, m_p, r_p = 0, 0, 0     # 左中右3点的位置
        for r in range(len(lanes[0])-1, -1, -1):     # 按y轴，行遍历(倒序，从车方向到远离车方向)
            # 判断这一行，有没有2个以上的非0点
            res = lanes[:, r] != 0
            count = sum(res[:, 0] * res[:, 1])
            if count < 2:
                continue
            
            # 认为在中间0.5左右的点，为自己所在的车道
            points = lanes[:, r]    # 获取这行的点坐标
            xs = points[:, 0]   # 获取x坐标集合
            ys = points[:, 1]   # 获取y坐标集合
            lane_on = 0 # 找到所在车道
            for i in range(status.lane_num - 1):
                if xs[i] < 0.5 and xs[i+1] > 0.5:
                    lane_on = i + 1
                    l_p, r_p = xs[i], xs[i+1]
            status.lane_on = lane_on
            break   # 找到该行后，不管其它行了
        
        rospy.loginfo(f'{l_p}, {r_p}')
        l_p, r_p = (l_p-0.5)*2, (r_p-0.5)*2 # 以图像中间为0点
        m_p = (l_p + r_p) / 2   # 找到中间点
        steer_rate = 1.0    # 转向系数
        angle = max(min(- m_p * steer_rate, 1), -1) # 限制值的范围-1.~1.
        twist = set_twist(twist, angle, 1.) # 设置转向角度，速度

        # rospy.loginfo(f'{(0.5 - m_p) * 2 * steer_rate}')
        rospy.loginfo(twist)
        return twist, status