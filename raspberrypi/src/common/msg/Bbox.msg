std_msgs/Header header
uint32 num                  # 数量
float64[] boxes             # [x1, y1, x2, y2, x1, y1, x2, y2, ...]
std_msgs/String[] classes   # 类别
float64[] confidence        # 置信度