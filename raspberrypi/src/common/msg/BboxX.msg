std_msgs/Header header
uint32 num                  # 数量
float64[] boxes             # [x1, y1, x2, y2, x1, y1, x2, y2, ...]
std_msgs/String[] classes   # 类别
float64[] confidence        # 置信度
float64[] distance          # 距离
float64[] distance_lv       # 距离等级 0:危险 1：太近 2：适中 3：稍远（有影响） 4.太远（影响小） 5.没有影响
