# v1.6 0x0201 机器人性能体系数据
uint8 robot_id # 机器人ID （1~7->红，101~107->蓝）
uint16 current_hp # 血量
bool friend_colcor #友方机器人颜色
# v1.6 0x0202 实时功率热量数据
uint16 shooter_heat # 枪口热量 
uint16 remain_time #剩余时间
uint8 game_process #比赛进程
uint8 hit_warning #受击警告
uint8 base_virtual_shield #基地虚拟护盾
uint8 resurrection #复活
uint16 outpost_hp #己方前哨战
uint16 enemy_outpost_hp #敌方前哨战
uint16 base_hp #己方基地
uint16 enemy_base_hp #敌方基地
uint16 bullets_fired #剩余发弹量
uint16 remaining_energy #剩余能量
#视觉消息
uint8 self_aim #是否识别到 识别到为1
float64 target_x #目标X坐标
float64 target_y #目标Y坐标
#雷达
uint8 radar_set
float64 radar_x
float64 radar_y
#云台手标点
float64 ext_x #手标点X坐标
float64 ext_y #手标点Y坐标
