uint8 UAV_Num			# 个体编号
uint8 flag_detect_bricks		# 发现砖堆标志位（++至4，搜索砖堆任务完成）
float64 x_r				# 红色位置x坐标
float64 y_r				# 红色位置y坐标
float64 radius_r	    # 红色区域半径
float64 x_g				# 绿色位置x坐标
float64 y_g				# 绿色位置y坐标
float64 radius_g	    # 绿色区域半径
float64 x_b				# 蓝色位置x坐标
float64 y_b				# 蓝色位置y坐标
float64 radius_b	    # 蓝色区域半径
float64 x_o				# 橙色位置x坐标
float64 y_o				# 橙色位置y坐标
float64 radius_o	    # 橙色区域半径
uint8 flag_detect_L			# 获取建筑位置标志位
float64 x_L_cross			# L型交点位置x坐标
float64 y_L_cross			# L型交点位置y坐标
float64 x_L_1				# L型1位置x坐标
float64 y_L_1				# L型1位置y坐标
float64 x_L_2				# L型2位置x坐标
float64 y_L_2				# L型2位置y坐标
string state				# 当前任务状态
uint8 flag_droped			# 砖块放置完成标志位
float64 x					# 当前本机位置x坐标
float64 y					# 当前本机位置y坐标
float64 z					# 当前本机位置z坐标
float64 vx					# 当前本机速度x方向的大小
float64 vy					# 当前本机速度y方向的大小
float64 vz					# 当前本机速度z方向的大小