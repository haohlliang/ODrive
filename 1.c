# 配置电机0控制模式，为位置闭环控制
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis1.controller.config.vel_limit = 60
odrv0.axis1.controller.config.vel_limit_tolerance = 1000

# 配置电机0最大转速（转/秒）（电机kv值 * 电压 / 60）
odrv0.axis1.controller.config.vel_limit = 460

# 配置位置环增益：20
odrv0.axis1.controller.config.pos_gain = 20

# 配置速度环增益：0.05
odrv0.axis1.controller.config.vel_gain = 0.025

# 配置积分增益：0.02
odrv0.axis1.controller.config.vel_integrator_gain = 0.01

# 配置输入模式为：梯形轨迹模式
odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ

# 配置梯形模式下的电机转速阈值（转/秒）
odrv0.axis1.trap_traj.config.vel_limit = 50

# 配置梯形运动模式下的加速加速度
# 数值大小影响动作跟随效果，大则跟随快；小则跟随慢。
odrv0.axis1.trap_traj.config.accel_limit = 10

# 配置梯形运动模式下的减速加速度
# 数值大小影响动作跟随效果，大则跟随快；小则跟随慢。
odrv0.axis1.trap_traj.config.decel_limit = 10

# 配置电机0的电流采样阈值（A）
odrv0.axis1.motor.config.requested_current_range = 10

# 配置电机0极对数，根据博客开篇的介绍，自己去数磁极个数，然后/2
odrv0.axis1.motor.config.pole_pairs = 7

# 配置电机0的限制电流（A）（根据自己电机的额定电流更改）
odrv0.axis1.motor.config.current_lim = 1

# 配置电机0校准时的电流阈值（根据自己电机的负载状况酌情配置）（A）
odrv0.axis1.motor.config.calibration_current = 1

# 配置电机0类型。
# 目前支持两种电机：大电流电机（MOTOR_TYPE_HIGH_CURRENT）和云台电机（MOTOR_TYPE_GIMBAL）
odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_GIMBAL

# 保存参数
odrv0.save_configuration()

# 重启驱动器
odrv0.reboot()

##############################################################################

# 配置电机0编码器类型。当前使用的是ABI正交（增量）编码器。
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL

# 配置电机0编码器CPR（每转一圈，编码器的计数），为编码器线束*4，博客开篇有讲
odrv0.axis1.encoder.config.cpr = 4000

# 保存参数
odrv0.save_configuration()

# 进行电机参数和编码器校准（运行后电机会发出哔~的一声，并电机会正转一圈再反转一圈）
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# 设置电机预校准。（不用每次上电都哔~的一声）
# 驱动器会将本次校准值保存，避免上电启动后自动校准，以加快启动速度。
odrv0.axis1.motor.config.pre_calibrated = True

# 配置电机为闭环模式
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# 设置ODrive上电启动时，自动校准编码器
odrv0.axis1.config.startup_encoder_offset_calibration = True

# 设置ODrive上电启动时，自动进入闭环模式
odrv0.axis1.config.startup_closed_loop_control = True

# 保存参数
odrv0.save_configuration()

# 重启驱动器
odrv0.reboot()

##############################################################################

# 配置电机0编码器类型。ENCODER_MODE_INCREMENTAL 使用的是ABI正交（增量）编码器。
# 值 ENCODER_MODE_SPI_ABS_AMS 使用AMS磁编码器-AS5047/AS5048。
odrv0.axis1.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS

# SPI是绝对值编码器，没有索引信号，无须索引校准
odrv0.axis1.encoder.config.use_index = False

# 设置CSn片选的引脚，ODrive J3接口GPIO3-8任选一个作为CS，这里我使用GPIO4
odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 8

# 配置电机0编码器CPR（每转一圈，编码器的计数），AS5047P的最大分辨率为14位
odrv0.axis1.encoder.config.cpr = 2**14

# 编码器带宽设置，CPR值越高带宽设置的也越高
odrv0.axis1.encoder.config.bandwidth = 3000

# 编码器精度，类型为 [float]，单位为 [圆周角度∠] （这个值可以适当的大一些，避免环境干扰）
# 电机实际转动角度和开环移动距离之间允许的最大误差，超过此误差将报错ERROR_CPR_OUT_OF_RANGE。
odrv0.axis1.encoder.config.calib_range = 10

# 保存参数
odrv0.save_configuration()

# 进行电机参数校准（运行后电机会发出哔~的一声）
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION

# 设置电机预校准。（不用每次上电都哔~的一声）
# 驱动器会将本次校准值保存，避免上电启动后自动校准，以加快启动速度。
odrv0.axis1.motor.config.pre_calibrated = True

# 进行编码器校准（运行后，电机会正转一圈再反转一圈）
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

# 查看错误，如果为0，则为无错。否则请断电后重启，重试校准。
odrv0.axis1.error

# 配置电机为闭环模式
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# 控制电机运行到10圈的位置
odrv0.axis1.controller.input_pos = 10

# 设置编码器预校准。（不用每次上电都右转一圈又左转一圈）
# 驱动器会将本次校准值保存，避免上电启动后自动校准，以加快启动速度。
odrv0.axis1.encoder.config.pre_calibrated = True

# 关闭ODrive上电启动时，自动校准编码器
odrv0.axis1.config.startup_encoder_offset_calibration = False

# 设置ODrive上电启动时，自动校准编码器
# odrv0.axis1.config.startup_encoder_offset_calibration = True

# 设置ODrive上电启动时，自动进入闭环模式
odrv0.axis1.config.startup_closed_loop_control = True

# 保存参数
odrv0.save_configuration()

# 重启驱动器
odrv0.reboot()