import rclpy
import DR_init
from datetime import datetime
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 50, 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

# class force_sub(Node):
#     def __init__(self):
#         super().__init__("force_sub_node")
#         self.force_sub = self.create_subscription(Float64MultiArray, "dsr01/msg/tool_force", self.force_callback, 10)

#     def force_callback(self, msg):
#         if msg.data[1] >= 3.0:
#             print(msg.data)

# force_node = force_sub()

def main(args=None):
    rclpy.init(args = args)    
    node = rclpy.create_node("jenga", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_AXIS_Y,
            DR_BASE,
            get_current_posx,
            set_digital_output,
            get_digital_input,
            wait,
            parallel_axis,
            get_tool_force,
            get_external_torque,
        )

        from DR_common2 import posx
        
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def release():
        set_digital_output(2, ON)  
        set_digital_output(1, OFF)        
        wait_digital_input(2)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)   
        wait_digital_input(1)

    JReady = [-0.59, -0.56, 58.47, -3.69, 83.47, -0.04]
    set_tool("Tool Weight")
    set_tcp("GripperSA_v1_test_1")

    start_pose = posx([501.05, 90.43, 222.07, 90.00, -90.00, 90.00]) # 3th

    way_point = posx([451.13, 143.48, 480.15, 125.36, -124.93, 109.03])

    way_point_2 = posx([502.64, -116.64, 469.43, 153.77, -155.43, 154.54])

    grip_pose = posx([496.80, -42.97, 229.46, 90.00, 90.00, 90.00])

    can_push = False

    push_power = [6.8, 6.0, 10.0, 4.5, 3.5, 3.0]

    idx = 0

    # movej(JReady, vel = VELOCITY, acc = ACC)
    movel(way_point, vel = VELOCITY, acc = ACC, ref = DR_BASE)

    release()
    
    wait(1)

    grip()

    while rclpy.ok():

        movel(start_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        parallel_axis([0, -1, 0], 2, 0) # parallel tool's z on base y

        wait(3)
        
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100]) # 컴플라이언스 설정
        set_desired_force(fd=[0, -10, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
        # 힘 설정, y축 -10
        push_time = datetime.now()
        while not check_force_condition(DR_AXIS_Y, max=push_power[idx]): # 힘을 받을 때까지 반복 2.65
            #ush_time_2 = datetime.now()
            push_diff = datetime.now() - push_time
            if push_diff.total_seconds() >= 14:
                print("can push")
                can_push = True
                break
            pass
        idx += 1

        # print(get_tool_force(ref = 0))
        # print(get_external_torque())             
            
        release_compliance_ctrl() # 컴플라이언스 해제

        if can_push == True:
            movel(start_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)
            release()
            movel(way_point, vel = VELOCITY, acc = ACC, ref = DR_BASE)
            movel(way_point_2, vel = VELOCITY, acc = ACC, ref = DR_BASE)
            grip_pose[2] = start_pose[2]
            movel(grip_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)
            grip()
            grip_pose[1] -= 100.0
            movel(grip_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)
            movel(way_point_2, vel = VELOCITY, acc = ACC, ref = DR_BASE)                   
            movel(way_point, vel = VELOCITY, acc = ACC, ref = DR_BASE)       
            movej([52.6, 20.33, 43.66, -62.24, 117.7, 5.29], vel = VELOCITY, acc = ACC)                              
            movej([52.6, 20.33, 43.66, -62.24, 117.7, 95.29], vel = VELOCITY, acc = ACC)        
            movej([52.6, 20.33, 43.66, -62.24, 117.7, 5.29], vel = VELOCITY, acc = ACC)                              
            movej([52.6, 20.33, 43.66, -62.24, 117.7, 95.29], vel = VELOCITY, acc = ACC)        
            movej([52.6, 20.33, 43.66, -62.24, 117.7, 5.29], vel = VELOCITY, acc = ACC)                              
            movej([52.6, 20.33, 43.66, -62.24, 117.7, 95.29], vel = VELOCITY, acc = ACC)                                                                  
            movel(posx([482.80, 142.50, 220.29, 137.62, 180.0, 138.19]), vel = VELOCITY, acc = ACC, ref = DR_BASE)            
            release()
            break

        else:
            movel(start_pose, vel = 10, acc = 10, ref = DR_BASE)
            start_pose[2] += 29.0
            if start_pose[2] >= 390.0:
                print("oh no")
                break


    rclpy.shutdown()

# def f_print():
#     force_node = force_sub()
#     rclpy.spin(force_node)
    
if __name__ == "__main__":
    main()



