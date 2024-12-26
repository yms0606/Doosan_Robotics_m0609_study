import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 700, 700

# Control CONSTANT
CUP_DIAMETER_X = 80             # CUP x좌표 가중치
CUP_DIAMETER_Y = 73             # CUP y좌표 가중치
HEIGHT_PLACE_OFFEST = 30        # CUP 이동시 충돌 방지를 위한 OFFSET
HEIGHT_TARGET_OFFSET = 20       # CUP 회전시 충돌 방지를 위한 OFFSET
MAX_FORCE = 8

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

spline_list = [] # movesx를 위한 list

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cup_stacking", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_digital_output,            
            set_desired_force,
            get_digital_input,               
            get_current_posx,
            DR_FC_MOD_REL,      
            DR_AXIS_Z,            
            set_tool,
            set_tcp,
            DR_BASE,    
            movesx,                       
            movej,
            movel,
            wait,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    # init pose
    JReady = [0, 0, 90, 0, 90, 0]


    # 컵 쌓여있는 위치
    world_cup_pose = posx([420.50, -209.33, 246.11, 0.0, 180.0, 0.0])
    # 뒤집는 컵 위치
    last_cup_grip_pose = posx([421.75, -179.06, 100.73, 88.28, 90.0, 90.0])


    # 시작 좌표
    place_start_pose = posx([561.52, 221.63, 85.13, 0.0, 180.0, 0.0])
    fourth_floor_start_pose = posx([561.52, 221.63, 306.26, 95.25, 90.0, -90.0])
    
    floor_list = [
        [(0, 0), (1.0, 0), (2.0, 0), (0.5, 1.0), (1.5, 1.0), (1.0, 2.0)],
        [(0.5, 0.5), (1.5, 0.5), (1.0, 1.5)],
        [(1.0,1.0)]
    ]

    fourth_floor_list = (1.0,0.6)

    floor_z = [
        85.13,
        178.1,
        272.07
    ]

    # 특이점 탈출 좌표
    out_of_sing = ([-44.23, 26.22, 89.20, 63.99, 98.94, -28.57])
    
    # 컴플라이언스
    compliance_ctrl_stx = [500, 500, 500, 100, 100, 100]
    fd_z_minus = [0, 0, -80, 0, 0, 0]
    fd_dir_z = [0, 0, 1, 0, 0, 0]



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

    def world_cup():

        movel(world_cup_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)
        grip()        

        task_compliance_ctrl(stx=compliance_ctrl_stx)
        set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass

        pos = get_current_posx()[0]

        release_compliance_ctrl()

        pos[2] += 10
        movel(pos, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        world_cup_z = get_current_posx()[0][2]
        world_cup_pose[2] = world_cup_z        

        release()

        pos[2] -= 18
        movel(pos, vel = VELOCITY, acc = ACC, ref = DR_BASE)
        grip()

        pos_2 = get_current_posx()[0]
        pos_2[2] += 125 # 125

        global spline_list
        spline_list.append(posx(pos_2))
   
    def stacking(start_pose,pos_z,coordinate):
        global spline_list

        for xy in coordinate:
            world_cup()        

            pose = start_pose.copy()

            pose[2] = pos_z
            pose[0] -= CUP_DIAMETER_X * xy[0]
            pose[1] -= CUP_DIAMETER_Y * xy[1]
            pose[2] += HEIGHT_PLACE_OFFEST

            spline_list.append(posx(pose))
            pose[2] -= HEIGHT_PLACE_OFFEST
            spline_list.append(posx(pose))

            movesx(spline_list, vel = VELOCITY, acc = ACC, ref = DR_BASE)

            spline_list = []

            task_compliance_ctrl(stx=compliance_ctrl_stx)
            set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=MAX_FORCE):
                pass

            release_compliance_ctrl()

            release()

    def fourth__floor(start_pose):
            #fourth floor
            release()

            pose = start_pose.copy()

            pose[0] -= CUP_DIAMETER_X * fourth_floor_list[0]
            pose[1] -= CUP_DIAMETER_Y * fourth_floor_list[1]

            last_cup_grip_pose[2] += HEIGHT_PLACE_OFFEST            
            movel(last_cup_grip_pose, vel = 100, acc = 100)
            last_cup_grip_pose[2] -= HEIGHT_PLACE_OFFEST            
            movel(last_cup_grip_pose, vel = VELOCITY, acc = ACC)            

            grip()

            last_cup_grip_pose[2] += HEIGHT_PLACE_OFFEST

            movel(last_cup_grip_pose, vel = VELOCITY - 400, acc = ACC - 400, ref = DR_BASE)
            movej(out_of_sing, vel = 80, acc = 80)

            pose[2] += HEIGHT_TARGET_OFFSET
            movel(pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)
            pose[2] -= HEIGHT_TARGET_OFFSET
            movel(pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

            task_compliance_ctrl(stx=compliance_ctrl_stx)
            set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=MAX_FORCE):
                pass

            release_compliance_ctrl()

            release()

    set_tool("Tool Weight")
    set_tcp("GripperSA_v1_test_1")
    
    movej(JReady, vel=30, acc=30)           # 초기 위치로

    for coor,pos_z in zip(floor_list,floor_z):
        stacking(place_start_pose,pos_z,coor)

    fourth__floor(fourth_floor_start_pose)

    rclpy.shutdown()


if __name__ == "__main__":
    main()