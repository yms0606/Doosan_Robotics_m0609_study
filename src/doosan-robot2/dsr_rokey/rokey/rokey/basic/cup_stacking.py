# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cup_stacking", namespace=ROBOT_ID)

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
            DR_BASE,
            get_current_posx,
            set_digital_output,
            get_digital_input,
            wait,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    # init pose
    JReady = [0, 0, 90, 0, 90, 0]

    world_cup_pose = posx([446.92, -229.70, 240.77, 0.0, 180.0, 0.0])

    first_line = posx([561.52, 221.63, 125.13, 0.0, -180.0, 0.0])

    compliance_ctrl_stx = [500, 500, 500, 100, 100, 100]

    fd_z_minus = [0, 0, -30, 0, 5, 0]

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
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        pos = get_current_posx()[0]

        release_compliance_ctrl()

        pos[2] += 10

        movel(pos, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        release()

        pos[2] -= 18

        movel(pos, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        grip()

        pos[2] += 100   

        movel(pos, vel = VELOCITY, acc = ACC, ref = DR_BASE)

    def three_world_cup(start_pose):
        # 1st cup        
        world_cup()

        start_pose[2] += 100

        movel(start_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        start_pose[2] -= 100

        movel(start_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        task_compliance_ctrl(stx=compliance_ctrl_stx)
        set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()

        release()

        # 2nd cup
        world_cup()

        start_pose[0] -= 80
        start_pose[2] += 100

        movel(start_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        start_pose[2] -= 100

        movel(start_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        task_compliance_ctrl(stx=compliance_ctrl_stx)
        set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()

        release()

        # 3rd cup
        world_cup()

        start_pose[0] +=40
        start_pose[2] += 100

        movel(start_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        task_compliance_ctrl(stx=compliance_ctrl_stx)
        set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()

        release()

        movel(start_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE) 

    def six_world_cup(start_pose):

        first_floor_poses = start_pose.copy()
        second_floor_pose = start_pose.copy()
        third_floor_pose = start_pose.copy()        
        for _ in range(3): # 1st floor
            world_cup()       
            first_floor_poses[2] += 100

            movel(first_floor_poses, vel = VELOCITY, acc = ACC, ref = DR_BASE)

            first_floor_poses[2] -= 100

            movel(first_floor_poses, vel = VELOCITY, acc = ACC, ref = DR_BASE)

            task_compliance_ctrl(stx=compliance_ctrl_stx)
            set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass

            release_compliance_ctrl()

            release()

            first_floor_poses[0] -= 80

        second_floor_pose[0] -= 40

        for _ in range(2): # 2nd floor
            world_cup()       
            second_floor_pose[2] += 100

            movel(second_floor_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

            task_compliance_ctrl(stx=compliance_ctrl_stx)
            set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass

            release_compliance_ctrl()

            release()

            second_floor_pose[0] -= 80

        third_floor_pose[0] -= 80

        world_cup()       
        third_floor_pose[2] += 170

        movel(third_floor_pose, vel = VELOCITY, acc = ACC, ref = DR_BASE)

        task_compliance_ctrl(stx=compliance_ctrl_stx)
        set_desired_force(fd=fd_z_minus, dir=fd_dir_z, mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()

        release()



    set_tool("Tool Weight")
    set_tcp("GripperSA_v1_test_1")
    
    movej(JReady, vel=30, acc=30)    

    while rclpy.ok():

        six_world_cup(first_line)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
