import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):


    rclpy.init(args=args)
    node = rclpy.create_node("pick_and_place", namespace=ROBOT_ID)

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

    def close():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    pose_start = [
        posx([348.35, -104.71, 80.0, 90.00, 180.0, 90.00]),
        posx([349.65, -53.87, 80.0, 90.00, 180.0, 90.00]),
        posx([350.50, -1.27, 80.0, 90.00, 180.0, 90.00]),
        posx([397.60, -106.03, 80.0, 90.00, 180.0, 90.00]),
        posx([399.35, -55.09, 80.0, 90.00, 180.0, 90.00]),
        posx([401.78, -0.85, 80.0, 90.00, 180.0, 90.00]),
        posx([448.25, -107.62, 80.0, 90.00, 180.0, 90.00]),
        posx([450.39, -55.68, 80.0, 90.00, 180.0, 90.00]),
        posx([452.60, -5.49, 80.0, 90.00, 180.0, 90.00])
    ]

    pose_goal = [
        posx([349.06, 46.91, 150.0, 4.69, 180.0, 3.15]),
        posx([351.27, 97.81, 150.0, 56.20, -179.99, 54.36]),
        posx([353.43, 148.13, 150.0, 51.98, 179.98, 49.90]),
        posx([399.95, 45.36, 150.0, 10.79, 180.0, 9.03]),
        posx([401.58, 95.75, 150.0, 14.53, -180.0, 12.32]),
        posx([402.36, 148.86, 150.0, 16.96, -180.0, 14.95]),
        posx([448.92, 42.35, 150.0, 48.65, -179.96, 46.69]),
        posx([451.98, 93.08, 150.0, 126.11, -179.96, 124.11]),
        posx([453.38, 144.75, 150.0, 5.74, -180.0, 3.46])
    ]

    start2goal = [
        (1, 1),
        (2, 2),
        (3, 3),
        (4, 4),
        (5, 5),
        (6, 6),
        (7, 7),
        (8, 8),
        (9, 9)
    ]

    cnt_low = 1

    cnt_mid = 1

    cnt_high = 1

    JReady = [0, 0, 90, 0, 90, 0]
    set_tool("Tool Weight")
    set_tcp("GripperSA_v1_test_1")

    grip()

    for item in pose_start:        # start pose
        
        movej(JReady, vel=VELOCITY, acc=ACC)    # 초기 위치로

        movel(item, vel=VELOCITY, acc=ACC) # 옮길 블럭 위치로

        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100]) # 컴플라이언스 설정
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        # 힘 설정, z축 -15
        while not check_force_condition(DR_AXIS_Z, max=5): # 힘을 받을 때까지 반복
            pass

        pos1 = get_current_posx()[0]

        print(pos1[2])

        height = pos1[2]

        release_compliance_ctrl() # 컴플라이언스 해제

        pos1[2] += 10.0                # 다시 z축 10만큼

        movel(pos1, vel=VELOCITY, acc=ACC, ref=DR_BASE) # 10만큼 이동
        
        release()       # 그리퍼 열기
        
        pos1[2] -= 32.0                                 # 다시 z축 -12만큼
        movel(pos1, vel=VELOCITY, acc=ACC, ref=DR_BASE) # -12만큼 이동

        grip()          # 그리퍼 잡기

        pos1[2] += 60.0
        movel(pos1, vel=VELOCITY, acc=ACC, ref=DR_BASE) # z 축으로 60만큼 이동

        if height >= 65:
            idx = cnt_high * 3
            cnt_high += 1
        elif height >= 55:
            idx = cnt_mid * 3 - 1
            cnt_mid += 1
        elif height >= 45:
            idx = cnt_low * 3 - 2
            cnt_low += 1


        movel(pose_goal[idx-1], vel=VELOCITY, acc=ACC) # 목표 위치로 이동

        release_pos_set = get_current_posx()[0]

        release_pos_set[2] -= 70.0

        movel(release_pos_set, vel=VELOCITY, acc=ACC) # 목표 위치로 이동
        
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100]) # 컴플라이언스 설정
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        # 힘 설정, z축 -15
        while not check_force_condition(DR_AXIS_Z, max=5): # 힘을 받을 때까지 반복
            pass

        release_compliance_ctrl() # 컴플라이언스 해제        

        release()     # 그리퍼 열기

        movel(pose_goal[idx-1], vel=VELOCITY, acc=ACC)  # 다시 위로

        grip()      # 그리퍼 닫기

    rclpy.shutdown()

if __name__ == "__main__":
    main()


