import time
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

####### 유리병 따개
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            check_position_condition,
            get_tool_force,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            get_current_posx,
            get_current_posj,
            set_digital_output,
            get_digital_input,
            amove_periodic,
            move_periodic,
            movej,
            movel,
            movesj,
            movesx,
            amovel,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    # def wait_digital_input(sig_num):
    #     while not get_digital_input(sig_num):
    #         time.sleep(0.5)
    #         print(f"Wait for digital input: {sig_num}")
    #         pass


    def release():
        print("set for digital output 0 1 for release")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        # wait_digital_input(2)
        time.sleep(1.0)

    def grip():
        print("set for digital output 1 0 for grip")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        # wait_digital_input(1)
        time.sleep(1.0)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    ############################# 위치 저장 #################################

    # pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])
    pos_glass_bottle = posx([495.38, 143.45, 311.82, 8.58, -179.48, 8.7])
    pos_to_opener_1 = posj([-128.85, 0.01, 90.0, 0.0, 90, 0.0])
    pos_to_opener_2 = posj([-164.17, 26.14, 95.48, -2.38, 15.98, 0.0])
    # pos_to_opener_3 = posj([-168.4, 44.25, 99.59, 18.99, -25.26, -100.67])
    pos_to_opener_3 = posj([-163.03, 50.58, 102.46, 28.39, -47.42, -100.67])  

    pos_cap_for_force = posj([156.86, -23.76, -109.9, 93.24, -81.58, 99.71-180])

    JReady = posj([0, 0, 90, 0, 90, 0])

    #######################################################################

    # release()
    grip()
    
    while rclpy.ok():
        
        # 홈위치
        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)

        # 병뚜껑 위치
        print(f"Moving to task position: {pos_glass_bottle}")
        movel(pos_glass_bottle, vel=VELOCITY, acc=ACC)

        # 힘제어로 병뚜껑 위치 찾기
        print("Starting task_compliance_ctrl")
        task_compliance_ctrl(stx=[3000, 3000, 500, 100, 100, 100])
        time.sleep(0.5)

        print("Starting set_desired_force")
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max = 12):
            print("Waiting for an external force greater than 12")
            time.sleep(0.5)
            pass

        # 병뚜껑 위치 저장
        c_pos_lid, _ = get_current_posx()
        print(f"Lid position x: {c_pos_lid[0]}, y: {c_pos_lid[1]} , z: {c_pos_lid[2]}, a: {c_pos_lid[3]}, b: {c_pos_lid[4]} , c: {c_pos_lid[5]}")

        print("Starting release_force")
        release_force()
        time.sleep(0.5)
        
        print("Starting release_compliance_ctrl")      
        release_compliance_ctrl()

        movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)


        # 병따개 위치로 이동
        movesj([pos_to_opener_1, pos_to_opener_2, pos_to_opener_3], vel=VELOCITY, acc=ACC)

        release()
        time.sleep(1.0)

        # 병따개 잡기
        movel([0, 0, 30, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        # movesx([pos_to_opener,pos_opener], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        grip()

        # 병따개 빼기
        movesx([posx([0, 0, 13, 0, 0, 0]),posx([0, 15, 0, 0, 0, 0]),posx([70, 0, 0, 0, 0, 0])], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod= DR_MV_MOD_REL)
        # movel([0, 0, 13, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        # movel([15, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        time.sleep(0.5)

        # 병따개 거는 위치 찾기
        movesj([pos_to_opener_1, pos_cap_for_force], vel=VELOCITY, acc=ACC)

        find_opener_pos,_ = get_current_posx()
        print(f'opener position x: {find_opener_pos[0]}, y: {find_opener_pos[1]} , z: {find_opener_pos[2]}, a: {find_opener_pos[3]}, b: {find_opener_pos[4]} , c: {find_opener_pos[5]}')
        # c_pos[1] -= 200
        # movel(c_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        # 힘제어로 병따개 위치 맞추기
        print("Starting task_compliance_ctrl for pos_open")
        task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
        time.sleep(0.5)

        print("Starting set_desired_force for pos_open")
        set_desired_force(fd=[15, 15, -15, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        

        while not check_force_condition(DR_AXIS_Z, max = 12):
            print("Waiting for an external force greater than 12")
            time.sleep(0.5)
            pass

        print('Find opener position!!')

        print("Starting release_force by find opener position")
        release_force()
        time.sleep(0.5)
        
        print("Starting release_compliance_ctrl by find opener position")      
        release_compliance_ctrl()
        
        break

    rclpy.shutdown()


if __name__ == "__main__":
    main()
    rclpy.shutdown()
