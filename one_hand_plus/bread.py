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


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_ref_coord,
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
            movej,
            movel,
            movesj,
            amovel,
            drl_script_stop,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_FC_MOD_ABS,
            DR_AXIS_Z,
            DR_AXIS_Y,
            DR_BASE,
            DR_TOOL,
            DR_SSTOP
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            time.sleep(0.5)
            print(f"Wait for digital input: {sig_num}")
            pass


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

    def NoBread():
        release_force()
        time.sleep(0.3)
        release_compliance_ctrl()
        time.sleep(0.3)
        drl_script_stop(DR_SSTOP)
        print("No bread detected")
        movel([0, 0, 100, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        ready_to_go = input("check bread")
        time.sleep(1.0)
        movel([0, 0, -50, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        task_compliance_ctrl(stx=[1000, 500, 300, 100, 100, 100])       # 힘제어 키고 하강
        time.sleep(0.3)
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.3)
        while not check_force_condition(DR_AXIS_Z, max=4):      # 빵 만나면 periodic 비동기 실행
            print("Starting check_force_condition")
            time.sleep(0.5)
            if check_position_condition(DR_AXIS_Z, min=50, ref=DR_BASE) == -1:
                NoBread()
            pass

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
# 96.45
# 50.59

    JReady = posj([0, 0, 90, 0, 90, 0])

    Knife = posj([-3.59, 19.83, 119.88, -2.42, -49.43, 91.43])
    mov_1 = posj([-11.27, 6.04, 106.79, -23.6, -24.34, 111.6])
    Chopping = posj([-26.24, 38.36, 60.44, -0.14, 81.4, 154.61])
    Slope_knife = posj([-18.16, 32.62, 66.16, -16.32, 76.96,165.61])
    Slice_push = posj([-31.52, 44.3, 53.66, -0.25, 82.03,149.44])
    Bread_push = posj([-31.52, 44.3, 53.66, -0.25, 82.03,149.44]) ################################ 사용할 시 티칭 필요 #########################
    Vertical_knife = posj([-27.49, 3.4, 89.29, -8.28, 2.97,99.88])
    Upper_knife = posj([-3.24, 12.65, 100.93, -4.09, -23.28, 93.76])

    ##########################################
    x_Knife = posx([0, 0, 90, 0, 90, 0])
    x_Chopping = posx([0, 0, 90, 0, 90, 0])
    x_Slice_push = posx([0, 0, 90, 0, 90, 0])
    x_Vertical_knife = posx([0, 0, 90, 0, 90, 0])
    x_Upper_knife = posx([0, 0, 90, 0, 90, 0])


    
    '''
    1. 홈위치
    2. 칼집 위치로 이동
    3. 그리퍼 닫아서 칼집고 z축으로 들어 올리기
    4. 빵 좌표로
    5. 힘제어 키고 하강
    6. 빵 만나면 periodic 비동기 실행 + z축 하강
    7. 좌표지정위치에서 빵 썰기 멈춤 + 힘제어 끄기(순응제어는 유지)
    8. z축으로만 이동하면서 빵 위로 나온 뒤 순응제어 종료

    12. 칼 세우고 칼집 위쪽으로 이동
    13. 천천히 하강(순응제어키고) periodic 비동기
    14. 좌표지정위치에서 periodic 끄기
    15. checkforce로 끝까지 밀어넣고 그리퍼 열기
    16. 힘제어 끄고 홈위치로 이동
    '''

    # 9. 자른 빵을 밀 위치로 이동 후 순응제어 키고 밀기
    # 10. 빵본체를 밀수 있는 위치로 이동 후 순응제어 키고 밀기
    # 11. 4~10 n회 반복

    while rclpy.ok():
        release()
        print(f"Moving to joint position: {JReady}")    
        movej(JReady, vel=VELOCITY, acc=ACC)    # 홈위치

        movej(Knife, vel=VELOCITY, acc=ACC)     # 칼집 위치로 이동
        movel([50, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        grip()      # 그리퍼 닫아서 칼집고 z축으로 들어 올리기
        movel([0, 0, 160, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        Upper_knife = get_current_posj()

        ## fix
        movesj([mov_1, Chopping, Slope_knife], vel=VELOCITY, acc=ACC)
        # movej(mov_1, vel=VELOCITY, acc=ACC)
        # movej(Chopping, vel=VELOCITY, acc=ACC)
        time.sleep(0.1)
        # for _ in range(1):
        # 빵 좌표로 이동
        # movej(Chopping, vel=VELOCITY, acc=ACC)      
        # movel([0, 0, -10, 0, -20, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        task_compliance_ctrl(stx=[1000, 500, 300, 100, 100, 100])       # 힘제어 키고 하강
        time.sleep(0.3)
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.3)
        while not check_force_condition(DR_AXIS_Z, max=4):      # 빵 만나면 periodic 비동기 실행
            print("Starting check_force_condition")
            time.sleep(0.5)
            if check_position_condition(DR_AXIS_Z, min=50, ref=DR_BASE) == -1:
                NoBread()
            pass
        # while not check_position_condition(DR_AXIS_Z, min=80, ref=DR_BASE):
        #     print("Starting check_position_condition")
        #     time.sleep(0.5)
        #     pass
        
        periodic_amp = [0, 15.0, 0.0, 0.0, 0.0, 0.0]
        amove_periodic(amp=periodic_amp, period=1.0, atime=0.02, repeat=20, ref=DR_TOOL)

        while not check_position_condition(DR_AXIS_Z, min=38.00, ref=DR_BASE):    # 좌표지정위치에서 빵 썰기 멈춤 + 힘제어 끄기(순응제어는 유지)
            # print("Periodic check_position_condition")
            time.sleep(0.5)
            pass
        # time.sleep(1.0)
        # while not check_force_condition(DR_AXIS_Y, min=5):      # 바닥 만나면 빵 썰기 멈춤 + 힘제어 끄기(순응제어는 유지)
        #     print("Starting check_force_condition")
        #     time.sleep(0.5)
        #     pass
        release_force()
        time.sleep(0.3)
        release_compliance_ctrl()
        time.sleep(1.0)
        drl_script_stop(DR_SSTOP)
        
        set_ref_coord(DR_BASE)
        movel([0, 0, 120, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)      # z축으로만 이동하면서 빵 위로 나온 뒤 순응제어 종료
        

        # movej(Slice_push, vel=VELOCITY, acc=ACC)     # 자른 빵을 밀 위치로 이동 후 순응제어 키고 밀기
        # task_compliance_ctrl(stx=[1000, 500, 1000, 100, 100, 100]) 
        # movel([0, 130, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        # release_compliance_ctrl()

        # movej(Bread_push, vel=VELOCITY, acc=ACC)     # 빵본체를 밀 위치로 이동 후 순응제어 키고 밀기
        # task_compliance_ctrl(stx=[300, 2000, 2000, 100, 100, 100]) 
        # movel([140, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        # release_compliance_ctrl()


        # 칼 세우고 칼집 위치로 이동
        # movej(Vertical_knife, vel=VELOCITY, acc=ACC)     
        # movej(Upper_knife, vel=VELOCITY, acc=ACC)
        movesj([Vertical_knife, Upper_knife], vel=VELOCITY, acc=ACC)
        movel([-5, -5, 10, 0, -10, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)


        # 천천히 하강(순응제어키고) periodic 비동기
        task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
        time.sleep(0.5)
        print("1")
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_ABS)
        time.sleep(0.5)
        print("2")
        # set_ref_coord(DR_BASE)
        # periodic_amp_2 = [0, 10.0, 0.0, 0.0, 0.0, 0.0]
        # print("3")
        # amove_periodic(amp=periodic_amp_2, period=3.0, atime=0.02, repeat=20, ref=DR_BASE)
        # periodic_amp_1 = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # amove_periodic(amp=periodic_amp_1, period=3.0, atime=0.02, repeat=20, ref=DR_BASE)


        # # 좌표지정위치에서 periodic 끄기
        # while not check_position_condition(DR_AXIS_Z, min=372.63, ref=DR_BASE):
        #     # print("Periodic check_position_condition")
        #     time.sleep(0.5)
        #     pass

        # checkforce로 끝까지 밀어넣고 그리퍼 열기
        while not check_force_condition(DR_AXIS_Z, max=10):
            time.sleep(0.5)
            print("check_force_condition")
            pass
        
        release_force()
        time.sleep(0.3)
        release_compliance_ctrl()
        time.sleep(0.3)
        # drl_script_stop(DR_SSTOP)
        release()

        set_ref_coord(DR_BASE)
        movel([-50, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movej(JReady, vel=VELOCITY, acc=ACC)    # 홈위치
        break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
