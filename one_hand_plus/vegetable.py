import time
import rclpy
import DR_init
import rclpy.logging
from std_msgs.msg import String
import threading
from queue import Queue

task_queue = Queue()

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1
is_task_running = False
is_task_done = False
task_status_pub = None

def run_vegetable_task():
    print('receive topic')
    global is_task_running, is_task_done
    if is_task_running:
        print("Task already running. Skipping.")
        return
    is_task_running = True

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
            get_current_posj,
            set_digital_output,
            amove_periodic,
            movej,
            movel,
            movesj,
            movesx,
            drl_script_stop,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_FC_MOD_ABS,
            DR_AXIS_Z,
            DR_AXIS_Y,
            DR_AXIS_X,
            DR_BASE,
            DR_TOOL,
            DR_SSTOP
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def release():
        print("set for digital output 0 1 for release")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        time.sleep(1.0)

    def grip():
        print("set for digital output 1 0 for grip")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        time.sleep(1.0)

    def noBread():
        release_force()
        time.sleep(0.3)
        release_compliance_ctrl()
        time.sleep(0.3)
        drl_script_stop(DR_SSTOP)
        print("No bread detected")
        movel([0, 0, 100, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        # ready_to_go = input("check bread")

        # while get_tool_force()[0] > 0 or get_tool_force()[1] > 0 or get_tool_force()[2] > 0:
        # rclpy.logging(get_tool_force())

        while get_tool_force()[2] > -2:
            # print(get_tool_force())
            pass

        time.sleep(1.0)
        movel([0, 0, -50, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        task_compliance_ctrl(stx=[1000, 500, 300, 100, 100, 100])
        time.sleep(0.3)
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.3)
        while not check_force_condition(DR_AXIS_Z, max=4):
            print("Starting check_force_condition")
            time.sleep(0.5)
            if check_position_condition(DR_AXIS_Z, min=50, ref=DR_BASE) == -1:
                noBread()
            pass

    def noSheaf():
        release_force()
        time.sleep(0.3)
        release_compliance_ctrl()
        time.sleep(0.3)
        drl_script_stop(DR_SSTOP)
        print("can't find knife sheaf")
        movel([0, 0, 30, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        periodic_amp_2 = [10, 10, 0.0, 0.0, 0.0, 0.0]
        amove_periodic(amp=periodic_amp_2, period=3.0, atime=0.02, repeat=20, ref=DR_BASE)
        while not check_position_condition(DR_AXIS_Z, min=372.63, ref=DR_BASE):
            time.sleep(0.5)
            pass
        drl_script_stop(DR_SSTOP)
        time.sleep(0.5)
        task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
        time.sleep(0.5)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.5)
        while not check_force_condition(DR_AXIS_Z, max=10):
            time.sleep(0.5)
            print("check_force_condition")
            pass
        if not check_position_condition(DR_AXIS_Z, min=360, ref=DR_BASE):
            noSheaf()

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")


    JReady = posj([0, 0, 90, 0, 90, 0])
    Knife = posj([-3.59, 19.83, 119.88, -2.42, -49.43, 91.43])
    mov_1 = posj([-11.27, 6.04, 106.79, -23.6, -24.34, 111.6])
    Chopping = posj([-26.24, 38.36, 60.44, -0.14, 81.4, 154.61])
    Slope_knife = posj([-18.16, 32.62, 66.16, -16.32, 76.96,165.61])
    Vertical_knife = posj([-27.49, 3.4, 89.29, -8.28, 2.97,99.88])
    Upper_knife = posj([-3.24, 12.65, 100.93, -4.09, -23.28, 93.76])
    mov_2 = posj([-26.66, -7.48, 118.37, -0.07, 69.21, 245.59])
    Bread_push = posj([-21.2, 5.86, 108.31, -4.67, 78.61, 251.55])
        
        
    '''
    1. 홈위치
    2. 칼집 위치로 이동
    3. 그리퍼 닫아서 칼집고 z축으로 들어 올리기
    4. 빵 좌표로
    5. 힘제어 키고 하강
    6. 빵 만나면 periodic 비동기 실행
    7. 좌표지정위치에서 빵 썰기 멈춤 + 힘제어 끄기
    8. 칼집 위쪽으로 이동
    9. 칼집에 칼 집어넣기
    10. 힘제어 끄고 홈위치로 이동
    '''
    try:
        release()
        print(f"Moving to joint position: {JReady}")    
        movej(JReady, vel=VELOCITY, acc=ACC)    # 홈위치
        movej(Knife, vel=VELOCITY, acc=ACC)     # 칼집 위치로 이동
        movel([50, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        grip()      # grip해서 칼집고 z축으로 들어 올리기
        movel([0, 0, 160, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        Upper_knife = get_current_posj()
        movej(mov_1, vel=VELOCITY, acc=ACC)
        
        # 빵 좌표로 이동 후 힘제어 키고 하강
        for i in range(3):
            movesj([Chopping, Slope_knife], vel=VELOCITY, acc=ACC)

            movel([3, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
            time.sleep(0.1)
            task_compliance_ctrl(stx=[1000, 500, 300, 100, 100, 100])       
            time.sleep(0.3)
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            time.sleep(0.3)
            while not check_force_condition(DR_AXIS_Z, max=5):
                print("Starting check_force_condition")
                time.sleep(0.5)
                if check_position_condition(DR_AXIS_Z, min=50, ref=DR_BASE) == -1:      # 빵이 없을경우 예외처리
                    noBread()
                pass
            
            periodic_amp = [0, 15.0, 0.0, 0.0, 0.0, 0.0]
            amove_periodic(amp=periodic_amp, period=1.0, atime=0.02, repeat=20, ref=DR_TOOL)

            # 좌표지정위치에서 빵 썰기 멈추고 힘제어 끄기
            while not check_position_condition(DR_AXIS_Z, min=40.00, ref=DR_BASE):    
                time.sleep(0.5)
                pass
            release_force()
            time.sleep(0.3)
            release_compliance_ctrl()
            time.sleep(1.0)
            drl_script_stop(DR_SSTOP)
            set_ref_coord(DR_BASE)
            movel([0, 0, 120, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

            # 빵본체를 밀 위치로 이동 후 순응제어 키고 밀기
            if i == 2:
                break
            movesj([mov_2,Bread_push], vel=VELOCITY, acc=ACC)
            movel([27+20*i, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
            movel([0, 0, -14, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
            task_compliance_ctrl(stx=[300, 2000, 2000, 100, 100, 100])
            time.sleep(0.3)
            set_desired_force(fd=[10, 0, 0, 0, 0, 0], dir=[1, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
            time.sleep(0.3)
            while not check_force_condition(DR_AXIS_Y, max=6, ref=DR_TOOL):
                print('push block force', get_tool_force())
                print("Starting check_force_condition")
                time.sleep(0.5)
            release_force()
            time.sleep(0.3)
            release_compliance_ctrl()
            time.sleep(1.0)
            movel([17, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
            movel([-10, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)            
            movej(mov_2, vel=VELOCITY, acc=ACC)
            
        
        # 칼집 위치로 이동 후 칼집에 넣기
        movesj([Vertical_knife, Upper_knife], vel=VELOCITY, acc=ACC)
        movel([-5, -5, 10, 0, -10, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
        time.sleep(0.5)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_ABS)
        time.sleep(0.5)
        while not check_force_condition(DR_AXIS_Z, max=10):
            if not check_position_condition(DR_AXIS_Z, max=350, ref=DR_BASE):
                break
            time.sleep(0.5)
            print("check_force_condition")
            pass
        if not check_position_condition(DR_AXIS_Z, min=360, ref=DR_BASE):       # 칼이 제대로 안들어가면 예외처리
            noSheaf()
        release_force()
        time.sleep(0.3)
        release_compliance_ctrl()
        time.sleep(0.3)
        movel([0, 0, -70, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        release()
        set_ref_coord(DR_BASE)
        movel([-50, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movej(JReady, vel=VELOCITY, acc=ACC)

    except KeyboardInterrupt:
        release_force()
        time.sleep(0.5)
        release_compliance_ctrl()
        time.sleep(0.5)

    finally:
        is_task_running = False
        is_task_done = True



def callback(msg):
    if msg.data == "vegetable":
        print("[vegetable Node] Received 'vegetable' command")
        task_queue.put(run_vegetable_task)
        # print(task_queue.empty())


from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    global is_task_running, is_task_done, task_status_pub
    rclpy.init(args=args)
    node = rclpy.create_node("vegetable_listener", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    node.create_subscription(String, "/robot_task_cmd", callback, 10)

    task_status_pub = node.create_publisher(String, "/vegetable_task_status", 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)


    def publish_status():
        status_msg = String()
        status_msg.data = "running" if is_task_running else "idle"
        task_status_pub.publish(status_msg)



    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)

        if not task_queue.empty():
            # print('task_queue.empty')
            task = task_queue.get()
            print(task)
            thread = threading.Thread(target=task)
            thread.start()

        publish_status()

        if is_task_done:
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
