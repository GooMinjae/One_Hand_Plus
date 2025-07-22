import time
import rclpy
import DR_init
from std_msgs.msg import String
import threading
from queue import Queue

task_queue = Queue()

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

is_task_running = False
is_task_done = False
task_status_pub = None

####### 유리병 따개
def run_glass_task():
    print('receive topic')
    global is_task_running, is_task_done
    if is_task_running:
        print("Task already running. Skipping.")
        return
    is_task_running = True

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            get_tool_force,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            get_current_posx,
            get_current_posj,
            drl_script_stop,
            set_digital_output,
            movej,
            movel,
            movesj,
            movesx,
            amovel,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_AXIS_Z,
            DR_AXIS_Y,
            DR_AXIS_X,
            DR_AXIS_C,
            DR_BASE,
            DR_TOOL,
            DR_QSTOP,
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

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    ############################# 위치 저장 #################################

    pos_glass_bottle = posx([495.38, 137., 311.82, 8.58, -179.48, 8.7])
    pos_to_opener_1 = posj([-128.85, 0.01, 90.0, 0.0, 90, 0.0])
    pos_to_opener_2 = posj([-164.17, 26.14, 95.48, -2.38, 15.98, 0.0])
    pos_to_opener_3 = posj([-163.03, 50.58, 102.46, 28.39, -47.42, -100.67])  

    pos_cap_for_force = posj([156.86-360, -23.76, -109.9, 93.24, -81.58, 99.71-180])
    pos_cap_for_force_1 = posj([-178.69, -3.81, 95.93, -7.73, 72.49, -100.67])
    pos_cap_for_force_2 = posj([-184.32, -8.81, -50.93, 14.23, 73.04, -100.67])
    pos_cap_for_force_3 = posj([-214.51, -0.80, -113.54, 78.42, -44.66, -82.85])

    JReady = posj([0, 0, 90, 0, 90, 0])

    criterion_lid_z = 299.51

    #######################################################################

    
    # while rclpy.ok():
    try:
        
        # 홈위치
        release()
        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)
        grip()

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
        
        criterion_diff_z = criterion_lid_z - c_pos_lid[2]

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

        grip()

        # 병따개 빼기
        movesx([posx([0, 0, 13, 0, 0, 0]),posx([0, 15, 0, 0, 0, 0]),posx([70, 0, 0, 0, 0, 0])], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod= DR_MV_MOD_REL)
        c_pos_opener_out= get_current_posj()    # 다시 돌려놓을때 위치를 알아내기 위해 저장
        time.sleep(0.5)

        # 병따개 거는 위치 찾기
        movesj([pos_cap_for_force_1, pos_cap_for_force_2, pos_cap_for_force_3, pos_cap_for_force], vel=VELOCITY, acc=ACC)

        # find_opener_pos,_ = get_current_posx()
        # print(f'opener position x: {find_opener_pos[0]}, y: {find_opener_pos[1]} , z: {find_opener_pos[2]}, a: {find_opener_pos[3]}, b: {find_opener_pos[4]} , c: {find_opener_pos[5]}')
        # diff_lid_z = c_pos_lid[2] - find_opener_pos[2]

        # 병 높이차이만큼 움직이기
        movel([0, 0, -criterion_diff_z, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        # 힘제어로 병따개 위치 맞추기
        print("Starting task_compliance_ctrl for pos_open")
        task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
        time.sleep(0.5)

        print("Starting set_desired_force for pos_open")
        set_desired_force(fd=[-15, 5, -5, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.5)
        

        while (not check_force_condition(DR_AXIS_X, max = 8) or not check_force_condition(DR_AXIS_Y, max = 3)):
            print("Waiting for an external force greater than 8")
            time.sleep(0.5)
            pass

        print('Find opener position!!')

        print("Starting release_force by find opener position")
        release_force()
        time.sleep(0.5)
        
        print("Starting release_compliance_ctrl by find opener position")      
        release_compliance_ctrl()
        time.sleep(0.5)

        # 병뚜껑 따기
        print('Starting open lid')
        amovel(posx(20, -13, 0, 0, 0, -30), vel=VELOCITY-30, acc=ACC-30, mod=DR_MV_MOD_REL, ref=DR_TOOL)
        time.sleep(2)

        while True:
            cnt = 1

            rz_force = get_tool_force(ref=DR_BASE)
            time.sleep(0.1)
            print(f'rz: {rz_force[2]}')

            # 병따개 위치가 위로 잡았을 때
            if rz_force[2] < 20:
                print('Wrong opener position!!')
                drl_script_stop(DR_SSTOP)
                movel(find_opener_pos, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS)
                movel([0, 0, -5, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
                find_opener_pos, _ = get_current_posx()
                time.sleep(0.3)


                # 힘제어로 병따개 위치 맞추기
                print(f"Starting task_compliance_ctrl for pos_open {cnt} try")
                task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
                time.sleep(0.5)

                print(f"Starting set_desired_force for pos_open {cnt} try")
                set_desired_force(fd=[-15, 5, -5, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                time.sleep(0.5)
                

                while (not check_force_condition(DR_AXIS_X, max = 8) or not check_force_condition(DR_AXIS_Y, max = 3)):
                    print("Waiting for an external force greater than 8")
                    time.sleep(0.5)
                    pass

                print('Find opener position!!')

                print("Starting release_force by find opener position")
                release_force()
                time.sleep(0.5)
                
                print("Starting release_compliance_ctrl by find opener position")      
                release_compliance_ctrl()
                time.sleep(0.5)

                # 병뚜껑 따기
                print('Starting open lid')

                amovel(posx(20, -13, 0, 0, 0, -30), vel=VELOCITY-30, acc=ACC-30, mod=DR_MV_MOD_REL, ref=DR_TOOL)
                time.sleep(2)
                cnt += 1
                pass

            # 병따개 위치가 아래로 잡았을 때
            elif rz_force[2] > 25:
                print('Wrong opener position!!')
                drl_script_stop(DR_SSTOP)
                movel(find_opener_pos, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS)
                movel([0, 0, 5, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
                find_opener_pos, _ = get_current_posx()
                time.sleep(0.3)


                # 힘제어로 병따개 위치 맞추기
                print(f"Starting task_compliance_ctrl for pos_open {cnt} try")
                task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
                time.sleep(0.5)

                print(f"Starting set_desired_force for pos_open {cnt} try")
                set_desired_force(fd=[-15, 5, -5, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                time.sleep(0.5)
                

                while (not check_force_condition(DR_AXIS_X, max = 8) or not check_force_condition(DR_AXIS_Y, max = 3)):
                    print("Waiting for an external force greater than 8")
                    time.sleep(0.5)
                    pass

                print('Find opener position!!')

                print("Starting release_force by find opener position")
                release_force()
                time.sleep(0.5)
                
                print("Starting release_compliance_ctrl by find opener position")      
                release_compliance_ctrl()
                time.sleep(0.5)

                # 병뚜껑 따기
                print('Starting open lid')

                amovel(posx(20, -13, 0, 0, 0, -30), vel=VELOCITY-30, acc=ACC-30, mod=DR_MV_MOD_REL, ref=DR_TOOL)
                time.sleep(2)
                cnt += 1
                pass

            else:
                print('Correct opener position!!')
                break
        
        print('Finish open!!')

        print("Starting release_force by finish open")
        release_force()
        time.sleep(0.5)


        
        print("Starting release_compliance_ctrl by finish open")      
        release_compliance_ctrl()
        time.sleep(0.5)

        # 병따개 다시 갖다 놓기
        print("Move to original place")
        movel([0, 0, 30, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movesj([pos_cap_for_force_3, pos_cap_for_force_2, pos_cap_for_force_1, c_pos_opener_out], vel=VELOCITY, acc=ACC)
        time.sleep(0.5)

        print("Adjust position for original place")
        movesx([posx([-70, 0, 0, 0, 0, 0]),posx([0, -15, 0, 0, 0, 0]),posx([0, 0, -13, 0, 0, 0])], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod= DR_MV_MOD_REL)

        release()

        # 홈위치로
        print("Get home position")
        movel([0, 0, -30, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_TOOL)

        movesj([pos_to_opener_3, pos_to_opener_2, pos_to_opener_1,JReady], vel=VELOCITY, acc=ACC)


    except KeyboardInterrupt:
        release_force()
        time.sleep(0.5)
        release_compliance_ctrl()
        time.sleep(0.5)

    finally:
        is_task_running = False
        is_task_done = True


def callback(msg):
    if msg.data == "glass":
        print("[glass Node] Received 'glass' command")
        task_queue.put(run_glass_task)
        # print(task_queue.empty())


from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    global is_task_running, is_task_done, task_status_pub
    rclpy.init(args=args)
    node = rclpy.create_node("glass_bottle_listener", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    node.create_subscription(String, "/robot_task_cmd", callback, 10)

    task_status_pub = node.create_publisher(String, "/glass_task_status", 10)
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
