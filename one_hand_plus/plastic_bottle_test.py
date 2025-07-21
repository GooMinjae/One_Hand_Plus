# import time
# import rclpy
# import DR_init
# from std_msgs.msg import String

# # for single robot
# ROBOT_ID = "dsr01"
# ROBOT_MODEL = "m0609"
# VELOCITY, ACC = 60, 60

# DR_init.__dsr__id = ROBOT_ID
# DR_init.__dsr__model = ROBOT_MODEL

# OFF, ON = 0, 1

# global_c_pos = [0] * 6

# def main(args=None):
#     rclpy.init(args=args)
#     node = rclpy.create_node("rokey_force_control", namespace=ROBOT_ID)

#     DR_init.__dsr__node = node

#     try:
#         from DSR_ROBOT2 import (
#             release_compliance_ctrl,
#             release_force,
#             check_force_condition,
#             check_position_condition,
#             get_tool_force,
#             task_compliance_ctrl,
#             set_desired_force,
#             set_tool,
#             set_tcp,
#             get_current_posx,
#             set_digital_output,
#             get_digital_input,
#             amove_periodic,
#             drl_script_stop,
#             move_periodic,
#             movej,
#             movel,
#             amovel,
#             DR_FC_MOD_REL,
#             DR_MV_MOD_REL,
#             DR_MV_MOD_ABS,
#             DR_AXIS_Z,
#             DR_BASE,
#             DR_TOOL,
#             DR_SSTOP,
#             DR_AXIS_X,
#             DR_FC_MOD_ABS,
#         )

#         from DR_common2 import posx, posj

#     except ImportError as e:
#         print(f"Error importing DSR_ROBOT2 : {e}")
#         return

#     # pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])
#     pos_bottle = posj([15.40, 23.25, 47.42, -0.39, 109.19, 1.74])
#     pos_cap_pre = posx([502.66, 17.85, 276.85, 143.82, 179.79, -37.07])
#     pos_cap = posx([502.66, 17.85, 112.85, 143.82, 179.79, -37.07])

#     JReady = posj([0, 0, 90, 0, 90, 0])
        
#     def wait_digital_input(sig_num):
#         while not get_digital_input(sig_num):
#             time.sleep(0.5)
#             print(f"Wait for digital input: {sig_num}")
#             pass


#     def release():
#         print("set for digital output 0 1 for release")
#         set_digital_output(2, ON)
#         set_digital_output(1, OFF)
#         # wait_digital_input(2)
#         time.sleep(1.0)

#     def grip():
#         print("set for digital output 1 0 for grip")
#         set_digital_output(1, ON)
#         set_digital_output(2, OFF)
#         # wait_digital_input(1)
#         time.sleep(1.0)

#     def start():
#         release()

#         print(f"Moving to joint position: {JReady}")
#         movej(JReady, vel=VELOCITY, acc=ACC)
        
#         grip()

#     def force_control():
#         global global_c_pos

#         print("Starting task_compliance_ctrl")
#         task_compliance_ctrl(stx=[3000, 3000, 500, 100, 100, 100])
#         time.sleep(0.5)

#         print("Starting set_desired_force")
#         set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

#         # 외력이 0 이상 5 이하이면 0
#         # 외력이 5 초과이면 -1
#         while not check_force_condition(DR_AXIS_Z, max=12):
#             print("Waiting for an external force greater than 5 ")
#             time.sleep(0.5)
#             pass

#         global_c_pos, _ = get_current_posx()

#         print("Starting release_force")
#         release_force()
#         time.sleep(0.5)
        
#         print("Starting release_compliance_ctrl")      
#         release_compliance_ctrl()

    
#     def grip_lid_for_open():

#         print(f"Moving to task position: {pos_bottle}")
#         movej(pos_bottle, vel=VELOCITY, acc=ACC)

#         force_control()
#         real_bottle_pos = global_c_pos
#         # movej(pos, vel=VELOCITY, acc=ACC)
#         movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        
#         release()

#         movel([0, 0, -23, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

#         grip()
    
#     def open_lid(motion):
        
#         if motion == 'open':
#             direction = 1
#         elif motion == 'close':
#             direction = -1

#         # print("Starting task_compliance_ctrl")
#         # task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
#         # time.sleep(0.5)

#         movel([0, 0, 4 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         release()
#         movel([0, 0, -4 * direction, 0, 0, -179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         grip()
#         movel([0, 0, 4 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         release()
#         movel([0, 0, -4 * direction, 0, 0, -179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         grip()
#         movel([0, 0, 4 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         movel([0, 0, 20, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        
#     def close_lid(motion):
#         if motion == 'open':
#             direction = 1
#         elif motion == 'close':
#             direction = -1



#         movel([0, 0, 3 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         release()
#         movel([0, 0, -4 * direction, 0, 0, -179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         grip()
#         movel([0, 0, 3 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         release()
#         movel([0, 0, -4 * direction, 0, 0, -179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         grip()
#         movel([0, 0, 3 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
#         release()
#         movel([0, 0, 20, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

#         # print("Starting release_compliance_ctrl")      
#         # release_compliance_ctrl()


#     def regrip_lid():
#         print(f"moving back to cap position: {pos_cap_pre}")
#         movel(pos_cap_pre, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS)
#         release()

#         # force_control()
#         movel(cap_release_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

#         grip()

# # //////////////////////////////////////////////////////////////////////////////
#     def check_force_and_release():
#         print("Starting task_compliance_ctrl")
#         task_compliance_ctrl(stx=[3000, 3000, 300, 200, 200, 200])
#         time.sleep(0.5)

#         print("Start set_desired_force")
#         set_desired_force(fd=[0, 0, -25, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_ABS)
#         print("Confirm set_desired_force")
#         time.sleep(0.5)
        
#         while True:
#             print("Waiting for an external force greater than 5 ")
#             print(get_tool_force())
#             if not check_force_condition(DR_AXIS_Z, min=24, ref=DR_BASE):
#                 break
#         # set_desired_force(fd=[0, 0, -12, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_ABS)

#         # # 외력이 0 이상 5 이하이면 0
#         # # 외력이 5 초과이면 -1
#         # while not check_force_condition(DR_AXIS_Z, max=10, ref=DR_BASE):
#         #     print("Waiting for an external force greater than 10 ")
#         #     # c_pos, _ = get_current_posx()
#         #     # time.sleep(0.5)
#         #     break


#         print("Starting release_force")
#         release_force()
#         time.sleep(0.5)
        
#         print("Starting release_compliance_ctrl")      
#         release_compliance_ctrl()

#         release()

#         #///////////////////////////////////////////////////////////////////////////////
    
#     set_tool("Tool Weight_2FG")
#     set_tcp("2FG_TCP")


#     '''
#     # 450 deg, 10 mm
#     example_amp = [0.0, 0.0, 2.0, 0.0, 0.0, -90.0]
#     amove_periodic(amp=example_amp, period=8.0, atime=0.02, repeat=3, ref=DR_TOOL)
#     time.sleep(2.0)

#     release()
#     time.sleep(4.0)

#     grip()
#     '''


#     while rclpy.ok():
#         try:
#             start()
#             grip_lid_for_open()

#             open_lid('open')

#             movel(pos_cap_pre, vel=VELOCITY, acc=ACC,ref=DR_BASE, mod=DR_MV_MOD_ABS)
#             movel(pos_cap, vel=VELOCITY, acc=ACC,ref=DR_BASE, mod=DR_MV_MOD_ABS)

#             force_control()
#             cap_release_pos = global_c_pos
#             release()

#             movej(JReady, vel=VELOCITY, acc=ACC) # return to the original spot
#     #/////////////////////////////////////////////////////////////////////////////////////////////////////
#             regrip_lid()

#             movel(posx(0, 0, 200, 0, 0, 0), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

#             movej(pos_bottle, vel=VELOCITY, acc=ACC)   #다시 병 위로 복귀시킴.

#             #여기다가 이제 오늘 배운 무브피리오딕 사용해서 돌려돌려 병 뚜껑에 장착 
            
#             print("Starting task_compliance_ctrl")
#             task_compliance_ctrl(stx=[3000, 3000, 500, 100, 100, 100])
#             time.sleep(0.5)

#             print("Starting set_desired_force")
#             set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        
#             while not check_force_condition(DR_AXIS_Z, max=12):
#                 print("Waiting for an external force greater than 5 ")
#                 time.sleep(0.5)
#                 pass

#             example_amp = [-4.0, -4.0, 0.0, 0.0, 0.0, 0.0]
#             print(f"Starting amove_periodic: {example_amp}")
#             amove_periodic(amp=example_amp, period=5, atime=0.02, repeat=3, ref=DR_TOOL)

#             time.sleep(1.0)

#             while not check_force_condition(DR_AXIS_X, min=2, ref=DR_BASE):
#                 print('check for hole')
#                 pass


#             print("Starting release_force")
#             release_force()
#             time.sleep(0.5)
            
#             print("Starting release_compliance_ctrl")      
#             release_compliance_ctrl()

#             release()
#             movej(pos_bottle, vel=VELOCITY, acc=ACC)
#             grip()

#             force_control()

#             movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
            
#             release()

#             movel([0, 0, -23, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

#             grip()

            
#             close_lid('close')

            
#             movej(JReady, vel=VELOCITY, acc=ACC) # return to home 

#             break

#         except KeyboardInterrupt:
#             release_force()
#             time.sleep(0.5)
#             release_compliance_ctrl()
#             time.sleep(0.5)
#         finally:
#             break

#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
#     rclpy.shutdown()


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
global_c_pos = [0] * 6


def run_plastic_task():
    print('receive topic')

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
            set_digital_output,
            get_digital_input,
            amove_periodic,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
            DR_AXIS_X,
            DR_FC_MOD_ABS,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    print('Finish import')

    # pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])
    pos_bottle = posj([15.40, 23.25, 47.42, -0.39, 109.19, 1.74])
    pos_cap_pre = posx([502.66, 17.85, 276.85, 143.82, 179.79, -37.07])
    pos_cap = posx([502.66, 17.85, 112.85, 143.82, 179.79, -37.07])

    JReady = posj([0, 0, 90, 0, 90, 0])
        
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

    def start():
        release()

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        grip()

    def force_control():
        global global_c_pos

        print("Starting task_compliance_ctrl")
        task_compliance_ctrl(stx=[3000, 3000, 500, 100, 100, 100])
        time.sleep(0.5)

        print("Starting set_desired_force")
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        # 외력이 0 이상 5 이하이면 0
        # 외력이 5 초과이면 -1
        while not check_force_condition(DR_AXIS_Z, max=12):
            print("Waiting for an external force greater than 5 ")
            time.sleep(0.5)
            pass

        global_c_pos, _ = get_current_posx()

        print("Starting release_force")
        release_force()
        time.sleep(0.5)
        
        print("Starting release_compliance_ctrl")      
        release_compliance_ctrl()

    
    def grip_lid_for_open():

        print(f"Moving to task position: {pos_bottle}")
        movej(pos_bottle, vel=VELOCITY, acc=ACC)

        force_control()
        real_bottle_pos = global_c_pos
        # movej(pos, vel=VELOCITY, acc=ACC)
        movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        
        release()

        movel([0, 0, -23, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        grip()
    
    def open_lid(motion):
        
        if motion == 'open':
            direction = 1
        elif motion == 'close':
            direction = -1

        # print("Starting task_compliance_ctrl")
        # task_compliance_ctrl(stx=[1000, 1000, 500, 100, 100, 100])
        # time.sleep(0.5)

        movel([0, 0, 4 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        release()
        movel([0, 0, -4 * direction, 0, 0, -179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        grip()
        movel([0, 0, 4 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        release()
        movel([0, 0, -4 * direction, 0, 0, -179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        grip()
        movel([0, 0, 4 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        movel([0, 0, 20, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        
    def close_lid(motion):
        if motion == 'open':
            direction = 1
        elif motion == 'close':
            direction = -1



        movel([0, 0, 3 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        release()
        movel([0, 0, -4 * direction, 0, 0, -179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        grip()
        movel([0, 0, 3 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        release()
        movel([0, 0, -4 * direction, 0, 0, -179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        grip()
        movel([0, 0, 3 * direction, 0, 0, 179 * direction], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        release()
        movel([0, 0, 20, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        # print("Starting release_compliance_ctrl")      
        # release_compliance_ctrl()


    def regrip_lid():
        print(f"moving back to cap position: {pos_cap_pre}")
        movel(pos_cap_pre, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS)
        release()

        # force_control()
        movel(cap_release_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        grip()


    print('start set tool')
    try:
        set_tool("Tool Weight_2FG")
        set_tcp("2FG_TCP")
    except Exception as e:
        print(f"[ERROR] Failed to set tool or tcp: {e}")

    print('end set tool')

    '''
    # 450 deg, 10 mm
    example_amp = [0.0, 0.0, 2.0, 0.0, 0.0, -90.0]
    amove_periodic(amp=example_amp, period=8.0, atime=0.02, repeat=3, ref=DR_TOOL)
    time.sleep(2.0)

    release()
    time.sleep(4.0)

    grip()
    '''


    # while rclpy.ok():
    try:
        print('start node')
        start()
        grip_lid_for_open()

        open_lid('open')

        movel(pos_cap_pre, vel=VELOCITY, acc=ACC,ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(pos_cap, vel=VELOCITY, acc=ACC,ref=DR_BASE, mod=DR_MV_MOD_ABS)

        force_control()
        cap_release_pos = global_c_pos
        release()

        movej(JReady, vel=VELOCITY, acc=ACC) # return to the original spot
#/////////////////////////////////////////////////////////////////////////////////////////////////////
        regrip_lid()

        movel(posx(0, 0, 200, 0, 0, 0), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

        movej(pos_bottle, vel=VELOCITY, acc=ACC)   #다시 병 위로 복귀시킴.

        #여기다가 이제 오늘 배운 무브피리오딕 사용해서 돌려돌려 병 뚜껑에 장착 
        
        print("Starting task_compliance_ctrl")
        task_compliance_ctrl(stx=[3000, 3000, 500, 100, 100, 100])
        time.sleep(0.5)

        print("Starting set_desired_force")
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    
        while not check_force_condition(DR_AXIS_Z, max=12):
            print("Waiting for an external force greater than 5 ")
            time.sleep(0.5)
            pass

        example_amp = [-4.0, -4.0, 0.0, 0.0, 0.0, 0.0]
        print(f"Starting amove_periodic: {example_amp}")
        amove_periodic(amp=example_amp, period=5, atime=0.02, repeat=3, ref=DR_TOOL)

        time.sleep(1.0)

        while not check_force_condition(DR_AXIS_X, min=2, ref=DR_BASE):
            print('check for hole')
            pass


        print("Starting release_force")
        release_force()
        time.sleep(0.5)
        
        print("Starting release_compliance_ctrl")      
        release_compliance_ctrl()

        release()
        movej(pos_bottle, vel=VELOCITY, acc=ACC)
        grip()

        force_control()

        movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        
        release()

        movel([0, 0, -23, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        grip()

        
        close_lid('close')

        
        movej(JReady, vel=VELOCITY, acc=ACC) # return to home 

        # break

    except KeyboardInterrupt:
        release_force()
        time.sleep(0.5)
        release_compliance_ctrl()
        time.sleep(0.5)


def callback(msg):
    if msg.data == "plastic":
        print("[Plastic Node] Received 'plastic' command")
        task_queue.put(run_plastic_task)
from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("plastic_bottle_listener", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    node.create_subscription(String, "/robot_task_cmd", callback, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

            if not task_queue.empty():
                task = task_queue.get()
                task()  # 같은 스레드에서 실행됨 = 안전함
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
