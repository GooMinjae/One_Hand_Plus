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
            set_digital_output,
            get_digital_input,
            amove_periodic,
            move_periodic,
            movej,
            movel,
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

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")



    # pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])
    pos_glass_bottle = posx([495.38, 143.45, 311.82, 8.58, -179.48, 8.7])
    pos_to_opener = posx([-764.65, -236.99, -42.13, 8.94, -105.67, -86.10])
    pos_opener = posx([-794.65, -236.99, -42.13, 8.94, -105.67, -86.10])
    pos_cup = posx([502.66, 17.85, 112,85, 143.82, 179.79, -37.07])

    JReady = posj([0, 0, 90, 0, 90, 0])
    # release()
    grip()
    
    while rclpy.ok():

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)

        print(f"Moving to task position: {pos_glass_bottle}")
        movel(pos_glass_bottle, vel=VELOCITY, acc=ACC)

        print("Starting task_compliance_ctrl")
        task_compliance_ctrl(stx=[3000, 3000, 500, 100, 100, 100])
        time.sleep(0.5)

        print("Starting set_desired_force")
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max = 12):
            print("Waiting for an external force greater than 12")
            time.sleep(0.5)
            pass
        c_pos_x,c_pos_y, c_pos_z = get_current_posx()[0], get_current_posx()[1], get_current_posx()[2]
        print(f"x: {c_pos_x}, y: {c_pos_y} , z: {c_pos_z}")

        # movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        print("Starting release_force")
        release_force()
        time.sleep(0.5)
        
        print("Starting release_compliance_ctrl")      
        release_compliance_ctrl()

        # movej(pos, vel=VELOCITY, acc=ACC)
        movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        # release()
        movel(pos_to_opener, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        release()
        time.sleep(1.0)

        movel(pos_opener, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        # movesx([pos_to_opener,pos_opener], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        grip()

        '''
        # 450 deg, 10 mm
        example_amp = [0.0, 0.0, 2.0, 0.0, 0.0, -90.0]
        amove_periodic(amp=example_amp, period=8.0, atime=0.02, repeat=3, ref=DR_TOOL)
        time.sleep(2.0)

        release()
        time.sleep(4.0)

        grip()
        '''
        # movej([0, 0, 0, 0, 0, -180], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        # movel([0, 0, 4, 0, 0, 179], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        # release()
        # movel([0, 0, -4, 0, 0, -179], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        # grip()
        # movel([0, 0, 4, 0, 0, 179], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        # release()
        # movel([0, 0, -4, 0, 0, -179], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        # grip()
        # movel([0, 0, 4, 0, 0, 179], vel=VELOCITY-40, acc=VELOCITY-40, mod=DR_MV_MOD_REL)
        # movel([0, 0, 20, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

    
        # movel(pos_cup_pre, vel=VELOCITY, acc=ACC,ref=DR_BASE, mod=DR_MV_MOD_ABS, radius=10)
        movel(pos_cup, vel=VELOCITY, acc=ACC,ref=DR_BASE, mod=DR_MV_MOD_ABS)

        break

    rclpy.shutdown()


if __name__ == "__main__":
    main()
    rclpy.shutdown()
