import os
import rclpy
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
NUM_CABLES = 9
DXL_MINIMUM_POSITION_VALUE = -4100
DXL_MAXIMUM_POSITION_VALUE = 4100
dxl_velocity = [0] * NUM_CABLES

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def main():
    rclpy.init()
    node = rclpy.create_node('Keyboard_Controller')
    
    publishers = []
    positions = {}  # Store positions for each cable
    
    for i in range(1, NUM_CABLES + 1):
        topic_name = f'Robot/Cable{i}/state/displacement'
        publisher = node.create_publisher(SetPosition, topic_name, 10)
        publishers.append(publisher)
        
        # Use the GetPosition service to get the initial position for each motor
        try:
            get_position_client = node.create_client(GetPosition, 'get_position')
            while not get_position_client.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('GetPosition service not available, waiting...')
            request = GetPosition.Request()
            request.id = i
            future = get_position_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None:
                initial_position = future.result().position
            else:
                node.get_logger().info('Failed to get initial position')
                return
        except Exception as e:
            node.get_logger().info(f'Error calling GetPosition service: {str(e)}')
            return
        
        positions[i] = initial_position
    
    current_cable = None
    try:
        while True:
            print("Enter a cable ID (1-9), or press ESC to quit:")
            key = getch()
            if key == chr(0x1b):  # ESC key to quit
                break
            elif key.isdigit() and '1' <= key <= '9':
                cable_number = int(key)
                current_cable = cable_number
                print(f"Selected Cable {current_cable}")
                
                while current_cable:
                    print(f"Current Cable: {current_cable}")
                    print("Press + to increase position, - to decrease, or press any digit to change cable. Motor position range is between 0 and 4100.")
                    key = getch()
                    if key == chr(0x1b):  # ESC key to quit
                        break
                    elif key == '+':
                        positions[current_cable] += 100  # Increase position by 100
                        positions[current_cable] = clamp(positions[current_cable], DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)
                        print(f"Position increased by 100. New position: {positions[current_cable]}")
                    elif key == '-':
                        positions[current_cable] -= 100  # Decrease position by 100
                        positions[current_cable] = clamp(positions[current_cable], DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)
                        print(f"Position decreased by 100. New position: {positions[current_cable]}")
                    elif key.isdigit() and '1' <= key <= '9':
                        current_cable = int(key)
                        print(f"Selected Cable {current_cable}")
                    else:
                        print("Invalid key. Press +, -, or any digit between 1 and 9.")
                    
                    msg = SetPosition()
                    msg.id = current_cable
                    msg.position = positions[current_cable]
                    publishers[current_cable - 1].publish(msg)
                    print(f"Published position for Cable {current_cable}: {msg.position}")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
