# coding: utf8
#!/usr/bin/env python3
import sys
import rclpy
import std_msgs
from std_msgs.msg import Float32MultiArray
import termios
import tty

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""
moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1)
   }

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
   }

def getKey(settings):

   tty.setraw(sys.stdin.fileno())
   # sys.stdin.read() returns a string on Linux
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

def saveTerminalSettings():
   return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
   return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

def main():
   settings = saveTerminalSettings()

   rclpy.init()

   node = rclpy.create_node('teleop_twist_keyboard')
   pub = node.create_publisher(Float32MultiArray, '/summit_xl_control/cmd_vel', 10)

   speed = 0.1
   turn = 0.1
   x = 0.0
   y = 0.0
   z = 0.0
   th = 0.0
   status = 0.0
   linear_vel = [0., 0., 0.]
   angular_vel = [0., 0., 0.]

   try:
      print(msg)
      print(vels(speed, turn))
      while True:
            key = getKey(settings)
            if key in moveBindings.keys():
               x = moveBindings[key][0]
               y = moveBindings[key][1]
               z = moveBindings[key][2]
               th = moveBindings[key][3]
            elif key in speedBindings.keys():
               speed = speed * speedBindings[key][0]
               turn = turn * speedBindings[key][1]

               print(vels(speed, turn))
               if (status == 14):
                  print(msg)
               status = (status + 1) % 15
            else:
               x = 0.0
               y = 0.0
               z = 0.0
               th = 0.0
               if (key == '\x03'):
                  break

            linear_vel[0] = x * speed
            #linear_vel[1] = y * speed
            #linear_vel[2] = z * speed
            angular_vel[2] = th * turn
            twist = Float32MultiArray(layout=std_msgs.msg.MultiArrayLayout(data_offset=0),
                                      data=[linear_vel[0],linear_vel[1],
                                       linear_vel[2],angular_vel[0],
                                       angular_vel[1],angular_vel[2]])
            pub.publish(twist)

   except Exception as e:
      print(e)

   finally:
      linear_vel = [0., 0., 0.]
      angular_vel = [0., 0., 0.]

      twist = Float32MultiArray(layout=std_msgs.msg.MultiArrayLayout(data_offset=0),
                                 data=[linear_vel[0],linear_vel[1],
                                 linear_vel[2],angular_vel[0],
                                 angular_vel[1],angular_vel[2]])
      pub.publish(twist)

      restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
