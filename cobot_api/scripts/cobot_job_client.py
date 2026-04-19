#!/usr/bin/env python3


import rclpy, time, re
from rclpy.node import Node
from cobot_msgs.srv import TaskParams
from cobot_msgs.srv import CtrlParam

# ros2 launch cobot_api cobot_api.launch.py move_group_demo:=job

class MoveItServiceClient(Node):
    """ MoveItServiceClient node """
    def __init__(self):
        super().__init__('cobot_job_client')

        # # Declare all parameters and Read parameters
        self.parameters = []
        self.declare_parameter("goal_program", ["pos1", "pos2"])
        self.goal_program = self.get_parameter('goal_program').get_parameter_value().string_array_value
        self.get_logger().info('-- goal program -- \n'+str(self.goal_program))

        # Create service clients
        self.go_to_cart_goal_client = self.create_client(TaskParams, '/cobot_api/go_to_cart_goal')
        self.go_to_joint_goal_client = self.create_client(TaskParams, '/cobot_api/go_to_joint_goal')
        self.vary_speed_client = self.create_client(TaskParams, '/cobot_api/vary_speed')
        self.stop_motion_client = self.create_client(CtrlParam, '/cobot_api/stop_motion')

        self.cylic_func = True
        self.run()

    def gripper_control(self, onoff):
        """ gripper control """
        if onoff == 0.0:
            # self.write_DO_req.data = False
            pass
        elif onoff == 1.0:
            # self.write_DO_req.data = True
            pass



    def run(self):
        """ run """
        self.get_logger().info('starting...')
        self.send_vary_speed_request(1.0)
        time.sleep(0.3)

        while True:
            # Read all positions
            for name in self.goal_program:
                self.declare_parameter(name, [0.0])
                self.parameters.append(name)
                goal = self.get_parameter(name).value
                self.get_logger().info(' \n goal: '+str(goal))

                float_goal = [float(value) for value in goal]

                self.get_logger().info('name: '+str(name))
                match = re.search(r'_([^:]+)', name)
                self.get_logger().info('-- debug 1 -- \n'+str(match))

                if match:
                    name = match.group(1)
                    # print(f"Command name: {name}")

                    if name == 'movel':
                        self.get_logger().info('-- debug 2 -- \n'+str(float_goal))
                        result = self.send_go_to_cart_goal_request(float_goal)
                        if result is not None and result.status == 0:
                            self.get_logger().error('task failed!')
                            return
                        elif result is not None and result.status == 1:
                            self.get_logger().info('task succeeded!')
                        else:
                            self.get_logger().error('Service call failed!')
                            return
                        # self.send_stop_motion_request()

                    elif name == 'movej':
                        result = self.send_go_to_joint_goal_request(float_goal)
                        if result is not None and result.status == 0:
                            self.get_logger().error('task failed!')
                            return
                        elif result is not None and result.status == 1:
                            self.get_logger().info('task succeeded!')
                        else:
                            self.get_logger().error('Service call failed!')
                            return
                        # self.send_stop_motion_request()

                    elif name == 'wait':
                        self.get_logger().info('Executing... Wait')
                        time.sleep(float_goal[0])

                    elif name == 'gripper':
                        self.get_logger().info('Executing... Gripper')
                        cmd = float_goal[0]
                        if(not self.gripper_control(cmd)):
                            print('Gripper Execution Error! ')


            # Undeclare the parameters
            for name in self.parameters:
                self.undeclare_parameter(name)
            self.parameters = []  # Clear the list of parameter names

            if self.cylic_func is False:
                self.get_logger().info('-- debug: i will only jiggle once.')
                break
            else:
                self.get_logger().info('-- debug: i will jiggle again.')


    def send_go_to_cart_goal_request(self, cart_goal):
        """ send_go_to_cart_goal_request """
        request = TaskParams.Request()
        request.cmd1 = cart_goal[0]
        request.cmd2 = cart_goal[1]
        request.cmd3 = cart_goal[2]
        request.cmd4 = cart_goal[3]
        request.cmd5 = cart_goal[4]
        request.cmd6 = cart_goal[5]
        request.cmd7 = cart_goal[6]

        # Wait for services to be available
        while not self.go_to_cart_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        future = self.go_to_cart_goal_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response


    def send_go_to_joint_goal_request(self, joint_goal):
        """ send_go_to_joint_goal_request """
        request = TaskParams.Request()
        request.cmd1 = joint_goal[0]
        request.cmd2 = joint_goal[1]
        request.cmd3 = joint_goal[2]
        request.cmd4 = joint_goal[3]
        request.cmd5 = joint_goal[4]
        request.cmd6 = joint_goal[5]

        while not self.go_to_joint_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        future = self.go_to_joint_goal_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response


    def send_vary_speed_request(self, vel):
        """ send_vary_speed_request """
        request = TaskParams.Request()
        request.cmd1 = vel

        while not self.vary_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('vary_speed: service not available, waiting again...')

        future = self.vary_speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'vary_speed: Service response: {response.status}')
        else:
            self.get_logger().warn('vary_speed: Service call failed')



    def send_stop_motion_request(self):
        """ send_stop_motion_request """
        request = CtrlParam.Request()

        while not self.stop_motion_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        future = self.stop_motion_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Stop motion response: {response.status}')
        else:
            self.get_logger().info('Service call failed!')


def main(args=None):
    """ main """
    rclpy.init(args=args)
    try:
        moveit_service_client = MoveItServiceClient()
        rclpy.spin(moveit_service_client)
    except KeyboardInterrupt:
        pass
    finally:
        moveit_service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()








# -----------------------------------------------------------------------------------
# JOB program goals demo sample yaml content:
# cobot_job_client:
#   ros__parameters:

#     goal_program: [
#       "1_movej",
#       "2_movej",
#       "3_movel",
#       "4_wait",
#       "5_gripper",
#     ]

#     1_movej: [j1, j2, j3, j4, j5, j6]
#     2_movej: [j1, j2, j3, j4, j5, j6]
#     3_movel: [x, y, z, qx, qy, qz, qw]
#     4_wait: [seconds]
#     5_gripper: [1 or 0]
# -----------------------------------------------------------------------------------
