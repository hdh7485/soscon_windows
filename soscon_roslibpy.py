import os
import time
import copy
from soscon.env import Env
from soscon.status import Status
from soscon.data.observation import Observation
import roslibpy as rp

class RobotController():

    LIDAR_DATA_SIZE = 360
    IR_DATA_SIZE = 5

    def __init__(self):
        self._env = Env()

        self._total_dx = 0
        self._total_dy = 0
        self._prev_obs = None

        self._env.on_observation = self.on_observation
        self.stop()

    def _cls(self):
        os.system("cls" if os.name == "nt" else "clear")

    def on_observation(self, obs: Observation, status: Status):
        if status is Status.OK:
            self._prev_obs = copy.deepcopy(obs)
            self._total_dx += obs.delta.x
            self._total_dy += obs.delta.y

    def print_current_state(self):
        if self._prev_obs is None:
            print("[ERROR] No measured not yet")
            return

        if len(self._prev_obs.lidar) != self.LIDAR_DATA_SIZE:
            print("[ERROR] LiDAR was not measured")
            return

        if len(self._prev_obs.ir) != self.IR_DATA_SIZE:
            print("[ERROR] IR was not measured")
            return

        self._cls()

        print("[Compass] %.1f deg" % self._prev_obs.compass)
        print("[Encoder] Left: %d, Right: %d" % (self._prev_obs.encoder.left, self._prev_obs.encoder.right))
        print("[IR] %.1f %.1f %.1f %.1f %.1f cm" % (self._prev_obs.ir[0], self._prev_obs.ir[1], self._prev_obs.ir[2], self._prev_obs.ir[3], self._prev_obs.ir[4]))
        print("[LiDAR] %.1f %.1f %.1f cm" % (self._prev_obs.lidar[330], self._prev_obs.lidar[0], self._prev_obs.lidar[30]))
        print("[Delta] X: %.1f, Y: %.1f" % (self._total_dx, self._total_dy))

    def stop(self):
        print("[STOP]")
        self._env.control_robot(0.0, 0.0)
        self._total_dx = 0
        self._total_dy = 0

    def move(self, linear_velocity: float, angular_velocity: float):
        # print("[MOVE] Linear(cm/s): %.1f, Angular(deg/s): %.1f" % (linear_velocity, angular_velocity))
        self._env.control_robot(linear_velocity, angular_velocity)

    def check_front_ir(self) -> float:
        if self._prev_obs is None:
            raise RuntimeError("[ERROR] No measured not yet")

        if len(self._prev_obs.ir) != self.IR_DATA_SIZE:
            raise RuntimeError("[ERROR] IR was not measured")

        return min(self._prev_obs.ir[1:4])

    def check_front_lidar(self) -> float:
        if self._prev_obs is None:
            raise RuntimeError("[ERROR] No measured not yet")

        if len(self._prev_obs.lidar) != self.LIDAR_DATA_SIZE:
            raise RuntimeError("[ERROR] LiDAR was not measured")

        left_min = min(self._prev_obs.lidar[345:360])
        right_min = min(self._prev_obs.lidar[0:16])
        return min([left_min, right_min])

    def move_forward(self, distance_cm: float):
        self.stop()
        print("[MOVE_FORWARD] Distance(cm): %.1f" % (distance_cm))

        while True:
            time.sleep(0.05)

            target_velocity = distance_cm - self._total_dx
            if target_velocity <= 5:
                target_velocity = 5
            elif target_velocity > 30:
                target_velocity = 30

            robot.move(target_velocity, 0)
            if self._total_dx >= distance_cm:
                self.stop()
                break

    def rotate_cw(self, target_angle_deg: float):
        self.stop()
        print("[ROTATE_CW] Angle(deg): %.1f" % (target_angle_deg))

        prev_compass = self._prev_obs.compass
        while True:
            robot.print_current_state()
            time.sleep(0.05)

            current_compass = self._prev_obs.compass
            delta_deg = current_compass - prev_compass
            if delta_deg < 0:
                delta_deg = 360 + current_compass - prev_compass

            target_velocity = target_angle_deg - delta_deg
            if target_velocity <= 5:
                target_velocity = 5
            elif target_velocity > 30:
                target_velocity = 30

            if delta_deg >= target_angle_deg:
                self.stop()
                break
            else:
                self.move(0, -1 * target_velocity)


    def callback(self, message):
        print(message['linear'], message['angular'])
        self.move(message['linear']['x'], message['angular']['z'])

if __name__ == "__main__":    
    ros = rp.Ros(host = '192.168.24.203', port=9090)
    sub = rp.Topic(ros,
                    'cmd_vel',
                    'geometry_msgs/Twist',
                    queue_length = 10)
    

    robot = RobotController()
    sub.subscribe(robot.callback)
    ros.run_forever()