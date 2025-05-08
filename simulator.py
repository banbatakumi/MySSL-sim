import pygame
import socket
import json
import time
import math
import threading
import sys

# Add project root to sys.path to allow importing config and lib
sys.path.append('.')
try:
    import config
except ImportError as e:
    print(f"Error importing project modules: {e}")
    print("Please ensure simulator.py is in the same directory as main.py, config.py and the lib/ folder.")
    sys.exit(1)

# --- Simulator Constants ---
ROBOT_DIAMETER_M = 0.18
ROBOT_RADIUS_M = ROBOT_DIAMETER_M / 2.0
COURT_WIDTH_M = config.COURT_WIDTH / 100.0
COURT_HEIGHT_M = config.COURT_HEIGHT / 100.0
ROBOT_MAX_SPEED_MPS = 1.5
ROBOT_MAX_ACCE_MPSS = 5
ROBOT_MAX_ANGULAR_SPEED_RADPS = 6 * math.pi  # 3 RPS

BALL_RADIUS_M = 0.0215
BALL_FRICTION_COEFF = 0.4
GRAVITY_MPSS = 9.81

SENSOR_FOV_HALF_ANGLE_RAD = math.radians(40)

KICK_POWER_TO_SPEED_MPS = 3.0 / 100.0
DRIBBLE_PULL_FACTOR = 8.0

# --- Initial Drawing Constants (will be updated for resizable window) ---
INITIAL_PIXELS_PER_METER = 300
INITIAL_SCREEN_PADDING_PX = 50
INITIAL_SCREEN_WIDTH_PX = int(
    COURT_WIDTH_M * INITIAL_PIXELS_PER_METER) + 2 * INITIAL_SCREEN_PADDING_PX
INITIAL_SCREEN_HEIGHT_PX = int(COURT_HEIGHT_M * INITIAL_PIXELS_PER_METER) + \
    2 * INITIAL_SCREEN_PADDING_PX

FIELD_MARKING_WIDTH_PX = 2
ROBOT_OUTLINE_WIDTH_PX = 2

COLOR_BACKGROUND = (0, 50, 0)
COLOR_FIELD_LINES = (255, 255, 255)
COLOR_YELLOW_ROBOT = (255, 255, 0)
COLOR_BLUE_ROBOT = (0, 0, 255)
COLOR_BALL = (255, 165, 0)
COLOR_TEXT = (200, 200, 200)
COLOR_ROBOT_FRONT = (200, 0, 0)
# ### ADDED ###: Color for debug vector
COLOR_DEBUG_VECTOR = (0, 255, 0)  # Green

SIMULATION_TIMESTEP = 0.01
FPS = 60

# --- Helper Functions ---


def normalize_angle_rad(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad

# --- Simulated Ball Class ---


class SimulatedBall:
    def __init__(self, x_m=0.0, y_m=0.0, radius_m=BALL_RADIUS_M):
        self.x_m = x_m
        self.y_m = y_m
        self.vx_mps = 0.0
        self.vy_mps = 0.0
        self.radius_m = radius_m
        self.is_dribbled_by = None

    def update_physics(self, dt):
        if self.is_dribbled_by:
            robot = self.is_dribbled_by
            overlap_depth_m = self.radius_m * 0.2
            dribbler_offset_m = robot.radius_m - overlap_depth_m

            target_x = robot.x_m + dribbler_offset_m * \
                math.cos(robot.angle_rad)
            target_y = robot.y_m + dribbler_offset_m * - \
                math.sin(robot.angle_rad)

            self.vx_mps = (target_x - self.x_m) * DRIBBLE_PULL_FACTOR
            self.vy_mps = (target_y - self.y_m) * DRIBBLE_PULL_FACTOR

            self.x_m += self.vx_mps * dt
            self.y_m += self.vy_mps * dt
        else:
            current_speed_mps = math.hypot(self.vx_mps, self.vy_mps)
            if current_speed_mps > 0:
                friction_decel = BALL_FRICTION_COEFF * GRAVITY_MPSS
                if current_speed_mps < friction_decel * dt:
                    self.vx_mps = 0.0
                    self.vy_mps = 0.0
                else:
                    self.vx_mps -= (self.vx_mps /
                                    current_speed_mps) * friction_decel * dt
                    self.vy_mps -= (self.vy_mps /
                                    current_speed_mps) * friction_decel * dt

            self.x_m += self.vx_mps * dt
            self.y_m += self.vy_mps * dt

        half_court_width = COURT_WIDTH_M / 2.0
        half_court_height = COURT_HEIGHT_M / 2.0

        if self.x_m - self.radius_m < -half_court_width:
            self.x_m = -half_court_width + self.radius_m
            self.vx_mps *= -0.7
        elif self.x_m + self.radius_m > half_court_width:
            self.x_m = half_court_width - self.radius_m
            self.vx_mps *= -0.7

        if self.y_m - self.radius_m < -half_court_height:
            self.y_m = -half_court_height + self.radius_m
            self.vy_mps *= -0.7
        elif self.y_m + self.radius_m > half_court_height:
            self.y_m = half_court_height - self.radius_m
            self.vy_mps *= -0.7

    def kick(self, robot, power):
        self.is_dribbled_by = None
        kick_speed_boost_mps = power * KICK_POWER_TO_SPEED_MPS
        kick_angle_rad = robot.angle_rad

        self.vx_mps = robot.vx_mps + \
            kick_speed_boost_mps * math.cos(kick_angle_rad)
        self.vy_mps = robot.vy_mps + \
            kick_speed_boost_mps * -math.sin(kick_angle_rad)

    def start_dribble(self, robot):
        self.is_dribbled_by = robot
        overlap_depth_m = self.radius_m * 0.2
        dribbler_offset_m = robot.radius_m - overlap_depth_m
        self.x_m = robot.x_m + dribbler_offset_m * math.cos(robot.angle_rad)
        self.y_m = robot.y_m + dribbler_offset_m * -math.sin(robot.angle_rad)
        self.vx_mps = robot.vx_mps
        self.vy_mps = robot.vy_mps

    def stop_dribble(self):
        if self.is_dribbled_by:
            self.is_dribbled_by = None

    def get_vision_data(self):
        return {
            "pos": (round(self.x_m * 100.0, 2), round(self.y_m * 100.0, 2))
        }

    def draw(self, screen, simulator):
        screen_x, screen_y = simulator.world_to_screen_pos(self.x_m, self.y_m)
        pygame.draw.circle(screen, COLOR_BALL, (screen_x, screen_y), int(
            self.radius_m * simulator.current_pixels_per_meter))

# --- Simulated Robot Class ---


class SimulatedRobot:
    def __init__(self, robot_id_num, color_name, initial_x_m, initial_y_m, initial_angle_deg, radius_m=ROBOT_RADIUS_M):
        self.color_name = color_name
        self.x_m = initial_x_m
        self.y_m = initial_y_m
        self.angle_rad = math.radians(initial_angle_deg)
        self.radius_m = radius_m

        self.vx_mps = 0.0
        self.vy_mps = 0.0
        self.omega_radps = 0.0

        self.current_command = None
        self.last_command_time = 0

        # ### ADDED ###: Store effective move angle for debug drawing
        self.debug_effective_move_angle_global_rad = 0.0
        self.debug_move_speed_mps = 0.0
        # ### END ADDITION ###

    def set_command(self, command_dict):
        self.current_command = command_dict.get("cmd")
        self.last_command_time = time.time()

    def update_physics(self, dt, ball):
        # Reset debug values
        # Default to current angle if no move cmd
        self.debug_effective_move_angle_global_rad = self.angle_rad
        self.debug_move_speed_mps = 0.0

        if self.current_command is None or (time.time() - self.last_command_time > 0.5):
            self.vx_mps *= 0.9
            self.vy_mps *= 0.9
            self.omega_radps *= 0.9
            if abs(self.vx_mps) < 0.01:
                self.vx_mps = 0
            if abs(self.vy_mps) < 0.01:
                self.vy_mps = 0
            if abs(self.omega_radps) < 0.01:
                self.omega_radps = 0
        else:
            cmd = self.current_command
            if cmd.get("stop", False):
                self.vx_mps = 0.0
                self.vy_mps = 0.0
                self.omega_radps = 0.0
                if ball.is_dribbled_by == self:
                    ball.stop_dribble()
            else:
                move_speed_mps = cmd.get("move_speed", 0.0)
                move_angle_relative_deg = cmd.get("move_angle", 0.0)
                move_angle_relative_rad = math.radians(move_angle_relative_deg)
                effective_move_angle_global_rad = normalize_angle_rad(
                    self.angle_rad + move_angle_relative_rad)

                # ### ADDED ###: Store for debug drawing
                self.debug_effective_move_angle_global_rad = effective_move_angle_global_rad
                self.debug_move_speed_mps = move_speed_mps
                # ### END ADDITION ###

                target_vx_global = move_speed_mps * \
                    math.cos(effective_move_angle_global_rad)
                target_vy_global = move_speed_mps * - \
                    math.sin(effective_move_angle_global_rad)

                max_accel_this_command = cmd.get(
                    "move_acce", ROBOT_MAX_ACCE_MPSS)
                if max_accel_this_command == 0:
                    max_accel_this_command = ROBOT_MAX_ACCE_MPSS
                elif max_accel_this_command == 1 and ROBOT_MAX_ACCE_MPSS > 1:
                    max_accel_this_command = ROBOT_MAX_ACCE_MPSS

                accel_x = (target_vx_global - self.vx_mps) / \
                    dt if dt > 0 else 0
                accel_y = (target_vy_global - self.vy_mps) / \
                    dt if dt > 0 else 0
                current_accel_magnitude = math.hypot(accel_x, accel_y)

                if current_accel_magnitude > max_accel_this_command:
                    scale = max_accel_this_command / current_accel_magnitude
                    accel_x *= scale
                    accel_y *= scale

                self.vx_mps += accel_x * dt
                self.vy_mps += accel_y * dt

                current_speed_magnitude = math.hypot(self.vx_mps, self.vy_mps)
                if current_speed_magnitude > ROBOT_MAX_SPEED_MPS:
                    scale = ROBOT_MAX_SPEED_MPS / current_speed_magnitude
                    self.vx_mps *= scale
                    self.vy_mps *= scale

                target_face_angle_deg = cmd.get(
                    "face_angle", math.degrees(self.angle_rad))
                target_face_angle_rad = math.radians(target_face_angle_deg)

                face_speed_limit_radps = cmd.get(
                    "face_speed", ROBOT_MAX_ANGULAR_SPEED_RADPS)
                if cmd.get("face_axis") == 0:
                    face_speed_limit_radps = ROBOT_MAX_ANGULAR_SPEED_RADPS

                angle_diff_rad = normalize_angle_rad(
                    target_face_angle_rad - self.angle_rad)

                if dt > 0:
                    if abs(angle_diff_rad) < abs(face_speed_limit_radps * dt):
                        self.omega_radps = angle_diff_rad / dt
                    else:
                        self.omega_radps = math.copysign(
                            face_speed_limit_radps, angle_diff_rad)
                else:
                    self.omega_radps = 0

                if abs(self.omega_radps) > ROBOT_MAX_ANGULAR_SPEED_RADPS:
                    self.omega_radps = math.copysign(
                        ROBOT_MAX_ANGULAR_SPEED_RADPS, self.omega_radps)

            kick_power = cmd.get("kick", 0)
            dribble_power = cmd.get("dribble", 0)

            kicked_this_step = False
            if isinstance(kick_power, (int, float)) and kick_power > 0:
                dist_to_ball_center = math.hypot(
                    ball.x_m - self.x_m, ball.y_m - self.y_m)
                max_kick_dist = self.radius_m + ball.radius_m * 0.5

                if dist_to_ball_center < max_kick_dist:
                    angle_to_ball_global_cw = - \
                        math.atan2(ball.y_m - self.y_m, ball.x_m - self.x_m)
                    relative_angle_to_ball = normalize_angle_rad(
                        angle_to_ball_global_cw - self.angle_rad)
                    if abs(relative_angle_to_ball) < math.radians(30):
                        if ball.is_dribbled_by == self:
                            ball.stop_dribble()
                        ball.kick(self, kick_power)
                        kicked_this_step = True

            if not kicked_this_step:
                if isinstance(dribble_power, (int, float)) and dribble_power > 0:
                    dist_center_to_center = math.hypot(
                        ball.x_m - self.x_m, ball.y_m - self.y_m)
                    touch_distance = self.radius_m + ball.radius_m - 0.005
                    initiate_dribble_max_dist = self.radius_m + ball.radius_m + 0.02

                    if dist_center_to_center < initiate_dribble_max_dist:
                        angle_to_ball_global_cw = - \
                            math.atan2(ball.y_m - self.y_m,
                                       ball.x_m - self.x_m)
                        relative_angle_to_ball = normalize_angle_rad(
                            angle_to_ball_global_cw - self.angle_rad)
                        if abs(relative_angle_to_ball) < math.radians(70):
                            if dist_center_to_center < touch_distance or ball.is_dribbled_by == self:
                                ball.start_dribble(self)
                else:
                    if ball.is_dribbled_by == self:
                        ball.stop_dribble()

        self.x_m += self.vx_mps * dt
        self.y_m += self.vy_mps * dt
        self.angle_rad = normalize_angle_rad(
            self.angle_rad + self.omega_radps * dt)

        half_court_width = COURT_WIDTH_M / 2.0
        half_court_height = COURT_HEIGHT_M / 2.0
        self.x_m = max(-half_court_width + self.radius_m,
                       min(self.x_m, half_court_width - self.radius_m))
        self.y_m = max(-half_court_height + self.radius_m,
                       min(self.y_m, half_court_height - self.radius_m))

    def get_vision_data(self):
        return {
            "type": self.color_name,
            "angle": round(math.degrees(self.angle_rad), 2),
            "pos": (round(self.x_m * 100.0, 2), round(self.y_m * 100.0, 2))
        }

    def get_sensor_data(self, ball):
        photo_front = False
        photo_back = False

        dx = ball.x_m - self.x_m
        dy = ball.y_m - self.y_m
        dist_center_to_center = math.hypot(dx, dy)

        overlap_depth_m = ball.radius_m * 0.2
        dribbler_offset_m = self.radius_m - overlap_depth_m

        if dist_center_to_center <= dribbler_offset_m + 0.005:
            angle_to_ball_global_cw = -math.atan2(dy, dx)
            relative_angle_to_ball_cw = normalize_angle_rad(
                angle_to_ball_global_cw - self.angle_rad)

            if abs(relative_angle_to_ball_cw) < SENSOR_FOV_HALF_ANGLE_RAD:
                photo_front = True

            angle_to_back_sensor_cw = normalize_angle_rad(
                relative_angle_to_ball_cw - math.pi)
            if abs(angle_to_back_sensor_cw) < SENSOR_FOV_HALF_ANGLE_RAD:
                photo_back = True

        return {
            "type": "sensor_data",
            "photo": {"front": photo_front, "back": photo_back}
        }

    def draw(self, screen, simulator):
        robot_color_rgb = COLOR_YELLOW_ROBOT if self.color_name == "yellow" else COLOR_BLUE_ROBOT
        screen_x, screen_y = simulator.world_to_screen_pos(self.x_m, self.y_m)

        robot_screen_radius = int(
            self.radius_m * simulator.current_pixels_per_meter)
        if robot_screen_radius < 1:
            robot_screen_radius = 1

        pygame.draw.circle(screen, robot_color_rgb,
                           (screen_x, screen_y), robot_screen_radius)
        pygame.draw.circle(screen, (0, 0, 0), (screen_x, screen_y),
                           robot_screen_radius, ROBOT_OUTLINE_WIDTH_PX)

        front_indicator_len_world = self.radius_m * 0.8
        front_indicator_len_screen = front_indicator_len_world * \
            simulator.current_pixels_per_meter

        front_x_indicator = screen_x + \
            front_indicator_len_screen * math.cos(self.angle_rad)
        front_y_indicator = screen_y + \
            front_indicator_len_screen * math.sin(self.angle_rad)
        pygame.draw.line(screen, COLOR_ROBOT_FRONT, (screen_x, screen_y), (int(
            front_x_indicator), int(front_y_indicator)), 3)

        # ### ADDED ###: Draw debug move vector
        if simulator.show_debug_vectors and self.current_command and not self.current_command.get("stop", False) and self.debug_move_speed_mps > 0:
            # Scale vector length by speed (e.g., max speed = 0.5m on screen)
            # Max visual length of vector can be, for example, 2 * robot_radius
            # Or a fixed world length like 0.3m
            vec_len_world = 0.3 * \
                (self.debug_move_speed_mps / ROBOT_MAX_SPEED_MPS)
            vec_len_screen = vec_len_world * simulator.current_pixels_per_meter

            vec_end_x_world = self.x_m + vec_len_world * \
                math.cos(self.debug_effective_move_angle_global_rad)
            vec_end_y_world = self.y_m + vec_len_world * - \
                math.sin(
                    self.debug_effective_move_angle_global_rad)  # World Y is up

            vec_end_screen_x, vec_end_screen_y = simulator.world_to_screen_pos(
                vec_end_x_world, vec_end_y_world)

            pygame.draw.line(screen, COLOR_DEBUG_VECTOR, (screen_x,
                             screen_y), (vec_end_screen_x, vec_end_screen_y), 2)
        # ### END ADDITION ###


# --- UDP Communication Handler for Simulator ---
class SimulatorUDP:
    def __init__(self, robots_dict, ball_sim, simulator_instance_ref):
        self.robots = robots_dict
        self.ball = ball_sim
        self.running = True
        self.simulator_instance = simulator_instance_ref

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.controller_listen_ip = config.LISTEN_IP
        if self.controller_listen_ip == "0.0.0.0":
            self.controller_listen_ip = "127.0.0.1"

        self.cmd_sockets = {}
        self.cmd_threads = {}

        if config.ENABLE_YELLOW_ROBOT and "yellow" in self.robots:
            try:
                ip = config.YELLOW_ROBOT_IP
                port = config.YELLOW_SEND_PORT
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((ip, port))
                sock.settimeout(0.1)
                self.cmd_sockets["yellow"] = sock
                print(
                    f"Simulator listening for Yellow robot commands on {ip}:{port}")
            except OSError as e:
                print(
                    f"ERROR: Could not bind to {ip}:{port} for Yellow robot commands: {e}")

        if config.ENABLE_BLUE_ROBOT and "blue" in self.robots:
            try:
                ip = config.BLUE_ROBOT_IP
                port = config.BLUE_SEND_PORT
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((ip, port))
                sock.settimeout(0.1)
                self.cmd_sockets["blue"] = sock
                print(
                    f"Simulator listening for Blue robot commands on {ip}:{port}")
            except OSError as e:
                print(
                    f"ERROR: Could not bind to {ip}:{port} for Blue robot commands: {e}")

        for color, sock in self.cmd_sockets.items():
            thread = threading.Thread(
                target=self._receive_commands, args=(color, sock), daemon=True)
            self.cmd_threads[color] = thread
            thread.start()

    def _receive_commands(self, robot_color, sock):
        while self.running:
            try:
                data, _ = sock.recvfrom(config.BUFFER_SIZE)
                command_dict = json.loads(data.decode('utf-8'))
                if robot_color in self.robots:
                    self.robots[robot_color].set_command(command_dict)
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                print(f"Sim received invalid JSON command for {robot_color}")
            except Exception as e:
                if self.running:
                    print(f"Error receiving command for {robot_color}: {e}")
                break

    def send_vision_data(self):
        vision_payload = {
            "fps": round(self.simulator_instance.clock.get_fps(), 2),
            "is_calibrated": True,
        }
        yellow_robot_data_list = []
        blue_robot_data_list = []

        if "yellow" in self.robots and config.ENABLE_YELLOW_ROBOT:
            yellow_robot_data_list.append(
                self.robots["yellow"].get_vision_data())
        if "blue" in self.robots and config.ENABLE_BLUE_ROBOT:
            blue_robot_data_list.append(self.robots["blue"].get_vision_data())

        vision_payload["yellow_robots"] = yellow_robot_data_list
        vision_payload["blue_robots"] = blue_robot_data_list
        vision_payload["orange_balls"] = [
            self.ball.get_vision_data()] if self.ball else []

        try:
            msg = json.dumps(vision_payload).encode('utf-8')
            self.send_socket.sendto(
                msg, (self.controller_listen_ip, config.VISION_LISTEN_PORT))
        except Exception as e:
            print(f"Error sending vision data: {e}")

    def send_sensor_data(self):
        if "yellow" in self.robots and config.ENABLE_YELLOW_ROBOT:
            sensor_payload = self.robots["yellow"].get_sensor_data(self.ball)
            try:
                msg = json.dumps(sensor_payload).encode('utf-8')
                self.send_socket.sendto(
                    msg, (self.controller_listen_ip, config.YELLOW_SENSOR_LISTEN_PORT))
            except Exception as e:
                print(f"Error sending yellow sensor data: {e}")

        if "blue" in self.robots and config.ENABLE_BLUE_ROBOT:
            sensor_payload = self.robots["blue"].get_sensor_data(self.ball)
            try:
                msg = json.dumps(sensor_payload).encode('utf-8')
                self.send_socket.sendto(
                    msg, (self.controller_listen_ip, config.BLUE_SENSOR_LISTEN_PORT))
            except Exception as e:
                print(f"Error sending blue sensor data: {e}")

    def stop(self):
        self.running = False
        for color, thread in self.cmd_threads.items():
            if thread.is_alive():
                thread.join(timeout=0.2)
        for sock in self.cmd_sockets.values():
            sock.close()
        if self.send_socket:
            self.send_socket.close()


# --- Main Simulator Class ---

class Simulator:
    def __init__(self):
        pygame.init()
        pygame.font.init()

        self.current_screen_width_px = INITIAL_SCREEN_WIDTH_PX
        self.current_screen_height_px = INITIAL_SCREEN_HEIGHT_PX
        self.current_screen_padding_px = INITIAL_SCREEN_PADDING_PX
        self.current_pixels_per_meter = INITIAL_PIXELS_PER_METER

        self.screen = pygame.display.set_mode(
            (self.current_screen_width_px, self.current_screen_height_px),
            pygame.RESIZABLE
        )
        self._update_drawing_parameters()

        pygame.display.set_caption("Robot Soccer Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 16)
        self.running = True
        # ### ADDED ###: Flag for showing debug vectors
        self.show_debug_vectors = False

        self.robots = {}
        if config.ENABLE_YELLOW_ROBOT:
            self.robots["yellow"] = SimulatedRobot(
                0, "yellow", -COURT_WIDTH_M/4, 0, 0)
        if config.ENABLE_BLUE_ROBOT:
            blue_initial_x = COURT_WIDTH_M/4 if config.ENABLE_YELLOW_ROBOT else 0
            self.robots["blue"] = SimulatedRobot(
                0, "blue", blue_initial_x, 0, 180)

        self.ball = SimulatedBall(0, 0)
        self.udp_handler = SimulatorUDP(self.robots, self.ball, self)

        self.last_vision_send_time = 0
        self.last_sensor_send_time = 0
        self.data_send_interval = config.CONTROL_LOOP_INTERVAL

    def world_to_screen_pos(self, x_m, y_m):
        center_x_px = self.current_screen_width_px / 2
        center_y_px = self.current_screen_height_px / 2
        screen_x = center_x_px + x_m * self.current_pixels_per_meter
        screen_y = center_y_px - y_m * self.current_pixels_per_meter
        return int(screen_x), int(screen_y)

    def screen_to_world_pos(self, screen_x_px, screen_y_px):
        center_x_px = self.current_screen_width_px / 2
        center_y_px = self.current_screen_height_px / 2

        if self.current_pixels_per_meter == 0:
            return 0.0, 0.0

        world_x_m = (screen_x_px - center_x_px) / self.current_pixels_per_meter
        world_y_m = (center_y_px - screen_y_px) / self.current_pixels_per_meter
        return world_x_m, world_y_m

    def _update_drawing_parameters(self):
        effective_width_for_court = self.current_screen_width_px - \
            2 * self.current_screen_padding_px
        effective_height_for_court = self.current_screen_height_px - \
            2 * self.current_screen_padding_px

        if COURT_WIDTH_M > 0 and COURT_HEIGHT_M > 0 and \
           effective_width_for_court > 0 and effective_height_for_court > 0:
            ppm_if_width_limited = effective_width_for_court / COURT_WIDTH_M
            ppm_if_height_limited = effective_height_for_court / COURT_HEIGHT_M
            self.current_pixels_per_meter = min(
                ppm_if_width_limited, ppm_if_height_limited)
        else:
            self.current_pixels_per_meter = 1

        if self.current_pixels_per_meter <= 0:
            self.current_pixels_per_meter = 1

    def run(self):
        while self.running:
            dt = SIMULATION_TIMESTEP

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.current_screen_width_px = event.w
                    self.current_screen_height_px = event.h
                    self.screen = pygame.display.set_mode(
                        (self.current_screen_width_px,
                         self.current_screen_height_px),
                        pygame.RESIZABLE
                    )
                    self._update_drawing_parameters()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        mouse_x_px, mouse_y_px = event.pos
                        world_x_m, world_y_m = self.screen_to_world_pos(
                            mouse_x_px, mouse_y_px)

                        half_court_width = COURT_WIDTH_M / 2.0 - self.ball.radius_m
                        half_court_height = COURT_HEIGHT_M / 2.0 - self.ball.radius_m

                        clamped_world_x_m = max(-half_court_width,
                                                min(world_x_m, half_court_width))
                        clamped_world_y_m = max(-half_court_height,
                                                min(world_y_m, half_court_height))

                        if self.ball.is_dribbled_by:
                            self.ball.stop_dribble()

                        self.ball.x_m = clamped_world_x_m
                        self.ball.y_m = clamped_world_y_m
                        self.ball.vx_mps = 0.0
                        self.ball.vy_mps = 0.0
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        if self.ball.is_dribbled_by:
                            self.ball.stop_dribble()
                        self.ball = SimulatedBall(0, 0)
                        self.udp_handler.ball = self.ball
                    # ### ADDED ###: Toggle debug vectors with 'm' key
                    elif event.key == pygame.K_m:
                        self.show_debug_vectors = not self.show_debug_vectors
                    # ### END ADDITION ###
                    elif event.key == pygame.K_ESCAPE:
                        self.running = False

            if dt <= 0:
                time.sleep(0.01)
                continue

            for robot in self.robots.values():
                robot.update_physics(dt, self.ball)
            self.ball.update_physics(dt)

            current_time = time.time()
            if current_time - self.last_vision_send_time >= self.data_send_interval:
                self.udp_handler.send_vision_data()
                self.last_vision_send_time = current_time

            if current_time - self.last_sensor_send_time >= self.data_send_interval:
                self.udp_handler.send_sensor_data()
                self.last_sensor_send_time = current_time

            self.draw()
            self.clock.tick(FPS)

        self.cleanup()

    def draw_field(self):
        self.screen.fill(COLOR_BACKGROUND)
        hw_m, hh_m = COURT_WIDTH_M / 2.0, COURT_HEIGHT_M / 2.0

        field_top_left_screen_x, field_top_left_screen_y = self.world_to_screen_pos(
            -hw_m, hh_m)
        field_width_px = max(
            1, int(COURT_WIDTH_M * self.current_pixels_per_meter))
        field_height_px = max(
            1, int(COURT_HEIGHT_M * self.current_pixels_per_meter))

        field_rect_on_screen = pygame.Rect(field_top_left_screen_x, field_top_left_screen_y,
                                           field_width_px, field_height_px)

        pygame.draw.rect(self.screen, COLOR_FIELD_LINES,
                         field_rect_on_screen, FIELD_MARKING_WIDTH_PX)

        center_x_screen, center_y_screen = self.world_to_screen_pos(0, 0)
        center_circle_radius_px = int(0.25 * self.current_pixels_per_meter)
        if center_circle_radius_px > 0:
            pygame.draw.circle(self.screen, COLOR_FIELD_LINES, (center_x_screen, center_y_screen),
                               center_circle_radius_px, FIELD_MARKING_WIDTH_PX)

        cl_top_s = self.world_to_screen_pos(0, hh_m)
        cl_bot_s = self.world_to_screen_pos(0, -hh_m)
        pygame.draw.line(self.screen, COLOR_FIELD_LINES,
                         cl_top_s, cl_bot_s, FIELD_MARKING_WIDTH_PX)

    def draw_hud(self):
        # ### MODIFIED ###: Only draw FPS
        y_offset = 10
        sim_fps_text = f"Sim FPS: {self.clock.get_fps():.1f}"
        surf_fps = self.font.render(sim_fps_text, True, COLOR_TEXT)
        self.screen.blit(surf_fps, (self.current_screen_width_px -
                         surf_fps.get_width() - 10, y_offset))
        # ### END MODIFICATION ###

    def draw(self):
        self.draw_field()
        for robot in self.robots.values():
            # Pass simulator instance for debug flag
            robot.draw(self.screen, self)
        self.ball.draw(self.screen, self)
        self.draw_hud()
        pygame.display.flip()

    def cleanup(self):
        print("Simulator shutting down...")
        self.udp_handler.stop()
        pygame.quit()
        print("Simulator finished.")


if __name__ == "__main__":
    print("Starting Robot Soccer Simulator...")
    if not (config.ENABLE_YELLOW_ROBOT or config.ENABLE_BLUE_ROBOT):
        print("Warning: No robots enabled in config.py. Simulator will run but control no robots.")

    sim = Simulator()
    sim.run()
