import random
import pygame
import math
import math
import numpy as np
from scipy.signal import StateSpace
import matplotlib.pyplot as plt

dt = 0.01

# Robot Parameters
mass = 15 / 2.205
mu = 0.1
inertia = 0.1
track_radius = 6 * 0.0254
gear_ratio = 1/1.5
wheel_radius = 2 * 0.0254
num_motors_per_side = 3

# Motor Parameters
nominal_voltage = 12.0
stall_torque = 2.1286320584285905 * num_motors_per_side
stall_current = 2.5029144189407573 * num_motors_per_side
free_current = 0.13 * num_motors_per_side
free_speed = 200

# convert rpm to rad/s
free_speed = free_speed / 60 * (2 * np.pi)


resistance = nominal_voltage / stall_current
torque_const = stall_torque / stall_current
angular_vel_const = free_speed / (nominal_voltage - resistance * free_current)

# Inputs
input_voltage_left = 10
input_voltage_right = 12


C1 = -(gear_ratio ** 2 * torque_const) / \
    (angular_vel_const * resistance * wheel_radius**2)
C2 = (gear_ratio * torque_const) / (resistance * wheel_radius)

A = np.array(
    [
        [0, -1/(2 * track_radius), 1/(2 * track_radius)],
        [0, (1/mass + track_radius**2 / inertia) * C1,
         (1/mass - track_radius**2 / inertia) * C1],
        [0, (1/mass - track_radius**2 / inertia) * C1,
         (1/mass + track_radius**2 / inertia) * C1]
    ]
)
B = np.array(
    [
        [0, 0],
        [(1/mass + track_radius**2 / inertia) * C2,
         (1/mass - track_radius**2 / inertia) * C2],
        [(1/mass - track_radius**2 / inertia) * C2,
         (1/mass + track_radius**2 / inertia) * C2]
    ]
)

C = np.eye(3)
D = np.zeros((3, 2))

sys = StateSpace(A, B, C, D)

sim = sys.to_discrete(dt)





def inches_to_pixels(inches):
    return inches * (4881.0 / 140.41) / 5

def pixels_to_inches(pixels):
    return pixels * (140.41 / 4881.0) * 5

dt = 0.01
class Pose:
    leftSpeed = 0
    rightSpeed = 0
    angularSpeed = 0
    theta = 0
    state = np.zeros((3, 1))
    def __init__(self, x, y, theta):
        self.x_pos = x
        self.y_pos = y
        self.xSpeed = 0
        self.ySpeed = 0
        self.theta = theta
        self.lastLeftWheelSpeed = 0
        self.lastRightWheelSpeed = 0
    def move(self, right, left, frameTime):
        # left = max(min(left, 1), -1)
        # right = max(min(right, 1), -1)
        max_speed = inches_to_pixels(4 * 3.14159 * (300 / 60)) * 10000 #62.8318
        max_accel = inches_to_pixels(1500000)


        leftWheelTarget = min(max(left, -max_speed), max_speed)
        rightWheelTarget = min(max(right, -max_speed), max_speed)
        
        self.state = sim.A @ self.state + \
            sim.B @ np.array([[left], [right]])

        leftSpeed = self.state[1][0]
        rightSpeed = self.state[2][0]


        if leftSpeed == rightSpeed:
            xSpeed = leftSpeed * math.cos(self.theta) * dt
            ySpeed = leftSpeed * math.sin(self.theta) * dt
            angularSpeed = 0
        else:
            turn_radius = track_radius * \
                (leftSpeed + rightSpeed) / (rightSpeed - leftSpeed)
            ICC_x = self.x_pos - inches_to_pixels(turn_radius/0.0254) * math.sin(self.theta)
            ICC_y = self.y_pos + \
                inches_to_pixels(turn_radius/0.0254) * math.cos(self.theta)

            # draw line to ICC
            # print((int(
            #     ICC_x), int(ICC_y)))
            pygame.draw.circle(screen, (0, 255, 0), (int(
                ICC_x), int(ICC_y)), abs(int(turn_radius)), 1)

            vel = (leftSpeed + rightSpeed) / 2
            omega = (rightSpeed - leftSpeed) / (track_radius * 2)

            # print(pixels_to_inches(ICC_R))
            # centipetal_force = mass * vel ** 2 / abs(turn_radius)
            # centipetal_force = 0
            # friction_force = mu * 9.81 * mass

            # if centipetal_force < friction_force:
            #     centipetal_force = friction_force
            
            # net_force = centipetal_force - friction_force

            # net_accel = net_force / mass

            # net_accel = math.copysign(net_accel, turn_radius)

            # Apply Acceleration
        
            xSpeed = vel * math.cos(self.theta) * dt
            ySpeed = vel * math.sin(self.theta) * dt

            angularSpeed = omega * dt
            
            # centrifugalAccel = 2 * self.angularSpeed * ICC_R
        # print(pixels_to_inches(x_posSpeed), pixels_to_inches(self.ySpeed))

        # Apply centrifugal acceleration
        # x_pos -= math.sin(self.theta) * centrifugalAccel * dt
        # self.y += math.cos(self.theta ) * centrifugalAccel * dt
        self.x_pos += inches_to_pixels(xSpeed / 0.0254)
        self.y_pos += inches_to_pixels(ySpeed / 0.0254)
        self.theta += angularSpeed
    def draw(self):
        robot_size = inches_to_pixels(15)
        image = pygame.Surface((robot_size, robot_size), pygame.SRCALPHA)
        pygame.draw.polygon(image, (255, 255, 255), ((0, 0), (0, robot_size), (robot_size, robot_size), (robot_size, 0)))
        # Mark front of robot
        pygame.draw.polygon(image, (255, 0, 0), ((robot_size, 0), (robot_size - inches_to_pixels(2), 0), (robot_size - inches_to_pixels(2), robot_size), (robot_size, robot_size)))
        orig_image = image

        rect = image.get_rect()

        rect.center = (self.x_pos, self.y_pos)
        image, rect = rotate(orig_image, rect, -self.theta * 180 / math.pi)
        screen.blit(image, rect)
    def __repr__(this):
        return "Pose(" + str(this.x_pos) + ", " + str(this.y_pos) + ", " + str(this.theta) + ")"
    def get_pose(this):
        return (pixels_to_inches(this.x_pos), pixels_to_inches(this.y_pos), this.theta)
    
def rotate(image, rect, angle):
    # Rotate the original image without modifying it.
    new_image = pygame.transform.rotate(image, angle)
    # Get a new rect with the center of the old rect.
    rect = new_image.get_rect(center=rect.center)
    return new_image, rect

# pose = Pose(inches_to_pixels(18), inches_to_pixels(18), 0)


pygame.init()
screen = pygame.display.set_mode((1000, 1000))
clock = pygame.time.Clock()

bg = pygame.image.load("Over_Under_Render.png")
bg = pygame.transform.scale(bg, (1000, 1000))

image = pygame.Surface((100, 100), pygame.SRCALPHA)
pygame.draw.polygon(image, (255, 255, 255), ((0, 0), (0, 100), (100, 100), (100, 0)))
# Mark front of robot
pygame.draw.polygon(image, (255, 0, 0), ((100, 0), (90, 0), (90, 100), (100, 100)))
orig_image = image


robot = Pose(inches_to_pixels(72), inches_to_pixels(72), -3.14159 / 2)
counter = 0


# joystick = []

for i in range(0, pygame.joystick.get_count()):
    joystick = pygame.joystick.Joystick(i)
joystick.init()


def wheel_speeds(velocity, angular_velocity):
    return (velocity - angular_velocity * (0.5 * 11), velocity + angular_velocity * (0.5 * 11))


speeds = []

lin_vels = []
ang_vels = []
# with open("cooperPath.txt", "r") as f:
#     f.readline()
#     lin_vels = f.readline().strip().replace("[", "").replace("]", "").split(", ")
#     ang_vels = f.readline().strip().replace("[", "").replace("]", "").split(", ")

# lin_vels = [float(i) for i in lin_vels]
# ang_vels = [float(i) for i in ang_vels]

# for i in range(len(lin_vels)):
#     speeds.append(wheel_speeds(lin_vels[i], ang_vels[i]))

# with open("new_arrana_profile.txt", "w") as f:
#     for i in range(len(speeds)):
#         f.write(f"{speeds[i][0]},{speeds[i][1]}\n")

i = 0
speeds.append((0, 0))


speeds = []
pathPoints = []
with open("profile.txt", "r") as f:
    path = f.readline().strip().split("|")[:-1]
    for line in path:
        line = line.strip()
        line = line.split(",")
        pathPoints.append((float(line[0]), float(line[1])))

    lines = f.readline().split("|")[:-1]
    with open("readable.txt", "w") as f:
        for line in lines:
            print(line)
            line = line.strip()
            line = ",".join(line.split(" "))
            line = line.split(",")
            speeds.append((float(line[0]), float(line[1]), float(line[2])))
            f.write(f"{speeds[-1][0]:.2f},{speeds[-1][1]:.2f},{speeds[-1][2]:.2f}\n")


# use linear interpolation to get the current velocity
def get_cur_vel(dist):
    index = int(dist / 0.1)
    if index >= len(speeds) - 1:
        return (0, 0)
    else:
        return (speeds[index][1] + (dist - index * 0.1) * (speeds[index + 1][1] - speeds[index][1]), speeds[index][2] + (dist - index * 0.1) * (speeds[index + 1][2] - speeds[index][2]))

positions = []

curDist = 0.2
lastPos = robot.get_pose()

while True:
    realDt = clock.tick(100)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
    keys=pygame.key.get_pressed()
    screen.blit(bg, (0, 0))

    left_input = -joystick.get_axis(1) * 12
    right_input = -joystick.get_axis(3)  * 12
    # left_input, right_input = wheel_speeds(*get_cur_vel(curDist))
    if i == -1:
        left_input = 0
        right_input = 0
    # print(f"Left In :{right_input:.2f}, right in: {left_input:.2f}")
    robot.move(left_input, right_input, realDt)
    positions.append((robot.x_pos, robot.y_pos))
    for pos in positions:
        pygame.draw.circle(screen, (255, 0, 0), (int(pos[0]), int(pos[1])), 2)
    # for pos in pathPoints:
    #     pygame.draw.circle(screen, (0, 255, 0), (inches_to_pixels(pos[0]) + inches_to_pixels(84), inches_to_pixels(pos[1]) + inches_to_pixels(108)), 2)
    robot.draw()
    
    # if i < len(speeds) - 1 and i != -1:
    #     i += 1
    #     print(f"Left In :{right_input:.2f}, right in: {left_input:.2f}")
    # else:
    #     i = -1
    #     print(f"X: {pixels_to_inches(robot.x):.2f}, Y: {pixels_to_inches(robot.y):.2f}, Theta: {robot.theta * (180 / math.pi):.2f}")
    curPose = robot.get_pose()
    curDist += math.sqrt((lastPos[0] - curPose[0]) ** 2 + (lastPos[1] - curPose[1]) ** 2)
    lastPos = curPose
    pygame.display.update()
