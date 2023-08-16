import random
import pygame
import math

def inches_to_pixels(inches):
    return inches * (4881.0 / 140.41) / 5

def pixels_to_inches(pixels):
    return pixels * (140.41 / 4881.0) * 5

dt = 0.01
class Pose:
    x = 0
    y = 0
    leftSpeed = 0
    rightSpeed = 0
    angularSpeed = 0
    theta = 0
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.xSpeed = 0
        self.ySpeed = 0
        self.theta = theta
        self.lastLeftWheelSpeed = 0
        self.lastRightWheelSpeed = 0
    def move(self, right, left, frameTime):
        # left = max(min(left, 1), -1)
        # right = max(min(right, 1), -1)
        max_speed = inches_to_pixels(4 * 3.14159 * (300 / 60)) * 1000 #62.8318
        max_accel = inches_to_pixels(150.5)



        leftWheelTarget = min(max(left, -max_speed), max_speed)
        rightWheelTarget = min(max(right, -max_speed), max_speed)
        
        leftWheelTarget = inches_to_pixels(left)
        rightWheelTarget = inches_to_pixels(right)
        if (leftWheelTarget - self.lastLeftWheelSpeed) / dt > max_accel:
            print(f"Left Limited at {(leftWheelTarget - self.lastLeftWheelSpeed) / dt} m/s^2")
            leftWheelSpeed = self.lastLeftWheelSpeed + max_accel * dt
        elif (leftWheelTarget - self.lastLeftWheelSpeed) / dt < -max_accel:
            print(f"Left Limited at {(leftWheelTarget - self.lastLeftWheelSpeed) / dt} m/s^2")
            leftWheelSpeed = self.lastLeftWheelSpeed - max_accel * dt
        else:
            leftWheelSpeed = leftWheelTarget

        if (rightWheelTarget - self.lastRightWheelSpeed) / dt > max_accel:
            print(f"Right Limited at a{pixels_to_inches((rightWheelTarget - self.lastRightWheelSpeed) / dt)} m/s^2")
            rightWheelSpeed = self.lastRightWheelSpeed + max_accel * dt
        elif (rightWheelTarget - self.lastRightWheelSpeed) / dt < -max_accel:
            print(f"Right Limited at b{(rightWheelTarget - self.lastRightWheelSpeed) / dt} m/s^2")
            rightWheelSpeed = self.lastRightWheelSpeed - max_accel * dt
        else:
            rightWheelSpeed = rightWheelTarget
            
        self.lastLeftWheelSpeed = leftWheelSpeed
        self.lastRightWheelSpeed = rightWheelSpeed
        # skid steer drive
        # left and right are between -1 and 1

        track_width = inches_to_pixels(15)


        
        centrifugalAccel = 0
        if leftWheelSpeed == rightWheelSpeed:
            self.xSpeed = leftWheelSpeed * math.cos(self.theta) * dt
            self.ySpeed = leftWheelSpeed * math.sin(self.theta)  * dt
            self.angularSpeed = 0
        else:
            ICC_R = (track_width / 2) * ((leftWheelSpeed + rightWheelSpeed) / (rightWheelSpeed - leftWheelSpeed))
            omega = (rightWheelSpeed - leftWheelSpeed) / track_width


            # print(pixels_to_inches(ICC_R))
            
            ICC_x = self.x - ICC_R * math.sin(self.theta)
            ICC_y = self.y + ICC_R * math.cos(self.theta)

            xSpeed = (math.cos(omega * dt) * (self.x - ICC_x) - math.sin(omega * dt) * (self.y - ICC_y) + ICC_x) - self.x
            ySpeed = (math.sin(omega * dt) * (self.x - ICC_x) + math.cos(omega * dt) * (self.y - ICC_y) + ICC_y) - self.y
            
            # print(self.xSpeed - xSpeed, self.ySpeed - ySpeed)

            # Add vector components

            # # Get vector form
            # nextSpeedAngle = math.atan2(nextYSpeed, nextXSpeed)
            # nextSpeedMagnitude = math.sqrt(nextXSpeed * nextXSpeed + nextYSpeed * nextYSpeed)

            # # Get magnitude along robot angle
            # nextSpeedMagnitude = nextSpeedMagnitude * math.cos(nextSpeedAngle - self.theta)

            # # Convert back to components
            # nextXSpeed = nextSpeedMagnitude * math.cos(nextSpeedAngle)
            # nextYSpeed = nextSpeedMagnitude * math.sin(nextSpeedAngle)


            self.xSpeed = xSpeed
            self.ySpeed = ySpeed
            self.angularSpeed = omega * dt
            # centrifugalAccel = 2 * self.angularSpeed * ICC_R
        # print(pixels_to_inches(self.xSpeed), pixels_to_inches(self.ySpeed))

        # Apply centrifugal acceleration
        # self.x -= math.sin(self.theta) * centrifugalAccel * dt
        # self.y += math.cos(self.theta ) * centrifugalAccel * dt

        self.x += self.xSpeed
        self.y += self.ySpeed
        # print(centrifugalAccel)
        self.theta += self.angularSpeed
    def draw(self):
        robot_size = inches_to_pixels(18)
        image = pygame.Surface((robot_size, robot_size), pygame.SRCALPHA)
        pygame.draw.polygon(image, (255, 255, 255), ((0, 0), (0, robot_size), (robot_size, robot_size), (robot_size, 0)))
        # Mark front of robot
        pygame.draw.polygon(image, (255, 0, 0), ((robot_size, 0), (robot_size - inches_to_pixels(2), 0), (robot_size - inches_to_pixels(2), robot_size), (robot_size, robot_size)))
        orig_image = image

        rect = image.get_rect()

        rect.center = (self.x, self.y)
        image, rect = rotate(orig_image, rect, -self.theta * 180 / math.pi)
        screen.blit(image, rect)
    def __repr__(this):
        return "Pose(" + str(this.x) + ", " + str(this.y) + ", " + str(this.theta) + ")"
                
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


robot = Pose(inches_to_pixels(24 + 12), inches_to_pixels(144 - 12), -3.14159 / 2)
counter = 0


# joystick = []

# for i in range(0, pygame.joystick.get_count()):
#     joystick = pygame.joystick.Joystick(i)
# joystick.init()


def wheel_speeds(velocity, angular_velocity):
    return (velocity - angular_velocity * (0.5 * 15), velocity + angular_velocity * (0.5 * 15))


speeds = []

lin_vels = []
ang_vels = []
with open("cooperPath.txt", "r") as f:
    f.readline()
    lin_vels = f.readline().strip().replace("[", "").replace("]", "").split(", ")
    ang_vels = f.readline().strip().replace("[", "").replace("]", "").split(", ")

lin_vels = [float(i) for i in lin_vels]
ang_vels = [float(i) for i in ang_vels]

for i in range(len(lin_vels)):
    speeds.append(wheel_speeds(lin_vels[i], ang_vels[i]))

with open("new_arrana_profile.txt", "w") as f:
    for i in range(len(speeds)):
        f.write(f"{speeds[i][0]},{speeds[i][1]}\n")

i = 0
speeds.append((0, 0))


speeds = []
with open("profile.txt", "r") as f:
    for line in f:
        line = line.strip()
        line = line.split(",")
        speeds.append((float(line[2]), float(line[3])))



positions = []

while True:
    realDt = clock.tick(100)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
    keys=pygame.key.get_pressed()
    screen.blit(bg, (0, 0))

    # left_input = -joystick.get_axis(1)
    # right_input = -joystick.get_axis(3) 
    left_input = speeds[i][0]
    right_input = speeds[i][1]
    if i == -1:
        left_input = 0
        right_input = 0
    robot.move(left_input, right_input, realDt)
    positions.append((robot.x, robot.y))
    for pos in positions:
        pygame.draw.circle(screen, (255, 0, 0), (int(pos[0]), int(pos[1])), 2)
    robot.draw()
    if i < len(speeds) - 1 and i != -1:
        i += 1
        print(f"Left In :{right_input:.2f}, right in: {left_input:.2f}")
    else:
        i = -1
        print(f"X: {pixels_to_inches(robot.x):.2f}, Y: {pixels_to_inches(robot.y):.2f}, Theta: {robot.theta * (180 / math.pi):.2f}")
    pygame.display.update()
