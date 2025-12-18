import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation


# animation settings
n_positions = 30  # interveral between start & end positions
frame_delay = 200  # Set delay (between frames), smaller delay = faster animation
delay = 2  # Set time delays (seconds) in User Interface
rc = 1  # rounding constant (can be adjusted for accuracy)

#init values
theta = []
Arm_lengths = []
theta_1 = []
theta_2 = []
theta_3 = []

# Flags
isForward = False  # for forward Kinematics
isInverse = False  # for Invers Kinematics
is3DOF = False  # for 3 Degrees Of Freedom Robot
is2DOF = False  # for 2 Degrees Of Freedom Robot
isLine = False  # for Stright Line motion (of end-effector)
isCurve = False  # for Curve motion


# Information collect
class ArmController:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.grid(True)
        self.angle_starts = []
        self.angle_ends = []
        self.arm_lengths = []
        self.start_points = {}  # Store start points
        self.end_points = {}  # Store end points
        self.current_step = 0
        self.point_counter = 1

        self.current_step = 0
        self.moving_line = None
        self.angle_text = None
        self.length_text = None
        self.radius = None  # To store the arm length for the circle

        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid2 = self.fig.canvas.mpl_connect('motion_notify_event', self.on_move)



    def on_click(self, event):
        point_name = f"{'start' if self.current_step % 2 == 0 else 'end'}_point{self.point_counter}"
        if self.current_step % 2 == 0:
            # Set starting angle and calculate arm length
            angle = np.degrees(np.arctan2(event.ydata, event.xdata))
            self.angle_starts.append(angle)
            self.radius = np.hypot(event.xdata, event.ydata)
            self.arm_lengths.append(self.radius)
            self.start_points[point_name] = (event.xdata, event.ydata)  # Store the original point
            # Draw a line from origin to the point
            self.ax.plot([0, event.xdata], [0, event.ydata], 'bo-')
            # Draw a circle for arm reach
            circle = plt.Circle((0, 0), self.radius, color='r', fill=False, linestyle='--')
            self.ax.add_artist(circle)
        else:
            # Set ending angle, point must lie on the circle
            corrected_x, corrected_y = self.get_corrected_point(event.xdata, event.ydata)
            angle = np.degrees(np.arctan2(corrected_y, corrected_x))
            self.angle_ends.append(angle)
            # Draw a line from origin to the corrected point
            self.ax.plot([0, corrected_x], [0, corrected_y], 'go-')

            # Print the start point for debugging
        if self.current_step % 2 == 0:
            print(f"{point_name}: {self.start_points[point_name]}")
        else:
            # Set ending angle, point must lie on the circle
            corrected_x, corrected_y = self.get_corrected_point(event.xdata, event.ydata)
            angle = np.degrees(np.arctan2(corrected_y, corrected_x))
            self.angle_ends.append(angle)
            self.end_points[point_name] = (corrected_x, corrected_y)  # Store the corrected point
            # Draw a line from origin to the corrected point
            self.ax.plot([0, corrected_x], [0, corrected_y], 'go-')

            # Print the end point for debugging
            print(f"{point_name}: {self.end_points[point_name]}")
            self.point_counter += 1  # Increment point counter after end point is added

        self.current_step += 1
        self.fig.canvas.draw()

    def on_move(self, event):
        if event.inaxes:
            x, y = event.xdata, event.ydata
            distance = np.hypot(x, y)

            # Check if we are in the step of setting a starting point (new radius)
            if self.current_step % 2 == 0:
                # Update radius with the current mouse position distance
                self.radius = distance

            # Get the corrected point on the circle if the radius is defined
            corrected_x, corrected_y = (x, y) if self.radius is None else self.get_corrected_point(x, y)
            angle = np.degrees(np.arctan2(corrected_y, corrected_x))

            # Round angle and distance to the nearest integer
            rounded_angle = round(angle)
            rounded_distance = round(self.radius)

            # Update or create the moving line
            if self.moving_line:
                self.moving_line.set_data([0, corrected_x], [0, corrected_y])
            else:
                self.moving_line, = self.ax.plot([0, corrected_x], [0, corrected_y], 'k--')

            # Update or create the text showing angle
            if self.angle_text:
                self.angle_text.set_position((corrected_x, corrected_y))
                self.angle_text.set_text(f"Angle: {rounded_angle}°")
            else:
                self.angle_text = self.ax.text(corrected_x, corrected_y, f"Angle: {rounded_angle}°", fontsize=12, bbox=dict(facecolor='white', alpha=0.7))

            # Update or create the text showing arm length
            if self.length_text:
                self.length_text.set_position((0, -9))  # Positioning it consistently
                self.length_text.set_text(f"Arm Length: {rounded_distance}")
            else:
                self.length_text = self.ax.text(0, -9, f"Arm Length: {rounded_distance}", fontsize=12, bbox=dict(facecolor='white', alpha=0.7))

            self.fig.canvas.draw()

    def get_corrected_point(self, x, y):
        """Return the point on the circle defined by self.radius closest to (x, y)."""
        if self.radius is None:
            return x, y
        angle = np.arctan2(y, x)
        corrected_x = self.radius * np.cos(angle)
        corrected_y = self.radius * np.sin(angle)
        return corrected_x, corrected_y

    def show(self):
        plt.show()

controller = ArmController()
controller.show()
#def show(self):
    #plt.show(block=False)



for i, (start, end) in enumerate(zip(controller.angle_starts, controller.angle_ends)):
    rounded_start = round(start)
    rounded_end = round(end)
    rounded_length = round(controller.arm_lengths[i])

    theta.append({
        'joint': i + 1,
        'angle_start': rounded_start,
        'angle_end': rounded_end,
    })

    Arm_lengths.append(rounded_length)  # 直接存储长度值

# 循环结束后一次性打印所有结果
print("Theta Data:", theta)
print("Arm Lengths Data:", Arm_lengths)

# 将 arm_lengths 的值分别赋予 L1, L2, L3
if len(Arm_lengths) == 3:  # 确保有3个值
    L1, L2, L3 = Arm_lengths
    print(f"L1 = {L1}, L2 = {L2}, L3 = {L3}")
else:
    print("Error: arm_lengths does not contain exactly 3 values")

#Animation
class ForwardKinematics:
    def __init__(self, lengths, start_angles):
        """
        初始化三自由度机械臂
        :param lengths: 各臂段的长度 [L1, L2, L3]
        :param start_angles: 各关节的初始角度 [theta1, theta2, theta3]
        """
        self.lengths = lengths
        self.angles = np.radians(start_angles)  # 将角度转换为弧度制

    def forward_kinematics(self):
        """
        计算前向运动学得到末端执行器的位置
        :return: 各关节的位置坐标 [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
        """
        L1, L2, L3 = self.lengths
        theta1, theta2, theta3 = self.angles

        x1 = L1 * np.cos(theta1)
        y1 = L1 * np.sin(theta1)

        x2 = x1 + L2 * np.cos(theta1 + theta2)
        y2 = y1 + L2 * np.sin(theta1 + theta2)

        x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + L3 * np.sin(theta1 + theta2 + theta3)

        return [(0, 0), (x1, y1), (x2, y2), (x3, y3)]

    def update_angles(self, new_angles):
        self.angles = np.radians(new_angles)

def animate_robot_arm(start_angles, end_angles, lengths, steps=100, interval=50):
    robot_arm = ForwardKinematics(lengths, start_angles)

    #Generate angle path
    angles_path = np.linspace(np.radians(start_angles), np.radians(end_angles), steps)

    #set up plot
    fig, ax = plt.subplots(figsize=(7, 7))

    ax.set_xlim(-sum(lengths), sum(lengths))
    ax.set_ylim(-sum(lengths), sum(lengths))
    ax.set_aspect('equal')

    #Initialize the line of the connecting rod
    line, = ax.plot([], [], 'o-', lw=2)


    # 在 animate_robot_arm 里，初始化 plot 后
    circles = [
        plt.Circle((0, 0), lengths[0], fill=False, linestyle='--', color='gray'),
        plt.Circle((0, 0), lengths[1], fill=False, linestyle='--', color='gray', visible=False),
        plt.Circle((0, 0), lengths[2], fill=False, linestyle='--', color='gray', visible=False)
    ]

    angle_texts = [
        ax.text(0, 0, "", fontsize=10, color='blue'),
        ax.text(0, 0, "", fontsize=10, color='green'),
        ax.text(0, 0, "", fontsize=10, color='red')
    ]

    for c in circles:
        ax.add_patch(c)

    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        robot_arm.update_angles(np.degrees(frame))  # Convert radians to angles

        #=========== 更新圆
        positions = robot_arm.forward_kinematics()

        (x0, y0), (x1, y1), (x2, y2), (x3, y3) = positions

        line.set_data([x0, x1, x2, x3], [y0, y1, y2, y3])

        # 更新圆心
        circles[0].center = (x0, y0)
        circles[0].set_visible(True)

        circles[1].center = (x1, y1)
        circles[1].set_visible(True)

        circles[2].center = (x2, y2)
        circles[2].set_visible(True)
        '''
        print(circles[0].center)
        print(circles[1].center)
        print(circles[2].center)
        '''
        # =========更新角度
        theta1, theta2, theta3 = np.degrees(frame)

        # joint positions
        (x0, y0), (x1, y1), (x2, y2), _ = positions

        angle_texts[0].set_position((x0 + 2, y0 + 2))
        angle_texts[0].set_text(f"θ1={theta1:.1f}°")

        angle_texts[1].set_position((x1 + 2, y1 + 2))
        angle_texts[1].set_text(f"θ2={theta2:.1f}°")

        angle_texts[2].set_position((x2 + 2, y2 + 2))
        angle_texts[2].set_text(f"θ3={theta3:.1f}°")

        return line, *circles, *angle_texts

    #set up animation
    ani = animation.FuncAnimation(
        fig, update, frames=angles_path, init_func=init,
        blit=True, interval=interval
    )

    plt.show()


# 使用存储的数据
lengths = [L1, L2, L3]
start_angles = [theta[0]['angle_start'], theta[1]['angle_start'], theta[2]['angle_start']]
end_angles = [theta[0]['angle_end'], theta[1]['angle_end'], theta[2]['angle_end']]

# 动画代码
animate_robot_arm( start_angles, end_angles, lengths)