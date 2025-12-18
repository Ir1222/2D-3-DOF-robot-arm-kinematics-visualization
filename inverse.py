import numpy as np
import math
import sys
import matplotlib.cm as cm
import matplotlib.pyplot as plt

'''
五、下一步我可以怎么帮你（你来选）

如果你愿意，我们可以直接进阶：

1️⃣ 把 inverse_k 改成 轨迹规划 + IK 分离
2️⃣ 加 elbow-up / elbow-down 选择
3️⃣ 改成 Jacobian-based IK（为后面 RL 铺路）
4️⃣ 把这套逻辑 迁移到 ROS2 / Isaac / PyBullet
'''

def draw_colored_path(ax, P1, P2, lengths, n=100):
    L1, L2, L3 = lengths
    r_min = abs(L1 - L2)
    r_max = L1 + L2

    cmap = cm.get_cmap("RdYlGn")

    for i, t in enumerate(np.linspace(0, 1, n)):
        x = P1[0] + t * (P2[0] - P1[0])
        y = P1[1] + t * (P2[1] - P1[1])

        r = np.hypot(x, y)
        r2 = r - L3

        safety = min(r2 - r_min, r_max - r2)
        safety_norm = np.clip(safety / (0.3 * r_max), 0, 1)

        color = cmap(safety_norm)

        ax.plot(x, y, 'o', color=color, markersize=4)


def path_is_reachable(P1, P2, lengths, n=50):
    L1, L2, L3 = lengths

    for t in np.linspace(0, 1, n):
        x = P1[0] + t * (P2[0] - P1[0])
        y = P1[1] + t * (P2[1] - P1[1])

        r = np.hypot(x, y)
        r2 = r - L3

        if r2 < abs(L1 - L2) or r2 > (L1 + L2):
            return False

    return True


# Inverse Kinematics function
def inverse_k(x_init, y_init, gamma_init, x_final, y_final, gamma_final, x_mid, y_mid, gamma_mid, L1, L2, L3):
    n_positions = 100
    is3DOF = True
    theta1, theta2, theta3, Px1, Py1, Px2, Py2, Px3, Py3 = [], [], [], [], [], [], [], [], []

    def get_incr(j, v_init, v_final):
        return v_init + j * (v_final - v_init) / (n_positions - 1)

    def get_y(x, isLine=True):
        if isLine:
            m = (y_final - y_init) / (x_final - x_init)
            b = y_init - (m * x_init)
            y_value = (m * x) + b
        return y_value

    def get_theta2_theta1(x, y):
        try:
            value2 = math.acos((x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2))
        except:
            raise ValueError("Selected points cause the robot to exit its reachable envelope.")
        value1 = math.atan2(y, x) - math.atan2(L2 * math.sin(value2), L1 + (L2 * math.cos(value2)))
        return value2, value1

    def get_x1_y1(theta):
        x_value = L1 * math.cos(theta)
        y_value = L1 * math.sin(theta)
        return x_value, y_value

    for i in range(0, n_positions):
        x3 = get_incr(i, x_init, x_final)
        y3 = get_y(x3)
        gamma = math.atan2(y3, x3)
        x2 = x3 - (L3 * math.cos(gamma))
        y2 = y3 - (L3 * math.sin(gamma))

        theta_2, theta_1 = get_theta2_theta1(x2, y2)
        theta_3 = gamma - theta_2 - theta_1

        x1, y1 = get_x1_y1(theta_1)

        theta1.append(theta_1)
        theta2.append(theta_2)
        if is3DOF: theta3.append(theta_3)
        if is3DOF: Px3.append(x3)
        if is3DOF: Py3.append(y3)
        Px1.append(x1)
        Py1.append(y1)
        Px2.append(x2)
        Py2.append(y2)

    return theta1, theta2, theta3, Px1, Py1, Px2, Py2, Px3, Py3

class RobotArmPlot:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-100, 100)
        #self.ax.set_ylim(-100, 100)
        #self.ax.set_aspect('equal')
        self.ax.grid(True, which='both')

        self.lengths = []
        self.points = []
        self.arm_lines = None
        self.dashed_line = None

        self.length_preview_line = None
        self.length_preview_circle = None

        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_move)
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_close = self.fig.canvas.mpl_connect('close_event', self.on_close)

        self.outer_circle = None
        self.inner_circle = None
        self.hover_mask = None

        self.projection_point = None

    def preview_length(self, event):
        x, y = event.xdata, event.ydata
        r = np.hypot(x, y)

        # 清除旧的预览
        if self.length_preview_line:
            self.length_preview_line.remove()
        if self.length_preview_circle:
            self.length_preview_circle.remove()

        # 虚线（原点 → 鼠标）
        self.length_preview_line, = self.ax.plot(
            [0, x], [0, y],
            'k--', lw=1
        )

        # 圆（表示当前连杆长度）
        self.length_preview_circle = plt.Circle(
            (0, 0),
            r,
            color='blue',
            fill=False,
            linestyle=':',
            lw=1
        )
        self.ax.add_artist(self.length_preview_circle)

        self.ax.set_title(f"Select L{len(self.lengths) + 1}: {r:.2f}")
        self.fig.canvas.draw_idle()

    def on_move(self, event):
        if event.inaxes is None:
            return

        # ====== 阶段 1：机械臂长度输入阶段 ======
        if len(self.lengths) < 3:
            self.preview_length(event)
            return

        # ====== 阶段 2：workspace / hover 阶段 ======
        # （你现在已有的 unreachable / projection 逻辑）

        x, y = event.xdata, event.ydata
        r = np.hypot(x, y)

        L1, L2, L3 = self.lengths
        r2 = r - L3

        r_min = abs(L1 - L2)
        r_max = L1 + L2

        unreachable = (r2 < r_min or r2 > r_max)

        # ===== 增加不可达直接变灰
        # 删除旧遮罩
        if self.hover_mask:
            self.hover_mask.remove()
            self.hover_mask = None

        if unreachable:
            self.hover_mask = plt.Circle(
                (0, 0),
                L1 + L2 + L3,
                color='gray',
                alpha=0.15,
                zorder=0
            )
            self.ax.add_artist(self.hover_mask)
            self.ax.set_title("Unreachable")
        else:
            self.ax.set_title("Reachable")

        self.fig.canvas.draw_idle()


        # ====== 增加最近可达点
        # 删除旧投影点
        if self.projection_point:
            self.projection_point.remove()
            self.projection_point = None

        if unreachable:
            # 单位方向
            dx, dy = x / r, y / r

            if r2 > r_max:
                r2_proj = r_max
            else:
                r2_proj = r_min

            r_proj = r2_proj + L3
            x_proj = dx * r_proj
            y_proj = dy * r_proj

            # 画最近可达点
            self.projection_point, = self.ax.plot(
                x_proj, y_proj, 'bo', markersize=8
            )

            # 连线提示
            self.ax.plot(
                [x, x_proj], [y, y_proj],
                'b:', lw=1
            )
            

    def on_click(self, event):
        if event.inaxes is not None:
            if len(self.lengths) < 3:
                length = np.sqrt(event.xdata**2 + event.ydata**2)
                self.lengths.append(length)
                print(f'Stored Length L{len(self.lengths)}: {length:.2f}')
                if len(self.lengths) == 3:
                    self.draw_reachable_area()
                    # 确认后清除预览
                    if self.length_preview_line:
                        self.length_preview_line.remove()
                        self.length_preview_line = None
                    if self.length_preview_circle:
                        self.length_preview_circle.remove()
                        self.length_preview_circle = None


            elif len(self.points) < 2:
                point = (event.xdata, event.ydata)
                self.points.append(point)
                self.ax.plot(point[0], point[1], 'ro')
                self.fig.canvas.draw()
                if len(self.points) == 2:
                    self.ax.plot([self.points[0][0], self.points[1][0]],
                                 [self.points[0][1], self.points[1][1]], 'r--')
                    self.fig.canvas.draw()
                    self.animate_robot_arm()


            elif len(self.points) == 1:
                P1 = self.points[0]
                P2 = (event.xdata, event.ydata)

                draw_colored_path(self.ax, P1, P2, self.lengths)
                self.fig.canvas.draw_idle()

                if not path_is_reachable(P1, P2, self.lengths):
                    self.ax.set_title("Path is not fully reachable")
                    self.fig.canvas.draw_idle()
                    return

                self.points.append(P2)
                self.animate_robot_arm()

    def draw_reachable_area(self):
        L1, L2, L3 = self.lengths

        R_outer = L1 + L2 + L3
        R_inner = max(0, abs(L1 - L2) - L3)

        margin = 0.1 * R_outer
        lim = R_outer + margin

        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_aspect('equal')

        # 清理旧的 circle
        if self.outer_circle:
            self.outer_circle.remove()
        if self.inner_circle:
            self.inner_circle.remove()

        # 外可达
        self.outer_circle = plt.Circle(
            (0, 0), R_outer, color='green', fill=False, lw=2
        )
        self.ax.add_artist(self.outer_circle)

        # 内不可达
        if R_inner > 0:
            self.inner_circle = plt.Circle(
                (0, 0), R_inner, color='red', fill=False, linestyle='--', lw=2
            )
            self.ax.add_artist(self.inner_circle)

        self.ax.set_title("Workspace: Green = Reachable, Red = Unreachable")
        self.fig.canvas.draw_idle()

    def animate_robot_arm(self):
        P1, P2 = self.points
        num_frames = 100
        L1, L2, L3 = self.lengths

        t_vals = np.concatenate([np.linspace(0, 1, num_frames), np.linspace(1, 0, num_frames)])
        for t in t_vals:
            target_x = P1[0] + t * (P2[0] - P1[0])
            target_y = P1[1] + t * (P2[1] - P1[1])

            theta1, theta2, theta3, Px1, Py1, Px2, Py2, Px3, Py3 = inverse_k(
                P1[0], P1[1], 0, P2[0], P2[1], 0, (P1[0]+P2[0])/2, (P1[1]+P2[1])/2, 0, L1, L2, L3)

            # 更新机械臂的绘图
            for i in range(num_frames):
                if self.arm_lines:
                    self.arm_lines[0].set_data([0, Px1[i]], [0, Py1[i]])
                    self.arm_lines[1].set_data([Px1[i], Px2[i]], [Py1[i], Py2[i]])
                    self.arm_lines[2].set_data([Px2[i], Px3[i]], [Py2[i], Py3[i]])
                else:
                    line1, = self.ax.plot([0, Px1[i]], [0, Py1[i]], 'ko-', lw=2)
                    line2, = self.ax.plot([Px1[i], Px2[i]], [Py1[i], Py2[i]], 'ko-', lw=2)
                    line3, = self.ax.plot([Px2[i], Px3[i]], [Py2[i], Py3[i]], 'ko-', lw=2)
                    self.arm_lines = [line1, line2, line3]

                self.fig.canvas.draw()
                plt.pause(0.05)

    def on_close(self, event):
        plt.close(self.fig)
        sys.exit()  # 结束程序运行

    def show(self):
        plt.show()

# 使用该类创建并展示机器人臂绘图
robot_arm_plot = RobotArmPlot()
robot_arm_plot.show()
