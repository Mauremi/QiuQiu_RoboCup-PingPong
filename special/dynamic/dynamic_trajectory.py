import matplotlib.pyplot as plt
import pandas as pd
import matplotlib
import os
import time
from matplotlib.animation import FuncAnimation

# 使用非阻塞后端
matplotlib.use('TkAgg')

# 乒乓球桌尺寸
table_length = 2740  # mm
table_width = 1525  # mm

# 初始化图形
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(-50, table_length + 50)
ax.set_ylim(-table_width / 2 - 100, table_width / 2 + 100)

# 绘制乒乓球桌边界 - 最低图层 (zorder=1)
table = ax.add_patch(plt.Rectangle((0, -table_width / 2), table_length, table_width,
                                   fill=True, color="#311B92", linewidth=2, zorder=1))
# 绘制中线 - 低图层 (zorder=2)
center_line_x = ax.plot([0, table_length], [0, 0], color='white', linestyle='-', linewidth=1, zorder=2)
center_line_y = ax.plot([table_length / 2, table_length / 2], [table_width / 2, -table_width / 2],
                        color='white', linestyle='-', linewidth=1, zorder=2)

# 绘制门（9个）- 最高图层 (zorder=5)
door_width = 20
door_spacing = 180
door_color = 'black'
doors = []
for i in range(9):
    door_center_y = (i - 4) * door_spacing
    door = ax.add_patch(plt.Rectangle((table_length - door_width, door_center_y - 43),
                                      door_width, 86, fill=True, color=door_color, zorder=5))  # 设置高zorder
    doors.append(door)

ax.set_xticks([])
ax.set_yticks([])

# 初始化轨迹线 - 中间图层 (zorder=3)
line, = ax.plot([], [], color='orange', linewidth=0.5, label='轨迹', zorder=3)
scatter = ax.scatter([], [], color='#FF851A', s=40, zorder=4)  # 散点图层略高于轨迹线

# 文件路径
file_path = 'data_trajectory.csv'  # 修改为你的文件路径
last_mod_time = 0


def update_plot(frame):
    global last_mod_time

    try:
        # 检查文件是否更新
        current_mod_time = os.path.getmtime(file_path)
        if current_mod_time > last_mod_time:
            last_mod_time = current_mod_time

            # 读取新数据
            data = pd.read_csv(file_path)
            if not data.empty:
                x = data['x']
                y = data['y']

                # 更新轨迹线
                line.set_data(x, y)

                # 更新散点（先移除旧的，再添加新的）
                if hasattr(update_plot, 'old_scatter') and update_plot.old_scatter in ax.collections:
                    ax.collections.remove(update_plot.old_scatter)
                new_scatter = ax.scatter(x, y, color='#FF851A', s=40, zorder=4)  # 保持zorder
                update_plot.old_scatter = new_scatter

                # 重新绘制
                fig.canvas.draw_idle()

    except Exception as e:
        print(f"Fail to load the file: {e}")

    return line, scatter


# 创建动画
ani = FuncAnimation(fig, update_plot, interval=500, blit=False)  # 每500毫秒更新一次

# 非阻塞式显示图形
plt.tight_layout()
plt.show(block=False)

# 保持窗口打开
print("Dynamic drawing. Close the window to stop.")
try:
    while plt.fignum_exists(fig.number):  # 检查图形窗口是否还存在
        plt.pause(0.5)  # 短暂暂停，让窗口保持响应
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    plt.close()