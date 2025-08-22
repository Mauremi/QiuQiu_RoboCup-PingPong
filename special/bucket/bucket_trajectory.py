import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import matplotlib
import os
import time
from matplotlib.animation import FuncAnimation

# 使用非阻塞后端
matplotlib.use('TkAgg')

# 乒乓球桌尺寸
table_length = 2740  # mm，乒乓球桌的宽度
table_width = 1525  # mm，乒乓球桌的高度

# 初始化图形
fig, ax = plt.subplots(figsize=(15, 9))
ax.set_xlim(0, table_length + 1000)  # 扩展X轴以容纳目标桶
ax.set_ylim(-table_width / 2, table_width / 2)

# 绘制边界线
ax.add_patch(plt.Rectangle((0, -table_width / 2), table_length, table_width,
                           fill=True, color='#311B92', linewidth=2))

# 绘制中线
ax.plot([0, table_length], [0, 0], color='white', linestyle='-', linewidth=1)
ax.plot([table_length / 2, table_length / 2], [table_width / 2, -table_width / 2],
        color='white', linestyle='-', linewidth=1)

# 添加目标桶（圆形和矩形）
bucket_color = 'purple'  # 目标桶的颜色

# 圆形目标桶 (半径 100)
ax.add_patch(plt.Circle((table_length + 280, -560), 100, color=bucket_color, fill=False))
ax.add_patch(plt.Circle((table_length + 280, 560), 100, color=bucket_color, fill=False))

# 圆形目标桶 (半径 80)
ax.add_patch(plt.Circle((table_length + 640, -250), 80, color=bucket_color, fill=False))
ax.add_patch(plt.Circle((table_length + 640, 250), 80, color=bucket_color, fill=False))

# 圆形目标桶 (半径 70)
ax.add_patch(plt.Circle((table_length + 880, 0), 70, color=bucket_color, fill=False))

# 矩形目标桶及半圆
rect_x_left = table_length + 180
rect_x_right = table_length + 380
rect_y_bottom = -250
rect_y_top = 250
ax.add_patch(plt.Rectangle((rect_x_left, rect_y_bottom),
                           rect_x_right - rect_x_left, rect_y_top - rect_y_bottom,
                           color=bucket_color, fill=False))

# 上方半圆 (使用 Wedge)
ax.add_patch(patches.Wedge((rect_x_left + 100, rect_y_top), 100, 0, 180,
                           color=bucket_color, lw=2, fill=False))

# 下方半圆 (使用 Wedge)
ax.add_patch(patches.Wedge((rect_x_left + 100, rect_y_bottom), 100, 180, 360,
                           color=bucket_color, lw=2, fill=False))

# 初始化轨迹线 - 最后添加，确保散点在最上层
line, = ax.plot([], [], color='orange', linewidth=0.5, label='轨迹')
scatter = ax.scatter([], [], color='#FF851A', s=10)  # 散点图最后添加，自然在最上层

# 关闭坐标轴显示
ax.set_xticks([])  # 关闭 x 轴
ax.set_yticks([])  # 关闭 y 轴
ax.set_aspect('equal', adjustable='box')

# 文件路径
file_path = 'data_trajectory.csv'  # 修改为你的文件路径
last_mod_time = 0


def update_plot(frame):
    global last_mod_time

    try:

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
            new_scatter = ax.scatter(x, y, color='#FF851A', s=40)  # 新散点添加到最上层
            update_plot.old_scatter = new_scatter

            # 重新绘制
            fig.canvas.draw_idle()

    except Exception as e:
        print(f"读取文件时出错: {e}")

    return line, scatter


# 创建动画
ani = FuncAnimation(fig, update_plot, interval=500, blit=False)  # 每500毫秒更新一次

# 非阻塞式显示图形
plt.tight_layout()
plt.show(block=False)

# 保持窗口打开
print("动态绘图已启动，关闭窗口以退出...")
print("散点图层设置为最上方（通过添加顺序控制）")
try:
    while plt.fignum_exists(fig.number):  # 检查图形窗口是否还存在
        plt.pause(0.5)  # 短暂暂停，让窗口保持响应
except KeyboardInterrupt:
    print("程序被用户中断")
finally:
    plt.close()