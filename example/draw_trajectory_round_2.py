import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd

# 读取数据
data = pd.read_csv('pingpong_trajectory.csv')
x = data['x']
y = data['y']

# 绘图
fig, ax = plt.subplots(figsize=(15, 9))

# 画乒乓球桌轮廓
table_length = 2740  # mm，乒乓球桌的宽度
table_width = 1525  # mm，乒乓球桌的高度
ax.set_xlim(0, table_length + 1000)  # 扩展X轴以容纳目标桶
ax.set_ylim(-table_width / 2, table_width / 2)

# 绘制边界线
ax.add_patch(plt.Rectangle((0, -table_width / 2), table_length, table_width, fill=False, edgecolor='blue', linewidth=2))

# 绘制中线
ax.plot([0, table_length], [0, 0], color='black', linestyle='--', linewidth=2)
ax.plot([table_length / 2, table_length / 2], [table_width / 2, -table_width / 2], color='black', linestyle='--', linewidth=2)


# 画出轨迹线
ax.plot(x, y, color='red', linewidth=2, label='轨迹')
ax.scatter(x, y, color='black', s=10)  # 加点以辅助查看

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
ax.add_patch(plt.Rectangle((rect_x_left, rect_y_bottom), rect_x_right - rect_x_left, rect_y_top - rect_y_bottom, color=bucket_color, fill=False))

# 上方半圆 (使用 Wedge)
ax.add_patch(patches.Wedge((rect_x_left + 100, rect_y_top), 100, 0, 180, color=bucket_color, lw=2, fill=False))

# 下方半圆 (使用 Wedge)
ax.add_patch(patches.Wedge((rect_x_left + 100, rect_y_bottom), 100, 180, 360, color=bucket_color, lw=2, fill=False))

# 关闭坐标轴显示
ax.set_xticks([])  # 关闭 x 轴
ax.set_yticks([])  # 关闭 y 轴

ax.set_aspect('equal', adjustable='box')

# 显示图形
plt.show()
