import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import os
from matplotlib import animation
from matplotlib import rcParams
import matplotlib
matplotlib.use('TkAgg')  # 使用非阻塞后端

# ---------- 字体设置 ----------
rcParams['font.sans-serif'] = ['Microsoft YaHei']
rcParams['axes.unicode_minus'] = False

# ---------- 参数 ----------
radius = 50  # mm
spacing = 120  # mm
csv_file = "data_score.csv"
refresh_interval = 500  # 毫秒（500ms = 0.5秒）

# ---------- 计算目标坐标 ----------
positions = []

# 第一排（5个）
for i in range(5):
    x = (i - 2) * spacing
    positions.append((x, 0))

# 第二排（3个）
for i in range(3):
    x = (i - 1) * spacing
    positions.append((x, spacing))

# 第三排（1个）
positions.append((0, 2 * spacing))

# ---------- 初始化绘图 ----------
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(-spacing * 3, spacing * 3)
ax.set_ylim(-spacing, spacing * 3)
ax.axis('off')
plt.title("动态目标得分可视化", fontsize=16)

# ---------- 绘制圆形对象 ----------
circles = []
labels = []

for i, (x, y) in enumerate(positions):
    c = patches.Circle((x, y), radius, color='orange')
    ax.add_patch(c)
    circles.append(c)
    text = ax.text(x, y, str(i + 1), color='white', ha='center', va='center',
                   fontsize=10, fontweight='bold')
    labels.append(text)


# ---------- 更新函数 ----------
def update(frame):
    if not os.path.exists(csv_file):
        print(f"⚠️ 未找到文件 {csv_file}")
        return circles

    try:
        with open(csv_file, 'r') as f:
            content = f.read().strip()
            scores = [int(x) for x in content.split(',')[:-1]]

        if len(scores) != 9:
            print("⚠️ 数据长度错误，应为9个数。")
            return circles

        for i, c in enumerate(circles):
            c.set_color("green" if scores[i] == 1 else "orange")

    except Exception as e:
        print("读取文件时出错：", e)

    return circles


# ---------- 启动动画 ----------
ani = animation.FuncAnimation(
    fig, update, interval=refresh_interval, blit=False
)

plt.show()
