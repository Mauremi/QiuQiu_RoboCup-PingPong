import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import time

matplotlib.use('TkAgg')

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 定义发球区参数（单位：mm）
SERVE_AREA_WIDTH = 1000  # 发球区宽度
SERVE_AREA_LENGTH = 250  # 发球区长度
GRID_SIZE = 50  # 格子大小
X_MIN, X_MAX = -500, 500  # X轴范围
Y_MIN, Y_MAX = 0, 250  # Y轴范围

# 避开黄色禁区（70mm*240mm）
FORBIDDEN_X_MIN, FORBIDDEN_X_MAX = -120, 120
FORBIDDEN_Y_MIN, FORBIDDEN_Y_MAX = 0, 70

# 高斯核函数参数
SIGMA = 80  # 高斯核的标准差，控制影响范围

# 创建图形和子图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))
plt.subplots_adjust(hspace=0.3)

# 初始化图像对象
im1 = ax1.imshow(np.zeros((len(np.arange(Y_MIN, Y_MAX + GRID_SIZE, GRID_SIZE)),
                           len(np.arange(X_MIN, X_MAX + GRID_SIZE, GRID_SIZE)))),
                 extent=[X_MIN, X_MAX, Y_MIN, Y_MAX],
                 origin='lower', cmap='RdBu_r', aspect='equal', vmin=-15, vmax=15)

im2 = ax2.imshow(np.zeros((len(np.arange(Y_MIN, Y_MAX + GRID_SIZE, GRID_SIZE)),
                           len(np.arange(X_MIN, X_MAX + GRID_SIZE, GRID_SIZE)))),
                 extent=[X_MIN, X_MAX, Y_MIN, Y_MAX],
                 origin='lower', cmap='PiYG_r', aspect='equal', vmin=-15, vmax=15)

# 添加颜色条
cbar1 = fig.colorbar(im1, ax=ax1, label='偏差 (mm)', extend='both')
cbar2 = fig.colorbar(im2, ax=ax2, label='偏差 (mm)', extend='both')

# 设置标题
ax1.set_title('X方向偏差热力图（红色-向右偏, 蓝色-向左偏）', fontsize=12, fontweight='bold')
ax2.set_title('Y方向偏差热力图（绿色-打远, 粉色-打近）', fontsize=12, fontweight='bold')

# 创建网格
x_grid = np.arange(X_MIN, X_MAX + GRID_SIZE, GRID_SIZE)
y_grid = np.arange(Y_MIN, Y_MAX + GRID_SIZE, GRID_SIZE)

# 添加网格线和禁区标记（只需要添加一次）
for ax in [ax1, ax2]:
    # 添加网格线
    for x in x_grid:
        ax.axvline(x=x, color='gray', linestyle='-', alpha=0.3, linewidth=0.5)
    for y in y_grid:
        ax.axhline(y=y, color='gray', linestyle='-', alpha=0.3, linewidth=0.5)

    # 标记黄色禁区
    rect = patches.Rectangle(
        (FORBIDDEN_X_MIN, FORBIDDEN_Y_MIN),
        FORBIDDEN_X_MAX - FORBIDDEN_X_MIN,
        FORBIDDEN_Y_MAX - FORBIDDEN_Y_MIN,
        linewidth=2,
        edgecolor='red',
        facecolor='white',
        alpha=0.9,
        zorder=10,
        label='禁区',
        hatch='//',
    )
    ax.add_patch(rect)

    # 添加原点和坐标轴
    ax.axhline(y=0, color='black', linestyle='-', alpha=0.5)
    ax.axvline(x=0, color='black', linestyle='-', alpha=0.5)

# 初始化散点图
scatter1 = ax1.scatter([], [], c=[], cmap='RdBu_r', s=50, edgecolors='black', alpha=0.8)
scatter2 = ax2.scatter([], [], c=[], cmap='PiYG_r', s=50, edgecolors='black', alpha=0.8)


def is_in_forbidden_zone(x, y):
    """检查是否在黄色禁区内"""
    return (FORBIDDEN_X_MIN <= x <= FORBIDDEN_X_MAX and
            FORBIDDEN_Y_MIN <= y <= FORBIDDEN_Y_MAX)


def update_plot(frame):
    try:
        df = pd.read_csv('data_offsets.csv')

        # 计算格子中心点
        x_centers = (x_grid[:-1] + x_grid[1:]) / 2
        y_centers = (y_grid[:-1] + y_grid[1:]) / 2
        Xc, Yc = np.meshgrid(x_centers, y_centers)

        # 初始化偏差矩阵
        x_bias_grid = np.zeros_like(Xc, dtype=float)
        y_bias_grid = np.zeros_like(Yc, dtype=float)

        # 遍历每个格子中心点
        for i, gy in enumerate(y_centers):
            for j, gx in enumerate(x_centers):

                if is_in_forbidden_zone(gx, gy):
                    continue

                # 遍历所有数据点并累积高斯权重
                for _, row in df.iterrows():
                    x, y = row['x'], row['y']
                    x_bias, y_bias = row['x_offset'], row['y_offset']

                    # ⚠️ 左右隔离：只允许同一半边的数据影响格子
                    if (gx < 0 and x >= 0) or (gx >= 0 and x < 0):
                        continue

                    distance = (gx - x) ** 2 + (gy - y) ** 2
                    w = np.exp(-distance / (2 * SIGMA ** 2))

                    x_bias_grid[i, j] += x_bias * w
                    y_bias_grid[i, j] += y_bias * w

        # 更新热力图
        im1.set_data(x_bias_grid)
        im2.set_data(y_bias_grid)

        im1.set_extent([X_MIN, X_MAX, Y_MIN, Y_MAX])
        im2.set_extent([X_MIN, X_MAX, Y_MIN, Y_MAX])

        im1.set_interpolation('nearest')
        im2.set_interpolation('nearest')

        # 更新散点图（不隔离，保持完整展示）
        scatter1.set_offsets(np.column_stack([df['x'], df['y']]))
        scatter1.set_array(df['x_offset'])

        scatter2.set_offsets(np.column_stack([df['x'], df['y']]))
        scatter2.set_array(df['y_offset'])

        ax1.set_title(f'X方向偏差热力图 - 数据点: {len(df)}', fontsize=12, fontweight='bold')
        ax2.set_title(f'Y方向偏差热力图 - 数据点: {len(df)}', fontsize=12, fontweight='bold')

        print(f"更新时间: {time.strftime('%H:%M:%S')} - 数据点: {len(df)}")

    except Exception as e:
        print(f"读取数据时出错: {e}")

    return [im1, im2, scatter1, scatter2]


# 创建动画
ani = FuncAnimation(fig, update_plot, interval=500, blit=False, cache_frame_data=False)

plt.tight_layout()
plt.show()