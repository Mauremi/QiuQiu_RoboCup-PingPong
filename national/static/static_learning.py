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

# -------------------- 基本参数 --------------------
SERVE_AREA_WIDTH = 1000  # 发球区宽度
SERVE_AREA_LENGTH = 250  # 发球区长度
GRID_SIZE = 50  # 格子大小
Y_MIN, Y_MAX = -500, 500  # 水平范围（左右）
X_MIN, X_MAX = 0, 250     # 垂直范围（上下）

# 禁区（按物理坐标：水平x，垂直y）
FORBIDDEN_Y_MIN, FORBIDDEN_Y_MAX = -120, 120
FORBIDDEN_X_MIN, FORBIDDEN_X_MAX = 0, 70

# 计算模式与系数（可调）
CALC_MODE = 1  # 1 -> case1, 2 -> case2
DEVISER_X = 100  # 控制远近的范围
DEVISER_Y = 200  # 控制左右的范围
CORR_COEFF_X = 0.8  # 远近偏差的放缩系数
CORR_COEFF_Y = 0.8  # 左右偏差的放缩系数

# 高斯核参数（这里只保留 SIGMA 做备用，主计算使用上面的 DEVISER_X/Y）
SIGMA = 60

# -------------------- 网格/中心点 --------------------
y_grid = np.arange(Y_MIN, Y_MAX + GRID_SIZE, GRID_SIZE)  # 边界线（水平）
x_grid = np.arange(X_MIN, X_MAX + GRID_SIZE, GRID_SIZE)  # 边界线（垂直）

# 格子中心（注意：矩阵大小应为边界数-1）
y_centers = (y_grid[:-1] + y_grid[1:]) / 2.0  # 水平每格中心，长度 = len(x_grid)-1
x_centers = (x_grid[:-1] + x_grid[1:]) / 2.0  # 垂直每格中心，长度 = len(y_grid)-1

# -------------------- 画图初始化 --------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))
plt.subplots_adjust(hspace=0.3)

# 使用格子数初始化热力图矩阵（行 = 垂直格子数, 列 = 水平格子数）
init_shape = (len(x_centers), len(y_centers))
im1 = ax1.imshow(np.zeros(init_shape),
                 extent=[Y_MAX, Y_MIN, X_MIN, X_MAX],
                 origin='lower', cmap='RdBu_r', aspect='equal', vmin=-30, vmax=30,
                 interpolation='nearest')
im2 = ax2.imshow(np.zeros(init_shape),
                 extent=[Y_MAX, Y_MIN, X_MIN, X_MAX],
                 origin='lower', cmap='PiYG_r', aspect='equal', vmin=-30, vmax=30,
                 interpolation='nearest')

cbar1 = fig.colorbar(im1, ax=ax1, label='偏差 (mm)', extend='both')
cbar2 = fig.colorbar(im2, ax=ax2, label='偏差 (mm)', extend='both')

# 绘制网格线与禁区（一次）
for ax in [ax1, ax2]:
    for x in y_grid:
        ax.axvline(x=x, color='gray', linestyle='-', alpha=0.3, linewidth=0.5)
    for y in x_grid:
        ax.axhline(y=y, color='gray', linestyle='-', alpha=0.3, linewidth=0.5)

    rect = patches.Rectangle(
        (FORBIDDEN_Y_MIN, FORBIDDEN_X_MIN),
        FORBIDDEN_Y_MAX - FORBIDDEN_Y_MIN,
        FORBIDDEN_X_MAX - FORBIDDEN_X_MIN,
        linewidth=2, edgecolor='red', facecolor='white', alpha=0.9, zorder=10, hatch='//'
    )
    ax.add_patch(rect)
    ax.axhline(y=0, color='black', linestyle='-', alpha=0.5)
    ax.axvline(x=0, color='black', linestyle='-', alpha=0.5)

# 散点（展示原始点，但热力学习已做左右隔离）
scatter1 = ax1.scatter([], [], c=[], cmap='RdBu_r', s=40, edgecolors='black', alpha=0.8)
scatter2 = ax2.scatter([], [], c=[], cmap='PiYG_r', s=40, edgecolors='black', alpha=0.8)


def is_in_forbidden_zone(x, y):
    """是否落在禁区（x水平, y垂直）"""
    return (FORBIDDEN_Y_MIN <= x <= FORBIDDEN_Y_MAX and FORBIDDEN_X_MIN <= y <= FORBIDDEN_X_MAX)


def update_plot(frame):
    try:
        df = pd.read_csv('data_offsets.csv')

        # 初始化格子累积矩阵（注意：不做归一化）
        x_bias_grid = np.zeros(init_shape, dtype=float)
        y_bias_grid = np.zeros(init_shape, dtype=float)

        # 对每个格子中心，累积来自同一侧（左右隔离）的点的贡献
        for i, gy in enumerate(x_centers):     # gy: 垂直坐标（0..250）
            for j, gx in enumerate(y_centers[::-1]): # gx: 水平坐标（-500..500）

                # 如果格子中心在禁区就跳过（也可以设为 NaN）
                if is_in_forbidden_zone(gx, gy):
                    continue

                # 遍历所有数据点，累积贡献（注意：保持左右隔离）
                for _, row in df.iterrows():
                    px = row['y']        # 原始点的水平坐标（-500..500）
                    py = row['x']        # 原始点的垂直坐标（0..250）
                    x_off = row['x_offset']
                    y_off = row['y_offset']

                    # 左右隔离：格子中心 (gx) 与 发球点 (px) 必须在同一半边
                    if (gx < 0 and px >= 0) or (gx >= 0 and px < 0):
                        continue

                    # ------------------ 关键的轴映射 ------------------
                    # 为了使 DEVISER_Y 控制“水平”扩散（左右），而 DEVISER_X 控制“垂直”扩散（上下）：
                    # 我们把水平差值除以 DEVISER_Y，垂直差值除以 DEVISER_X。
                    dx_scaled = (gx - px) / DEVISER_Y   # 水平方向归一化（受 DEVISER_Y 控制）
                    dy_scaled = (gy - py) / DEVISER_X   # 垂直方向归一化（受 DEVISER_X 控制）

                    # 两种模式（与原始C++同步）
                    if CALC_MODE == 1:
                        w = np.exp(-np.sqrt(dx_scaled**2 + dy_scaled**2))
                    else:  # case 2
                        w = np.exp(-(dx_scaled**2 - dy_scaled**2))

                    # 累积（乘上各自的校正系数）
                    x_bias_grid[i, j] += CORR_COEFF_Y * x_off * w
                    y_bias_grid[i, j] += CORR_COEFF_X * y_off * w

        # 更新热力图（按格子）
        im1.set_data(x_bias_grid)
        im2.set_data(y_bias_grid)
        im1.set_extent([Y_MAX, Y_MIN, X_MIN, X_MAX])
        im2.set_extent([Y_MAX, Y_MIN, X_MIN, X_MAX])
        im1.set_interpolation('nearest')
        im2.set_interpolation('nearest')

        # 散点保持展示所有原始点（不作左右隔离）
        if len(df) > 0:
            scatter1.set_offsets(np.column_stack([df['y'], df['x']]))
            scatter1.set_array(df['x_offset'])
            scatter2.set_offsets(np.column_stack([df['y'], df['x']]))
            scatter2.set_array(df['y_offset'])
        else:
            scatter1.set_offsets([])
            scatter2.set_offsets([])

        ax1.set_title(f'Y方向偏差热力图（蓝左红右） - 数据点: {len(df)}', fontsize=12, fontweight='bold')
        ax2.set_title(f'X方向偏差热力图（粉近绿远） - 数据点: {len(df)}', fontsize=12, fontweight='bold')

        print(f"更新时间: {time.strftime('%H:%M:%S')} - 数据点: {len(df)}")

    except Exception as e:
        print(f"读取数据时出错: {e}")

    return [im1, im2, scatter1, scatter2]


ani = FuncAnimation(fig, update_plot, interval=500, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()
