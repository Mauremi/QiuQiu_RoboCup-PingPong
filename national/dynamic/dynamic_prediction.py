import os
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 使用非阻塞后端
matplotlib.use('TkAgg')

# 乒乓球桌尺寸（单位：mm）
TABLE_LENGTH = 2740
TABLE_WIDTH = 1525

# 预测所对应的固定 x（与 C++ 中一致：x = 200）
PREDICT_X = 200

# CSV 路径（与 C++ 写入路径保持一致：repo_path/national/dynamic/data_prediction.csv）
CSV_PATH = os.path.join('national', 'dynamic', 'data_prediction.csv')


def draw_table(ax):
    # 桌面
    ax.add_patch(plt.Rectangle((0, -TABLE_WIDTH / 2), TABLE_LENGTH, TABLE_WIDTH,
                               fill=True, color='#311B92', linewidth=2))
    # 中线
    ax.plot([0, TABLE_LENGTH], [0, 0], color='white', linestyle='-', linewidth=1)
    # 中垂线（半场）
    ax.plot([TABLE_LENGTH / 2, TABLE_LENGTH / 2], [TABLE_WIDTH / 2, -TABLE_WIDTH / 2],
            color='white', linestyle='-', linewidth=1)

    # 预测 x 位置参考线
    ax.axvline(PREDICT_X, color='orange', linestyle='--', linewidth=1, alpha=0.8)


def setup_axes(ax, title):
    ax.set_xlim(0, TABLE_LENGTH)
    ax.set_ylim(-TABLE_WIDTH / 2, TABLE_WIDTH / 2)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=12)
    draw_table(ax)


fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
ax_linear = axes[0, 0]
ax_batch = axes[0, 1]
ax_full = axes[1, 0]
ax_half = axes[1, 1]

setup_axes(ax_linear, 'Linear')
setup_axes(ax_batch, 'Batch Quadratic')
setup_axes(ax_full, 'Full-table Quadratic')
setup_axes(ax_half, 'Half-table Quadratic')

# 初始化散点容器（历史点轨迹，使用渐变颜色表示先后顺序）
cmap = plt.cm.plasma
scatter_linear = ax_linear.scatter([], [], c=[], cmap=cmap, s=30)
scatter_batch = ax_batch.scatter([], [], c=[], cmap=cmap, s=30)
scatter_full = ax_full.scatter([], [], c=[], cmap=cmap, s=30)
scatter_half = ax_half.scatter([], [], c=[], cmap=cmap, s=30)

# 轴上的文本提示（显示最新 y 值）
text_linear = ax_linear.text(5, TABLE_WIDTH / 2 - 60, '', color='white', fontsize=10)
text_batch = ax_batch.text(5, TABLE_WIDTH / 2 - 60, '', color='white', fontsize=10)
text_full = ax_full.text(5, TABLE_WIDTH / 2 - 60, '', color='white', fontsize=10)
text_half = ax_half.text(5, TABLE_WIDTH / 2 - 60, '', color='white', fontsize=10)


def read_prediction_csv():
    if not os.path.exists(CSV_PATH):
        return None
    try:
        # 文件为追加写入且会重复写入表头，这里仅解析最后一段（最后一次表头之后的数据）
        with open(CSV_PATH, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        # 找到所有表头行的索引（不区分大小写，兼容潜在空格）
        header_marks = [i for i, l in enumerate(lines) if l.strip().lower().startswith('stage,')]
        if not header_marks:
            return None
        start_idx = header_marks[-1]

        from io import StringIO
        segment = ''.join(lines[start_idx:])
        if segment.strip() == '':
            return None
        df = pd.read_csv(StringIO(segment))
        return df
    except Exception:
        return None


def update(_frame):
    df = read_prediction_csv()
    if df is None or df.empty:
        return scatter_linear, scatter_batch, scatter_full, scatter_half, text_linear, text_batch, text_full, text_half

    # 过滤非法/空值
    y_linear = df['linear'].dropna().values if 'linear' in df.columns else []
    y_batch = df['batch_quadratic'].dropna().values if 'batch_quadratic' in df.columns else []
    y_full = df['full_table_quadratic'].dropna().values if 'full_table_quadratic' in df.columns else []
    # half 可能为空，使用 dropna 过滤
    y_half = df['half_table_quadratic'].dropna().values if 'half_table_quadratic' in df.columns else []

    # x 坐标固定为 PREDICT_X，长度等于历史点个数
    x_linear = [PREDICT_X] * len(y_linear)
    x_batch = [PREDICT_X] * len(y_batch)
    x_full = [PREDICT_X] * len(y_full)
    x_half = [PREDICT_X] * len(y_half)

    # 为每个子图生成颜色序列（0..n-1）并归一化，形成渐变
    if len(x_linear) > 0:
        scatter_linear.set_offsets(list(zip(x_linear, y_linear)))
        colors_l = np.arange(len(y_linear))
        scatter_linear.set_array(colors_l)
        scatter_linear.set_clim(0, max(len(y_linear) - 1, 1))
        text_linear.set_text(f'latest y: {y_linear[-1]:.1f} mm')
    if len(x_batch) > 0:
        scatter_batch.set_offsets(list(zip(x_batch, y_batch)))
        colors_b = np.arange(len(y_batch))
        scatter_batch.set_array(colors_b)
        scatter_batch.set_clim(0, max(len(y_batch) - 1, 1))
        text_batch.set_text(f'latest y: {y_batch[-1]:.1f} mm')
    if len(x_full) > 0:
        scatter_full.set_offsets(list(zip(x_full, y_full)))
        colors_f = np.arange(len(y_full))
        scatter_full.set_array(colors_f)
        scatter_full.set_clim(0, max(len(y_full) - 1, 1))
        text_full.set_text(f'latest y: {y_full[-1]:.1f} mm')
    if len(x_half) > 0:
        scatter_half.set_offsets(list(zip(x_half, y_half)))
        colors_h = np.arange(len(y_half))
        scatter_half.set_array(colors_h)
        scatter_half.set_clim(0, max(len(y_half) - 1, 1))
        text_half.set_text(f'latest y: {y_half[-1]:.1f} mm')

    return (scatter_linear, scatter_batch, scatter_full, scatter_half,
            text_linear, text_batch, text_full, text_half)


ani = FuncAnimation(fig, update, interval=500, blit=False)

plt.show(block=False)

print('动态预测可视化已启动：')
print(f'读取文件: {CSV_PATH}')
try:
    while plt.fignum_exists(fig.number):
        plt.pause(0.5)
except KeyboardInterrupt:
    pass
finally:
    plt.close()


