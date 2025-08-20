import matplotlib.pyplot as plt
import pandas as pd
import matplotlib
# 弹出窗口
matplotlib.use('TkAgg')

# 读取数据
data = pd.read_csv(r'C:\Robot\2025PingPangCampus\QiuQiuPingPang_allCodes\campus_example\pingpong_trajectory.csv')  # 修改为你的文件路径
x = data['x']
y = data['y']

# 绘图
fig, ax = plt.subplots(figsize=(10, 6))

# 画乒乓球桌轮廓
table_length = 2740  # mm，乒乓球桌的长度
table_width = 1525  # mm，乒乓球桌的宽度
ax.set_xlim(0, table_length)
ax.set_ylim(-table_width / 2, table_width / 2)

# 绘制边界线
ax.add_patch(plt.Rectangle((0, -table_width / 2), table_length, table_width, fill=False, edgecolor='blue', linewidth=2))

# 绘制中线
ax.plot([0, table_length], [0, 0], color='black', linestyle='--', linewidth=2)
ax.plot([table_length / 2, table_length / 2], [table_width / 2, -table_width / 2], color='black', linestyle='--', linewidth=2)

# 绘制发球区线（可选）
# 发球区根据比赛规则，可以绘制，也可以不绘制
# ax.plot([0, 0], [0, table_width], color='green', linestyle='-', linewidth=2, label='发球区')

# 画出轨迹线
ax.plot(x, y, color='red', linewidth=2, label='轨迹')
ax.scatter(x, y, color='black', s=10)  # 加点以辅助查看

# 添加门的标识（9个门）
door_width = 20  # 门的宽度
door_spacing = 180  # 每两个门中心的间隔
door_color = 'green'  # 门的颜色

# 计算门的位置，并绘制
for i in range(9):
    door_center_y = (i - 4) * door_spacing  # 第5个门的Y坐标是0，其他门按此规律调整
    ax.add_patch(plt.Rectangle((table_length - door_width, door_center_y - 50), door_width, 100, fill=True, color=door_color))

# ax.text(0, -table_width / 2, f'(0, {-table_width / 2})', fontsize=12, ha='center', va='center', color='green')
# ax.text(table_length, -table_width / 2, f'({table_length}, {-table_width / 2})', fontsize=12, ha='center', va='center', color='green')
# ax.text(0, table_width / 2, f'(0, {table_width / 2})', fontsize=12, ha='center', va='center', color='green')
# ax.text(table_length, table_width / 2, f'({table_length}, {table_width / 2})', fontsize=12, ha='center', va='center', color='green')

ax.set_xticks([])  # 关闭 x 轴
ax.set_yticks([])  # 关闭 y 轴

# 显示图形
plt.show()
