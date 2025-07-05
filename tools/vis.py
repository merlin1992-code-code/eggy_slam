import pandas as pd
import matplotlib.pyplot as plt

# 读取数据，自动识别列名（不加 comment 参数）
df = pd.read_csv('rtk_odom.txt', sep='\s+')

# 检查实际列名
print(df.columns)

# 画速度随时间变化
df['time_ms'] = (df['#timestamp'] - df['#timestamp'].iloc[0]) / 1e6
plt.figure(figsize=(10,6))
plt.plot(df['time_ms'], df['vel_x'], label='vel_x')
plt.plot(df['time_ms'], df['vel_y'], label='vel_y')
plt.plot(df['time_ms'], df['vel_z'], label='vel_z')
plt.xlabel('Time (ms)')
plt.ylabel('Velocity')
plt.legend()
plt.title('Velocity over Time')
plt.show()

# 画轨迹（以第一帧为原点）
x0, y0 = df['vel_x'][0], df['vel_y'][0]
plt.figure(figsize=(8,6))
plt.plot(df['vel_x'] - x0, df['vel_y'] - y0, marker='o')
plt.xlabel('vel_x (relative)')
plt.ylabel('vel_y (relative)')
plt.title('Trajectory (First Frame as Origin)')
plt.axis('equal')
plt.grid(True)
plt.show()