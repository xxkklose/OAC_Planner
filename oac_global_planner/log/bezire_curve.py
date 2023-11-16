import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.special import comb

def bernstein_poly(i, n, t):
    return comb(n, i) * ( t**(n-i) ) * ( (1-t)**i )

def bezier_curve(control_points, num_points=1000):
    n = len(control_points) - 1
    curve = np.zeros((num_points, len(control_points[0])))
    
    for i in range(num_points):
        t = i / (num_points - 1)
        curve[i] = sum([bernstein_poly(j, n, t) * control_points[j] for j in range(n + 1)])
    
    return curve

def bezier_curve_derivative(control_points, num_points=1000):
    n = len(control_points) - 1
    curve_derivative = np.zeros((num_points, len(control_points[0])))
    
    for i in range(num_points):
        t = i / (num_points - 1)
        curve_derivative[i] = n * sum([(control_points[j+1] - control_points[j]) * 
                                       bernstein_poly(j, n-1, t) for j in range(n)])
    
    return curve_derivative

# 输入三维点
points = np.array([
    (0.00294652, 0.00168454, 0.0160459),
    (0.408545, 0.294071, 0.00682506),
    (0.824201, 0.571975, 0.00682506),
    (0.859152, 1.07075, 0.00682506),
    (1.32386, 0.590322, 0.00682506),
    (1.19764, 1.43875, 0.0160459)
])

# 生成三维贝塞尔曲线
bezier_points = bezier_curve(points)

# 生成三维贝塞尔曲线的导数
bezier_derivative_points = bezier_curve_derivative(points)

# 显示原始折线图
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.plot(points[:, 0], points[:, 1], points[:, 2], marker='o', label='Original Points')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

# 显示三维贝塞尔曲线图
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
ax2.plot(bezier_points[:, 0], bezier_points[:, 1], bezier_points[:, 2], label='Bezier Curve')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

# 显示三维贝塞尔曲线的导数图
fig3 = plt.figure()
ax3 = fig3.add_subplot(111, projection='3d')
ax3.plot(bezier_derivative_points[:, 0], bezier_derivative_points[:, 1], bezier_derivative_points[:, 2], label='Bezier Curve Derivative')
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')

# 设置z轴尺度为0到1
ax1.set_zlim(0, 1)
ax2.set_zlim(0, 1)
ax3.set_zlim(0, 1)

# 显示图形
plt.show()
