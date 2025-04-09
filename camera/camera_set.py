#AI-generated
import cv2
import numpy as np
import glob

# 定义棋盘格内角点的行数和列数
pattern_size = (9, 13)
# 创建一个零矩阵，用于存储棋盘格角点在真实世界坐标系中的 3D 坐标
# 这里假设棋盘格位于 Z = 0 的平面上
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
# 生成棋盘格角点的 X 和 Y 坐标
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# 用于存储所有图像中棋盘格角点在真实世界坐标系中的 3D 坐标
objpoints = []
# 用于存储所有图像中棋盘格角点在图像平面上的 2D 坐标
imgpoints = []

print("初始化完成")

# 获取所有用于相机标定的棋盘格图像的文件路径
images = glob.glob('data/*.png')

print("已执行读取")

# 遍历所有棋盘格图像
for fname in images:
    # 读取图像
    img = cv2.imread(fname)
    # 将图像转换为灰度图像，因为角点检测通常在灰度图像上进行
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 查找棋盘格角点
    # ret 表示是否成功找到角点，corners 是找到的角点的坐标
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    # 如果成功找到角点
    if ret:
        # 将当前图像对应的棋盘格角点的 3D 坐标添加到 objpoints 列表中
        objpoints.append(objp)

        # 定义角点亚像素级检测的终止条件
        # 当迭代次数达到 30 次或者角点位置变化小于 0.001 时停止迭代
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # 进行亚像素级角点检测，提高角点检测的精度
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # 将当前图像对应的棋盘格角点的 2D 坐标添加到 imgpoints 列表中
        imgpoints.append(corners2)

        # 在图像上绘制检测到的角点
        cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
        # 显示绘制了角点的图像
        cv2.imshow('img', img)
        # 等待 500 毫秒，以便查看图像
        cv2.waitKey(500)

# 关闭所有打开的窗口
cv2.destroyAllWindows()

# 进行相机标定
# ret 表示标定是否成功
# mtx 是相机内参矩阵
# dist 是畸变系数
# rvecs 是旋转向量
# tvecs 是平移向量
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 打印相机内参矩阵
print("相机内参矩阵:")
print(mtx)

# 初始化总重投影误差
total_error = 0
# 遍历所有图像
for i in range(len(objpoints)):
    # 将 3D 对象点投影到图像平面上，得到投影点的坐标
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    # 计算投影点与实际检测到的图像点之间的 L2 范数误差，并除以点数得到单张图像的重投影误差
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    # 累加单张图像的重投影误差到总重投影误差中
    total_error += error

# 计算平均重投影误差
mean_error = total_error / len(objpoints)
# 打印平均重投影误差
print("重投影误差: {}".format(mean_error))

# 将相机标定结果保存到一个 .npz 文件中
np.savez('calibration_results.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

# 读取一张测试图像，用于验证标定结果
test_img = cv2.imread(images[0])
# 获取测试图像的高度和宽度
h, w = test_img.shape[:2]
# 计算最优的新相机内参矩阵，用于去畸变处理
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# 对测试图像进行去畸变处理
dst = cv2.undistort(test_img, mtx, dist, None, newcameramtx)

# 获取裁剪区域的坐标和尺寸
x, y, w, h = roi
# 裁剪去畸变后的图像，去除边缘的无效区域
dst = dst[y:y + h, x:x + w]

# 显示去畸变后的图像
cv2.imshow('Undistorted Image', dst)
# 等待用户按下任意键
cv2.waitKey(0)
# 关闭所有打开的窗口
cv2.destroyAllWindows()
