import numpy as np
import cv2

# 准备标定图像
images = []  # 存放标定图像路径
# 加载图像
for i in range(1, 83):
    img = cv2.imread(f'/home/nuc_12/img/img_{i}.jpg')
    images.append(img)

# 准备物理世界坐标
# 这里假设棋盘格每个格子的大小为 25mm x 25mm
# 生成物理世界坐标
objp = np.zeros((6 * 9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 25

# 用于存储检测到的角点
objpoints = []  # 存储物理世界坐标
imgpoints = []  # 存储图像坐标

for img in images:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 查找角点
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# 执行标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 打印相机参数
print("相机矩阵:")
print(mtx)
print("畸变参数:")
print(dist)

# # 校正畸变
# img = cv2.imread('test_image.jpg')
# h, w = img.shape[:2]
# newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
# dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# # 显示校正后的图像
# cv2.imshow('Original Image', img)
# cv2.imshow('Undistorted Image', dst)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
