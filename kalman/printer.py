import cv2
import numpy as np

# 读取文件中的点集
def read_points_from_file(file_path):
    points = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                # 假设文件中每行是一个点，格式为 "x,y"
                x, y = map(float, line.strip().split(' '))
                points.append((x, y))
    except FileNotFoundError:
        print(f"错误：未找到文件 {file_path}")
    except ValueError:
        print("错误：文件中的数据格式不正确，请确保每行是 'x,y' 格式。")
    return points

# 假设文件路径
file_path_1 = 'in/homework_data_4.txt'
file_path_2 = 'out/homework_data_4.txt'
points_1 = read_points_from_file(file_path_1)
points_2 = read_points_from_file(file_path_2)

# 创建一个空白图像
image = np.zeros((800, 800, 3), dtype=np.uint8)

# 定义点的颜色（BGR格式）和半径
color_1 = (0, 255, 0)
color_2 = (0, 0, 255)
radius = 1

# 定义缩放因子和平移量
scale_factor = 100 # 缩放因子，可根据需要调整
translation_x = 50  # x 方向的平移量，可根据需要调整
translation_y = 50  # y 方向的平移量，可根据需要调整

# 绘制坐标轴
axis_color = (255, 255, 255)
# 绘制 x 轴
cv2.line(image, (translation_x, translation_y), (450, translation_y), axis_color, 2)
# 绘制 y 轴
cv2.line(image, (translation_x, translation_y), (translation_x, 450), axis_color, 2)

# 添加刻度和标签
tick_length = 10
tick_interval = 100
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.5
font_color = (255, 255, 255)

# x 轴刻度和标签
for i in range(0, 400, tick_interval):
    x = translation_x + i
    cv2.line(image, (x, translation_y), (x, translation_y - tick_length), axis_color, 1)
    cv2.putText(image, str(i // scale_factor), (x - 5, translation_y + 20), font, font_scale, font_color, 1)

# y 轴刻度和标签
for i in range(0, 400, tick_interval):
    y = translation_y + i
    cv2.line(image, (translation_x, y), (translation_x + tick_length, y), axis_color, 1)
    cv2.putText(image, str(i // scale_factor), (translation_x - 30, y + 5), font, font_scale, font_color, 1)

# 遍历第一个点集并在图像上绘制点
for point in points_1:
    # 对坐标进行缩放和平移
    scaled_x = point[0]* 1 * scale_factor + translation_x
    scaled_y = point[1] * scale_factor + translation_y
    int_point = (int(scaled_x), int(scaled_y))
    cv2.circle(image, int_point, radius, color_1, -1)

# 遍历第二个点集并在图像上绘制点
for point in points_2:
    # 对坐标进行缩放和平移
    scaled_x = point[0] * 1 * scale_factor + translation_x
    scaled_y = point[1] * scale_factor + translation_y
    int_point = (int(scaled_x), int(scaled_y))
    cv2.circle(image, int_point, radius, color_2, -1)

# 显示图像
cv2.imshow('Points', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
    