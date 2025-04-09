import pandas as pd
import matplotlib.pyplot as plt

def plot_points(csv_file, txt_file):
    # 读取 CSV 文件
    csv_data = pd.read_csv(csv_file, header = 0,names=['DATA','PRICE'])
    csv_x = csv_data['DATA']
    csv_y = csv_data['PRICE']

    # 读取 TXT 文件，假设 TXT 文件中的数据以空格分隔
    txt_data = pd.read_csv(txt_file, sep=' ', header=None)
    txt_x = txt_data[0]
    txt_y = txt_data[1]

    # 绘制点集
    plt.scatter(csv_x, csv_y, label='CSV Points', color='blue')
    plt.scatter(txt_x, txt_y, label='TXT Points', color='red')

    # 设置图形标题和坐标轴标签
    plt.title('Point Set Plot')
    plt.xlabel('X')
    plt.ylabel('Y')

    # 显示图例
    plt.legend()

    # 显示图形
    plt.show()

# 指定 CSV 文件和 TXT 文件的路径
csv_file = 'in/stock_prices.csv'
txt_file = 'out/result.txt'
# 调用绘图函数
plot_points(csv_file, txt_file)