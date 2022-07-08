'''
直线拟合, 趋势预测是否直行, 路口检测, 十字路口还是T字路口
'''

import numpy as np
import matplotlib.pyplot as plt
import random


def lineFit(curb_point):

    # print(curb_point[:, 0])
    # print(curb_point[:, 1])
    z1 = np.polyfit(curb_point[:, 0], curb_point[:, 1], 1)
    p1 = np.poly1d(z1)

    return p1


def fig2data(fig):
    """
    fig = plt.figure()
    image = fig2data(fig)
    @brief Convert a Matplotlib figure to a 4D numpy array with RGBA channels and return it
    @param fig a matplotlib figure
    @return a numpy 3D array of RGBA values
    """
    import PIL.Image as Image
    # draw the renderer
    fig.canvas.draw()

    # Get the RGBA buffer from the figure
    w, h = fig.canvas.get_width_height()
    buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
    buf.shape = (w, h, 4)

    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    buf = np.roll(buf, 3, axis=2)
    image = Image.frombytes("RGBA", (w, h), buf.tostring())
    image = np.asarray(image)
    return image

def create_data():

    # 数据量
    SIZE = 100
    # 产生数据 np.linspace 返回一个一维数组，SIZE指定数组长度
    # 数组最小值是0, 最大值是10, 所有元素间隔相等
    X = np.linspace(0, 10, SIZE)
    Y = 3 * X + 10

    # 让散点图的数据更加随机并且添加一些噪声
    random_x = []
    random_y = []
    # 添加直线随机噪声
    for i in range(SIZE):
        random_x.append(X[i] + random.uniform(-0.5, 0.5))
        random_y.append(Y[i] + random.uniform(-0.5, 0.5))
        # 添加随机噪声
    for i in range(20):
        random_x.append(random.uniform(0, 5))
        random_y.append(random.uniform(10, 25))
    RANDOM_X = np.array(random_x)  # 散点图的横轴
    RANDOM_Y = np.array(random_y)  # 散点图的纵轴
    return RANDOM_X, RANDOM_Y

if __name__ == '__main__':

    fig = plt.figure()
    # 画图区域分成1行1列。选择第一块区域
    ax1 = fig.add_subplot(1, 1, 1)
    # 标题
    ax1.set_title("line_fit")

    RANDOM_X, RANDOM_Y = create_data()

    z1 = np.polyfit(RANDOM_X, RANDOM_Y, 1)
    p1 = np.poly1d(z1)

    # 画散点图
    ax1.scatter(RANDOM_X, RANDOM_Y)

    RANDOM_X = np.expand_dims(RANDOM_X, axis=1)
    RANDOM_Y = np.expand_dims(RANDOM_Y, axis=1)

    point = np.concatenate((RANDOM_X ,RANDOM_Y), axis=1)
    p1 = lineFit(point)



    print(p1)
    print(p1[0])
    print(p1[1])
    print(p1[2])
    print(p1[3])
    Y = p1[1] * RANDOM_X + p1[0]
    # print(RANDOM_X)
    # print(Y)

    ax1.plot(RANDOM_X, Y)

    plt.show()


    sub_curb_point = [[[1, 3], [2, 5], [3, 7]], [[4, 9], [5, 11], [6, 13]]]
    lineFit(sub_curb_point)
