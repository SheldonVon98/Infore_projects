import numpy as np
import matplotlib.pyplot as plt
import random
import math


def line_fit(points):
    SIZE = len(points)

    X = np.array(points, dtype=int)[:, 0]
    Y = np.array(points, dtype=int)[:, 1]

    # 使用RANSAC算法估算模型
    # 迭代最大次数，每次得到更好的估计会优化iters的数值
    iters = 100
    # 数据和模型之间可接受的差值
    sigma = 0.25
    # 最好模型的参数估计和内点数目
    best_a = 0
    best_b = 0
    pretotal = 0
    # 希望的得到正确模型的概率
    P = 0.99
    for i in range(iters):
        # 随机在数据中红选出两个点去求解模型
        sample_index = random.sample(range(SIZE),2)
        x_1 = X[sample_index[0]]
        x_2 = X[sample_index[1]]
        y_1 = Y[sample_index[0]]
        y_2 = Y[sample_index[1]]

        # y = ax + b 求解出a，b
        if x_2 - x_1 == 0:
            a = 99999
        else:
            a = (y_2 - y_1) / (x_2 - x_1)
        b = y_1 - a * x_1

        # 算出内点数目
        total_inlier = 0
        for index in range(SIZE):
            y_estimate = a * X[index] + b
            if abs(y_estimate - Y[index]) < sigma:
                total_inlier = total_inlier + 1

        # 判断当前的模型是否比之前估算的模型好
        if total_inlier > pretotal:
            iters = math.log(1 - P) / math.log(1 - pow(total_inlier / (SIZE * 2), 2))
            pretotal = total_inlier
            best_a = a
            best_b = b

        # 判断是否当前模型已经符合超过一半的点
        if total_inlier > SIZE/1.5:
            break

    # 用我们得到的最佳估计画图
    x1 = X[-1]
    y1 = best_a * x1 + best_b
    return x1, y1, best_a

