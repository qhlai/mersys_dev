import numpy as np
import math
from matplotlib import pyplot as plt
from shapesimilarity import shape_similarity
# from scipy.interpolate import interp1d


def rmse(predictions, targets):
    return np.sqrt(((predictions - targets) ** 2).mean())
def mae(predictions, targets):
    return np.abs(predictions - targets).mean()
CONSTANTS_RADIUS_OF_EARTH = 6371000 
def GPStoXY(lat, lon, ref_lat, ref_lon):
        # input GPS and Reference GPS in degrees
        # output XY in meters (m) X:North Y:East
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        cos_d_lon = math.cos(lon_rad - ref_lon_rad)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
        c = math.acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c))

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH)

        return x, y

print(123)
data_est = []
x_est=[]
y_est=[]

time_real=[]
x_real=[]
y_real=[]
z_real=[]
x_real_bias=0
y_real_bias=0
# with open('./colive_backend/output/path_0_save_all_data.txt', 'r') as f:
#        line = f.readline()
#        while line:
#                line_split=line.split(',')
#                time = float(line_split[0])
#                x = float(line_split[1])
#                y = float(line_split[2])
#                z = float(line_split[3])
#                x_est.append(x)
#                y_est.append(y)
#                data_est.append([time,x,y])
#             #    print(line.split(','))
#                line = f.readline()
with open('./colive_backend/output/path0.txt', 'r') as f:
       line = f.readline()
       line = f.readline()
       line_split=line.split(',')
       x_real_bias = float(line_split[6])
       y_real_bias = float(line_split[7])
       while line:
               line_split=line.split(',')
               time = float(line_split[0])/1e9
               x_la = float(line_split[6])
               y_lo = float(line_split[7])               
               x,y = GPStoXY( x_la, y_lo, x_real_bias, y_real_bias)
               time_real.append(time)
               x_real.append(x)
               y_real.append(y)
            #    data_est.append([time,x,y])
            #    print(line.split(','))
               line = f.readline()
print(y_real)
# # x_real_bias
# # for point in y_est:
# #       print(point)
# plt.plot(x_est,y_est, color='b', label="Predicted curve")
# # x_est = 
# # np.random.seed(10)
# # x = np.linspace(0, 10, 100)
# # y_true = np.sin(x)
# # y_pred = y_true + np.random.randn(len(x)) * 0.1

# plt.plot(x_real,y_real, 'r', label="True curve")
# # plt.plot(x, y_pred, 'b', label="Predicted curve")
# plt.legend()
# plt.show()
#保存数据open函数
with open('./colive_backend/output/groundtruth0.txt','a',encoding='utf-8') as f:#使用with open()新建对象f
    for i in range(len(x_real)):
            print(i, time_real[i], x_real[i], y_real[i])
            a =str(time_real[i])+" "+str(x_real[i])+" "+ str(y_real[i])+ " 0 0 0 0 1"+'\n'
            f.write(a)#写入数据，文件保存在上面指定的目录，加\n为了换行更方便阅读  




# f_true = interp1d(x, y_true)
# f_pred = interp1d(x, y_pred)

# x_new = np.linspace(0, 10, 1000)

# y_true_new = f_true(x_new)
# y_pred_new = f_pred(x_new)

# print("RMSE:", rmse(y_pred_new, y_true_new))
# print("MAE:", mae(y_pred_new, y_true_new))