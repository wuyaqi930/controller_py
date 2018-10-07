#!/usr/bin/python

# 导入库文件
from numpy import *

# 定义运动学模型相关变量

P_t=mat(zeros((3,1)))#定义状态矩阵（X，Y，theta），并初始化

# P_t[0]=10 # 定义状态矩阵初始数值不为零
# P_t[1]=0
# P_t[2]=10

Pd_t=mat(zeros((3,1))) 

P_derivative_t=mat(zeros((3,1))) #定义状态矩阵的导数（X，Y，theta），并初始化
Pd_derivative_t=mat(zeros((3,1)))

U_t=mat(zeros((2,1))) # 定义输入U（线速度，角速度），并初始化
Ud_t=mat(zeros((2,1))) 

A=mat(zeros((3,2))) #定义运动学方程的转换矩阵，并初始化
Ad=mat(zeros((3,2)))

P_delta=mat(zeros((3,1))) #定义理想状态矩阵和实际状态矩阵的差值，并初始化

E_t=mat(zeros((3,1))) #定义Kanayama类型误差变量，并初始化

B=mat(zeros((3,3)))#定义Kanayama类型误差变量方程的转换矩阵，并初始化

Kx=1      #controller的参数（王凯公式（15））
K_theta=1

# 设置循环100s 

for num in range(1,100):

    #给理想输入 Ud 赋值（走圆形的话，线速度V和角速度w都是常数
    Ud_t[0]= 1
    Ud_t[1]= 1

    #---------------------------给实际输入 U 赋值（根据相关的公式进行输入）---------------------------

    #计算P_delta，求理想轨迹和实际轨迹的误差
    P_delta = Pd_t - P_t

    #计算Kanayama类型误差变量方程的转换矩阵B

    B[0,0]= cos(P_t[2])  #给转换矩阵B进行赋值
    B[0,1]= sin(P_t[2])
    B[1,0]= -sin(P_t[2])
    B[1,1]= cos(P_t[2])
    B[2,2]= 1

    #计算Kanayama类型误差变量
    E_t = B * P_delta

    print("E_t")
    print(E_t)

    # 计算输入 (按道理各个项目应当就到此时能够实现）
	# V（t）= U_t(0),W(t) = U_t(1)	Vd（t）= Ud_t(0),Wd(t) = Ud_t(1)
	# e_theta = E_t(2) e_x = E_t(0)

    if (E_t[2] == 0):
        U_t[0] = Ud_t[0]*cos(E_t[2]) + Kx * E_t[0]  #王凯公式（15）需要进行一个判断，否则会出现没有解的情况
        U_t[1]= Ud_t[1] + K_theta * E_t[2] + Ud_t[0]*E_t[1]
    else:
        U_t[0]= Ud_t[0]*cos(E_t[2]) + Kx * E_t[0]  #王凯公式（15）需要进行一个判断，否则会出现没有解的情况
        U_t[1]= Ud_t[1] + K_theta * E_t[2] + Ud_t[0]*E_t[1]*sin(E_t[2]) / E_t[2]
   
    #---------------------------下面进行理想轨迹和实际轨迹之间的误差分析---------------------------
    
    # 给理想运动学方程转换矩阵Ad进行赋值 
    Ad[0,0] = cos(Pd_t[2])
    Ad[1,0] = sin(Pd_t[2])
    Ad[2,1] = 1

    # 给实际运动学方程转换矩阵A进行赋值 
    A[0,0] = cos(P_t[2])
    A[1,0] = sin(P_t[2])
    A[2,1] = 1

    # 输出（ Xd，Yd，theta_d ）的导数
    Pd_derivative_t = Ad * Ud_t # 矩阵计算求得状态矩阵的导数
    P_derivative_t = A * U_t # 矩阵计算求得状态矩阵的导数

    # 输出（ Xd，Yd，theta_d ）
    Pd_t += Pd_derivative_t # 通过累加实现对导数的积分

    # 输出（ X，Y，theta ）
    P_t += P_derivative_t # 通过累加实现对导数的积分

    # 调试使用相关代码（必要时可以删除）
    print("\nU_t")
    print (U_t)
    
    print("\nPd_t")
    print (Pd_t)

    print("\nP_t")
    print (P_t)

    print("\nPd_t-P_t")
    print (Pd_t-P_t)



