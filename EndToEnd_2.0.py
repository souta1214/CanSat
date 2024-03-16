import math
import time
import datetime
import csv
import os
import numpy as np
import threading
from argparse import ArgumentParser
import sys
sys.path.append('../')
from functions import motorFunctions
from functions import gpsFunctions
from functions import cameraFunctions
from functions.cameraFunctions import takepic_openDetect
# from functions.camerafunction2 import takepic_openDetect
# from functions import camerafunction2
from functions.nineaxisFunction import getMag
from functions.recordFunctions import csv_write_f
from functions.heatUpNichrome import heatUpNichrome
from functions.rmNoiseFunction import rmnoise
from functions.barometerFunction import getPressure


# from PIL import Image,ImageOps
# import cv2
# import re


################## モータに関する定数の定義 #########################
# duty比
R_forward_duty = 55
L_forward_duty = 50  
R_rotateRight_duty = 33
L_rotateRight_duty = 30
R_rotateLeft_duty = 33
L_rotateLeft_duty = 30 
R_backward_duty = 55
L_backward_duty = 50
# モータ回転時間
forward_dt = 1.0  # (s/m)
backward_dt = 1.0  # (s/m)
rotate_dt = 1/90  # (s/deg)
######################## 現在の状態 ##############################
failed_counter = 0
gps_thread_flag = True
latitude = 36.111956
longitude = 140.098956
nineaxis_thread_flag = False
dis_x_list = []
dis_y_list = []
######################################################################


def landDetectPhase():
    #################### 地上の気圧を取得 #########################
    preslist = []
    for i in range(10):
        time.sleep(1)
        pressure = getPressure()
        preslist.append(pressure)
    pressure_ground = (sum(preslist) / len(preslist)) - 1.5
    pressure_sky = pressure_ground - 1.5
#   pressure_sky = pressure_ground*((1-(0.0065*h)/(t0+0.0065*h+273.15))**5.257)
    # h = 標高(m)
    # t0 = 海抜0mの気温(℃)
    n = 0  # 初期化
    ####################### 上空検知 ############################
    while True:
        time.sleep(1)
        pressure = getPressure()
        print(f"pressure={pressure}")
        if pressure < pressure_sky:
            n += 1
            print(f"sky_count={n}")
        else:
            n = 0
        if n >= 10:
            break
    print("In the sky!")
    ####################### 地上検知 ############################
    n = 0  # 初期化
    while True:
        time.sleep(1)
        pressure = getPressure()
        print(f"pressure={pressure}")
        if pressure > pressure_ground:
            n += 1
            print(f"ground_count={n}")
        else:
            n = 0
        if n >= 10:
            break
    print("On the ground!")
    
def openDetectPhase():
    global gps_thread_flag
    gps_thread_flag = True
    heatUpNichrome(3.5)
    try:
        motorFunctions.GoForward(R_forward_duty, L_forward_duty, 0)
    except KeyboardInterrupt:
        motorFunctions.CleanUp()
    open_counter = 0  
    global failed_counter
    while True: #明るい箇所の割合が一定以上になるまで繰り返す
        time.sleep(1)
        data=takepic_openDetect()
        lightning_prop=data[1] #明るい箇所の割合取得
        write_data = ("open_detect",lightning_prop)
        print(f"lightning_prop={lightning_prop}")
        if lightning_prop > 5: 
            open_counter += 1
            print(f"open_count={open_counter}")
        else:
            heatUpNichrome(1)
            open_counter = 0
            failed_counter += 1
            print(f"failed_count={failed_counter}")
        if open_counter >= 5:
            print("open!!")
            break
        if failed_counter >= 3:
            print("failed;;")
            break
            
    try:
        motorFunctions.GoForward(R_forward_duty, L_forward_duty, 3)
    except KeyboardInterrupt:
        motorFunctions.CleanUp()


def guidePhase1():
    ###################################################################
    ########################## guidePhase1 ############################
    ###################################################################
    global gps_thread_flag
    global nineaxis_thread_flag
    ################### gpsのデータをアップデート ######################
    print("===== gps data update =====")
    for i in range(20):
        time.sleep(1)
        if i%10==0:
            print(f"{20-i}")
    ##################### ゴール座標を読み込み ########################
    with open ('goal/goal.csv', 'r') as f :
        reader = csv.reader(f)
        line = [row for row in reader]
        goal_latitude = float(line[ 1 ] [ 0 ])
        goal_longitude = float(line[ 1 ] [ 1 ])
    ####################### gpsから位置情報を取得 ######################
    x_now, y_now = gpsFunctions.getXY(latitude, longitude, goal_latitude, goal_longitude)
    x_now = -x_now
    goal_distance = math.sqrt(x_now**2 + y_now**2)
    ################## 制御履歴保存ファイルの作成 ######################
    csv_write = csv_write_f()


    while goal_distance > 3:  # ゴールまでの距離が3m以内になるとbreak
        ################### 9軸キャリブレーション #####################
        print("======= calibration =======")
        gps_thread_flag = False
        nineaxis_thread_flag = True
        print("======= motor start ========")
        motorFunctions.RotateRight(R_rotateRight_duty, L_rotateRight_duty, 15)
        print("======== motor stop ========")
        x_mag_max = max(rmnoise(dis_x_list,2))
        x_mag_min = min(rmnoise(dis_x_list,2))
        y_mag_max = max(rmnoise(dis_y_list,2))
        y_mag_min = min(rmnoise(dis_y_list,2))
        dis_x = -(x_mag_max+x_mag_min)/2
        dis_y = -(y_mag_max+y_mag_min)/2
        nineaxis_thread_flag = False
        for i in range(2):
            ######################### 角度計算 ############################
            # 9軸から方向ベクトルを取得
            x_mag_list, y_mag_list = getMag(dis_x, dis_y)
            x_mag = np.average(rmnoise(x_mag_list,2))
            y_mag = np.average(rmnoise(y_mag_list,2))
            # gpsの位置情報から角度を取得
            theta_goal = math.atan2(y_now, x_now) * 180/math.pi + 180
            theta_nineaxis = math.atan2(y_mag, x_mag) * 180/math.pi
            if (theta_nineaxis < 0): theta_nineaxis = 360 + theta_nineaxis


            theta_theta = theta_goal - theta_nineaxis
            if(abs(theta_theta) >= 180): theta_rotate = 360 - abs(theta_theta)
            elif(abs(theta_theta) < 180): theta_rotate = abs(theta_theta)
            ###################### 進行方向の修正 #########################
            if -180 <= theta_theta <= 0 or 180 <= theta_theta:
                rotateRight_t = rotate_dt * theta_rotate
                try:
                    motorFunctions.RotateRight(R_rotateRight_duty, L_rotateRight_duty, rotateRight_t)
                except KeyboardInterrupt:
                    motorFunctions.CleanUp()
                theta = - theta_rotate
            if 0 < theta_theta < 180 or theta_theta < -180:
                rotateLeft_t = rotate_dt * theta_rotate
                try:
                    motorFunctions.RotateLeft(R_rotateLeft_duty, L_rotateLeft_duty, rotateLeft_t)
                except KeyboardInterrupt:
                    motorFunctions.CleanUp()
                theta = theta_rotate
        ########################### 直進 ##############################
        gps_thread_flag = True
        try:
            motorFunctions.GoForward(R_forward_duty, L_forward_duty, goal_distance*forward_dt)
        except KeyboardInterrupt:
            motorFunctions.CleanUp()
        ######################### 位置情報の更新 #######################
        x_now, y_now = gpsFunctions.getXY(latitude, longitude, goal_latitude, goal_longitude)
        x_now = -x_now
        # ゴールまでの距離を算出
        goal_distance = math.sqrt(x_now**2 + y_now**2)
        ######################## 制御履歴の保存 ##########################
        write_data = ["guidePhase1", theta, theta_nineaxis, latitude, longitude, x_now, y_now, goal_distance]
        csv_write(*write_data)
        ####################### 現在の状態を出力 #########################
        print(f" x_now={x_now}, y_now={y_now}, goal_distance={goal_distance}")


def guidePhase2():
    global gps_thread_flag
    global nineaxis_thread_flag
    global R_forward_duty
    global L_forward_duty
    global count
    global takepic_count
    R_forward_duty = 50
    L_forward_duty = 50
    takepic_counter = 0
    ################# 保存先のディレクトリを作成 ####################
    image_folder="image_jpg_folder"
    scanth_folder="scanth_jpg_folder"
    os.makedirs(image_folder, exist_ok=True)
    os.makedirs(scanth_folder, exist_ok=True)


    for i in range(5): # 接近の上限を5回に制限する
        ################### 9軸キャリブレーション #####################
        print("======= calibration =======")
        gps_thread_flag = False
        nineaxis_thread_flag = True
        motorFunctions.RotateRight(R_rotateRight_duty, L_rotateRight_duty, 15)
        x_mag_max = max(rmnoise(dis_x_list,2))
        x_mag_min = min(rmnoise(dis_x_list,2))
        y_mag_max = max(rmnoise(dis_y_list,2))
        y_mag_min = min(rmnoise(dis_y_list,2))
        dis_x = -(x_mag_max+x_mag_min)/2
        dis_y = -(y_mag_max+y_mag_min)/2
        nineaxis_thread_flag = False
        ##################### 赤コーン探索フェーズ#######################
        max_prop = 0
        x_mag_list = []
        y_mag_list = []
        for j in range(1, 16):
            data = cameraFunctions.takepic()
            print('{}回目, prop{}'.format(j, data[1]))  #デバック用
            # 20回中propが最大のもののtheta_reltiveを取得
            if max_prop < data[1]:
                # 最大の更新＋値の代入
                max_prop = data[1]
                max_takepic_counter = data[3]
                x_mag_list, y_mag_list = getMag(dis_x, dis_y)
            try: 
                motorFunctions.RotateRight(R_rotateRight_duty, L_rotateRight_duty, 50*rotate_dt)
            except KeyboardInterrupt:
                motorFunctions.CleanUp()
            time.sleep(0.3)
        takepic_counter = 0
        count =  0
        # 探索結果の出力
        print("find!!!")
        print(f"max_prop={max_prop}")
        print(f"max_takepic_counter = {max_takepic_counter}")
        ##################### 赤コーン接近フェーズ #######################
        x_mag = np.average(rmnoise(x_mag_list,2))
        y_mag = np.average(rmnoise(y_mag_list,2))
        max_prop_theta = math.atan2(y_mag, x_mag) * 180/math.pi
        for i in range(2):
            # 現在の方向を取得
            x_mag_list, y_mag_list = getMag(dis_x, dis_y)
            x_mag = np.average(rmnoise(x_mag_list,2))
            y_mag = np.average(rmnoise(y_mag_list,2))
            theta = math.atan2(y_mag, x_mag) * 180/math.pi

            theta_to_rotate = max_prop_theta -theta
            if(theta_to_rotate >= 180): theta_to_rotate = -360 + theta_to_rotate
            elif(theta_to_rotate <= -180): theta_to_rotate = 360 + theta_to_rotate

            #　角度修正
            if theta_to_rotate > 0:
                try:
                    motorFunctions.RotateLeft(R_rotateLeft_duty, L_rotateLeft_duty, abs(theta_to_rotate)*rotate_dt)
                except KeyboardInterrupt:
                    motorFunctions.CleanUp()
            else:
                try:
                    motorFunctions.RotateRight(R_rotateRight_duty, L_rotateRight_duty, abs(theta_to_rotate)*rotate_dt)
                except KeyboardInterrupt:
                    motorFunctions.CleanUp()
        # 1m前進
        try:
            motorFunctions.GoForward(R_forward_duty, L_forward_duty, 0.8*forward_dt)
        except KeyboardInterrupt:
                motorFunctions.CleanUp()
        ########################### 終了判定 ###############################
        # 終了判定（コーンとの距離をコーン画像の縦の大きさで判定）5回以内に0mと判断した場合
        if cameraFunctions.fin_detect(data[2])== 1:
            break
    # 1m前進
    try:
        motorFunctions.GoForward(R_forward_duty, L_forward_duty, 0.8*forward_dt)
    except KeyboardInterrupt:
        motorFunctions.CleanUp()
    
    print("Goal!!!")


def gps_thread():
    global latitude
    global longitude
    
    while True:
        if gps_thread_flag:
                latitude_list, longitude_list = gpsFunctions.getGps()
                latitude = np.average(rmnoise(latitude_list,2))
                longitude = np.average(rmnoise(longitude_list,2))
        time.sleep(0.3)
        
def nineaxis_thread():
    global dis_x_list
    global dis_y_list
    
    while True:
        if nineaxis_thread_flag:
            x_mag_list, y_mag_list = getMag(0, 0)
            x_mag = rmnoise(x_mag_list,2)
            y_mag = rmnoise(y_mag_list,2)
            dis_x_list.extend(x_mag)
            dis_y_list.extend(y_mag)
        time.sleep(0.1)
    
def argparse():
    parser = ArgumentParser()
    parser.add_argument("--field", type=str, default="out")
    args = parser.parse_args()
    return args



if __name__ == "__main__":

    args = argparse()
    if args.field == "out":
        th_gps_thread = threading.Thread(target=gps_thread)
        th_gps_thread.setDaemon(True)
        th_gps_thread.start()
        th_nineaxis_thread = threading.Thread(target=nineaxis_thread)
        th_nineaxis_thread.setDaemon(True)
        th_nineaxis_thread.start()
        print("====== outdoor ======")
    else:
        print("====== indoor =======")
    
    print("====== landDetectPhase ======")
    landDetectPhase()
    print("====== openDetectPhase ======")
    openDetectPhase()
    if failed_counter < 3:
        print("======= guidePhase1 =======")
        guidePhase1()
        print("======= guidePhase2 =======")
        guidePhase2()
