import time
import csv
import numpy as np
import sys
sys.path.append('../')
from functions import gpsFunctions
from functions.rmNoiseFunction import rmnoise

try:
    print("===== gps data update =====")
    while True:
        latitude_list, longitude_list = gpsFunctions.getGps()
        latitude = np.average(rmnoise(latitude_list,2))
        longitude = np.average(rmnoise(longitude_list,2))
        print(f"latitude={latitude},longitude={longitude}")
        time.sleep(0.3)


except KeyboardInterrupt:
    print("===== save goal poit =====")
    # 10個のデータの平均値をゴール座標とする．
    sum_latitude = 0
    sum_longitude = 0

    for i in range(10):
        latitude_list, longitude_list = gpsFunctions.getGps()
        latitude = np.average(rmnoise(latitude_list,2))
        longitude = np.average(rmnoise(longitude_list,2))
        print(f"{i+1}:latitude={latitude},longitude={longitude}")
        sum_latitude += latitude
        sum_longitude += longitude
        time.sleep(0.3)
    
    goal_latitude = sum_latitude/10
    goal_longitude = sum_longitude/10

    with open('goal/goal.csv',mode='w',newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["goal_latitude", "goal_longitude"])

    with open('goal/goal.csv',mode='a',newline='') as f:
            writer = csv.writer(f)
            writer.writerow([goal_latitude, goal_longitude]) 
