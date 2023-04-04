import math

def callback(data):
    # 라이다에서 측정한 거리, 각도 정보를 가져옴
    distances = data.ranges
    angles = [math.radians(i) for i in range(len(distances))]

    # 거리, 각도 정보에서 가장 가까운 객체의 거리 및 각도 계산
    closest_distance = min(d for d in distances if d > 0.0)
    closest_angle = angles[distances.index(closest_distance)]

    # 객체의 좌표 계산
    x = closest_distance * math.cos(closest_angle)
    y = closest_distance * math.sin(closest_angle)
    print("Object coordinates: ({}, {})".format(x, y))

'''
레퍼런스 
http://wiki.ros.org/laser_geometry#Coordinate_Calculation
https://www.jbmpa.com/python_tip/10
좌표 출력'''