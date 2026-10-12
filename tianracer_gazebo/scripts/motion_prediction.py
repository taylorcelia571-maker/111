"""Bounded constant-body-twist compensation for short feedback transport delays."""
import math


def predict_rear_pose(x,y,yaw,speed,yaw_rate,age,lateral_speed=0.0):
    if not all(math.isfinite(v) for v in (x,y,yaw,speed,yaw_rate,age,lateral_speed)) or not 0<=age<=.08:
        raise ValueError('Invalid bounded pose prediction input')
    end_yaw=yaw+yaw_rate*age
    if abs(yaw_rate)<1e-5:
        x+=(speed*math.cos(yaw)-lateral_speed*math.sin(yaw))*age
        y+=(speed*math.sin(yaw)+lateral_speed*math.cos(yaw))*age
    else:
        x+=(speed*(math.sin(end_yaw)-math.sin(yaw))+lateral_speed*(math.cos(end_yaw)-math.cos(yaw)))/yaw_rate
        y+=(speed*(math.cos(yaw)-math.cos(end_yaw))+lateral_speed*(math.sin(end_yaw)-math.sin(yaw)))/yaw_rate
    return x,y,end_yaw
