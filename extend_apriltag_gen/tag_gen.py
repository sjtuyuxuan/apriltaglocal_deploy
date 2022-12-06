import cv2
import apriltag
import numpy as np
import yaml
fs = open(("config.yaml"),encoding="UTF-8")
datas = yaml.load(fs,Loader=yaml.FullLoader)

for i in datas["GenTags"]:
    tag = cv2.imread(datas["ImagePath"] + datas[i]["File"])
    resize = cv2.resize(tag, (datas[i]["Size"] * datas[i]["Pixpermm"] ,datas[i]["Size"] * datas[i]["Pixpermm"]), interpolation=cv2.INTER_NEAREST)
    canvas = np.zeros((datas[i]["Canvas"][0] * datas[i]["Pixpermm"], datas[i]["Canvas"][1] * datas[i]["Pixpermm"], 3),dtype=np.uint8)
    canvas = ~canvas
    start_x = int(datas[i]["Canvas"][0] * datas[i]["Pixpermm"]/2 - datas[i]["Size"] * datas[i]["Pixpermm"]/2)
    end_x = int(datas[i]["Canvas"][0] * datas[i]["Pixpermm"]/2 + datas[i]["Size"] * datas[i]["Pixpermm"]/2)
    start_y = int(datas[i]["Canvas"][1] * datas[i]["Pixpermm"]/2 - datas[i]["Size"] * datas[i]["Pixpermm"]/2)
    end_y = int(datas[i]["Canvas"][1] * datas[i]["Pixpermm"]/2 + datas[i]["Size"] * datas[i]["Pixpermm"]/2)
    canvas[start_x:end_x, start_y:end_y] = resize
    for dot in datas[i]["Extand_Dot"]:
        center_x = int((datas[i]["Canvas"][0]/2 + dot[0]) * datas[i]["Pixpermm"])
        center_y = int((datas[i]["Canvas"][1]/2 + dot[1]) * datas[i]["Pixpermm"])
        radius = dot[2] * datas[i]["Pixpermm"]
        cv2.circle(canvas, (center_y, center_x), radius, (0, 0, 0), -1)
    cv2.imwrite(datas["GenPath"] + datas[i]["File"], canvas)
    