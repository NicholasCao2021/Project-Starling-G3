import cv2
from cv2 import circle
from cv2 import sort
import numpy as np
import time
import heapq
from math import pi, sqrt, tan, cos, sin, atan, acos
import matplotlib.pyplot as plt
# import rclpy  # type: ignore
# from rclpy.node import Node
# from sensor_msgs.msg import Image
import cv2
# from cv_bridge import CvBridge
import os

# class ImageProcessor():
#     # 获取完坐标后，进行A*或其他导航，返回waypoints

#     def __init__(self):  # ,node
#         # super().__init__('image_processor')
#         # self.br = CvBridge()
#         self.yellow_threadhold = 124
#         self.red_threadhold = 0
#         self.vertical_angel = 1.72525
#         self.horizon_angel = 2
#         self.al_drone = 0
#         self.camera_angel = 0
#         self.detect_red_num = 0
#         self.r = 0
#         self.center_point_of_area = [0, 0]
#         self.centers_of_arc = []
#         self.landing_point = [0, 0]
#         # self.node = node

#     def start(self):
#         msg = None
#         # set up subscriber for image
#         # state_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10)
#         img = cv2.imread('51.421895  _2.6686701 170.442555377.png')
#         red_and_yellow_and_center_points_3d = self.image_callback(msg, img)
#         return red_and_yellow_and_center_points_3d

#     def image_callback(self, msg, img):
#         # img = self.br.imgmsg_to_cv2(msg)

#         self.al_drone = 20  # subscribe to sw
#         self.camera_angel = pi / 6  # subscribe to sw + shake of attack angel
#         # points = [[0,0],[0,320],[0,640],[240,0],[240,320],[240,640],[480,0],[480,320],[480,640]]
#         # points = [[0,480],[120,480],[240,480],[360,480],[480,480]]

#         # get points of red and yellow zones in meters, also the center points of red zones for further processing and graphing
#         points_r_in_meters, points_y_in_meters, center_red_points_in_meters = self.return_shape(img)

#         # update center point of the area of interest
#         red_centers = [x[:2] for x in center_red_points_in_meters]
#         self.cal_center_circle(red_centers)

#         # update center points of arc for all possible landing points
#         self.get_centers_of_arc(red_centers, self.center_point_of_area)

#         # update the best landding points
#         self.find_best_landing_point(points_y_in_meters)

#         #Astar path planning
#         point_astar_path = self.get_astar_path(center_red_points_in_meters)
#         print(point_astar_path)
#         return points_r_in_meters + points_y_in_meters + center_red_points_in_meters + point_astar_path
#         # return point_astar_path

#     def get_astar_path(self, red_center_points, r=10):
#         img = np.array(([[255] * 480] * 240)).astype('uint8')
#         for p in red_center_points:
#             img = cv2.circle(img, (int(p[1] * 2 + 240), int(p[0] * 2)), r, 0, cv2.FILLED)  # self.red_threadhold
#         # img = cv2.circle(img, (240, 0), 5, 50, cv2.FILLED)  # self.red_threadhold
#         # img = cv2.circle(img, (int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2)), 5, 124, cv2.FILLED)  # self.red_threadhold
#         # img = np.rot90(img, 2)
#         # img = np.flip(img, 1)
#         # cv2.imshow("img_contours", img)
#         # cv2.waitKey(0)
#         # cv2.destroyAllWindows()
#         start_p = [(240, 0), 700, (240, 0)]
#         end_p = [(int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2)), 0, (int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2))]
#         path = self.astar_algrithm(img, start_p, end_p)
#         path.reverse()
#         return path

#     def astar_algrithm(self, maps, start, end):
#         maps_size = np.array(maps)
#         hight = maps_size.shape[0]  # ->y
#         width = maps_size.shape[1]  # ->x

#         # start  = [(60, 50), 700, (60, 50)]  # 'position','cost','parent_node'
#         # end = [(630, 420), 0, (630, 420)]

#         openlist = []
#         closelist = [start]
#         step_size = 8
#         diagonal_step_size = int(step_size / 1.4)

#         add = ([0, step_size], [0, -step_size], [step_size, 0], [-step_size, 0], [diagonal_step_size, diagonal_step_size], [diagonal_step_size, -diagonal_step_size],
#                [-diagonal_step_size, diagonal_step_size], [-diagonal_step_size, -diagonal_step_size])  # 8 directions
#         while 1:
#             s_point = closelist[-1][0]  #获取close列表最后一个点position，S点

#             for i in add:
#                 x = s_point[0] + i[0]
#                 if x < 0 or x >= width:
#                     continue
#                 y = s_point[1] + i[1]
#                 if y < 0 or y >= hight:
#                     continue
#                 G = sqrt((x - start[0][0])**2 + (y - start[0][1])**2)  #计算cost
#                 H = sqrt((x - end[0][0])**2 + (y - end[0][1])**2)  #计算cost
#                 F = G + H

#                 addpoint = [(x, y), F, s_point]  #更新position
#                 count = 0
#                 for i in openlist:
#                     if i[0] == addpoint[0]:
#                         count = 1
#                         break
#                 for i in closelist:
#                     if i[0] == addpoint[0]:
#                         count = 1
#                         break
#                 if count == 0 and maps[y, x] != 0:  #新增点不在open和close列表中 #非障碍物
#                     openlist.append(addpoint)
#                 if H < step_size:  #当逐渐靠近终点时，搜索的步长变小
#                     step_size = 1
#                     add = ([0, step_size], [0, -step_size], [step_size, 0], [-step_size, 0])

#             t_point = [(240, 0), 10000, (240, 0)]
#             for j in range(len(openlist)):
#                 if openlist[j][1] < t_point[1]:
#                     t_point = openlist[j]
#             for j in range(len(openlist)):
#                 if t_point == openlist[j]:
#                     openlist.pop(j)
#                     break
#             closelist.append(t_point)
#             if t_point[0] == end[0]:
#                 print("found the destionation")
#                 break
#         road = []
#         road.append(closelist[-1])
#         point = road[-1]
#         while 1:
#             for i in closelist:
#                 if i[0] == point[2]:
#                     point = i
#                     road.append(point)
#             if point == start:
#                 print("A-star finished")
#                 break
#         return [(rd[0][1] / 2, (rd[0][0] - 240) / 2, 2) for rd in road]

#     def print_path(self, maps, road):
#         informap = np.array(maps)
#         for i in road:
#             cv2.circle(informap, i[0], 1, 200, cv2.FILLED)
#         cv2.circle(informap, road[0], 5, 100, cv2.FILLED)
#         cv2.circle(informap, road[-1], 5, 100, cv2.FILLED)
#         informap = np.rot90(informap, 2)
#         informap = np.flip(informap, 1)
#         cv2.imwrite("informap.png", informap)

#     def find_best_landing_point(self, points_y):
#         re1 = []
#         re2 = []
#         for c in self.centers_of_arc:
#             c_dmin = 100000000
#             for p in points_y:
#                 temp = (c[0] - p[0])**2 + (c[1] - p[1])**2
#                 if (c_dmin > temp):
#                     c_dmin = temp
#             if (c_dmin > 300):
#                 dis = c[0]**2 + c[1]**2
#                 re1.append(c)
#                 re2.append(dis)
#         best = map(re2.index, heapq.nsmallest(1, re2))
#         self.landing_point = re1[list(best)[0]]

#     def return_shape(self, img):
#         points_r = []
#         points_y = []
#         center_red_points = []
#         threadthold_bgr = [100, 150, 120]
#         img_b = img[:, :, 0] < threadthold_bgr[0]
#         img_g = img[:, :, 1] > threadthold_bgr[1]
#         img_r = img[:, :, 2] > threadthold_bgr[2]
#         map_color_r = (img_r & img_b) ^ (img_g & img_b)
#         map_color_y = img_r & img_g & img_b
#         map_color_r = np.array(map_color_r, dtype='uint8') * 255
#         map_color_y = np.array(map_color_y, dtype='uint8') * 255
#         contours_r = cv2.findContours(map_color_r, 1, 1)[0]
#         contours_y = cv2.findContours(map_color_y, 1, 1)[0]
#         for j in contours_y:
#             area = cv2.contourArea(j)
#             if (area > 10):
#                 # img_contours = cv2.drawContours(img_contours, [j], -1, (0, 124, 255), cv2.FILLED)  # self.yellow_threadhold
#                 if len(points_y) == 0:
#                     points_y = j.sum(axis=1)
#                 else:
#                     points_y = np.vstack((points_y, j.sum(axis=1)))
#         for i in contours_r:
#             area = cv2.contourArea(i)
#             if (area > 10):
#                 # img_contours = cv2.drawContours(img_contours, [i], -1, (0, 0, 255), cv2.FILLED)  # self.red_threadhold
#                 if len(points_r) == 0:
#                     points_r = i.sum(axis=1)
#                     center_red_points = self.cal_center_points(i)
#                 else:
#                     points_r = np.vstack((points_r, i.sum(axis=1)))
#                     center_red_points = np.vstack((center_red_points, self.cal_center_points(i)))
#                 self.detect_red_num += 1
#         points_r_after = [self.cal_point(np.array(points), 2) for points in points_r]
#         points_y_after = [self.cal_point(np.array(points), 1) for points in points_y]
#         points_center_red_points_after = [self.cal_point(np.array(points), 0) for points in center_red_points]
#         # cv2.imshow("img_contours", img_contours)
#         # cv2.waitKey(0)
#         # cv2.destroyAllWindows()
#         # cv2.imwrite("red_contours.png", img_contours)

#         # np.savetxt('new.csv',np.array(map1)*255,delimiter = ',',fmt='%d')
#         return points_r_after, points_y_after, points_center_red_points_after

#     def cal_center_circle(self, points):
#         a = 2 * (points[1][0] - points[0][0])
#         b = 2 * (points[1][1] - points[0][1])
#         c = points[1][0]**2 + points[1][1]**2 - points[0][0]**2 - points[0][1]**2
#         d = 2 * (points[2][0] - points[1][0])
#         e = 2 * (points[2][1] - points[1][1])
#         f = points[2][0]**2 + points[2][1]**2 - points[1][0]**2 - points[1][1]**2
#         x = (b * f - e * c) / (b * d - e * a)
#         y = (d * c - a * f) / (b * d - e * a)
#         self.r = sqrt((x - points[0][0]) * (x - points[0][0]) + (y - points[0][1]) * (y - points[0][1]))  # radius
#         self.center_point_of_area = [x, y]

#     def cal_center_points(self, points):
#         red_p = [0, 0]
#         for p in points:
#             red_p += p
#         return red_p / len(points)

#     def cal_point(self, point, color):
#         pixel_y, pixel_x = point
#         camera_angel = self.camera_angel
#         vertical_angel = self.vertical_angel
#         al_drone = self.al_drone
#         cmv = camera_angel - vertical_angel / 2

#         point_center = (tan(camera_angel) * al_drone, 0, 0)
#         point_e = (tan(cmv) * al_drone, 0, 0)
#         dis_eh = cos(camera_angel) * (point_center[0] - point_e[0])  # cos*dis_ef
#         width_half = dis_eh * 4 / 3

#         # calculate point_x_world
#         pixel_x_portion = (240 - pixel_x) / 240
#         theta_x = atan(pixel_x_portion * tan(vertical_angel / 2))
#         point_x_world = tan(theta_x + camera_angel) * al_drone

#         # calculate point_y_world
#         pixel_y_portion = (480 - pixel_x) / 240
#         z_y = pixel_y_portion * sin(camera_angel) * dis_eh
#         point_y_max_world = al_drone / (al_drone - z_y) * width_half
#         point_y_world_portion = (320 - pixel_y) / 320
#         point_y_world = point_y_max_world * point_y_world_portion

#         # #four corners world positions
#         # p2 = []
#         # cpv = camera_angel + vertical_angel/2
#         # z_i = 2*sin(camera_angel)*dis_eh
#         # ex_width_half = al_drone/(al_drone-z_i)*width_half
#         # point_a = (tan(cmv)*al_drone,  width_half, 0)
#         # point_b = (tan(cmv)*al_drone,  -width_half, 0)
#         # point_c = (tan(cpv)*al_drone,  -ex_width_half, 0)
#         # point_d = (tan(cpv)*al_drone,  ex_width_half, 0)
#         # p2.append(point_center)
#         # p2.append(point_a)
#         # p2.append(point_b)
#         # p2.append(point_c)
#         # p2.append(point_d)
#         # p2 = np.array(p2, dtype='double')
#         # print(np.array(p2))
#         return [point_x_world, -point_y_world, color]

#     def cal_arc_centers(self, p1, p2, center_point_of_area):
#         x0, y0 = center_point_of_area
#         x1, y1 = p1
#         x2, y2 = p2
#         theta = acos((x1 * x2 - x1 * x0 - x2 * x0 + x0**2 + y1 * y2 - y0 * y2 - y1 * y0 + y0**2) / self.r**2) / 2
#         x_1 = x0 + (x1 - x0) * cos(theta) - (y1 - y0) * sin(theta)
#         y_1 = y0 + (x1 - x0) * sin(theta) + (y1 - y0) * cos(theta)
#         return [x_1, y_1]

#     def get_centers_of_arc(self, red_points, center_point_of_area):
#         self.centers_of_arc = []
#         ps1 = [x for x in red_points if x[0] - center_point_of_area[0] > 0]
#         ps1.sort(key=lambda x: x[1])
#         ps2 = [x for x in red_points if x[0] - center_point_of_area[0] < 0]
#         ps2.sort(key=lambda x: x[1], reverse=True)
#         ps = ps1 + ps2
#         for i in range(len(ps)):
#             if i == len(ps) - 1:
#                 p = self.cal_arc_centers(ps[-1], ps[0], center_point_of_area)
#                 self.centers_of_arc.append(p)
#                 break
#             else:
#                 p = self.cal_arc_centers(ps[i], ps[i + 1], center_point_of_area)
#                 self.centers_of_arc.append(p)

#     def show_3d_map(self, position_xyc):
#         position_xy = np.array([x[0:2] for x in position_xyc])
#         position_c = np.array([x[2] for x in position_xyc])
#         c_x = np.linspace(self.center_point_of_area[1] - self.r, self.center_point_of_area[1] + self.r, 1000)
#         c_y1 = np.sqrt(self.r**2 - (c_x - self.center_point_of_area[1])**2) + self.center_point_of_area[0]
#         c_y2 = -np.sqrt(self.r**2 - (c_x - self.center_point_of_area[1])**2) + self.center_point_of_area[0]
#         map_color = {0: 'black', 1: 'y', 2: 'r'}
#         color = list(map(lambda x: map_color[x], position_c))
#         plt.figure()
#         plt.axes().set_aspect('equal', 'datalim')
#         plt.scatter(position_xy[:, 1], position_xy[:, 0], s=10, c=color)
#         plt.scatter(np.array(self.centers_of_arc)[:, 1], np.array(self.centers_of_arc)[:, 0], s=10, c='g')
#         plt.scatter(self.center_point_of_area[1], self.center_point_of_area[0], s=10, c='g')
#         plt.scatter(self.landing_point[1], self.landing_point[0], s=40, c='g')
#         plt.scatter(0, 0, s=30, c="navy", marker="o")
#         plt.plot(c_x, c_y1, linewidth=20, alpha=0.2, c='blue')
#         plt.plot(c_x, c_y2, linewidth=20, alpha=0.2, c='blue')
#         plt.annotate("Drone postion", xy=(0, 0), xytext=(-20, 0.1))
#         plt.xlabel("Horizontal axis (y) of drone in meters")
#         plt.ylabel("Vertical axis (x) of drone in meters")
#         plt.title("2D image to 3D position (Altitude = 0)")
#         plt.show()





class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        self.br = CvBridge()
        self.image=None
        self.ready = False
        self.yellow_threadhold = 124
        self.red_threadhold = 0
        self.vertical_angel = 1.72525
        self.horizon_angel = 2
        self.al_drone = 0
        self.camera_angel = 0
        self.positions_of_points = []
        self.detect_red_num = 0
        self.r = 0
        self.center_point_of_area = [0, 0]
        self.centers_of_arc = []
        self.landing_point = [0, 0]
        self.get_logger().info('ImageProcessor created')
        

    def start(self):
        self.ready=True
        # set up subscriber for image
        
        # self.destroy_subscription(red_and_yellow_and_center_points_3d)
        
        # img = cv2.imread('51.4219364  _2.6695396 170.442555377.png')
        # red_and_yellow_and_center_points_3d = self.image_callback(msg,img)
        # self.return_shape()
        # return red_and_yellow_and_center_points_3d

    def image_callback(self, msg):
        if self.ready:    
            img = self.br.imgmsg_to_cv2(msg)
            red_and_yellow_and_center_points_3d = []
            self.al_drone = 20  # subscribe to sw
            self.camera_angel = pi/6  # subscribe to sw + shake of attack angel

            # get points of red and yellow zones in meters, also the center points of red zones for further processing and graphing
            points_r_in_meters, points_y_in_meters, center_red_points_in_meters = self.return_shape(img)

            # update center point of the area of interest
            red_centers = [x[:2] for x in center_red_points_in_meters]
            self.cal_center_circle(red_centers)

            # update center points of arc for all possible landing points
            self.get_centers_of_arc(red_centers, self.center_point_of_area)

            # update the best landding points
            self.find_best_landing_point(points_y_in_meters)

            #Astar path planning
            point_astar_path = self.get_astar_path(center_red_points_in_meters)
            self.ready=False
            self.get_logger().info('red zones detected: ',self.detect_red_num)
            self.get_logger().info('point_astar_path: ',point_astar_path)
            # return points_r_in_meters + points_y_in_meters + center_red_points_in_meters + point_astar_path
            # return point_astar_path

    def get_astar_path(self, red_center_points, r=10):
        img = np.array(([[255] * 480] * 240)).astype('uint8')
        for p in red_center_points:
            img = cv2.circle(img, (int(p[1] * 2 + 240), int(p[0] * 2)), r, 0, cv2.FILLED)  # self.red_threadhold
        # img = cv2.circle(img, (240, 0), 5, 50, cv2.FILLED)  # self.red_threadhold
        # img = cv2.circle(img, (int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2)), 5, 124, cv2.FILLED)  # self.red_threadhold
        # img = np.rot90(img, 2)
        # img = np.flip(img, 1)
        # cv2.imshow("img_contours", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        start_p = [(240, 0), 700, (240, 0)]
        end_p = [(int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2)), 0, (int(self.landing_point[1] * 2 + 240), int(self.landing_point[0] * 2))]
        path = self.astar_algrithm(img, start_p, end_p)
        path.reverse()
        return path

    def astar_algrithm(self, maps, start, end):
        maps_size = np.array(maps)
        hight = maps_size.shape[0]  # ->y
        width = maps_size.shape[1]  # ->x

        # start  = [(60, 50), 700, (60, 50)]  # 'position','cost','parent_node'
        # end = [(630, 420), 0, (630, 420)]

        openlist = []
        closelist = [start]
        step_size = 8
        diagonal_step_size = int(step_size / 1.4)

        add = ([0, step_size], [0, -step_size], [step_size, 0], [-step_size, 0], [diagonal_step_size, diagonal_step_size], [diagonal_step_size, -diagonal_step_size],
            [-diagonal_step_size, diagonal_step_size], [-diagonal_step_size, -diagonal_step_size])  # 8 directions
        while 1:
            s_point = closelist[-1][0]  #获取close列表最后一个点position，S点

            for i in add:
                x = s_point[0] + i[0]
                if x < 0 or x >= width:
                    continue
                y = s_point[1] + i[1]
                if y < 0 or y >= hight:
                    continue
                G = sqrt((x - start[0][0])**2 + (y - start[0][1])**2)  #计算cost
                H = sqrt((x - end[0][0])**2 + (y - end[0][1])**2)  #计算cost
                F = G + H

                addpoint = [(x, y), F, s_point]  #更新position
                count = 0
                for i in openlist:
                    if i[0] == addpoint[0]:
                        count = 1
                        break
                for i in closelist:
                    if i[0] == addpoint[0]:
                        count = 1
                        break
                if count == 0 and maps[y, x] != 0:  #新增点不在open和close列表中 #非障碍物
                    openlist.append(addpoint)
                if H < step_size:  #当逐渐靠近终点时，搜索的步长变小
                    step_size = 1
                    add = ([0, step_size], [0, -step_size], [step_size, 0], [-step_size, 0])

            t_point = [(240, 0), 10000, (240, 0)]
            for j in range(len(openlist)):
                if openlist[j][1] < t_point[1]:
                    t_point = openlist[j]
            for j in range(len(openlist)):
                if t_point == openlist[j]:
                    openlist.pop(j)
                    break
            closelist.append(t_point)
            if t_point[0] == end[0]:
                print("found the destionation")
                break
        road = []
        road.append(closelist[-1])
        point = road[-1]
        while 1:
            for i in closelist:
                if i[0] == point[2]:
                    point = i
                    road.append(point)
            if point == start:
                print("A-star finished")
                break
        return [(rd[0][1] / 2, (rd[0][0] - 240) / 2, 2) for rd in road]

    def print_path(self, maps, road):
        informap = np.array(maps)
        for i in road:
            cv2.circle(informap, i[0], 1, 200, cv2.FILLED)
        cv2.circle(informap, road[0], 5, 100, cv2.FILLED)
        cv2.circle(informap, road[-1], 5, 100, cv2.FILLED)
        informap = np.rot90(informap, 2)
        informap = np.flip(informap, 1)
        cv2.imwrite("informap.png", informap)

    def find_best_landing_point(self, points_y):
        re1 = []
        re2 = []
        for c in self.centers_of_arc:
            c_dmin = 100000000
            for p in points_y:
                temp = (c[0] - p[0])**2 + (c[1] - p[1])**2
                if (c_dmin > temp):
                    c_dmin = temp
            if (c_dmin > 300):
                dis = c[0]**2 + c[1]**2
                re1.append(c)
                re2.append(dis)
        best = map(re2.index, heapq.nsmallest(1, re2))
        self.landing_point = re1[list(best)[0]]

    def return_shape(self, img):
        points_r = []
        points_y = []
        center_red_points = []
        threadthold_bgr = [100, 150, 120]
        img_b = img[:, :, 0] < threadthold_bgr[0]
        img_g = img[:, :, 1] > threadthold_bgr[1]
        img_r = img[:, :, 2] > threadthold_bgr[2]
        map_color_r = (img_r & img_b) ^ (img_g & img_b)
        map_color_y = img_r & img_g & img_b
        map_color_r = np.array(map_color_r, dtype='uint8') * 255
        map_color_y = np.array(map_color_y, dtype='uint8') * 255
        contours_r = cv2.findContours(map_color_r, 1, 1)[0]
        contours_y = cv2.findContours(map_color_y, 1, 1)[0]
        for j in contours_y:
            area = cv2.contourArea(j)
            if (area > 10):
                # img_contours = cv2.drawContours(img_contours, [j], -1, (0, 124, 255), cv2.FILLED)  # self.yellow_threadhold
                if len(points_y) == 0:
                    points_y = j.sum(axis=1)
                else:
                    points_y = np.vstack((points_y, j.sum(axis=1)))
        for i in contours_r:
            area = cv2.contourArea(i)
            if (area > 10):
                # img_contours = cv2.drawContours(img_contours, [i], -1, (0, 0, 255), cv2.FILLED)  # self.red_threadhold
                if len(points_r) == 0:
                    points_r = i.sum(axis=1)
                    center_red_points = self.cal_center_points(i)
                else:
                    points_r = np.vstack((points_r, i.sum(axis=1)))
                    center_red_points = np.vstack((center_red_points, self.cal_center_points(i)))
                self.detect_red_num += 1
        points_r_after = [self.cal_point(np.array(points), 2) for points in points_r]
        points_y_after = [self.cal_point(np.array(points), 1) for points in points_y]
        points_center_red_points_after = [self.cal_point(np.array(points), 0) for points in center_red_points]
        # cv2.imshow("img_contours", img_contours)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # cv2.imwrite("red_contours.png", img_contours)

        # np.savetxt('new.csv',np.array(map1)*255,delimiter = ',',fmt='%d')
        return points_r_after, points_y_after, points_center_red_points_after

    def cal_center_circle(self, points):
        a = 2 * (points[1][0] - points[0][0])
        b = 2 * (points[1][1] - points[0][1])
        c = points[1][0]**2 + points[1][1]**2 - points[0][0]**2 - points[0][1]**2
        d = 2 * (points[2][0] - points[1][0])
        e = 2 * (points[2][1] - points[1][1])
        f = points[2][0]**2 + points[2][1]**2 - points[1][0]**2 - points[1][1]**2
        x = (b * f - e * c) / (b * d - e * a)
        y = (d * c - a * f) / (b * d - e * a)
        self.r = sqrt((x - points[0][0]) * (x - points[0][0]) + (y - points[0][1]) * (y - points[0][1]))  # radius
        self.center_point_of_area = [x, y]

    def cal_center_points(self, points):
        red_p = [0, 0]
        for p in points:
            red_p += p
        return red_p / len(points)

    def cal_point(self, point, color):
        pixel_y, pixel_x = point
        camera_angel = self.camera_angel
        vertical_angel = self.vertical_angel
        al_drone = self.al_drone
        cmv = camera_angel - vertical_angel / 2

        point_center = (tan(camera_angel) * al_drone, 0, 0)
        point_e = (tan(cmv) * al_drone, 0, 0)
        dis_eh = cos(camera_angel) * (point_center[0] - point_e[0])  # cos*dis_ef
        width_half = dis_eh * 4 / 3

        # calculate point_x_world
        pixel_x_portion = (240 - pixel_x) / 240
        theta_x = atan(pixel_x_portion * tan(vertical_angel / 2))
        point_x_world = tan(theta_x + camera_angel) * al_drone

        # calculate point_y_world
        pixel_y_portion = (480 - pixel_x) / 240
        z_y = pixel_y_portion * sin(camera_angel) * dis_eh
        point_y_max_world = al_drone / (al_drone - z_y) * width_half
        point_y_world_portion = (320 - pixel_y) / 320
        point_y_world = point_y_max_world * point_y_world_portion

        # #four corners world positions
        # p2 = []
        # cpv = camera_angel + vertical_angel/2
        # z_i = 2*sin(camera_angel)*dis_eh
        # ex_width_half = al_drone/(al_drone-z_i)*width_half
        # point_a = (tan(cmv)*al_drone,  width_half, 0)
        # point_b = (tan(cmv)*al_drone,  -width_half, 0)
        # point_c = (tan(cpv)*al_drone,  -ex_width_half, 0)
        # point_d = (tan(cpv)*al_drone,  ex_width_half, 0)
        # p2.append(point_center)
        # p2.append(point_a)
        # p2.append(point_b)
        # p2.append(point_c)
        # p2.append(point_d)
        # p2 = np.array(p2, dtype='double')
        # print(np.array(p2))
        return [point_x_world, -point_y_world, color]

    def cal_arc_centers(self, p1, p2, center_point_of_area):
        x0, y0 = center_point_of_area
        x1, y1 = p1
        x2, y2 = p2
        theta = acos((x1 * x2 - x1 * x0 - x2 * x0 + x0**2 + y1 * y2 - y0 * y2 - y1 * y0 + y0**2) / self.r**2) / 2
        x_1 = x0 + (x1 - x0) * cos(theta) - (y1 - y0) * sin(theta)
        y_1 = y0 + (x1 - x0) * sin(theta) + (y1 - y0) * cos(theta)
        return [x_1, y_1]

    def get_centers_of_arc(self, red_points, center_point_of_area):
        self.centers_of_arc = []
        ps1 = [x for x in red_points if x[0] - center_point_of_area[0] > 0]
        ps1.sort(key=lambda x: x[1])
        ps2 = [x for x in red_points if x[0] - center_point_of_area[0] < 0]
        ps2.sort(key=lambda x: x[1], reverse=True)
        ps = ps1 + ps2
        for i in range(len(ps)):
            if i == len(ps) - 1:
                p = self.cal_arc_centers(ps[-1], ps[0], center_point_of_area)
                self.centers_of_arc.append(p)
                break
            else:
                p = self.cal_arc_centers(ps[i], ps[i + 1], center_point_of_area)
                self.centers_of_arc.append(p)








class Timer:

    def __init__(self, func):
        self.func = func

    def __call__(self):
        start = time.time()
        self.func()
        print(f"Operation time:{time.time() - start}")


@Timer
def main(args=None):
    image_node = ImageProcessor()
    for _ in range(1):
        image_node.__init__()
        # image_node.start()
        position_xyc = image_node.start()
        # print(position_xyc)
        image_node.show_3d_map(position_xyc)


if __name__ == '__main__':
    main()

    # 红点横坐标x基于tan(drone角加减1弧度)乘以高度得到
    # 红点纵坐标y和求abcd坐标的y一样，按比例得到每个x的y轴长度，再按比例进行向x轴放缩
