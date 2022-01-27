#!/usr/bin/env python
# coding: utf-8

# In[25]:


import cv2 #图像处理需要的库 OpenCV
import numpy as np
import math
import copy

#碰撞检测，检验路径上的点是否越界或为障碍物点（与check_path共同检验）
def check_point(point, map_img):
    point = np.mat(point) #先将这些点转换为矩阵
    not_obstacle = True #验证是否为障碍点

    #若该点仍在图像范围内
    if (point[:, 0] < map_img.shape[0] and 
        point[:, 1] < map_img.shape[1] and 
        point[:, 0] >= 0 and 
        point[:, 1] >= 0):
        #若在图像范围内但路径上有点的像素值为0（黑色，即该路径中间碰到了障碍）
        if map_img[point[:, 1], point[:, 0]] == 0:
            not_obstacle = False
    else:#路径上有某一点已经不在图像范围内
        not_obstacle = False

    return not_obstacle

#检验某点周围的邻域范围内是否无障碍点
def not_obstacle_in_area(x, y, d, map_img): 
    #d代表邻域范围，检验的范围为与该点距离为d的4邻域（上下左右）和距离为根号2d的8邻域（4个斜向方向）
    if x < map_img.shape[1] and x >= 0 and y < map_img.shape[0] and y >= 0 and map_img[x,y]!=0     and x-d < map_img.shape[1] and x-d >= 0 and y < map_img.shape[0] and y >= 0 and map_img[x-d,y]!=0     and x < map_img.shape[1] and x >= 0 and y-d < map_img.shape[0] and y-d >= 0 and map_img[x,y-d]!=0     and x-d < map_img.shape[1] and x-d >= 0 and y-d < map_img.shape[0] and y-d >= 0 and map_img[x-d,y-d]!=0     and x+d < map_img.shape[1] and x+d >= 0 and y < map_img.shape[0] and y >= 0 and map_img[x+d,y]!=0     and x < map_img.shape[1] and x >= 0 and y+d < map_img.shape[0] and y+d >= 0 and map_img[x,y+d]!=0     and x+d < map_img.shape[1] and x+d >= 0 and y+d < map_img.shape[0] and y+d >= 0 and map_img[x+d,y+d]!=0     and x+d < map_img.shape[1] and x+d >= 0 and y-d < map_img.shape[0] and y-d >= 0 and map_img[x+d,y-d]!=0     and x-d < map_img.shape[1] and x-d >= 0 and y+d < map_img.shape[0] and y+d >= 0 and map_img[x-d,y+d]!=0:
        return True #若8个邻域点和自身均不为障碍物点，则默认为该点邻域范围内无障碍点
    return False

#碰撞检测，检验路径上的点是否越界或为障碍物点（与check_point共同检验）
def check_path(point_current, point_other, map_img):
    #首先确保连线的两点周围的邻域范围内无障碍点
    if not_obstacle_in_area(point_current[0,1], point_current[0,0], 11, map_img)     and not_obstacle_in_area(point_other[0,1], point_other[0,0], 11, map_img):     
        #取横向、纵向较大值，确保经过的每个像素都被检测到
        step_length = max(abs(point_current[0, 0] - point_other[0, 0]), abs(point_current[0, 1] - point_other[0, 1]))
        path_x = np.linspace(point_current[0, 0], point_other[0, 0], step_length + 1)
        path_y = np.linspace(point_current[0, 1], point_other[0, 1], step_length + 1)
        #检验路径连线上的点是否越界或为障碍点（调用check_point函数）
        for i in range(int(step_length + 1)):
            if check_point([int(math.ceil(path_x[i])), int(math.ceil(path_y[i]))], map_img):
                return True
    return False

#计算两点间欧氏距离（即直线距离）
def EuclidenDistance(point_a, point_b):
    #point_a可以是矩阵形式的点集，该函数将返回一个矩阵，每行对应各点与point_b的直线距离
    distance = np.sqrt(np.sum(np.multiply(point_a - point_b, point_a - point_b), axis=1))
    return distance

#画出RRT树形图和最终路线图   
def result_plot(map_img, rrt_tree, length, width):
    # 首先绘制树形图
    # 设置树形图图相关参数：点的大小，颜色和线的粗细
    point_size = 3
    point_color = (0, 127, 0) 
    thickness = 4
    #将矩阵转化为数组并转为整型，再转化为元组，以供cv2使用   
    vertex = np.around(np.array(rrt_tree)).astype(int)
    vertex_tuple = tuple(map(tuple, vertex)) 
    map_img1 = copy.deepcopy(map_img) #需要进行深拷贝，而不是引用，否则会导致img1和img2一样

    #画出RRT树中所有点的节点并连线
    for point in vertex_tuple:
        cv2.circle(map_img1, point[0 : 2], point_size, point_color, thickness)
        if point[0] != 0:
            cv2.line(map_img1, point[0 : 2], vertex_tuple[point[2]][0 : 2], (255,150,150), 2)
    
    #通过回溯来绘制最优路径
    #并且通过优化连接点数量使得最终路径更加平滑
    #将离目标点最近的点a与其父节点b的父节点c进行连接，再从点c开始继续这个循环
    #三角形法则：(ac < ab + bc)，得到的路径比最初的路径会更短且更平滑
    point_a_index = -1 #用于定位rrt_tree数组的最后一个元素（离目标点最近的点a）
    while point_a_index != 0: #直至遍历到rrt_tree数组的第一个元素（即遍历结束）
        point_b_index = rrt_tree[point_a_index, 2] #point_b_index为a的父节点索引
        point_c_index = rrt_tree[point_b_index, 2] #point_c_index为b的父节点索引
        #连接a与c，(0,0,0)代表连线颜色的RGB(黑色)，3为线段粗细程度
        cv2.line(map_img1,vertex_tuple[point_a_index][0 : 2], vertex_tuple[point_c_index][0 : 2],(0,0,0),3)
        point_a_index = point_c_index #将起始点转移到c，继续该循环

    img1 = cv2.resize(map_img1,(width,length))#将图像尺寸变为初始尺寸
    cv2.imwrite('C:\\Users\\93508\\Desktop\\tree.png', img1)#图像存储路径

    #去掉无关点，绘制用于小车巡线的最终路线图
    map_img2 = copy.deepcopy(map_img) #需要进行深拷贝，而不是引用，否则会导致img1和img2一样
    #相同的步骤绘制路线图
    point_a_index = -1 #用于定位rrt_tree数组的最后一个元素（离目标点最近的点a）
    while point_a_index != 0: #直至遍历到rrt_tree数组的第一个元素（即遍历结束）
        point_b_index = rrt_tree[point_a_index, 2] #point_b_index为a的父节点索引
        point_c_index = rrt_tree[point_b_index, 2] #point_c_index为b的父节点索引
        #连接a与c，(0,0,0)代表连线颜色的RGB(黑色)，3为线段粗细程度
        cv2.line(map_img2,vertex_tuple[point_a_index][0 : 2], vertex_tuple[point_c_index][0 : 2],(0,0,0),3)
        point_a_index = point_c_index #将起始点转移到c，继续该循环

    img2 = cv2.resize(map_img2,(width,length))#将图像尺寸变为初始尺寸
    cv2.imwrite('C:\\Users\\93508\\Desktop\\result.png', img2)#图像存储路径

    #将img1和img2放在同一个窗口下展示
    imgs = np.hstack([img1,img2])
    cv2.imshow("1", imgs)
    cv2.waitKey()#防止图像一闪而过


#RoadMap类：读入图片，将其二值化为网格图，并进行一系列操作
class RoadMap(object):
    def __init__(self, map_img):
        #读取图像尺寸
        self.length = map_img.shape[0]
        self.width = map_img.shape[1]
        
        #为方便后续操作，将图像尺寸的长宽均设置为更小的那个值（图片转为正方形）
        squad = min(self.length, self.width)        
        map_img = cv2.resize(map_img,(squad,squad))
        self.map = map_img

        #s设置图像的起点和终点       
        self.point_start = np.mat([550, 20]) #运动规划的起点        
        self.point_end = np.mat([20,565]) #运动规划的终点
        
    # RRT算法                
    def rrt_planning(self, **param):
        ''' 快速扩展随机树算法（RRT算法）
        Args:
            **param: 关键字参数，用以配置规划参数    
                     step: 搜索步长，默认20。int
                     dis: 判断阈值，默认20。float
                     cnt: 尝试次数。默认20000。int
        Return:
            本函数没有返回值，但会根据计算结果赋值（或定义）以下属性变量：
                self.rrt_tree: 所生成的rrt树。numpy.mat
                    数据含义: [[横坐标, 纵坐标, 父节点索引]]，其中第一个点（起点）为根，最后一个点（终点）为树枝
        Example:
            mr = RoadMap(img)
            mr.rrt_planning(s=25, t=30, l=15000, pic='None')
        '''

        # 关键字参数处理
        if 'step' in param:
            step_size = param['step'] #搜索步长step_size
        if 'dis' in param:
            area = param['dis'] #判断阈值area
        if 'cnt' in param:
            limit_try = param['cnt'] #尝试次数limit_try

        # 地图灰度化
        image_gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)

        # 地图二值化
        # cv2.THRESH_BINARY表示阈值的二值化操作，大于阈值使用maxval表示，小于阈值使用0表示
        # 大于127的像素点置为255（白色），小于127的像素点置为0（黑色）
        ret,img_binary = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY) 
        
        # 初始化 RRT 树:[横坐标，纵坐标，父节点索引]
        rrt_tree = np.hstack((self.point_start, [[0]]))
        # 初始化尝试次数
        num_try = 0
        # 路径规划是否成功
        path_found = False
        
        #开始limit_try次随机尝试
        while num_try <= limit_try:
            #随机生成采样点
            #在每次选择生长方向时，有一定的概率会向着目标点延伸
            #也有一定的概率会随机在地图内选择一个方向延伸一段距离
            #在这里设置两种概率均为0.5
            if np.random.rand() < 0.5:
                #在地图范围内随机采样一个像素
                sample = np.mat(np.random.randint(0, img_binary.shape[0] - 1, (1, 2)))
            else:
                sample = self.point_end

            #计算各点与当前随机采样点的距离
            #找出rrt树中离当前随机采样点最近的点
            mat_distance = EuclidenDistance(rrt_tree[:, 0 : 2], sample)

            #argmin用于找出距离最小点的索引
            index_close = np.argmin(mat_distance, 0)[0, 0] #末尾索引用来取出数值，否则index_close变为矩阵
            point_close = rrt_tree[index_close, 0 : 2]

            #从距离最小点向当前采样点移动step_size距离，并进行碰撞检测
            #计算出移动方向（角度）
            theta_dir = math.atan2(sample[0, 0] - point_close[0, 0], sample[0, 1] - point_close[0, 1])
            #得到移动后的点point_new
            point_new = point_close + step_size * np.mat([math.sin(theta_dir), math.cos(theta_dir)])

            #将坐标化为整数（矩阵转数组，元素int化，再转矩阵）
            point_new = np.around(np.array(point_new)).astype(int)
            point_new = np.mat(point_new)
      
            #若两点间连线失败（有障碍物），则继续下一次循环
            if not check_path(point_close, point_new, img_binary):
                num_try = num_try + 1
                continue

            #若连线成功，则先检验point_new和终点point_end的距离是否小于判断阈值area
            #若在范围内则代表两个点属于同一个点，默认路径规划成功，已到达终点
            if EuclidenDistance(point_new, self.point_end) < area:
                path_found = True
                #将point_new加入到rrt树，设置为新节点
                point_new = np.hstack((point_new, [[index_close]]))
                rrt_tree = np.vstack((rrt_tree, point_new))
                break
          
            #若point_new尚未到达终点的邻域范围内，则计算rrt树中各点与point_new的距离
            mat_distance = EuclidenDistance(rrt_tree[:, 0 : 2], point_new)
            if np.min(mat_distance, 0) < area:
                num_try = num_try + 1 #若存在距离小于area的，则point_new与该点重合，直接继续下次循环
                continue
            #若均大于判断阈值area，则证明point_new未与rrt树中任何一个点重合，可将其添加到rrt树中
            #设置离新点point_new最近的节点为其父节点
            #父节点索引为index_close
            else:
                point_new = np.hstack((point_new, [[index_close]]))
                rrt_tree = np.vstack((rrt_tree, point_new))        
            
        #路径规划成功则开始画图，反之不画图
        if path_found == True:
            print('规划成功！')
            self.rrt_tree = rrt_tree
            result_plot(self.map, self.rrt_tree, self.length, self.width) #绘图
        else:
            print('没有找到解。')

#主函数
if __name__=="__main__": 

    #读取图像
    image_path = "C:\\Users\\93508\\Desktop\\maze.png"
    img = cv2.imread(image_path)
    
    #开始RRT路径规划
    print('开始RRT路径规划...')
    res = RoadMap(img)
    #这里可修改三项参数步长step，距离阈值dis，尝试次数cnt
    res.rrt_planning(step = 10, dis = 10, cnt = 200000)  

