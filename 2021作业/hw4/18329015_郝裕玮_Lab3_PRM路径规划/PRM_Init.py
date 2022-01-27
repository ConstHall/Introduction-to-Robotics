#!/usr/bin/env python
# coding: utf-8

# In[26]:


import math
from PIL import Image #用于对图像进行一系列处理
import numpy as np
import networkx as nx #对无向图进行相关处理 
import copy

STAT_OBSTACLE='#' #障碍点用 # 表示
STAT_NORMAL='.' #普通点用 . 表示

#RoadMap类：读入图片，将其二值化为有障碍物的二维网格化地图（即整张图只有 # 和 . ），并进行一系列操作
class RoadMap():
    
    #图像初始化处理
    def __init__(self,img_file): 
        temp_map = [] #临时变量存储结果
        img = Image.open(img_file) #读取图片
        img_gray = img.convert('L') #将地图转换为灰度图像，每个像素用8个bit表示，0表示黑，255表示白，其他数字表示不同的灰度。
        img_arr = np.array(img_gray) #将该图像转换为数组形式存储
        img_binary = np.where(img_arr<127,0,255) #若像素值小于127则将其设置为黑色(0)，反之设置为白色(0)
        
        #遍历地图将各像素点修改为'.'或者'#'
        for x in range(img_binary.shape[0]): #shape[0]代表该数组的行数
            temp_row = [] #每轮内部循环前先将temp_row清空
            for y in range(img_binary.shape[1]): #shape[1]代表该数组的列数
                if img_binary[x,y] == 0: 
                    status = STAT_OBSTACLE #若当前像素点为黑色则设置其为障碍点'#'
                else:
                    status = STAT_NORMAL #若为白色则设置其为普通点'.'
                temp_row.append(status) #将内部循环结果加入temp_row数组
            temp_map.append(temp_row) #内部循环结束后将当前temp_row的结果加入temp_map
 
        #给成员变量赋值
        self.map = temp_map 
        self.rows = img_binary.shape[0]
        self.cols = img_binary.shape[1]
    
    
    #判断当前点是否在地图范围内
    def is_valid_xy(self,x,y): 
        if x < 0 or x >= self.rows or y < 0 or y >= self.cols: #防止随机点越出地图范围
            return False
        return True
    
    
    #判断当前点是否为障碍点#
    def not_obstacle(self,x,y): 
        if self.map[x][y] != STAT_OBSTACLE:
            return True
        return False
    
    
    #计算两点间欧氏距离（即直线距离）
    def EuclidenDistance(self, xy1, xy2): 
        temp = (xy1[0]-xy2[0])**2 + (xy1[1]-xy2[1])**2
        dis = pow(temp,0.5)
        return dis
    
    
    #计算两点间曼哈顿距离
    def ManhattanDistance(self,xy1,xy2): 
        dis = abs(xy1[0]-xy2[0]) + abs(xy1[1]-xy2[1])
        return dis
    
    
    #碰撞检测，检查两点之间连线是否经过障碍物
    def check_path(self, xy1, xy2): 
        steps = max(abs(xy1[0]-xy2[0]), abs(xy1[1]-xy2[1])) # 取横向、纵向较大值，确保经过的每个像素都被检测到
        
        #将以两点之间连线为对角线的矩阵的长|xy1[0]-xy2[0]|和宽|xy1[1]-xy2[1]|进行均分
        x_matrix = np.linspace(xy1[0],xy2[0],steps+1) 
        y_matrix = np.linspace(xy1[1],xy2[1],steps+1)
        
        # 第一个节点(x_matrix[0],y_matrix[0])和最后一个节点(x_matrix[steps+1],y_matrix[steps+1])分别是 xy1，xy2，不需要检查
        for i in range(1, steps): 
            if not self.not_obstacle(math.ceil(x_matrix[i]), math.ceil(y_matrix[i])): #若连线中出现障碍物点，则立刻终止循环返回False
                return False
        return True
    
    
    #画出路线图
    def plot(self,path): 
        out = []
        
        #开始遍历整个地图
        for x in range(self.rows):
            temp = [] #每轮内部循环前先将temp清空
            for y in range(self.cols):
                if self.map[x][y]==STAT_OBSTACLE: #若为障碍物点则设置该点为黑色(0)
                    temp.append(0)
                elif self.map[x][y]==STAT_NORMAL: #若为普通点则设置该点为白色(255)
                    temp.append(255)
            out.append(temp) #内部循环结束后将当前temp的结果加入out
            
        for x,y in path:
            out[x][y] = 0 #将path数组中的所有位置点的像素值设置为127（灰色）
            
        #先将其转为数组再转为image格式
        out = np.array(out)
        img = Image.fromarray(np.uint8(out)) #不将数组转换为uint8格式会导致图片保存时全黑，无法保存原图（尽管调用show时没有问题）
        img.show() #展示结果
        img.save("345.png") #保存结果
        
        


#PRM类：使用PRM算法计算出最优路径、
#PRM类继承RoadMap类，所以可以使用RoadMap类中的实例方法
class PRM(RoadMap): 
    
    #初始化无向图并初始化相关数据
    def __init__(self, img_file, **param): 
        # 随机路线图算法(Probabilistic Roadmap, PRM)
        # **param: 参数以字典形式传入
        # num_sample: 采样点个数
        # distance_neighbor: 邻域距离
        
        RoadMap.__init__(self,img_file) #先继承父类，再构造子类，否则就是重构了（这样的话子类就无法继承父类）
        
        self.num_sample = param['num_sample']
        self.distance_neighbor = param['distance_neighbor']
        self.G = nx.Graph() # 构造无向图，保存构型空间的完整连接属性
    
    
    #学习阶段，构造无碰撞的无向图
    def learn(self): 
        # 随机“撒点”
        while len(self.G.nodes) < self.num_sample: #在地图上洒满num_sample个样点
            XY = (np.random.randint(0, self.rows),np.random.randint(0, self.cols)) # 在地图上随机取点
            #以下if的判定内容为：
            #（1）确保判定点仍在地图内（因为涉及到加减，可能使源位置超出地图范围）
            #（2）确保样点周围一定距离内没有障碍物点（这样可保证最终路线不会太靠近障碍物，避免小车行进时撞倒迷宫墙壁）
            #（3）这里设置（2）中的一定距离为20
            if self.is_valid_xy(XY[0],XY[1])             and self.is_valid_xy(XY[0]+20,XY[1])             and self.is_valid_xy(XY[0],XY[1]+20)             and self.is_valid_xy(XY[0]+20,XY[1]+20)             and self.is_valid_xy(XY[0]-20,XY[1])             and self.is_valid_xy(XY[0],XY[1]-20)             and self.is_valid_xy(XY[0]-20,XY[1]-20)             and self.not_obstacle(XY[0],XY[1])             and self.not_obstacle(XY[0]+20,XY[1])             and self.not_obstacle(XY[0],XY[1]+20)             and self.not_obstacle(XY[0]+20,XY[1]+20)             and self.not_obstacle(XY[0]-20,XY[1])             and self.not_obstacle(XY[0],XY[1]-20)             and self.not_obstacle(XY[0]-20,XY[1]-20):
                self.G.add_node(XY) #满足条件则将该点加入无向图节点
                
        # 检测邻域范围内的连线是否与障碍物发生碰撞，若无碰撞，则将该连线加入到无向图中。
        # 遍历所有节点
        for node1 in self.G.nodes:
            for node2 in self.G.nodes:
                if node1 == node2:
                    continue
                dis = self.EuclidenDistance(node1,node2) #计算两点间欧氏距离（直线距离）
                
                #若两节点间距离在邻域范围内且连线间无障碍物点，则将该连线加入无向图
                if dis < self.distance_neighbor and self.check_path(node1,node2):
                    self.G.add_edge(node1,node2,weight=dis) #边的权重为欧氏距离（直线距离）
    
    
    #利用learn中得到的无碰撞无向图进行寻路
    def find_path(self): 

        # 寻路时再将起点和终点添加进图中，以便一次学习多次使用 
        temp_G = copy.deepcopy(self.G) #对无向图进行深拷贝
        start = (30,550) #迷宫起点坐标
        end = (750, 10)  #迷宫终点坐标
        #将起点和终点加入无向图节点
        temp_G.add_node(start) 
        temp_G.add_node(end)
        
        # 将起点和终点与之前无向图中已存在的节点进行连线，并将满足同样条件的连线加入无向图
        for node1 in [start, end]: 
            for node2 in temp_G.nodes:
                dis = self.EuclidenDistance(node1,node2)
                if dis < self.distance_neighbor and self.check_path(node1,node2):
                    #若两节点间距离在邻域范围内且连线间无障碍物点，则将该连线加入无向图
                    temp_G.add_edge(node1,node2,weight=dis) #、#边的权重为欧氏距离（直线距离）
                    
        # 直接调用networkx中求最短路径的方法shortest_path来求出无向图中起点start和终点end间的最短路径
        path = nx.shortest_path(temp_G, source=start, target=end)
        
        return self.construct_path(path)
    
    
    #将最短路径上的各节点之间的连线所经过的像素点全部加入结果数组，便于绘图
    def construct_path(self, path): 
        out = []
        print('PRM最优路径长度为:',int(path_length(path))) #输出路径长度
        for i in range(len(path)-1):
            x1 = path[i][0]
            y1 = path[i][1]
            x2 = path[i+1][0]
            y2 = path[i+1][1]
            steps = max( abs(x1-x2), abs(y1-y2) ) #取横向、纵向较大值，确保经过的每个像素都被检测到
            #将以两点之间连线为对角线的矩阵的长|x1-x2|和宽|y1-y2|进行均分
            x_matrix = np.linspace(x1,x2,steps+1)
            y_matrix = np.linspace(y1,y2,steps+1)
            for i in range(0, steps+1): #将路径上等分的节点也加入结果数组
                for j in range (-2,2):
                    for k in range (-2,2):
                        out.append((math.ceil(x_matrix[i]+j), math.ceil(y_matrix[i]+k)))
                
        return out #返回最后的路径结果数组
    
    
#独立于两个类以外的函数
def path_length(path): #计算最短路径长度
    dis = 0
    for i in range(len(path)-1):
        x1 = path[i][0]
        y1 = path[i][1]
        x2 = path[i+1][0]
        y2 = path[i+1][1]

        dis+= pow( (x1-x2)**2+(y1-y2)**2 , 0.5 ) #依次累加连续两点间的距离
    return dis

        
#======= 主函数 ==============
num_sample=10000
distance_neighbor=35
prm = PRM('C:\\Users\\93508\\Desktop\\maze.png',num_sample=10000,distance_neighbor=35) #读入文件且设置样本点数量和邻域大小
print("样本点数量为:{}  邻域距离为:{}".format(num_sample,distance_neighbor))
prm.learn() #构建无障碍连通图
path = prm.find_path() #使用PRM算法寻找最优路径
prm.plot(path) #画出路径

