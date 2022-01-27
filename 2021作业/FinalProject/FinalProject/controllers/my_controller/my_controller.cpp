#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <map>
#include <unordered_map>
#include <queue>
#include <iomanip>
#include<tuple>


#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/Lidar.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>


using namespace std;
using namespace webots;
using namespace cv;

//小车各方向的速度
const double v = 20.0;
double speedForward[4] = {v, v, v, v};
double speedBackward[4] = {-v, -v, -v, -v};
double speedLeftward[4] = {0.4 * v, 0.4 * v, v, v};
double speedRightward[4] = {v, v, 0.4 * v, 0.4 * v};
double speedLeftCircle[4] = {-0.5 * v, -0.5 * v, 0.5 * v, 0.5 * v};
double speedRightCircle[4] = {0.5 * v, 0.5 * v, -0.5 * v, -0.5 * v};


// 地图参数
//地图以像素为单位时的长和宽
const int mapHeight = 500;
const int mapWidth = 500;
//地图以距离(m)为单位时的长和宽
const double worldHeight = 10;
const double worldWidth = 10;
//像素——距离转换公式
const double world2pixel = mapHeight / worldHeight;
const int outlierCnt = 3; // 离群点检测范围

using namespace std;
using namespace cv;

Mat image;//用于存储点云图
vector<tuple<int, int,int> > node;//存储路径结果序列(终点到起点)
vector<tuple<int, int,int> > node1;//存储路径结果序列(起点到终点)
int row, col;//存储图片的行数和列数
int endr, endc;//存储终点的像素点位置

//计算两点间的GPS距离
double dis(double x1,double y1,double x2,double y2){
  double len=pow((x2-x1),2)+pow((y2-y1),2);
  return sqrt(len);
}

//像素点转GPS
void PixelToGps(int Pixelx,int Pixely,double& outx,double& outy) {
    outx = 1.0*Pixelx/world2pixel - worldWidth / 2.0;
    outy = 1.0*(mapHeight-Pixely)/world2pixel - worldWidth / 2.0;
}

//GPS转像素点
void GpsToPixel(double Gpsx,double Gpsy,int& outx,int& outy) {
    outx = int((Gpsx+ worldWidth / 2.0) * world2pixel);
    outy = mapHeight-int((Gpsy+ worldWidth / 2.0) * world2pixel);
    if(outx<0){
        outx=0;
    }
    if(outy<0){
        outy=0;
    }
    if(outx>=500){
        outx=499;
    }
    if(outy>=500){
        outy=499;
    }
} 

//判断当前点是不是墙
bool is_wall(int r, int c) {
    if (r < 0 || c < 0 || r >= row || c >= col) {
        return true;
    }
    return image.at<Vec3b>(c, r)[0] < 50 && \
        image.at<Vec3b>(c, r)[1] < 50 && \
        image.at<Vec3b>(c, r)[2] < 50;
}

//判断两点间的连线是否存在墙
bool check(tuple<int, int,int>node1, tuple<int, int,int>node2) {
    int step = 100;
    double a = get<0>(node1), b = get<1>(node1);
    double deta_a = 1.0 * (get<0>(node2) - get<0>(node1)) / step;
    double deta_b = 1.0 * (get<1>(node2) - get<1>(node1)) / step;
    for (int i = 0; i < step; ++i) {
        a += deta_a;
        b += deta_b;
        if (is_wall(a, b)) return false;
    }
    return true;
}

//判断两点是否合法以及两点间的连线是否存在墙
bool ok(tuple<int, int,int>a, tuple<int, int,int>b) {
    if (get<0>(a) < 0 || get<0>(b) < 0 || get<0>(a) >= row || get<0>(b) >= row \
        || get<1>(a) < 0 || get<1>(b) < 0 || get<1>(a) >= col || get<1>(b) >= col \
        || is_wall(get<0>(a), get<1>(a)) || is_wall(get<0>(b), get<1>(b)))
        return false;
    return check(a, b);
}

//RRT规划
vector<tuple<int, int,int>> build_graph(double start1,double end1) {
    //对于tuple<int,int,int>元组的3个int型变量依次代表:像素横坐标,像素纵坐标,与该像素点连线的父节点序号
    //将当前所在位置的GPS转为像素点,并加入到node向量中
    int start,end;
    GpsToPixel(start1,end1,start,end);
    node.push_back({ start, end,-1 });

    //进行RRT树的生长
    //near:当前离终点最近的节点的序号
    //steps:撒点数量
    //length:每次生长的步长
    //rate:向随机点和终点生长的概率分布
    int near = 0;
    int steps = 100000, length = 40;
    double rate = 0.5;
    srand((unsigned)time(NULL));
    while (steps--) {
        int m = rand() % 100;
        double p = 1.0 * m / 100;
        bool flag = true;
        //向终点方向生长
        if (p < rate) {
            //通过三角形的比例法则得出生长后的点的坐标(nr,nc)
            int r = get<0>(node[near]), c = get<1>(node[near]);
            double p = sqrt((r - endr) * (r - endr) + (c - endc) * (c - endc));
            int nr = r + length / p * (endr - r);
            int nc = c + length / p * (endc - c);
            //若两点间连线不存在墙
            if (ok({ nr,nc,near }, node[near])) {
                for (int i = 0; i < node.size(); ++i) {
                    //若当前已经存在的点与新生长的点的距离均大于10则将该点加入到RRT树中
                    //反之则认为新生长的点与某一点重合,跳过此次循环
                    int m = (get<0>(node[i]) - nr) * (get<0>(node[i]) - nr) + (get<1>(node[i]) - nc) * (get<1>(node[i]) - nc);
                    if (m < 100) {
                        flag = false;
                        break;
                    }
                }
                //若当前已经存在的点与新生长的点的距离均大于10则将该点加入到RRT树中
                if (flag == true) {
                    node.push_back({ nr,nc,near });
                    near = node.size() - 1;
                }
            }
            flag = true;
        }
        //向随机点方向生长
        else {
            //生成新随机点(r1,c1)
            int r1 = rand() % row, c1 = rand() % col;
            int index = -1;
            int k = 10000000;
            //找出RRT树中与新随机点距离最近的节点
            for (int i = 0; i < node.size(); ++i) {
                int m = (get<0>(node[i]) - r1) * (get<0>(node[i]) - r1) + (get<1>(node[i]) - c1) * (get<1>(node[i]) - c1);
                if (m < k) {
                    k = m;
                    index = i;
                }
            }
            //若距离太近,小于10,则认为新随机点与某一点重合,跳过此次循环
            if (k < 100) {
                continue;
            }
            
            //通过三角形的比例法则得出生长后的点的坐标(nr,nc)
            int r = get<0>(node[index]), c = get<1>(node[index]);
            double p = sqrt((r - r1) * (r - r1) + (c - c1) * (c - c1));
            int nr = r + length / p * (r1 - r);
            int nc = c + length / p * (c1 - c);
            //若两点间连线不存在墙
            if (ok({ nr,nc,index }, node[index])) {
                //若当前RRT树中的node[near](即离终点最近的节点)与终点的距离 > 新生长点与终点的距离
                //则near+1
                //最后再将该点加入到RRT树中(不管距离关系如何,只要连线合法就可以将其加入RRT树)
                int a = (endr - get<0>(node[near])) * (endr - get<0>(node[near])) + (endc - get<1>(node[near])) * (endc - get<1>(node[near]));
                int b = (endr - nr) * (endr - nr) + (endc - nc) * (endc - nc);
                if (a > b) near = node.size();
                node.push_back({ nr,nc,index });
            }
        }
        //若当前RRT树中的最新节点离终点的距离 < 根号450(20多),则终止撒点,开始连线
        if (node.size()>1&&(endr - get<0>(node[node.size()-1])) * (endr - get<0>(node[node.size() - 1])) + (endc - get<1>(node[node.size() - 1])) * (endc - get<1>(node[node.size() - 1])) < 450) {
            node.push_back({ endr,endc,near });
            break;
        }
    }

    int a=node.size()-1;
    vector<tuple<int, int,int> > Node;//存储路径结果序列(终点到起点)
    vector<tuple<int, int,int> > Node1;//存储路径结果序列(起点到终点)
    //利用回溯法画出终点到起点的路线(因为每个节点的父节点是唯一的)
    while (get<2>(node[a]) != -1) {
        int b = get<2>(node[a]);
        line(image, Point(get<0>(node[a]), get<1>(node[a])), Point(get<0>(node[b]), get<1>(node[b])), Scalar(0, 0, 0), 5);
        Node.push_back(node[a]);
        Node.push_back(node[b]);
        a = b;
    }
    //将Node数组倒序赋值给Node1数组
    int len=Node.size();
    for(int i=0;i<=len-1;i++){
        Node1.push_back(Node[len-1-i]);
    }
    return Node1;
}


//主程序
int main(int argc, char **argv)
{
    //一些节点的初始化
    //如Robot,Lidar,GPS,Motor等
    Robot *robot = new Robot();
    bool flag=false;//用于判定是否抵达终点
    int timeStep = (int)robot->getBasicTimeStep();
    int num=0;
    Keyboard keyboard;
    keyboard.enable(timeStep);
    double init_x,init_y;
    Motor *motors[4];
    char wheelsNames[4][8] = {"motor1", "motor2", "motor3", "motor4"};
    double speed[4];

    for (size_t i = 0; i < 4; i++)
    {
        motors[i] = robot->getMotor(wheelsNames[i]);
        motors[i]->setPosition(std::numeric_limits<double>::infinity());
        motors[i]->setVelocity(0.0);
        speed[i] = 0;
    }

    const double *pos; // (x,y,z)
    GPS *gps[2];
    char gps_names[2][100]={"gps","head_gps"};
    for(int i=0;i<2;i++){
      gps[i]=robot->getGPS(gps_names[i]);
      gps[i]->enable(64);
    }

    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    imu->enable(timeStep);
    const double *rpy; // (roll,pitch,yaw)

    Lidar *lidar = robot->getLidar("lidar");
    lidar->enable(timeStep);
    const float *lidarImage;
    int lidarRes = lidar->getHorizontalResolution(); // number of lidar points per line

    Mat map(mapHeight, mapWidth, CV_8U, 255);//存储点云图
    //cnt:帧数计算
    //maxCnt:建立点云图的频率
    //cnt2:和cnt一起用于判断何时建立点云图
    int cnt = 0, maxCnt = 60, cnt2 = 0;

    //用于存储路线序列的GPS二维坐标
    vector<double>nodex;
    vector<double>nodey;

    while(true){
        flag=false;
        //65代表每隔1秒建立1次点云图
        while (robot->step(timeStep) != -1&&cnt2 <= 65)
        {
            int keyValue = 0;
            keyValue = keyboard.getKey();
            pos = gps[0]->getValues();
            double carX = pos[0];
            double carY = pos[1];
            int carPixelX = int((carX + worldWidth / 2.0) * world2pixel);
            int carPixelY = mapHeight - int((carY + worldHeight / 2.0) * world2pixel);

            rpy = imu->getRollPitchYaw();
            double carAngle = rpy[0]; // rpy[0] = roll, 但旋转小车时 yaw 不发生变化。
            // cout << "rpy = " << carAngle << endl;

            // step1 建图
            Mat lidarPoints(mapHeight, mapWidth, CV_8U, 255);
            // circle(lidarPoints, Point2d(carPixelX, carPixelY), 3, 0, -1);

            if (cnt < 4) // 减速过程
                keyValue = 0;
            if (cnt==4)
            {
                lidarImage = lidar->getRangeImage();
                double mapPointAngle, mapPointX, mapPointY;
                for (size_t i = outlierCnt; i < lidarRes - outlierCnt; ++i)
                {
                    if (isfinite(lidarImage[i]) && lidarImage[i] != 0)
                    {
                        double outlierCheck = 0.0;
                        for (int k = i - outlierCnt; k < i + outlierCnt; ++k)
                        {
                            outlierCheck += abs(lidarImage[i] - lidarImage[k]);
                        }
                        outlierCheck /= (2 * outlierCnt);
                        if (outlierCheck > 0.1 * lidarImage[i])
                            continue;

                        mapPointAngle = carAngle + M_PI - double(i) / double(lidarRes) * 2.0 * M_PI;
                        mapPointX = lidarImage[i] * cos(mapPointAngle);
                        mapPointY = lidarImage[i] * sin(mapPointAngle);
                        mapPointX += carX;
                        mapPointY += carY;

                        int imgX = int((mapPointX + worldWidth / 2.0) * world2pixel);
                        int imgY = mapHeight - int((mapPointY + worldHeight / 2.0) * world2pixel);
                        if (imgX >= 0 && imgX < map.cols && imgY >= 0 && imgY < map.rows)
                        {
                            // circle(lidarPoints, Point(imgX, imgY), 1, 0);
                            circle(map, Point(imgX, imgY), 1, 0);
                        }
                    }
                }
                // imshow("lidar points", lidarPoints);
                //展示实时点云图
                imshow("CloudMap", map);
                imwrite("test.png",map);
                waitKey(1);
            }
            cnt++;
            cnt2++;
            if (cnt >= maxCnt){
                cnt -= maxCnt;
                
            }

        }
        cnt=0;
        cnt2=0;

        //以下步骤读取最新的点云图进行路径规划并根据结果进行GPS导航
        image = imread("C:/Users/93508/Desktop/FinalProject/controllers/my_controller/test.png");
        //图片尺寸
        col = image.rows;
        row = image.cols;
        //终点的像素坐标
        endr = 30; endc = col-50;
        //根据最新一次的GPS扫描信息得到当前小车所在点(即路径规划的起点)
        double start=pos[0];
        double end=pos[1];

        //前几次GPS扫描出来的结果可能是非法值,重新进行1次循环即可
        if(isnan(start)||isnan(end)){
            continue;
        }

        //输入起点的像素坐标进行RRT规划
        //并将结果存储到node1中
        node1=build_graph(start,end);

        //展示局部路线规划图
        imshow("Local Path", image);
        //可选择将该结果写入到文件夹中
        //imwrite("Path.png", image);
        
        //将路线序列从像素点转化为GPS
        //并将GPS二维坐标存储到nodex和nodey中用于后续GPS控制导航
        int length=node1.size();
        for(int i=0;i<=length-1;i++){
            double x,y;
            PixelToGps(get<0>(node1[i]),get<1>(node1[i]),x,y);
            nodex.push_back(x);
            nodey.push_back(y);
        }

        //GPS控制导航
        //设置小车运动方向的各个轮子的速度
        double alpha=0.05;
        double speedForward[4] = {v, v, v, v};
        double speedBackward[4] = {-v, -v, -v, -v};
        double speedRightward[4] = {alpha* v, alpha * v, -alpha*v, -alpha*v}; //重要
        double speedLeftward[4] = {-alpha*v, -alpha*v, alpha * v, alpha* v};
        int target=1;
        int count=1;
        //每次只走路线规划中的前3个点
        //走完后跳出循环,进行下一次雷达扫描,重新建立点云图进行新一次的路径规划
        while(robot->step(timeStep) !=-1 && target<=3){
            double cur_x=0,cur_y=0,head_x=0,head_y=0;
            double gps_x,gps_y,head_gps_x,head_gps_y;             
            gps_x=gps[0]->getValues()[0];
            gps_y=gps[0]->getValues()[1];
            head_gps_x=gps[1]->getValues()[0];
            head_gps_y=gps[1]->getValues()[1];
            //若有非法值则进行下一次循环,重新获取GPS值
            if(isnan(gps_x)||isnan(gps_y)||isnan(head_gps_x)||isnan(head_gps_y)){
                continue;
            }

            cur_x=gps_x;
            cur_y=gps_y;
            head_x=head_gps_x;
            head_y=head_gps_y;

            int maxstep=500;//时间阈值
            double k=0.05;//阈值

            //判断是否到达目标点，到达则更新为下一个目标点，否则记录经过了一个时钟周期
            if (dis(cur_x, cur_y, nodex[target], nodey[target]) < k)
            {
                target++;
                count=0;
                continue;
            }
            else{
                count++;
            }

            //若500个时钟周期过后发现仍为到达下一个目标点
            //则跳出循环,进入到下一次的雷达扫描当中
            //即开始重新规划路线
            if(count>maxstep){
                break;
            }
            int keyValue = 0;

            // 通过计算点乘计算余弦值
            double dot = (head_x - cur_x) * (nodex[target] - cur_x) + (head_y - cur_y) * (nodey[target] - cur_y);
            double norm = dis(0, 0, head_x - cur_x, head_y - cur_y) * dis(0, 0, nodex[target] - cur_x, nodey[target] - cur_y);
            double res = dot / norm;
            
            // 计算叉乘
            double cross = (head_x - cur_x) * (nodey[target] - cur_y) - (head_y - cur_y) * (nodex[target] - cur_x);
            
            if (res >= 0.995)// 直走
            {
                keyValue = 'W';
            }

            else if (cross > 0)// 左转
            {
                keyValue = 'A';
            }
            else if (cross < 0)// 右转
            {
                keyValue = 'D';
            }

            //设置小车的行动
            if (keyValue == 'W')
            {
                for (int i = 0; i < 4; ++i)
                {
                    speed[i] = speedForward[i];
                }
            }
            else if (keyValue == 'S')
            {
                for (int i = 0; i < 4; ++i)
                {
                    speed[i] = speedBackward[i];
                }
            }
            else if (keyValue == 'A')
            {
                for (int i = 0; i < 4; ++i)
                {
                    speed[i] = speedLeftward[i];
                }
            }
            else if (keyValue == 'D')
            {
                for (int i = 0; i < 4; ++i)
                {
                    speed[i] = speedRightward[i];
                }
            }
            else
            {
                for (int i = 0; i < 4; ++i)
                {
                    speed[i] = 0;
                }
            }
            for (int i = 0; i < 4; ++i)
            {
                motors[i]->setVelocity(speed[i]);
            }

            //将终点转换为GPS坐标
            //计算当前点和终点的距离
            //若小于0.25则说明已经抵达终点,跳出循环
            double endx,endy;
            PixelToGps(endr,endc,endx,endy) ;
            if(dis(cur_x,cur_y,endx,endy)<=0.25){
                flag=true;
                break;
            }
        }
        //因为是全局变量,每次循环后需要将结果清空
        node.clear();
        nodex.clear();
        nodey.clear();
        node1.clear();
        //局部路线行驶完毕后将小车速度置0,便于雷达扫描
        for (int i = 0; i < 4; ++i)
        {
            speed[i] = 0;
        }
        for (int i = 0; i < 4; ++i)
        {
                motors[i]->setVelocity(speed[i]);
        }
        //若flag为true这说明到达终点,可跳出大循环
        if(flag==true){
            break;
        }
    }
    return 0;
}
