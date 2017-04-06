#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <algorithm>
#include <math.h>
#include "JpParticleFilter.h"
#include "JpParticleFilter2.h"

using namespace std;
using namespace cv;

void print(Mat temp)
{
    cout << temp << endl;
}

//注意：在粒子滤波中，噪声对跟踪影响很大；这个地方也困扰我多时
//比如之前我设置  m_processNoiseCov = 0.1 而 m_measureNoiseCov = 1  时，跟踪结果根本无法入眼

//double measureNoiseCov = 10;
//double processNoiseCov = 10;
double measureNoiseCov = 5;
double processNoiseCov = 5;

double true_x = 40, true_y = 40, true_vx = 10, true_vy = 10;
double measure_x, measure_y;

int main()
{
    RNG rng;

//    JpParticleFilter pf(measureNoiseCov, processNoiseCov);
    JpParticleFilter2 pf(measureNoiseCov, processNoiseCov);
    pf.init(100, 600, 450);

    while(1)
    {
        Mat image(450, 600, CV_8UC3, Scalar(0,0,0));
        rectangle(image,Point(30,30),Point(570,420),Scalar(255,255,255),2);//绘制目标弹球的“撞击壁”

        //假设真实运动为严格匀速运动
        true_x += true_vx;
        true_y += true_vy;
        //观测数据模拟，观测数据符合相应的高斯分布
        measure_x = true_x + rng.gaussian(measureNoiseCov);
        measure_y = true_y + rng.gaussian(measureNoiseCov);

        std::pair<double, double> pf_predict;
        pf_predict = pf.predict(measure_x, measure_y);
        pf.resample();


        for(int i=0; i< pf.particles().size(); ++i)
        {
            circle(image, Point(pf.particles()[i].x, pf.particles()[i].y) , 0.5, Scalar(255, 255, 255), 2); // 青色 - filter预测位置
        }
        circle(image, Point(pf_predict.first, pf_predict.second) , 4, Scalar(255, 255, 0), 2); // 青色 - filter预测位置
        circle(image, Point(true_x, true_y), 4, Scalar(0, 255, 255), 2); //黄色 - 真实位置
        circle(image, Point(measure_x, measure_y), 4, Scalar(0, 255, 0), 2); //绿色 - 观测位置

        if(true_x<30){  //当撞击到反弹壁时，对应轴方向取反
            true_vx *= -1;
        }
        if(true_x>570){
            true_vx *= -1;
        }
        if(true_y<30){  //当撞击到反弹壁时，对应轴方向取反
            true_vy *= -1;
        }
        if(true_y>420){
            true_vy *= -1;
        }

        imshow("ParitcleFilter", image);
        if( waitKey(100) == 'e')
        {
            break;
        }
    }
    return 0;
}