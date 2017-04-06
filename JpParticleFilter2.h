//
// Created by cvrsg on 17-4-2.
//

/*两个Particle Filter类的区别：
 *JpParticleFilter2粒子中的变量有当前坐标，当前速度，权值
 * 这里也证明了：我们可以将状态转移方程写成和Kalman Filter一样的形式
 * [x(t),y(t),vx(t),vy(t)] = [1,0,1,0
                              0,1,0,1
                              0,0,1,0
                              0,0,,1] * [x(t-1),y(t-1),vx(t-1),vy(t-1)]
 * */

#ifndef PARTICLEFILTER_JPPARTICLEFILTER2_H
#define PARTICLEFILTER_JPPARTICLEFILTER2_H

#include "opencv2/core.hpp"
#include <vector>
#define PI 3.1415926

class Particle2
{
public:
    double x; // current x
    double y;
    double vx; // previous x
    double vy;
    double weight;
};


class JpParticleFilter2 {

public:
    JpParticleFilter2(){}
    JpParticleFilter2(double measureNoiseCov, double processNoiseCov);
    void init(int n, double width, double height); //设置粒子大小
    std::pair<double, double> predict(double measure_x, double measure_y);
    void resample();
    const std::vector<Particle2>& particles();

private:
    void transition(Particle2 &p, double width, double height);
    void normalizeWeights();
    double computeWeights(double distance, double sigma);
    double computeDistance(double x1, double y1, double x2, double y2);


private:
    std::vector<Particle2> m_particles;
    cv::RNG m_rng;

    double m_width;
    double m_height;

    double m_processNoiseCov;
    double m_measureNoiseCov;

};


#endif //PARTICLEFILTER_JPPARTICLEFILTER2_H
