//
// Created by cvrsg on 17-4-2.
//
/*两个Particle Filter类的区别：
 *JpParticleFilter粒子中的变量有当前坐标，前一时刻坐标，权值
 * 其实两个类是等价的。
 * */

#ifndef PARTICLEFILTER_JPPARTICLEFILTER_H
#define PARTICLEFILTER_JPPARTICLEFILTER_H

#include "opencv2/core.hpp"
#include <vector>
#define PI 3.1415926

class Particle
{
public:
    double x; // current x
    double y;
    double px; // previous x
    double py;
    double weight;
};


class JpParticleFilter {

public:
    JpParticleFilter(){}
    JpParticleFilter(double measureNoiseCov, double processNoiseCov);
    void init(int n, double width, double height); //设置粒子大小
    std::pair<double, double> predict(double measure_x, double measure_y);
    void resample();
    const std::vector<Particle>& particles();

private:
    void transition(Particle &p, double width, double height);
    void normalizeWeights();
    double computeWeights(double distance, double sigma);
    double computeDistance(double x1, double y1, double x2, double y2);


private:
    std::vector<Particle> m_particles;
    cv::RNG m_rng;

    double m_width;
    double m_height;

    double m_processNoiseCov;
    double m_measureNoiseCov;

};


#endif //PARTICLEFILTER_JPPARTICLEFILTER_H
