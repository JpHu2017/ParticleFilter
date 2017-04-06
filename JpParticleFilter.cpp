//
// Created by cvrsg on 17-4-2.
//

#include "JpParticleFilter.h"
//#include <math.h>

double JpRandn(double start, double end)
{
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}

bool particle_cmp(Particle p1, Particle p2)
{
    return p1.weight > p2.weight;
}

//注意：在粒子滤波中，噪声对跟踪影响很大；这个地方也困扰我多时
//比如之前我设置m_processNoiseCov = 0.1 而 m_measureNoiseCov = 1时，跟踪结果根本无法入眼
JpParticleFilter::JpParticleFilter(double measureNoiseCov, double processNoiseCov)
        : m_measureNoiseCov(measureNoiseCov), m_processNoiseCov(processNoiseCov)
{

}

//粒子初始化
void JpParticleFilter::init(int n, double width, double height) {
    m_particles.resize(n);
    for(int i=0; i<n; ++i)
    {
        m_particles[i].x = JpRandn(0, width);
        m_particles[i].y = JpRandn(0, height);
        m_particles[i].px = m_particles[i].x;
        m_particles[i].py = m_particles[i].y;
        m_particles[i].weight = 1.0 / static_cast<double>(n);
    }
    m_width = width;
    m_height = height;
}

void JpParticleFilter::transition(Particle &p, double width, double height)
{
    Particle pn;
    pn.x = 2 * p.x - p.px + m_rng.gaussian(m_processNoiseCov); // x + v * t <=> x + (x - px)
    pn.y = 2 * p.y - p.py + m_rng.gaussian(m_processNoiseCov);
    //粒子不超过图片范围
    pn.x = std::max(0.0, std::min(pn.x, width));
    pn.y = std::max(0.0, std::min(pn.y, height));
    pn.px = p.x;
    pn.py = p.y;

    p.x = pn.x; p.y = pn.y;
    p.px = pn.px; p.py = pn.py;
}

void JpParticleFilter::normalizeWeights() {
    double weights_sum = 0;
    for(int i=0; i<m_particles.size(); ++i)
    {
        weights_sum += m_particles[i].weight;
    }
    for(int i=0; i<m_particles.size(); ++i)
    {
        m_particles[i].weight = m_particles[i].weight/weights_sum;
    }
}

std::pair<double, double> JpParticleFilter::predict(double measure_x, double measure_y) {
    for(int i=0; i<m_particles.size(); ++i)
    {
        transition(m_particles[i], m_width, m_height);
        double distance = computeDistance(m_particles[i].x, m_particles[i].y, measure_x, measure_y);
        m_particles[i].weight = computeWeights(distance, m_measureNoiseCov);
    }
    normalizeWeights();
    double avg_x = 0, avg_y = 0;
    for(int i=0; i<m_particles.size(); ++i)
    {
        avg_x += m_particles[i].x * m_particles[i].weight;
        avg_y += m_particles[i].y * m_particles[i].weight;
    }
    return std::pair<double, double>(avg_x, avg_y);
}

double JpParticleFilter::computeDistance(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

double JpParticleFilter::computeWeights(double distance, double sigma) {
    //两种计算权值的方式
    //1.高斯权值
    //return 1.0/sqrt(sigma)/sqrt(2.0*PI)*exp(-1.0*distance*distance/2.0/sigma);
    //2.距离倒数
    return 1.0/distance;
}

void JpParticleFilter::resample() {
    std::sort(m_particles.begin(), m_particles.end(), particle_cmp);
    std::vector<Particle> new_particles;
    for(int i=0; i<m_particles.size(); ++i)
    {
        double np = cvRound(m_particles[i].weight * m_particles.size());
        for(int j=0; j<np; ++j)
        {
            new_particles.push_back(m_particles[i]);
            if(new_particles.size() == m_particles.size())
                break;
        }
        if(new_particles.size() == m_particles.size())
            break;
    }
    while(new_particles.size() < m_particles.size())
    {
        new_particles.push_back(m_particles[0]);
    }
    m_particles.swap(new_particles);
}

const std::vector<Particle> &JpParticleFilter::particles() {
    return m_particles;
}


















