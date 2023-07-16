#pragma once
#ifndef MINIMUM_JERK_H
#define MINIMUM_JERK_H

#include <vector>
#include <Eigen/Dense>

class Minimum_jerk
{
private:
    double cost_func;
    // std::vector<Eigen::Vector3d> points;

public:
    Minimum_jerk(/* args */);
    ~Minimum_jerk();

    void solve_minimum_jerk(std::vector<Eigen::Vector3d> points);
};

Minimum_jerk::Minimum_jerk(/* args */)
{
}

Minimum_jerk::~Minimum_jerk()
{
}


#endif