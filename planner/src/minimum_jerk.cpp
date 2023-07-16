#include "minimum_jerk.h"
#include <iostream>
#include <fstream>
std::ofstream output;

void Minimum_jerk::solve_minimum_jerk(std::vector<Eigen::Vector3d> points){

    int n = points.size() - 1; //pieceNum
    output.open("/home/parallels/solve_minimum_jerk.txt", std::ios::app);

    Eigen::Vector3d start_pt;
    start_pt = points[0];
    output << "pieceNum" << n << std::endl;

    for(const auto &point : points){
        output << point << std::endl;
        std::cout << point << std::endl;
        output << "================================" << std::endl;
        std::cout << "================================" << std::endl;
    }
    output.close();
}
