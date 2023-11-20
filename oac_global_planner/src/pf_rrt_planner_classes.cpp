/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 *  Date: 2023-6-24
 */

#include "pf_rrt_planner_classes.h"
#include <execution>
#include <algorithm>
#include <iostream>
#include <chrono>

using namespace std;
using namespace Eigen;

namespace OAC
{

Node::Node():cost_(0.0f){}
Node::Node(const Node &node)
{
    children_=node.children_;
    parent_=node.parent_;
    position_=node.position_;
    cost_=node.cost_;

    plane_=new Plane;
    if(node.plane_!=NULL) *plane_=*node.plane_;
}
Node::~Node()
{
    delete plane_;
    plane_=NULL;
}

Path::Path():cost_(INF),type_(Empty){}
Path::~Path(){}

Plane::Plane(){}
Plane::Plane(const Eigen::Vector3d &p_surface,World* world,const double &radius,const FitPlaneArg &arg)
{
    init_coord=project2plane(p_surface);
    // Converts the coordinates of the point to the grid center coordinates under World
    Vector3d ball_center = world->coordRounding(p_surface);
    float resolution = world->getResolution();

    int fit_num=static_cast<int>(radius/resolution);
    Matrix<bool,Dynamic,Dynamic> vac(2*fit_num+1,2*fit_num+1);
    int vac_cout_init=(2*fit_num+1)*(2*fit_num+1);
    // Loop over each cell in the vac matrix
    for (int i = -fit_num; i <= fit_num; i++) {
        for (int j = -fit_num; j <= fit_num; j++) {
            // Initialize the vac matrix cell to false
            vac(i + fit_num, j + fit_num) = false;

            // Traverse the points around the cube
            for (int k = -3; k <= 3; k++) {
                Vector3d point = ball_center + resolution * Vector3d(i, j, k);

                // Check if the point is inside the world's borders and not free
                if (world->isInsideBorder(point) && !world->isFree(point)) {
                    // Add the point to the plane_pts vector
                    plane_pts.push_back(point);

                    // If the vac matrix cell was not set to true before, set it to true and decrement vac_count_init
                    if (!vac(i + fit_num, j + fit_num)) {
                        vac(i + fit_num, j + fit_num) = true;
                        vac_cout_init--;
                    }
                }
            }
        }
    }
    
    size_t pt_num=plane_pts.size();
    Vector3d center;
    for(const auto&pt:plane_pts) center+=pt;
    center /= pt_num;
    MatrixXd A(pt_num,3);
    for(size_t i = 0; i < pt_num; i++) A.row(i)=plane_pts[i]-center;

    //Key point : Planar fitting using SVD
    JacobiSVD<MatrixXd> svd(A,ComputeFullV);
    normal_vector=svd.matrixV().col(2);
    
    //calculate indicator1:flatness      
    float flatness = 0;
    for(size_t i = 0; i < pt_num; i++) flatness+=powf(normal_vector.dot(A.row(i)),2);
    flatness /= (1+pt_num);

    //calculate indicator2:slope
    Vector3d z_axies(0,0,1);
    float slope = 180.0f*(float)acos(z_axies.dot(normal_vector)) / PI;

    //calculate indicator3:sparsity
    float sparsity = 0.0f;

    if(vac_cout_init > 0)
    {
        int vac_cout = 0;
        MatrixXd M_vac(2,vac_cout_init);
        for(int i = 0;i < vac.rows();i++)
        {
            for(int j = 0;j < vac.cols();j++)
            {
                if(!vac(i,j))
                {
                    M_vac(0,vac_cout) = i;
                    M_vac(1,vac_cout) = j;
                    vac_cout++;
                }
            }  
        }
        
        MatrixXd meanVec = M_vac.colwise().mean();
        MatrixXd zeroMeanMat = M_vac;
        RowVectorXd meanVecRow(RowVectorXd::Map(meanVec.data(),M_vac.cols()));
        zeroMeanMat.rowwise() -= meanVecRow;
        MatrixXd covMat = (zeroMeanMat.adjoint()*zeroMeanMat)/float(M_vac.rows());
        float trace  = (covMat.transpose()*covMat(0,0)).trace();
        float ratio = vac_cout/(float)(vac.rows()*vac.cols());
 
        if(ratio > arg.ratio_max_) sparsity = 1;
        else if(ratio > arg.ratio_min_ && ratio < arg.ratio_max_ && (1/trace) > arg.conv_thre_) 
            //sparsity = ratio;
            sparsity=(ratio-arg.ratio_min_)/(arg.ratio_max_-arg.ratio_min_);
        else sparsity = 0;
    }

    //The traversability is linear combination of the three indicators
    traversability=arg.w_total_*(arg.w_flatness_*flatness+arg.w_slope_*slope+arg.w_sparsity_*sparsity);
    traversability = (1 < traversability)?1:traversability; 
}

Plane::Plane(const Eigen::Vector3d &p_surface, World* world)
{
   // 得到二维坐标及法向量
   grid_map::Position position(p_surface(0), p_surface(1));
   // 获取给定位置对应的栅格索引
   grid_map::Index index;
   world->subMap_.getIndex(position, index);
   // 获取栅格的中心坐标
   grid_map::Position gridCenter;
   world->subMap_.getPosition(index, gridCenter); 

   init_coord = Vector2d(gridCenter(0), gridCenter(1));
   normal_vector(0) = world->subMap_.atPosition("normal_vectors_x", gridCenter);
   normal_vector(1) = world->subMap_.atPosition("normal_vectors_y", gridCenter);
   normal_vector(2) = world->subMap_.atPosition("normal_vectors_z", gridCenter);
    
   //得到可通过性
   traversability = float(world->subMap_.atPosition("traversability", gridCenter));
}

World::World(const float &resolution, const ros::NodeHandle& node_handle) : 
        resolution_(resolution)
{
    resolution_=resolution;
    lowerbound_=INF*Vector3d::Ones();
    upperbound_=-INF*Vector3d::Ones();
    idx_count_=Vector3i::Zero();
    gridMap_.setBasicLayers({"elevation"});
    gridMap_.add("elevation", 0.0);
    gridMap_.setFrameId("world");
    gridMap_.setGeometry(grid_map::Length(100.0, 100.0), 0.08); // 设置地图尺寸和分辨率
}

World::~World(){clearMap();}

void World::clearMap()
{
    if(has_map_)
    {
        // grid_map_.clear();
        effect_grid_.clear();
        // map_.clear();
        // subMap_.clear();
    }
}

void World::initGridMap(grid_map::GridMap& gridMap)
{
    clearMap();
    grid_map::Size size = gridMap.getSize();
    // // 获取指定图层的最大值和最小值
    // double minValue, maxValue;
    // if(minValue < lowerbound_(2)) lowerbound_(2) = minValue;
    // if(maxValue > upperbound_(2)) upperbound_(2) = maxValue;

    grid_map::Position pos = gridMap.getPosition();
    grid_map::Length length = gridMap.getLength();

    if(pos(0) + length(0)/2 > upperbound_(0)) upperbound_(0) = pos(0) + length(0)/2;
    if(pos(0) - length(0)/2 < lowerbound_(0)) lowerbound_(0) = pos(0) - length(0)/2;
    if(pos(1) + length(1)/2 > upperbound_(1)) upperbound_(1) = pos(1) + length(1)/2;
    if(pos(1) - length(1)/2 < lowerbound_(1)) lowerbound_(1) = pos(1) - length(1)/2;
    upperbound_(2) = 5.0;
    lowerbound_(2) = -5.0;

    idx_count_ = ((upperbound_-lowerbound_)/resolution_).cast<int>() + Eigen::Vector3i::Ones();
    
    if(subMap_.exists("elevation_inpainted")){
        has_map_=true;
    }else has_map_=false;
}

void World::initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{   
    if(cloud.points.empty())
    {
        ROS_ERROR("Can not initialize the map with an empty point cloud!");
        return;
    }
    clearMap();
    auto start_tiem = std::chrono::steady_clock::now();
    for(const auto&pt:cloud.points)
    {
        if(pt.x < lowerbound_(0)) lowerbound_(0)=pt.x;
        if(pt.y < lowerbound_(1)) lowerbound_(1)=pt.y;
        if(pt.z < lowerbound_(2)) lowerbound_(2)=pt.z;
        if(pt.x > upperbound_(0)) upperbound_(0)=pt.x;
        if(pt.y > upperbound_(1)) upperbound_(1)=pt.y;
        if(pt.z + 1.0 > upperbound_(2)) upperbound_(2)=pt.z+1.0;
    }
    auto end_time1 = std::chrono::steady_clock::now();

    idx_count_ = ((upperbound_-lowerbound_)/resolution_).cast<int>() + Eigen::Vector3i::Ones();

    if(run_time_print_) ROS_WARN("idx_count_(0): %d , idx_count_(1): %d , idx_count_(2): %d", idx_count_(0), idx_count_(1), idx_count_(2));
    
    // grid_map_.resize(idx_count_(0), std::vector<std::vector<bool>>(idx_count_(1), std::vector<bool>(idx_count_(2), true)));

    auto end_time2 = std::chrono::steady_clock::now();
    if(run_time_print_) ROS_WARN("TimeIn1: %f , TimeIn2: %f", \
                std::chrono::duration_cast<std::chrono::duration<double>>(end_time1 - start_tiem).count(), \
                std::chrono::duration_cast<std::chrono::duration<double>>(end_time2 - end_time1).count());
    has_map_=true;
}
 
/*
    * @brief: judge the route from node_start to node_end is collision free or not
    * @param: node_start: start node
    * @param: node_end: end node
*/
bool World::collisionFree(const Node* node_start,const Node* node_end) 
{
    Vector3d e_z,e_y,e_x;
    Matrix3d rotation_matrix;

    Vector3d diff_pos=node_end->position_-node_start->position_;
    Vector3d diff_norm_vector=node_end->plane_->normal_vector-node_start->plane_->normal_vector;

    size_t step=20;
    bool isfree = true;

    for(size_t i = 0;i <= step;i++)
    {
        Vector3d check_center = node_start->position_ + diff_pos * i/(double)step;
        e_z=node_start->plane_->normal_vector + diff_norm_vector *i/(double)step;
        e_z.normalize();

        e_x = diff_pos-(diff_pos.dot(e_z))*diff_pos;
        e_x.normalize();

        e_y = e_z.cross(e_x); 

        rotation_matrix << e_x(0),e_y(0),e_z(0),
                           e_x(1),e_y(1),e_z(1),
                           e_x(2),e_y(2),e_z(2);

        Vector3d check_point;
        for(int y=-3; y <= 3; y++)
        {
            for(int z=-2; z<=2; z++)
            {
                for(int x=-4; x<=6; x++)
                {
                    // 整车长 75cm, 摆臂长25cm， 宽度45cm，高度25cm
                    check_point=check_center+rotation_matrix*Vector3d(-0.263 + 0.097 * x, 0.15 * y, 0.1 * z);
                    if(!isFree(check_point)) 
                    {
                        return false;
                    }
                }
            }
        }
    }
    return isfree;
}

void World::setObs(const Vector3d &point)
{   
    world_mutex_.lock();
    Vector3i idx=coord2index(point);
    grid_map_[idx(0)][idx(1)][idx(2)]=false;
    // grid_map_.push_back(idx);
    effect_grid_.push_back(idx);
    world_mutex_.unlock();
}

void World::setGrid(const Vector3d &point)
{   
    world_mutex_.lock();
    // 将点的坐标转换为GridMap坐标
    grid_map::Position position(point(0), point(1));
    // 检查点是否在GridMap范围内
    if (gridMap_.isInside(position))
    {
        // 将点的数据（例如高度）添加到GridMap中
        gridMap_.atPosition("elevation", position) = point(2);
    }
    world_mutex_.unlock();
}

bool World::isFree(const Vector3d &point)
{
    Vector3i idx = coord2index(point);
    // bool is_free = isInsideBorder(idx) && grid_map_[idx(0)][idx(1)][idx(2)];
    grid_map::Position position(point(0), point(1));
    if(!subMap_.isInside(position)) return false;
    bool is_free = isInsideBorder(idx) && subMap_.atPosition("elevation_inpainted", position) < point(2);
    return is_free;
}

Vector3d World::coordRounding(const Vector3d & coord)
{
    return index2coord(coord2index(coord));
}

bool World::project2surface(const float &x,const float &y,Vector3d* p_surface)
{
    bool ifsuccess=true;
    grid_map::Position position(x, y);
    if(subMap_.isInside(position))
    {
        // 获取给定位置对应的栅格索引
        grid_map::Index index;
        subMap_.getIndex(position, index);
        // 获取栅格的中心坐标
        grid_map::Position gridCenter;
        subMap_.getPosition(index, gridCenter);
        *p_surface=Vector3d(gridCenter(0), gridCenter(1), subMap_.atPosition("elevation_inpainted", gridCenter));
        
        if(std::isnan((*p_surface)(2))) ifsuccess=false;
    }
    return ifsuccess;
}

bool World::project2GridMapSurf(const float &x,const float &y,Eigen::Vector3d* p_surface)
{
    bool ifsuccess=true;
    grid_map::Position position(x, y);
    if(subMap_.isInside(position))
    {
        // 获取给定位置对应的栅格索引
        grid_map::Index index;
        subMap_.getIndex(position, index);

        // 获取栅格的中心坐标
        grid_map::Position gridCenter;
        subMap_.getPosition(index, gridCenter);

        *p_surface=Vector3d(gridCenter(0), gridCenter(1), subMap_.atPosition("elevation_inpainted", gridCenter));
        
        if(std::isnan((*p_surface)(2))) ifsuccess=false;
    }else return false;

    return ifsuccess;
}

bool World::isInsideBorder(const Vector3i &index)
{
    return index(0) >= 0 &&
           index(1) >= 0 &&
           index(0) < idx_count_(0)&&
           index(1) < idx_count_(1);
}
}

