/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * lqz.nnu.edu,   
 */
#include "far_planner/global_grid.h"
#include "far_planner/map_handler.h"

const char INIT_BIT = char(0);
const char OBS_BIT = char(1);
const char FREE_BIT = char(2);

void GlobalGrid::Init(const ros::NodeHandle& nh, const GlobalGridParams& params){
    nh_ = nh;
    gg_params_ = params;
    const int row_num = std::ceil(gg_params_.grid_max_length / gg_params_.cell_length);
    const int col_num = row_num;

    // initialize grid 
    Eigen::Vector3i grid_size(row_num,col_num,1);
    Eigen::Vector3d grid_origin(0,0,0);
    Eigen::Vector3d grid_resolution(FARUtil::kLeafSize, FARUtil::kLeafSize, FARUtil::kLeafSize);       //  0.15 * 0.15 * 0.15
    global_grids_ = std::make_unique<grid_ns::Grid<char>>(grid_size, INIT_BIT, grid_origin, grid_resolution, 3);   //cell里存放0初始，1障碍物，2空地
    global_grids_->ReInitGrid(INIT_BIT);

    const int n_cell  = global_grids_->GetCellNumber();
    visited_list_.resize(n_cell);
    std::fill(visited_list_.begin(), visited_list_.end(), 0);
    this->SetGlobalGridOrigin();
    init_grid = true;
}

void GlobalGrid::ResetGlobalGridOrigin(const Point3D& ori_robot_pos){
    Point3D map_origin;
    const Eigen::Vector3i dim = global_grids_->GetSize();
    map_origin.x = ori_robot_pos.x - (global_grids_->GetResolution().x() * dim.x()) /2;
    map_origin.y = ori_robot_pos.y - (global_grids_->GetResolution().y() * dim.y()) /2;
    map_origin.z = ori_robot_pos.z - (global_grids_->GetResolution().z()) / 2.0f - FARUtil::vehicle_height; // From Ground Level
    Eigen::Vector3d pointcloud_grid_origin(map_origin.x, map_origin.y, map_origin.z);
    global_grids_->SetOrigin(pointcloud_grid_origin);
    init_grid = true;
    if (FARUtil::IsDebug) ROS_INFO("MH: Global Cloud Map Grid Initialized.");
}

void GlobalGrid::SetGlobalGridOrigin(){
    Point3D map_origin;
    const Eigen::Vector3i dim = global_grids_->GetSize();
    map_origin.x = gg_params_.origin_x;
    map_origin.y = gg_params_.origin_y;
    map_origin.z = 0 - (global_grids_->GetResolution().z()) / 2.0f - FARUtil::vehicle_height; // From Ground Level
    Eigen::Vector3d pointcloud_grid_origin(map_origin.x, map_origin.y, map_origin.z);
    global_grids_->SetOrigin(pointcloud_grid_origin);
    init_grid = true;
    if (FARUtil::IsDebug) ROS_INFO("MH: Global Cloud Map Grid Initialized.");
}

void GlobalGrid::UpdateGlobalGraph(const Point3D& center,const PointCloudPtr& obsCloudIn,const PointCloudPtr& freeCloudIn){
    if (!init_grid){
        global_grids_->ReInitGrid(INIT_BIT);
    }
    if(!freeCloudIn->empty() || !obsCloudIn->empty()){
        for(const auto &point :obsCloudIn->points){
            Eigen::Vector3i c_sub = global_grids_->Pos2Sub(point.x, point.y, FARUtil::odom_pos.z);
            for(int i = -1;i < 1 ;i++){
                for(int j = -1;j < 1;j++){
                    Eigen::Vector3i sub = c_sub;
                    sub.x() += i, sub.y() += j, sub.z() = 0;
                    if (global_grids_->InRange(sub)) {
                        const int ind = global_grids_->Sub2Ind(sub);
                        global_grids_->GetCell(ind) =  global_grids_->GetCell(ind) | OBS_BIT;
                    }
                }
            }
        }
        for(const auto &point :freeCloudIn->points){
            Eigen::Vector3i c_sub = global_grids_->Pos2Sub(point.x, point.y, FARUtil::odom_pos.z);
            for(int i = -1;i < 1 ;i++){
                for(int j = -1;j < 1;j++){
                    Eigen::Vector3i sub = c_sub;
                    sub.x() += i, sub.y() += j, sub.z() = 0;
                    if (global_grids_->InRange(sub)) {
                        const int ind = global_grids_->Sub2Ind(sub);
                        global_grids_->GetCell(ind) = global_grids_->GetCell(ind) | FREE_BIT;  
                    }
                }
            }
        }
        
    }
}

void GlobalGrid::OutputGlobalGrid(){
    std::vector<char> output_grid;
    const int x = global_grids_->GetSize().x();
    const int y = global_grids_->GetSize().y();
    for(int i = 0; i < y; i++){
        for(int j = 0 ; j < x; j++){
            const char value = global_grids_->GetCell(i,j,0);
            output_grid.push_back(value);
        }
    }
    //存储网格信息
    std::fstream file("/home/liqunzhao/far_planner/output_grid.txt",std::ios::out);
    for(const auto status : output_grid){
        file<<(int)status<<',';
    }
    file.close();
    
}

Point3D GlobalGrid::UpdatePos(const Point3D &odom){
    return odom_pos_ = odom;
}
