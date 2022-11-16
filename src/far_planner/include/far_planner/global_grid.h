#ifndef GLOBAL_GRID_H
#define GLOBAL_GRID_H

#include "utility.h"
#include "dynamic_graph.h"

struct GlobalGridParams {
    GlobalGridParams() = default;
    std::string frame_id;
    float cell_length;
    float grid_max_length;
    float origin_x;
    float origin_y;    
};

class GlobalGrid {
public:
    GlobalGrid() = default;
    ~GlobalGrid() = default;
    PointCloudPtr cloud;
    void Init(const ros::NodeHandle& nh, const GlobalGridParams& params);
    void UpdateGlobalGraph(const Point3D& center,const PointCloudPtr& obsCloudIn,const PointCloudPtr& freeCloudIn);
    
    void OutputGlobalGrid();
    void ShowCornerImage(const cv::Mat& img_mat,const PointCloudPtr& pc);
    const cv::Mat       GetCloudImgMat() const { return img_mat_;};
    void ConvertNavNodePtrStackToPCL(const NodePtrStack &navnode_stack,const PointCloudPtr &point_cloud_ptr);
    Point3D UpdatePos(const Point3D& odom);
    
private:
    void SetGlobalGridOrigin();
    void ResetGlobalGridOrigin(const Point3D& ori_robot_pos);
    NavNodePtr odom_node_ptr_;
    Point3D odom_pos_;
    cv::Point2f free_odom_resized_;
    PointCloudPtr new_corners_cloud_;
    int MAT_SIZE, CMAT;
    int MAT_RESIZE, CMAT_RESIZE;
    cv::Mat img_mat_;
    std::size_t img_counter_;
    float VOXEL_DIM_INV;
    ros::NodeHandle nh_;
    GlobalGridParams gg_params_;

    PointCloudPtr  nodes_cloud_ptr_;
    std::unique_ptr<grid_ns::Grid<char>>  global_grids_ ;
    bool init_grid = false;
    std::vector<int> visited_list_;

};


#endif