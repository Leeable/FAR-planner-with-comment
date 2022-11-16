/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,
 */

#include "far_planner/far_planner.h"

/***************************************************************************************/

void FARMaster::Init()
{
    /* initialize subscriber and publisher */
    reset_graph_sub_ = nh.subscribe("/reset_visibility_graph", 5, &FARMaster::ResetGraphCallBack, this);
    odom_sub_ = nh.subscribe("/odom_world", 5, &FARMaster::OdomCallBack, this);
    terrain_sub_ = nh.subscribe("/terrain_cloud", 1, &FARMaster::TerrainCallBack, this);
    scan_sub_ = nh.subscribe("/scan_cloud", 5, &FARMaster::ScanCallBack, this);
    waypoint_sub_ = nh.subscribe("/goal_point", 1, &FARMaster::WaypointCallBack, this);
    terrain_local_sub_ = nh.subscribe("/terrain_local_cloud", 1, &FARMaster::TerrainLocalCallBack, this);
    joy_command_sub_ = nh.subscribe("/joy", 5, &FARMaster::JoyCommandCallBack, this);
    update_command_sub_ = nh.subscribe("/update_visibility_graph", 5, &FARMaster::UpdateCommandCallBack, this);
    goal_pub_ = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
    boundary_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("/navigation_boundary", 5);
    // Timers
    runtime_pub_ = nh.advertise<std_msgs::Float32>("/runtime", 1);
    planning_time_pub_ = nh.advertise<std_msgs::Float32>("/planning_time", 1);
    traverse_time_pub_ = nh.advertise<std_msgs::Float32>("/far_traverse_time", 5);
    laser_timer_pub_    = nh.advertise<std_msgs::Float32>("/laser_time",1);

    // planning status publisher
    reach_goal_pub_ = nh.advertise<std_msgs::Bool>("/far_reach_goal_status", 5);
    // Terminal formatting subscriber
    read_command_sub_ = nh.subscribe("/read_file_dir", 1, &FARMaster::ReadFileCommand, this);
    save_command_sub_ = nh.subscribe("/save_file_dir", 1, &FARMaster::SaveFileCommand, this);
    // DEBUG Publisher
    dynamic_obs_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_dynamic_obs_debug", 1);
    surround_free_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_free_debug", 1);
    surround_obs_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_obs_debug", 1);
    scan_grid_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_scanGrid_debug", 1);
    new_PCL_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_new_debug", 1);
    terrain_height_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_terrain_height_debug", 1);
    // 是否开启记录waypoint、odom、goal、connection
    record_data_ = nh.subscribe("/record_data", 5, &FARMaster::RecordDataCallBack, this);
    // 是否输出grid
    ouputgrid_ = nh.subscribe("/output_grid",5, &FARMaster::OutputGridCallBack, this);
    this->LoadROSParams();

    /*init path generation thred callback*/
    const float duration_time = 0.99f / master_params_.main_run_freq;
    planning_event_ = nh.createTimer(ros::Duration(duration_time), &FARMaster::PlanningCallBack, this);

    /* init Dynamic Planner Processing Objects */
    contour_detector_.Init(cdetect_params_);
    graph_manager_.Init(nh, graph_params_);     //  初始化terrain_grid
    graph_planner_.Init(nh, gp_params_);        //  初始化free_terrain_grid
    contour_graph_.Init(cg_params_);
    planner_viz_.Init(nh);
    map_handler_.Init(map_params_);             //  最大的gird pointcloud_grid 生成 world_obs_grid 和 world_free_grid  还有额外生成一个 terrain_height_grid
    scan_handler_.Init(scan_params_);           //  voxel_grid
    graph_msger_.Init(nh, msger_parmas_);
    //global_grid_.Init(nh, output_grid_params_); //自己写的部分

    /* init internal params */
    odom_node_ptr_ = NULL;
    is_cloud_init_ = false;
    is_odom_init_ = false;
    is_scan_init_ = false;
    is_planner_running_ = false;
    is_graph_init_ = false;
    is_reset_env_ = false;
    is_stop_update_ = false;
    is_output = false;

    // allocate memory to pointers
    new_vertices_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    temp_obs_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    temp_free_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    temp_cloud_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    scan_grid_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    local_terrain_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    terrain_height_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    viewpoint_around_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    kdtree_viewpoint_obs_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());

    // set kdtree sorted value
    FARUtil::kdtree_new_cloud_->setSortedResults(false);
    FARUtil::kdtree_filter_cloud_->setSortedResults(false);
    kdtree_viewpoint_obs_cloud_->setSortedResults(false);

    // init global utility cloud
    FARUtil::stack_new_cloud_->clear();
    FARUtil::stack_dyobs_cloud_->clear();

    // init TF listener
    tf_listener_ = new tf::TransformListener();

    // clear temp vectors and memory
    this->ClearTempMemory();
    FARUtil::robot_pos = Point3D(0, 0, 0);
    FARUtil::free_odom_p = Point3D(0, 0, 0);

    robot_pos_ = Point3D(0, 0, 0);
    nav_heading_ = Point3D(0, 0, 0);            //计算point_way的
    goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;
    printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
    std::cout << std::endl;
    if (master_params_.is_static_env)
    {
        std::cout << "\033[1;33m **************** STATIC ENV PLANNING **************** \033[0m\n"
                  << std::endl;
    }
    else
    {
        std::cout << "\033[1;33m **************** DYNAMIC ENV PLANNING **************** \033[0m\n"
                  << std::endl;
    }
    std::cout << "\n"
              << std::endl;
}

void FARMaster::ResetEnvironmentAndGraph()
{
    this->ResetInternalValues();
    if (!FARUtil::IsDebug)
    { // Terminal Output
        printf("\033[A"), printf("\033[A"), printf("\033[2K");
        std::cout << "\033[1;31m V-Graph Resetting...\033[0m\n"
                  << std::endl;
    }
    graph_manager_.ResetCurrentGraph();
    map_handler_.ResetGripMapCloud();
    graph_planner_.ResetPlannerInternalValues();
    contour_graph_.ResetCurrentContour();
    /* Reset clouds */
    FARUtil::surround_obs_cloud_->clear();
    FARUtil::surround_free_cloud_->clear();
    FARUtil::stack_new_cloud_->clear();
    FARUtil::stack_dyobs_cloud_->clear();
    FARUtil::cur_new_cloud_->clear();
    FARUtil::cur_dyobs_cloud_->clear();
    /* Stop the robot if it is moving */
    goal_waypoint_stamped_.header.stamp = ros::Time::now();
    goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
    goal_pub_.publish(goal_waypoint_stamped_);
    NodePtrStack empty_path;
    planner_viz_.VizPath(empty_path);
}

void FARMaster::Loop()
{
    ros::Rate loop_rate(master_params_.main_run_freq);
    while (ros::ok())
    {
        if (is_reset_env_)
        {
            this->ResetEnvironmentAndGraph();
            is_reset_env_ = false;
            if (FARUtil::IsDebug)
                ROS_WARN("****************** Graph and Env Reset ******************");
            loop_rate.sleep(); // skip this iteration
            continue;
        }
        /* Process callback functions */
        ros::spinOnce();
        if (!this->PreconditionCheck())
        {
            loop_rate.sleep();
            continue;
        }
        
        /* add main process after this line */
        graph_manager_.UpdateRobotPosition(robot_pos_); // 把odom加入到globalGraphNodes_里
        odom_node_ptr_ = graph_manager_.GetOdomNode();
        if (odom_node_ptr_ == NULL)
        {
            ROS_WARN("FAR: Waiting for Odometry...");
            loop_rate.sleep();
            continue;
        }
        /* Extract Vertices and new nodes */
        FARUtil::Timer.start_time("Total V-Graph Update");
        //! Algorithm 1: Polygon Extraction
        //! surround_obs_cloud是从TerrainCallBack来的
        contour_detector_.BuildTerrainImgAndExtractContour(odom_node_ptr_, FARUtil::surround_obs_cloud_, realworld_contour_);
        //? 得到refined_cotours和 realWorld_contours
        // realWorld_contours 存的是好几组点（[（x1,y1,z1）,(x2,y2,z2)...],[(x1,y1,z1),(x2,y2,z2)...]...）
        //也可以这么理解 （poly1, poly2, poly3）  而且这个是由obs_cloud得到的
        contour_graph_.UpdateContourGraph(odom_node_ptr_, realworld_contour_);
        //! 得到一个global_odom_p , 从refined_contours得到contour的nodes  ->  得到contour_graph_
        //! 得到每一个ctnode 的 dir<first,second> ；得到每个ctnode的freedirect属性 CONVEX CONCAVE PILLAR UNKOWN
        //?   主要填充CTNode结构体里的Point3D position      ;    NodeFreeDirect free_direct;
        //?   PointPair surf_dirs;PolygonPtr poly_ptr     ;    std::shared_ptr<CTNode> front
        //?   std::shared_ptr<CTNode> back;

        if (is_graph_init_)
        {
            if (!FARUtil::IsDebug)
                printf("\033[2K");
            std::cout << "    "
                      << "Local V-Graph Updated. Number of local vertices: " << ContourGraph::contour_graph_.size() << std::endl;
        }

        //! 第一次loop运行到这里是没有nav_graph_的,nav_node_id也都是0
        /* Adjust heights with terrain */       //还原node里的z坐标
        map_handler_.AdjustCTNodeHeight(ContourGraph::contour_graph_);  //处理ctnode_ptr->is_ground_associate的标识位
        map_handler_.AdjustNodesHeight(nav_graph_); //? 往后看 nav_graph就是一个global的地图的graph，它的更新依赖于globalGraphNodes_

        // Truncate for local range nodes
        // 把global_graph_node里分离出离odom一定范围内的nodes，而且分成了好几种不同程度的范围
        // 这些都是分割出去的范围都属于InternalStack
        /*
            第一次运行到这里，已经有了ctnode组成的contour_graph_ , 且odom已经push进globalGraphNodes_
        */
        graph_manager_.UpdateGlobalNearNodes();
        near_nav_graph_ = graph_manager_.GetExtendLocalNode(); //把extend_match_nodes_给返回出来给near_nav_graph_

        // Match near nav nodes with contour
        contour_graph_.MatchContourWithNavGraph(nav_graph_, near_nav_graph_, new_ctnodes_); //  从near_nav_graph里匹配，没有匹配到的就是new_ctnodes
        //! 从这里开始，匹配到到ctnode里的nav_node_id就是nav_node的id了。
        //! 从contour_graph_里得到没有global_match到的new_convex_vertices(也就是new_ctnodes_)

        if (master_params_.is_visual_opencv)
        {
            FARUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);  //画new_ctnode的点
            cv::Mat cloud_img = contour_detector_.GetCloudImgMat();
            contour_detector_.ShowCornerImage(cloud_img, new_vertices_ptr_);
        }

        

        //! Algorithm 2: Dynamic V-graph Update
        /* update planner graph */
        new_nodes_.clear();
        //!   get new_nodes_
        if (!is_stop_update_ && graph_manager_.ExtractGraphNodes(new_ctnodes_))         //  ctnode 转变为 nav_node，而ctnode变成该nav_node下的一个属性
        {
            //! 在执行  ExtractGraphNodes(new_ctnodes_)  的时候，就会得到new_nodes_ 并且判断它是否为NULL
            //返回new_nodes_ 而且这些new_nodes_（nav_node）里对应的navnode的属性只有一部分被设置好
            new_nodes_ = graph_manager_.GetNewNodes();
            //这个new_nodes_是一个 自身odom（如果需要的话） 和 新的contour的ctnode变化而来的一个新的nav_node
        }
        if (is_graph_init_)
        {
            if (!FARUtil::IsDebug)
                printf("\033[2K");
            std::cout << "    "
                      << "Number of new vertices adding to global V-Graph: " << new_nodes_.size() << std::endl;
        }

        //! 上面的函数让我得到了new_nodes_

        /* Graph Updating */
        //! 这个非常重要 通过上面的new_nodes_去更新global_graph_node
        //之前我们得到了new_nodes_ 这是很多navnodes 我们还要通过它们更新全局地图，更新他们navnode的属性

        graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_);
        runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_) / 1000.f; // Unit: second
        // runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_); // Unit: ms
        runtime_pub_.publish(runtimer_);
        /* Update v-graph in other modules */
        nav_graph_ = graph_manager_.GetNavGraph();
        if (is_graph_init_)
        {
            if (!FARUtil::IsDebug)
                printf("\033[2K");
            std::cout << "    "
                      << "Global V-Graph Updated. Number of global vertices: " << nav_graph_.size() << std::endl;
        }
        contour_graph_.ExtractGlobalContours();     // Global Polygon Update
        graph_planner_.UpdateVGraph(nav_graph_);    // Graph Planner Update             用来进行路径规划用的graph
        graph_msger_.UpdateGlobalGraph(nav_graph_); // Graph Messager Update

        /* Publish local boundary to lower level local planner */
        this->LocalBoundaryHandler(ContourGraph::local_boundary_);

        /* Viz Navigation Graph */
        const NavNodePtr last_internav_ptr = graph_manager_.GetLastInterNavNode();
        if (last_internav_ptr != NULL)
        {
            planner_viz_.VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
        }
        planner_viz_.VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
        planner_viz_.VizNodes(graph_manager_.GetOutContourNodes(), "out_contour", VizColor::YELLOW);
        planner_viz_.VizPoint3D(FARUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
        planner_viz_.VizGraph(nav_graph_);
        planner_viz_.VizContourGraph(ContourGraph::contour_graph_);
        planner_viz_.VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);

        if (is_graph_init_)
        {
            if (FARUtil::IsDebug)
            {
                std::cout << " ========================================================== " << std::endl;
            }
            else
            { // cleanup outputs in terminal
                for (int i = 0; i < 6; i++)
                {
                    printf("\033[A");
                }
            }
        }

        if (!is_graph_init_ && !nav_graph_.empty())
        {
            is_graph_init_ = true;
            printf("\033[A"), printf("\033[A"), printf("\033[2K");
            std::cout << "\033[1;32m V-Graph Initialized \033[0m\n"
                      << std::endl;
        }
        // if (master_params_.is_out_grid){
        //     Point3D odom_pos = global_grid_.UpdatePos(FARUtil::odom_pos);
        //     PointCloudPtr grid_obs(new pcl::PointCloud<PCLPoint>());
        //     PointCloudPtr grid_free(new pcl::PointCloud<PCLPoint>());
        //     map_handler_.GetCloudOfPoint(odom_pos,grid_obs,CloudType::OBS_CLOUD,true);
        //     map_handler_.GetCloudOfPoint(odom_pos,grid_free,CloudType::FREE_CLOUD,true);
        //     global_grid_.UpdateGlobalGraph(odom_pos,grid_obs,grid_free);
        //     if(is_output){
        //         global_grid_.OutputGlobalGrid();
        //         is_output = false;
        //     }
        // }
        loop_rate.sleep();
    }
}

void FARMaster::PlanningCallBack(const ros::TimerEvent &event)
{
    if (!is_graph_init_)        //只有收到了goal的信号，才会继续进行
        return;
    const NavNodePtr goal_ptr = graph_planner_.GetGoalNodePtr();
    if (goal_ptr == NULL)
    {
        /* Graph Traversablity Update */
        if (!FARUtil::IsDebug)
            printf("\033[2K");
        std::cout << "    "
                  << "Adding Goal to V-Graph "
                  << "Time: " << 0.f << "ms" << std::endl;
        graph_planner_.UpdateGraphTraverability(odom_node_ptr_, NULL);  //就算goal为空，我们也会更新分数fgsocre和gscore 这里用的是广度优先
        if (!FARUtil::IsDebug)
            printf("\033[2K");
        std::cout << "    "
                  << "Path Search "
                  << "Time: " << 0.f << "ms" << std::endl;
    }
    else
    {
        // Update goal postion with nearby terrain cloud   通过从world cloud obs 里抽取以goal为中心的free_terrain_grid的obs和free，并更新free_terrain_grid
        const Point3D ori_p = graph_planner_.GetOriginNodePos(true);      //得到z值修改过的goal 纯3d的点
        PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
        PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
        map_handler_.GetCloudOfPoint(ori_p, goal_obs, CloudType::OBS_CLOUD, true);      //在world_obs_cloud里把global为中心的sub的3x3x5范围内的点云提取出来(如果goal没有被发现，那么周围的点云就是init)
        map_handler_.GetCloudOfPoint(ori_p, goal_free, CloudType::FREE_CLOUD, true);    //同上
        graph_planner_.UpdateFreeTerrainGrid(ori_p, goal_obs, goal_free);               //用goal_obs和goal_free来更新27x27的grid
        graph_planner_.ReEvaluateGoalPosition(goal_ptr, !master_params_.is_multi_layer);    //调整goal_ptr的坐标

        // Adding goal into v-graph   把goal加入到当前的graph中，并更新goal和当前graph中node的边关系
        FARUtil::Timer.start_time("Adding Goal to V-Graph");
        graph_planner_.UpdateGoalNavNodeConnects(goal_ptr);     //更新goal和当前的nav_node的联系
        graph_planner_.UpdateVGraph(graph_manager_.GetNavGraph());
        if (!FARUtil::IsDebug)
            printf("\033[2K");
        FARUtil::Timer.end_time("Adding Goal to V-Graph");

        // Update v-graph traversibility            更新每个node的分数
        FARUtil::Timer.start_time("Path Search");
        graph_planner_.UpdateGraphTraverability(odom_node_ptr_, goal_ptr);

        // Construct path to goal and publish waypoint
        NodePtrStack global_path;
        Point3D current_free_goal;
        NavNodePtr last_nav_ptr = nav_node_ptr_;    //更新上一步走的node
        bool is_planning_fails = false;
        goal_waypoint_stamped_.header.stamp = ros::Time::now();
        bool is_current_free_nav = false;
        bool is_reach_goal = false;
        Point3D waypoint;
        // 输入终点坐标，输出调整过的终点坐标，路径，还有一个导航点nav_node_ptr_
        if (graph_planner_.PathToGoal(goal_ptr, global_path, nav_node_ptr_, current_free_goal, is_planning_fails, is_reach_goal, is_current_free_nav) && nav_node_ptr_ != NULL)
        {
            waypoint = nav_node_ptr_->position;
            // 如果导航点不是终点的话
            if (nav_node_ptr_ != goal_ptr)
            {
                waypoint = this->ProjectNavWaypoint(nav_node_ptr_, last_nav_ptr);   //  调整waypoint的位置
            }
            else if (master_params_.is_viewpoint_extend)
            {
                planner_viz_.VizViewpointExtend(goal_ptr, goal_ptr->position);
            }
            goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(waypoint);
            goal_pub_.publish(goal_waypoint_stamped_);  //  发送waypoint
            is_planner_running_ = true;
            planner_viz_.VizPoint3D(waypoint, "waypoint", VizColor::MAGNA, 1.5);
            planner_viz_.VizPoint3D(current_free_goal, "free_goal", VizColor::GREEN, 1.5);
            planner_viz_.VizPath(global_path, is_current_free_nav);
        }
        else if (is_planner_running_)
        {
            // stop robot
            global_path.clear();
            planner_viz_.VizPath(global_path);
            is_planner_running_ = false;
            nav_heading_ = Point3D(0, 0, 0);
            if (is_planning_fails)
            { // stops the robot
                goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
                goal_pub_.publish(goal_waypoint_stamped_);
            }
        }
        if (!FARUtil::IsDebug)
            printf("\033[2K");

        // publish planner status and timers
        std_msgs::Bool reach_goal_msg;
        reach_goal_msg.data = is_reach_goal;
        reach_goal_pub_.publish(reach_goal_msg);
        std_msgs::Float32 traverse_timer;
        traverse_timer.data = FARUtil::Timer.record_time("Overall_executing");
        traverse_time_pub_.publish(traverse_timer);
        if (is_reach_goal)
        {
            FARUtil::Timer.end_time("Overall_executing", false);
        }
        plan_timer_.data = FARUtil::Timer.end_time("Path Search");
        planning_time_pub_.publish(plan_timer_);
        //记录waypoint、odom、odom->connect_node
        // if(is_record){
        //     std::fstream file("/home/liqunzhao/far_planner/use_debug/record.txt",std::ios::app);
        //     //记录waypoint
        //     file<<(waypoint.x,waypoint.y,waypoint.z)<<' ';
        //     //记录odom
        //     file<<(odom_node_ptr_->position.x,odom_node_ptr_->position.y,odom_node_ptr_->position.z)<<' ';
        //     //记录connect_node
        //     for(auto const point : odom_node_ptr_->connect_nodes){
        //         file<<(point->position.x,point->position.y,point->position.z)<<' ';   
        //     }
        //     file<<"\n";
        //     if(is_reach_goal){
        //         file<<"done\n";
        //     }
        //     file.close();
        // }
    }
}

void FARMaster::LocalBoundaryHandler(const std::vector<PointPair> &local_boundary)
{
    if (!master_params_.is_pub_boundary || local_boundary.empty())
        return;
    geometry_msgs::PolygonStamped boundary_poly;
    boundary_poly.header.frame_id = master_params_.world_frame;
    boundary_poly.header.stamp = ros::Time::now();
    float index_z = robot_pos_.z;
    std::vector<PointPair> sorted_boundary;
    for (const auto &edge : local_boundary)
    {
        if (FARUtil::DistanceToLineSeg2D(robot_pos_, edge) > master_params_.local_planner_range)
            continue;
        sorted_boundary.push_back(edge);
    }
    FARUtil::SortEdgesClockWise(robot_pos_, sorted_boundary); /* For better rviz visualization purpose only! */
    for (const auto &edge : sorted_boundary)
    {
        geometry_msgs::Point32 geo_p1, geo_p2;
        geo_p1.x = edge.first.x, geo_p1.y = edge.first.y, geo_p1.z = index_z;
        geo_p2.x = edge.second.x, geo_p2.y = edge.second.y, geo_p2.z = index_z;
        boundary_poly.polygon.points.push_back(geo_p1), boundary_poly.polygon.points.push_back(geo_p2);
        index_z += 0.001f; // seperate polygon lines
    }
    boundary_pub_.publish(boundary_poly);
}

Point3D FARMaster::ProjectNavWaypoint(const NavNodePtr &nav_node_ptr, const NavNodePtr &last_point_ptr)
{
    bool is_momentum = false;
    if (last_point_ptr == nav_node_ptr || (last_point_ptr != NULL && (last_point_ptr->position - nav_node_ptr_->position).norm() < FARUtil::kNearDist))
    {
        is_momentum = true;
    }
    Point3D waypoint = nav_node_ptr->position;
    float free_dist = master_params_.local_planner_range;
    const Point3D extend_p = this->ExtendViewpointOnObsCloud(nav_node_ptr_, FARUtil::surround_obs_cloud_, free_dist);
    free_dist = std::max(free_dist, master_params_.robot_dim * 2.5f);
    if (master_params_.is_viewpoint_extend)
    {
        waypoint = extend_p;
        planner_viz_.VizViewpointExtend(nav_node_ptr_, waypoint);
    }
    const Point3D diff_p = waypoint - robot_pos_;
    Point3D new_heading;    //运动的方向（是一个单位向量）
    if (is_momentum && nav_heading_.norm() > FARUtil::kEpsilon)
    {
        const float hdist = free_dist / 2.0f;
        const float ratio = std::min(hdist, diff_p.norm()) / hdist;
        new_heading = diff_p.normalize() * ratio + nav_heading_ * (1.0f - ratio);
    }
    else
    {
        new_heading = diff_p.normalize();
    }
    if (nav_heading_.norm() > FARUtil::kEpsilon && new_heading.norm_dot(nav_heading_) < 0.0f)
    { // negative direction reproject
        Point3D temp_heading(nav_heading_.y, -nav_heading_.x, nav_heading_.z);
        if (temp_heading.norm_dot(new_heading) < 0.0f)
        {
            temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
        }
        new_heading = temp_heading;
    }
    nav_heading_ = new_heading.normalize();
    if (diff_p.norm() < free_dist)
    {
        waypoint = waypoint + nav_heading_ * (free_dist - diff_p.norm());
    }
    return waypoint;
}

Point3D FARMaster::ExtendViewpointOnObsCloud(const NavNodePtr &nav_node_ptr, const PointCloudPtr &obsCloudIn, float &free_dist)
{
    if (nav_node_ptr_->free_direct != NodeFreeDirect::CONVEX || obsCloudIn->empty())
        return nav_node_ptr_->position;
    FARUtil::CropPCLCloud(obsCloudIn, viewpoint_around_ptr_, nav_node_ptr_->position, free_dist + FARUtil::kNearDist);
    float maxR = std::min((nav_node_ptr_->position - robot_pos_).norm(), free_dist) - FARUtil::kNearDist;
    maxR = std::max(maxR, 0.0f);
    bool is_wall = false;
    const Point3D direct = -FARUtil::SurfTopoDirect(nav_node_ptr_->surf_dirs, is_wall);
    if (!is_wall)
    {
        Point3D waypoint = nav_node_ptr_->position;
        if (viewpoint_around_ptr_->empty())
        {
            waypoint = waypoint + direct * maxR;
        }
        else
        {
            kdtree_viewpoint_obs_cloud_->setInputCloud(viewpoint_around_ptr_);
            const int N_Thred = (int)std::floor(FARUtil::kNearDist / FARUtil::kLeafSize);
            const float R = FARUtil::kNearDist / 2.0f + FARUtil::kLeafSize;
            // ray tracing
            Point3D start_p = waypoint + direct * FARUtil::kNearDist;   //把该nav_node的坐标按小车的大小做一下修改，保证小车离该nav_node距离足够
            float ray_dist = FARUtil::kNearDist;
            bool is_occupied = FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_) > N_Thred; //判断该点有没有被障碍物占据
            waypoint = start_p; //往下就是判断这个反向方向向量延伸的点有没有穿过别的障碍物，它用的是cloudpoint来判断的，不是几何来判断的
            while (!is_occupied && ray_dist < free_dist)
            {
                start_p = start_p + direct * FARUtil::kNearDist;
                ray_dist += FARUtil::kNearDist;
                is_occupied = FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_) > N_Thred;
                if (ray_dist < maxR)
                {
                    waypoint = start_p;
                }
            }
            if (is_occupied)
            {
                waypoint = (nav_node_ptr_->position + waypoint - direct * FARUtil::kNearDist) / 2.0f;
                waypoint.z = nav_node_ptr_->position.z;
                free_dist = ray_dist - FARUtil::kNearDist;
            }
            return waypoint;
        }
    }
    return nav_node_ptr_->position;
}

void FARMaster::LoadROSParams()
{
    const std::string master_prefix = "/far_planner/";
    const std::string map_prefix = master_prefix + "MapHandler/";
    const std::string scan_prefix = master_prefix + "ScanHandler/";
    const std::string cdetect_prefix = master_prefix + "CDetector/";
    const std::string graph_prefix = master_prefix + "Graph/";
    const std::string viz_prefix = master_prefix + "Viz/";
    const std::string utility_prefix = master_prefix + "Util/";
    const std::string planner_prefix = master_prefix + "GPlanner/";
    const std::string contour_prefix = master_prefix + "ContourGraph/";
    const std::string msger_prefix = master_prefix + "GraphMsger/";
    const std::string op_grid_prefix = master_prefix + "GGrid/";

    // master params
    nh.param<float>(master_prefix + "main_run_freq", master_params_.main_run_freq, 5.0);
    nh.param<float>(master_prefix + "voxel_dim", master_params_.voxel_dim, 0.2);
    nh.param<float>(master_prefix + "robot_dim", master_params_.robot_dim, 0.8);
    nh.param<float>(master_prefix + "vehicle_height", master_params_.vehicle_height, 0.75);
    nh.param<float>(master_prefix + "sensor_range", master_params_.sensor_range, 50.0);
    nh.param<float>(master_prefix + "terrain_range", master_params_.terrain_range, 15.0);
    nh.param<float>(master_prefix + "local_planner_range", master_params_.local_planner_range, 5.0);
    nh.param<float>(master_prefix + "visualize_ratio", master_params_.viz_ratio, 1.0);
    nh.param<bool>(master_prefix + "is_viewpoint_extend", master_params_.is_viewpoint_extend, true);
    nh.param<bool>(master_prefix + "is_multi_layer", master_params_.is_multi_layer, false);
    nh.param<bool>(master_prefix + "is_opencv_visual", master_params_.is_visual_opencv, true);
    nh.param<bool>(master_prefix + "is_static_env", master_params_.is_static_env, true);
    nh.param<bool>(master_prefix + "is_pub_boundary", master_params_.is_pub_boundary, true);
    nh.param<bool>(master_prefix + "is_debug_output", master_params_.is_debug_output, false);
    nh.param<bool>(master_prefix + "is_attempt_autoswitch", master_params_.is_attempt_autoswitch, true);
    nh.param<bool>(master_prefix + "is_out_grid",master_params_.is_out_grid, true);
    nh.param<std::string>(master_prefix + "world_frame", master_params_.world_frame, "map");
    master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range);

    // map handler params
    nh.param<float>(map_prefix + "floor_height", map_params_.floor_height, 2.0);
    nh.param<float>(map_prefix + "cell_length", map_params_.cell_length, 5.0);
    nh.param<float>(map_prefix + "map_grid_max_length", map_params_.grid_max_length, 5000.0);
    nh.param<float>(map_prefix + "map_grad_max_height", map_params_.grid_max_height, 100.0);
    map_params_.height_voxel_dim = master_params_.voxel_dim * 2.0f;
    map_params_.cell_height = map_params_.floor_height / 2.5f;
    map_params_.sensor_range = master_params_.sensor_range;

    // utility params
    nh.param<float>(utility_prefix + "angle_noise", FARUtil::kAngleNoise, 15.0);
    nh.param<float>(utility_prefix + "accept_max_align_angle", FARUtil::kAcceptAlign, 15.0);
    nh.param<float>(utility_prefix + "new_intensity_thred", FARUtil::kNewPIThred, 2.0);
    nh.param<float>(utility_prefix + "nav_clear_dist", FARUtil::kNavClearDist, 0.5);
    nh.param<float>(utility_prefix + "terrain_free_Z", FARUtil::kFreeZ, 0.1);
    nh.param<int>(utility_prefix + "dyosb_update_thred", FARUtil::kDyObsThred, 4);
    nh.param<int>(utility_prefix + "new_point_counter", FARUtil::KNewPointC, 10);
    nh.param<float>(utility_prefix + "dynamic_obs_dacay_time", FARUtil::kObsDecayTime, 10.0);
    nh.param<float>(utility_prefix + "new_points_decay_time", FARUtil::kNewDecayTime, 2.0);
    nh.param<int>(utility_prefix + "obs_inflate_size", FARUtil::kObsInflate, 2);
    FARUtil::kLeafSize = master_params_.voxel_dim;
    FARUtil::kNearDist = master_params_.robot_dim;                                      //0.8
    FARUtil::kHeightVoxel = map_params_.height_voxel_dim;
    FARUtil::kMatchDist = master_params_.robot_dim * 2.0f + FARUtil::kLeafSize;         //1.75
    FARUtil::kNavClearDist = master_params_.robot_dim / 2.0f + FARUtil::kLeafSize;      //0.55
    FARUtil::kProjectDist = master_params_.voxel_dim;
    FARUtil::worldFrameId = master_params_.world_frame;
    FARUtil::kVizRatio = master_params_.viz_ratio;
    FARUtil::kTolerZ = map_params_.floor_height - FARUtil::kHeightVoxel;
    FARUtil::kCellLength = map_params_.cell_length;
    FARUtil::kCellHeight = map_params_.cell_height;
    FARUtil::kAcceptAlign = FARUtil::kAcceptAlign / 180.0f * M_PI;
    FARUtil::kAngleNoise = FARUtil::kAngleNoise / 180.0f * M_PI;
    FARUtil::robot_dim = master_params_.robot_dim;
    FARUtil::IsStaticEnv = master_params_.is_static_env;
    FARUtil::IsDebug = master_params_.is_debug_output;
    FARUtil::IsMultiLayer = master_params_.is_multi_layer;
    FARUtil::vehicle_height = master_params_.vehicle_height;
    FARUtil::kSensorRange = master_params_.sensor_range;
    FARUtil::kMarginDist = master_params_.sensor_range - FARUtil::kMatchDist;
    FARUtil::kMarginHeight = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f;
    FARUtil::kTerrainRange = master_params_.terrain_range;
    FARUtil::kLocalPlanRange = master_params_.local_planner_range;

    // graph planner params
    nh.param<float>(planner_prefix + "converge_distance", gp_params_.converge_dist, 1.0);
    nh.param<float>(planner_prefix + "goal_adjust_radius", gp_params_.adjust_radius, 10.0);
    nh.param<int>(planner_prefix + "free_counter_thred", gp_params_.free_thred, 5);
    nh.param<int>(planner_prefix + "reach_goal_vote_size", gp_params_.votes_size, 5);
    nh.param<int>(planner_prefix + "path_momentum_thred", gp_params_.momentum_thred, 5);
    gp_params_.momentum_dist = master_params_.robot_dim / 2.0f;
    gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;

    // contour graph params
    cg_params_.kPillarPerimeter = master_params_.robot_dim * 4.0f;

    // dynamic graph params
    nh.param<int>(graph_prefix + "connect_votes_size", graph_params_.votes_size, 10);
    nh.param<int>(graph_prefix + "clear_dumper_thred", graph_params_.dumper_thred, 3);
    nh.param<int>(graph_prefix + "node_finalize_thred", graph_params_.finalize_thred, 3);
    nh.param<int>(graph_prefix + "filter_pool_size", graph_params_.pool_size, 12);
    nh.param<float>(graph_prefix + "connect_angle_thred", graph_params_.kConnectAngleThred, 10.0);
    nh.param<float>(graph_prefix + "dirs_filter_margin", graph_params_.filter_dirs_margin, 10.0);
    graph_params_.filter_pos_margin = FARUtil::kNavClearDist;
    graph_params_.filter_dirs_margin = FARUtil::kAngleNoise;
    graph_params_.kConnectAngleThred = FARUtil::kAcceptAlign;
    graph_params_.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;

    // graph messager params
    nh.param<int>(msger_prefix + "robot_id", msger_parmas_.robot_id, 0);
    msger_parmas_.frame_id = master_params_.world_frame;
    msger_parmas_.votes_size = graph_params_.votes_size;
    msger_parmas_.pool_size = graph_params_.pool_size;
    msger_parmas_.dist_margin = graph_params_.filter_pos_margin;

    // scan handler params
    scan_params_.terrain_range = master_params_.terrain_range;
    scan_params_.voxel_size = master_params_.voxel_dim;
    scan_params_.ceil_height = map_params_.floor_height;

    // contour detector params
    nh.param<float>(cdetect_prefix + "resize_ratio", cdetect_params_.kRatio, 5.0);
    nh.param<int>(cdetect_prefix + "filter_count_value", cdetect_params_.kThredValue, 5);
    nh.param<bool>(cdetect_prefix + "is_save_img", cdetect_params_.is_save_img, false);
    nh.param<std::string>(cdetect_prefix + "img_folder_path", cdetect_params_.img_path, "");
    cdetect_params_.kBlurSize = (int)std::round(FARUtil::kNavClearDist / master_params_.voxel_dim);
    cdetect_params_.sensor_range = master_params_.sensor_range;
    cdetect_params_.voxel_dim = master_params_.voxel_dim;

    // output grid params
    nh.param<float>(op_grid_prefix + "cell_length", output_grid_params_.cell_length, 0.15);
    nh.param<float>(op_grid_prefix + "grid_max_length", output_grid_params_.grid_max_length,1000);
    nh.param<float>(op_grid_prefix + "origin_x", output_grid_params_.origin_x,-10);
    nh.param<float>(op_grid_prefix + "origin_y", output_grid_params_.origin_y,-10);
}

void FARMaster::OdomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    // transform from odom frame to mapping frame
    std::string odom_frame = msg->header.frame_id;
    tf::Pose tf_odom_pose;
    tf::poseMsgToTF(msg->pose.pose, tf_odom_pose);
    if (!FARUtil::IsSameFrameID(odom_frame, master_params_.world_frame))
    {
        if (FARUtil::IsDebug)
            ROS_WARN_ONCE("FARMaster: odom frame does NOT match with world frame!");
        tf::StampedTransform odom_to_world_tf_stamp;
        try
        {
            tf_listener_->waitForTransform(master_params_.world_frame, odom_frame, ros::Time(0), ros::Duration(1.0));
            tf_listener_->lookupTransform(master_params_.world_frame, odom_frame, ros::Time(0), odom_to_world_tf_stamp);
            tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Tracking odom TF lookup: %s", ex.what());
            return;
        }
    }
    robot_pos_.x = tf_odom_pose.getOrigin().getX();
    robot_pos_.y = tf_odom_pose.getOrigin().getY();
    robot_pos_.z = tf_odom_pose.getOrigin().getZ();
    // extract robot heading
    FARUtil::robot_pos = robot_pos_;
    double roll, pitch, yaw;
    tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);   // yaw 绕y轴旋转，三维空间的右手笛卡尔坐标
    robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);

    if (!is_odom_init_)
    {
        // system start time
        FARUtil::systemStartTime = ros::Time::now().toSec();
        FARUtil::map_origin = robot_pos_;
        map_handler_.UpdateRobotPosition(robot_pos_);
    }

    is_odom_init_ = true;
}

void FARMaster::PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr &pc,
                              const PointCloudPtr &cloudOut)
{

    pcl::PointCloud<PCLPoint> temp_cloud;
    pcl::fromROSMsg(*pc, temp_cloud);       //把msg里的点云信息传递给临时的pcl点云数组
    cloudOut->clear(), *cloudOut = temp_cloud;  //把目标指针指向临时的pcl点云数组
    if (cloudOut->empty())
        return;
    FARUtil::FilterCloud(cloudOut, master_params_.voxel_dim);
    // transform cloud frame
    std::string cloud_frame = pc->header.frame_id;
    FARUtil::RemoveNanInfPoints(cloudOut);
    if (!FARUtil::IsSameFrameID(cloud_frame, master_params_.world_frame))
    {
        if (FARUtil::IsDebug)
            ROS_WARN_ONCE("FARMaster: cloud frame does NOT match with world frame!");
        try
        {
            FARUtil::TransformPCLFrame(cloud_frame,
                                       master_params_.world_frame,
                                       tf_listener_,
                                       cloudOut);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Tracking cloud TF lookup: %s", ex.what());
            return;
        }
    }
}

void FARMaster::ScanCallBack(const sensor_msgs::PointCloud2ConstPtr &scan_pc)   //产生存放动态障碍物的cur_scan_cloud
{
    if (master_params_.is_static_env || !is_odom_init_)
        return;
    this->PrcocessCloud(scan_pc, FARUtil::cur_scan_cloud_);
    scan_handler_.UpdateRobotPosition(robot_pos_);
}

void FARMaster::TerrainLocalCallBack(const sensor_msgs::PointCloud2ConstPtr &pc)
{
    if (master_params_.is_static_env)
        return;
    this->PrcocessCloud(pc, local_terrain_ptr_);
    FARUtil::ExtractFreeAndObsCloud(local_terrain_ptr_, FARUtil::local_terrain_free_, FARUtil::local_terrain_obs_);
}
//生成grid，初始化obs_grid和free_grid 两个开始内容基本上都是一样的
void FARMaster::TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr &pc)
{
    if (!is_odom_init_)
        return;
    // update map grid robot center
    FARUtil::Timer.start_time("laser process");
    map_handler_.UpdateRobotPosition(FARUtil::robot_pos);
    if (!is_stop_update_)
    {
        this->PrcocessCloud(pc, temp_cloud_ptr_);
        FARUtil::CropBoxCloud(temp_cloud_ptr_, robot_pos_, Point3D(master_params_.terrain_range, master_params_.terrain_range, FARUtil::kTolerZ));
        FARUtil::ExtractFreeAndObsCloud(temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_);
        if (!master_params_.is_static_env)
        {
            FARUtil::RemoveOverlapCloud(temp_obs_ptr_, FARUtil::stack_dyobs_cloud_, true);
        }
        // 这里要用dilate grid
        map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
        map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
        // extract new points
        // 得到cur_new_cloud_
        FARUtil::ExtractNewObsPointCloud(temp_obs_ptr_,
                                         FARUtil::surround_obs_cloud_,
                                         FARUtil::cur_new_cloud_);
    }
    else
    { // stop env update
        temp_cloud_ptr_->clear();
        FARUtil::cur_new_cloud_->clear();
    }
    // extract surround free cloud & update terrain height
    map_handler_.GetSurroundFreeCloud(FARUtil::surround_free_cloud_);//在neighbour范围内的temp_free_cloud   -》surround_free_cloud_
    map_handler_.UpdateTerrainHeightGrid(FARUtil::surround_free_cloud_, terrain_height_ptr_);   //更新89x89x1的terrain 由surround_free_cloud_去更新terrain_height_ptr_
    // update surround obs cloud
    map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_); //直接从temp_obs_cloud转化来
    // extract dynamic obstacles
    FARUtil::cur_dyobs_cloud_->clear();
    if (!master_params_.is_static_env && !is_stop_update_)
    {
        this->ExtractDynamicObsFromScan(FARUtil::cur_scan_cloud_,
                                        FARUtil::surround_obs_cloud_,
                                        FARUtil::surround_free_cloud_,
                                        FARUtil::cur_dyobs_cloud_);
        if (FARUtil::cur_dyobs_cloud_->size() > FARUtil::kDyObsThred)
        {
            if (FARUtil::IsDebug)
                ROS_WARN("FARMaster: dynamic obstacle detected, size: %ld", FARUtil::cur_dyobs_cloud_->size());
            FARUtil::InflateCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim, 1, true);
            map_handler_.RemoveObsCloudFromGrid(FARUtil::cur_dyobs_cloud_);
            FARUtil::RemoveOverlapCloud(FARUtil::surround_obs_cloud_, FARUtil::cur_dyobs_cloud_);
            FARUtil::FilterCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim);
            // update new cloud
            *FARUtil::cur_new_cloud_ += *FARUtil::cur_dyobs_cloud_;
            FARUtil::FilterCloud(FARUtil::cur_new_cloud_, master_params_.voxel_dim);
        }
        // update world dynamic obstacles
        FARUtil::StackCloudByTime(FARUtil::cur_dyobs_cloud_, FARUtil::stack_dyobs_cloud_, FARUtil::kObsDecayTime);
    }

    // create and update kdtrees
    FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_, FARUtil::stack_new_cloud_, FARUtil::kNewDecayTime);
    //更新扫描出来的新obscloud，通过kdtree给他一个入口进行快速的搜索操作。
    FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);

    if (!FARUtil::surround_obs_cloud_->empty())
        is_cloud_init_ = true;

    /* visualize clouds */
    planner_viz_.VizPointCloud(new_PCL_pub_, FARUtil::stack_new_cloud_);
    planner_viz_.VizPointCloud(dynamic_obs_pub_, FARUtil::cur_dyobs_cloud_);
    planner_viz_.VizPointCloud(surround_free_debug_, FARUtil::surround_free_cloud_);
    planner_viz_.VizPointCloud(surround_obs_debug_, FARUtil::surround_obs_cloud_);
    planner_viz_.VizPointCloud(terrain_height_pub_, terrain_height_ptr_);
    // visualize map grid
    PointStack neighbor_centers, occupancy_centers;
    map_handler_.GetNeighborCeilsCenters(neighbor_centers);
    map_handler_.GetOccupancyCeilsCenters(occupancy_centers);
    planner_viz_.VizMapGrids(neighbor_centers, occupancy_centers, map_params_.cell_length, map_params_.cell_height);
    // DBBUG visual raycast grids
    if (!master_params_.is_static_env)
    {
        scan_handler_.GridVisualCloud(scan_grid_ptr_, GridStatus::RAY);
        planner_viz_.VizPointCloud(scan_grid_debug_, scan_grid_ptr_);
    }
    laser_timer_.data = FARUtil::Timer.end_time("laser process");
    laser_timer_pub_.publish(laser_timer_);
}

void FARMaster::ExtractDynamicObsFromScan(const PointCloudPtr &scanCloudIn,
                                          const PointCloudPtr &obsCloudIn,
                                          const PointCloudPtr &freeCloudIn,
                                          const PointCloudPtr &dyObsCloudOut)
{
    scan_handler_.ReInitGrids();
    scan_handler_.SetCurrentScanCloud(scanCloudIn, freeCloudIn);
    scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut);
}

//得到goal point的时候，ros_spinonce就会跳转到这里
void FARMaster::WaypointCallBack(const geometry_msgs::PointStamped &route_goal)
{
    if (!is_graph_init_)
    {
        if (FARUtil::IsDebug)
            ROS_WARN("FARMaster: wait for v-graph to init before sending any goals");
        return;
    }
    Point3D goal_p(route_goal.point.x, route_goal.point.y, route_goal.point.z);
    const std::string goal_frame = route_goal.header.frame_id;
    if (!FARUtil::IsSameFrameID(goal_frame, master_params_.world_frame))
    {
        if (FARUtil::IsDebug)
            ROS_WARN_THROTTLE(1.0, "FARMaster: waypoint published is not on world frame!");
        FARUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_listener_, goal_p);
    }
    graph_planner_.UpdateGoal(goal_p);
    FARUtil::Timer.start_time("Overall_executing", true);
    // visualize original goal
    planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
}


/* allocate static utility PointCloud pointer memory */
PointCloudPtr FARUtil::surround_obs_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::stack_new_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::cur_new_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::cur_dyobs_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::stack_dyobs_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::cur_scan_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::local_terrain_obs_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::local_terrain_free_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_new_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
/* init static utility values */
const float FARUtil::kEpsilon = 1e-7;
const float FARUtil::kINF = std::numeric_limits<float>::max();
std::string FARUtil::worldFrameId;
float FARUtil::kAngleNoise;
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float FARUtil::robot_dim;
float FARUtil::vehicle_height;
float FARUtil::kLeafSize;
float FARUtil::kHeightVoxel;
float FARUtil::kNavClearDist;
float FARUtil::kCellLength;
float FARUtil::kCellHeight;
float FARUtil::kNewPIThred;
float FARUtil::kSensorRange;
float FARUtil::kMarginDist;
float FARUtil::kMarginHeight;
float FARUtil::kTerrainRange;
float FARUtil::kLocalPlanRange;
float FARUtil::kFreeZ;
float FARUtil::kVizRatio;
double FARUtil::systemStartTime;
float FARUtil::kObsDecayTime;
float FARUtil::kNewDecayTime;
float FARUtil::kNearDist;
float FARUtil::kMatchDist;
float FARUtil::kProjectDist;
int FARUtil::kDyObsThred;
int FARUtil::KNewPointC;
int FARUtil::kObsInflate;
float FARUtil::kTolerZ;
float FARUtil::kAcceptAlign;
bool FARUtil::IsStaticEnv;
bool FARUtil::IsDebug;
bool FARUtil::IsMultiLayer;
TimeMeasure FARUtil::Timer;

/* Global Graph */
DynamicGraphParams DynamicGraph::dg_params_;
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t DynamicGraph::id_tracker_;
std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_;
// <Nav类型的node,<有效计数器,无序的Nav_node集合>>
std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> DynamicGraph::out_contour_nodes_map_;

/* init static contour graph values */
//points -> polygon -> ctnodes -> contour_graph
CTNodeStack ContourGraph::polys_ctnodes_;                       //存放每个polygon的第一个ctnode
CTNodeStack ContourGraph::contour_graph_;                       //存放由polygon转变为ctnodes的graph
PolygonStack ContourGraph::contour_polygons_;                   //储存每个polygon的一个graph
std::vector<PointPair> ContourGraph::global_contour_;
std::vector<PointPair> ContourGraph::unmatched_contour_;
std::vector<PointPair> ContourGraph::inactive_contour_;
std::vector<PointPair> ContourGraph::boundary_contour_;
std::vector<PointPair> ContourGraph::local_boundary_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::global_contour_set_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::boundary_contour_set_;

/* init terrain map values */
PointKdTreePtr MapHandler::kdtree_terrain_clould_;
std::vector<int> MapHandler::terrain_grid_occupy_list_;     //terrain_grid下探索过的点
std::vector<int> MapHandler::terrain_grid_traverse_list_;
std::unordered_set<int> MapHandler::neighbor_obs_indices_;
std::unordered_set<int> MapHandler::extend_obs_indices_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_free_cloud_grid_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_obs_cloud_grid_;
std::unique_ptr<grid_ns::Grid<std::vector<float>>> MapHandler::terrain_height_grid_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "far_planner_node");
    FARMaster dp_node;
    dp_node.Init();
    dp_node.Loop();
}