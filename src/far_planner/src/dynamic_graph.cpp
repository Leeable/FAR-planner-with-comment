/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/dynamic_graph.h"

/***************************************************************************************/

void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
    dg_params_ = params;
    //CONNECT_ANGLE_COS:0.997564
    CONNECT_ANGLE_COS = cos(dg_params_.kConnectAngleThred);
    //NOISE_ANGLE_COS:0.965926  
    NOISE_ANGLE_COS = cos(FARUtil::kAngleNoise);
    id_tracker_     = 1;
    last_connect_pos_ = Point3D(0,0,0);
    /* Initialize Terrian Planner */
    tp_params_.world_frame  = FARUtil::worldFrameId;
    tp_params_.voxel_size   = FARUtil::kLeafSize;           //  0.15
    tp_params_.radius       = FARUtil::kNearDist * 2.0f;    //  1.6
    tp_params_.inflate_size = FARUtil::kObsInflate;         //  1
    terrain_planner_.Init(nh, tp_params_);
}

void DynamicGraph::UpdateRobotPosition(const Point3D& robot_pos) {
    robot_pos_ = robot_pos;
    terrain_planner_.SetLocalTerrainObsCloud(FARUtil::local_terrain_obs_);//这个只有dynamic obs的时候才会用
    //odom_node_ptr_默认也是NULL 第一次就把robot_pos_更新到odom_node_ptr_ 然后odom_node_ptr_更新到graph
    if (odom_node_ptr_ == NULL) {
        this->CreateNavNodeFromPoint(robot_pos_, odom_node_ptr_, true);//is_odom为true
        this->AddNodeToGraph(odom_node_ptr_);       //把odom_node放到globalGraphNodes_里
        if (FARUtil::IsDebug) ROS_INFO("DG: Odom node has been initilaized.");
    } else {
        //第二次以后就一直走这个了
        this->UpdateNodePosition(odom_node_ptr_, robot_pos_);
    }
    FARUtil::odom_pos = odom_node_ptr_->position;
    terrain_planner_.VisualPaths();     //只有dynamic env才会有
}

// 判断是否需要设置导航点，返回true + last_connect_pos_ 或 返回true 或者 返回last_connect_pos_ + false
bool DynamicGraph::IsInterNavpointNecessary() {
    // cur_internav_ptr_ 在 UpdateGlobalNearNodes() 里会得到更新
    // 第一次运行 internav_near_nodes_为空，不更新cur_internav_ptr_ 所以第一次会走这个
    if (cur_internav_ptr_ == NULL ) { // create nearest nav point
        last_connect_pos_ = FARUtil::free_odom_p;
        return true;
    }
    //从odom的edge_vote里找到cur_internav_ptr_->id
    const auto it = odom_node_ptr_->edge_votes.find(cur_internav_ptr_->id);
    //如果is_bridge_internav_为true  或  it 指向迭代器end(没找到对应的id)  或  cur_internav_ptr_超出Internav的范围
    if (is_bridge_internav_ || it == odom_node_ptr_->edge_votes.end() || !this->IsInternavInRange(cur_internav_ptr_)) {
        float min_dist = FARUtil::kINF;
        //从internav_near_nodes里找每个internav_ptr ，算它和last_connect_pos的距离
        for (const auto& internav_ptr : internav_near_nodes_) {
            const float cur_dist = (internav_ptr->position - last_connect_pos_).norm();
            //更新最小距离
            if (cur_dist < min_dist) min_dist = cur_dist;
        }
        //循环更新以后会得到一个last_connect_pos和internav_ptr的最小距离
        //如果这个最小距离超过0.55 返回true 就需要一个inter_nav_point
        if (min_dist > FARUtil::kNavClearDist) return true;
    } 
    //free_odom和last_connect_pos距离如果超过0.8  或  （在odom_node的edge_vote找到了对应的id 且 second位的最后一个数值为1
    if ((FARUtil::free_odom_p - last_connect_pos_).norm() > FARUtil::kNearDist || 
        (it != odom_node_ptr_->edge_votes.end() && it->second.back() == 1)) 
    {
        //更新last_connect_pos
        last_connect_pos_ = FARUtil::free_odom_p;
    }
    return false;
}

bool DynamicGraph::ExtractGraphNodes(const CTNodeStack& new_ctnodes) {
    if (new_ctnodes.empty()) return false;
    NavNodePtr new_node_ptr = NULL;
    new_nodes_.clear();
    //检查是否需要导航点
    //第一步 先判断需不需要设置导航点(inter_navnode)，如果需要的话，从odom的last_connect里更新一下放到new_nodes里
    //第一次运行到这里，更新last_connect_position
    if (this->IsInterNavpointNecessary()) { // check wheter or not need inter navigation points
        if (FARUtil::IsDebug) ROS_INFO("DG: One trajectory node has been created.");
        //这个是从freeodom里(last_connect_pos_)得到new_node
        //此处建立的nav_node其属性is_navpoint为true
        this->CreateNavNodeFromPoint(last_connect_pos_, new_node_ptr, false, true);
        new_nodes_.push_back(new_node_ptr);
        //更新一下last_connect_pos_
        last_connect_pos_ = FARUtil::free_odom_p;
        if (is_bridge_internav_) is_bridge_internav_ = false;
    }
    // 上面在if条件下 完成得到new_nodes_
    //第二步，从new_ctnode里判断需不需要更新navnode
    for (const auto& ctnode_ptr : new_ctnodes) {
        bool is_near_new = false;
        //第一个if判断是不是新节点是不是附近的，且is_near_new也会改变
        if (this->IsAValidNewNode(ctnode_ptr, is_near_new)) {
            //通过contour产生一个新的navnode
            this->CreateNewNavNodeFromContour(ctnode_ptr, new_node_ptr);
            if (!is_near_new) {
                new_node_ptr->is_block_frontier = true;
            }
            //这个是从new_ctnodes里得到的new_nodes_
            new_nodes_.push_back(new_node_ptr);
        }
    }
    if (new_nodes_.empty()) return false;
    else return true;
}


void DynamicGraph::UpdateNavGraph(const NodePtrStack& new_nodes,
                                  const bool& is_freeze_vgraph,
                                  NodePtrStack& clear_node) 
{
    // clear false positive node detection
    clear_node.clear();
    if (!is_freeze_vgraph) {
        //  UpdateGlobalNearNodes()里得到过extend_match_nodes_  其实就是前面用到的near_nav_graph_
        for (const auto& node_ptr : extend_match_nodes_) {
            if (FARUtil::IsStaticNode(node_ptr) || node_ptr == cur_internav_ptr_) continue;
            //cur_internav_ptr_ 离odom最近的一个nav_node
            //放进去的是extend_match_nodes_的node
            if (!this->ReEvaluateCounter(node_ptr)) {
                //只有clear_dumper_count大于一定数值，才会返回true，需要合并，并把需要合并的点push进clear_node
                if (this->SetNodeToClear(node_ptr)) {
                    clear_node.push_back(node_ptr);
                }
            } else {
                //减少clear_dumper_count
                this->ReduceDumperCounter(node_ptr);
            }
        }
        // re-evaluate trajectory edge using terrain planner
        if (!FARUtil::IsStaticEnv && cur_internav_ptr_ != NULL) {
            //正常来说，surround_internav_nodes包含了cur_internav_ptr_这个点
            //如果这个点不在surround_internav_nodes里，就把点加进去
            NodePtrStack internav_check_nodes = surround_internav_nodes_;
            if (!FARUtil::IsTypeInStack(cur_internav_ptr_, internav_check_nodes)) {
                internav_check_nodes.push_back(cur_internav_ptr_);
            }
            //循环internav_check_nodes的每一个点
            for (const auto& sur_internav_ptr : internav_check_nodes) {
                //把当前internav的traj_connect拷贝一份
                const NodePtrStack copy_traj_connects = sur_internav_ptr->trajectory_connects;
                //循环traj_connect的traj_node
                for (const auto& tnode_ptr : copy_traj_connects) {
                    //把每个tnode和当前的sur_internav_ptr做个评估
                    if (this->ReEvaluateConnectUsingTerrian(sur_internav_ptr, tnode_ptr)) {
                            this->RecordValidTrajEdge(sur_internav_ptr, tnode_ptr);
                    } else {
                        this->RemoveInValidTrajEdge(sur_internav_ptr, tnode_ptr);
                    }
                }   
            }     
        }
    }

    // clear merged nodes in stacks
    this->ClearMergedNodesInGraph();
    // add matched margin nodes into near and wide near nodes
    this->UpdateNearNodesWithMatchedMarginNodes(margin_near_nodes_, near_nav_nodes_, wide_near_nodes_);

    // 处理odom  检查并添加odom到wide_near_nodes的connect
    // check-add connections to odom node with wider near nodes
    NodePtrStack codom_check_list = wide_near_nodes_;
    //把new_nodes加到codom_check_list（ wide_near_nodes_ ）的最末端
    /*
        第二次运行的时候，如果小车没有动，环境没有变化，那么这个codom_check_list和第一次的内容是一样的，只是new_nodes为空而已
    */
    codom_check_list.insert(codom_check_list.end(), new_nodes.begin(), new_nodes.end()); // add new nodes to check list
    for (const auto& conode_ptr : codom_check_list) {   // odom和ctnode建立edge_votes、potential_edge、poly_connect、connect_node
        //跳过小车自身的odom
        if (conode_ptr->is_odom) continue;
        //判断odom和当前的codom_check_list里的ctnode是否为有效连接

        if (this->IsValidConnect(odom_node_ptr_, conode_ptr, false)) {      //edge_vote、potential_edges
            //如果是有效连接   1、把node互相加入到对方的poly_connects   2、把node互相加到对方的connect_nodes里
            this->AddPolyEdge(odom_node_ptr_, conode_ptr), this->AddEdge(odom_node_ptr_, conode_ptr);
        } 
        //如果不是有效的连接，那么就要把对应的poly_connects和connect_nodes进行删除
        else {
            this->ErasePolyEdge(odom_node_ptr_, conode_ptr), this->EraseEdge(conode_ptr, odom_node_ptr_);
        }
    }

    //如果graph进行更新
    if (!is_freeze_vgraph) {

        // Adding new nodes to near nodes stack
        for (const auto& new_node_ptr : new_nodes) {
            //把new_nodes更新到global_Graph里
            this->AddNodeToGraph(new_node_ptr);
            //把new_node_ptr给push进near_nav_nodes，且new_node_ptr->is_near_nodes为true
            new_node_ptr->is_near_nodes = true;
            near_nav_nodes_.push_back(new_node_ptr);
            //再判断新的new_node_ptr里 is_navpoint是不是为true 如果是的话，就执行UpdateCurInterNavNode
            if (new_node_ptr->is_navpoint) this->UpdateCurInterNavNode(new_node_ptr);
            //若new_node_ptr的ctnode不为空，因为newnodes是navnode，它里面的ctnode可以和该navnode配对上
            if (new_node_ptr->ctnode != NULL) {
                ContourGraph::MatchCTNodeWithNavNode(new_node_ptr->ctnode, new_node_ptr);
            }
        }

        // connect outrange contour nodes
        for (const auto& out_node_ptr : out_contour_nodes_) {
            const NavNodePtr matched_node = ContourGraph::MatchOutrangeNodeWithCTNode(out_node_ptr, near_nav_nodes_);
            const auto it = out_contour_nodes_map_.find(out_node_ptr);
            if (matched_node != NULL) {
                this->RecordContourVote(out_node_ptr, matched_node);
                //把matched_node插入无序的navnode_set集合
                it->second.second.insert(matched_node);
            }
            for (const auto& reached_node_ptr : it->second.second) {
                //it->second.second里存放着许多nav_nodes，如果这个nav_node和matched_node不相同
                if (reached_node_ptr != matched_node) {
                    //delete contour vote(往里面填充0，使这个vote不生效)
                    this->DeleteContourVote(out_node_ptr, reached_node_ptr);
                }
            }
        }

        // reconnect between near nodes
        NodePtrStack outside_break_nodes;
        outside_break_nodes.clear();
        for (std::size_t i=0; i<near_nav_nodes_.size(); i++) {
            const NavNodePtr nav_ptr1 = near_nav_nodes_[i];
            if (nav_ptr1->is_odom) continue;
            // re-evaluate nodes which are not in near  重新判断nav_node里的connect_node是否是有效的，如果因为什么变化，node无效了，就把这些无效的node放进outside_break_nodes
            const NodePtrStack copy_connect_nodes = nav_ptr1->connect_nodes;    //创建接收nav_ptr的connect_nodes
            for (const auto& cnode : copy_connect_nodes) {
                //跳过一些类型的点
                if (cnode->is_odom || cnode->is_near_nodes || FARUtil::IsOutsideGoal(cnode) || FARUtil::IsTypeInStack(cnode, nav_ptr1->contour_connects)) continue;
                //判断nav_ptr1和它的connect_nodes里的cnode是不是一个validConnect
                if (this->IsValidConnect(nav_ptr1, cnode, false)) {     //edge_vote、potential_edges
                    //如果是有效连接   1、把node互相加入到对方的poly_connects   2、把node互相加到对方的connect_nodes里
                    //! 感觉有点奇怪，如果是直接从nav_ptr1的connect_nodes里取的点，
                    //! 那么为什么判断如果是有效连接的话，还要把点push到poly_connects和connect_nodes呢？本来就有了啊
                    //! 不是应该如果是有效连接就什么都不做，如果无效连接 就删除么？
                    this->AddPolyEdge(nav_ptr1, cnode), this->AddEdge(nav_ptr1, cnode);
                } else {
                    //不是有效连接，擦除
                    this->ErasePolyEdge(nav_ptr1, cnode) ,this->EraseEdge(nav_ptr1, cnode);
                    //并把该cnode给push进outside_break_nodes
                    outside_break_nodes.push_back(cnode);  
                } 
            }
            //判断near_nav_nodes里 node[i]和它之前的node[j]们的关系
            for (std::size_t j=0; j<near_nav_nodes_.size(); j++) {
                const NavNodePtr nav_ptr2 = near_nav_nodes_[j];
                if (i == j || j > i || nav_ptr2->is_odom) continue;
                //对于near_nav_nodes 如果连接有效（相当于是把polygon各自的ctnodes（nav_node）和别的polygon的ctnodes（nav_node）做了一下连接）
                //所以它的check_contour标识符为true
                if (this->IsValidConnect(nav_ptr1, nav_ptr2, true)) {   //contour_votes、 potential_contours、edge_vote、potential_edges
                    //如果是有效连接   1、把node互相加入到对方的poly_connects   2、把node互相加到对方的connect_nodes里
                    this->AddPolyEdge(nav_ptr1, nav_ptr2), this->AddEdge(nav_ptr1, nav_ptr2);
                } else {
                    this->ErasePolyEdge(nav_ptr1, nav_ptr2), this->EraseEdge(nav_ptr1, nav_ptr2);
                }
            }
            for (const auto& oc_node_ptr : out_contour_nodes_) {
                if (!oc_node_ptr->is_contour_match || !nav_ptr1->is_contour_match) continue;
                if (ContourGraph::IsNavNodesConnectFromContour(nav_ptr1, oc_node_ptr)) {
                    this->RecordContourVote(nav_ptr1, oc_node_ptr);
                } else {
                    this->DeleteContourVote(nav_ptr1, oc_node_ptr);
                }
            }
            //! generate boundary_contour_set_
            this->TopTwoContourConnector(nav_ptr1);
        }

        // update out range break nodes connects
        for (const auto& node_ptr : near_nav_nodes_) {
            for (const auto& ob_node_ptr : outside_break_nodes) {
                if (this->IsValidConnect(node_ptr, ob_node_ptr, false)) {
                    this->AddPolyEdge(node_ptr, ob_node_ptr), this->AddEdge(node_ptr, ob_node_ptr);
                } else {
                    this->ErasePolyEdge(node_ptr, ob_node_ptr), this->EraseEdge(node_ptr, ob_node_ptr);
                }
            }
        }

        // Analysisig frontier nodes
        for (const auto& node_ptr : near_nav_nodes_) {
            if (this->IsNodeFullyCovered(node_ptr)) {
                node_ptr->is_covered = true;
            } else {
                node_ptr->is_covered = false;
            }
            if (this->IsFrontierNode(node_ptr)) {
                node_ptr->is_frontier = true;
            } else {
                node_ptr->is_frontier = false;
            }
        }
    }
}

bool DynamicGraph::IsValidConnect(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2,
                                  const bool& is_check_contour) 
{
    //计算两点间的距离
    const float dist = (node_ptr1->position - node_ptr2->position).norm();
    if (dist < FARUtil::kEpsilon) return true;
    //如果node1或者node2有一个是odom  且  node1或node2 有一个is_navpoint为true (同一个node的is_odom和is_navpoint不会同时为true)
    if ((node_ptr1->is_odom || node_ptr2->is_odom) && (node_ptr1->is_navpoint || node_ptr2->is_navpoint)) {
        //如果距离小于0.55 返回true
        if (dist < FARUtil::kNavClearDist) return true; 
    } 

    /* check contour connection from node1 to node2 */
    if (is_check_contour) {
        if (this->IsBoundaryConnect(node_ptr1, node_ptr2) || (ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) && IsOnTerrainConnect(node_ptr1, node_ptr2, true))) {
            this->RecordContourVote(node_ptr1, node_ptr2);
        } else if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
            this->DeleteContourVote(node_ptr1, node_ptr2);
        }
    }
    //设置标识位
    bool is_connect = false;

    /* check polygon connections */
    //! 这一部分对应的是rviz里青色的线，如果缺少这一部分，那么机器人无法到达 非odom和goal直连的地方, 同时它也是产生edge_votes的地方
    // dg_params_.votes_size = 10
    const int vote_queue_size = (node_ptr1->is_odom || node_ptr2->is_odom) ? std::ceil(dg_params_.votes_size / 3.0f) : dg_params_.votes_size;
    if (IsConvexConnect(node_ptr1, node_ptr2) && this->IsInDirectConstraint(node_ptr1, node_ptr2) && ContourGraph::IsNavNodesConnectFreePolygon(node_ptr1, node_ptr2) && IsOnTerrainConnect(node_ptr1, node_ptr2, false)){
        //若满足上面的要求，接着判断这个
        if (this->IsPolyMatchedForConnect(node_ptr1, node_ptr2)) {
            //构成edge_vote、potential_edges
            RecordPolygonVote(node_ptr1, node_ptr2, vote_queue_size);
        }
    } 
    //如果不满足第一个if的判断，那么就要把node1和node2的edge_vote里的second里填0，代表不是
    else {
        DeletePolygonVote(node_ptr1, node_ptr2, vote_queue_size);
    }
    //判断vote是不是有效的
    if (this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2)) {
        //! 判断如果没有相似的connect 那么is_connect标识为true
        if (!this->IsSimilarConnectInDiection(node_ptr1, node_ptr2)) is_connect = true;
    } 
    //如果vote里0占了多数，导致其判断为fake votes，那么判断node1或node2是否为odom，如果是的话进入下面的判断 
    else if (node_ptr1->is_odom || node_ptr2->is_odom) {
        //node1中擦除node2的id
        node_ptr1->edge_votes.erase(node_ptr2->id);
        //node2中擦除node1的id
        node_ptr2->edge_votes.erase(node_ptr1->id);
        // clear potential connections
        // 从node_ptr1的potential_edges里擦除node_ptr2
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_edges);
        // 从node_ptr2的potential_edges里擦除node_ptr1
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_edges);
    }
    /*  通过上面的过程，is_connect可能为true 也可能为false  但至少已经经过了一次is_connect的判断*/
    /*  下面的部分就是 若上面判断完 is_connect为false的时候，接续的判断 */

    /* check if exsiting trajectory connection exist */
    //如果is_connect为false
    if (!is_connect) {
        //判断node1在不在node2的traj_connect里，如果在的话 is_connect为true
        if (FARUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) is_connect = true;
        //如果node1或node2是odom 且cur_internav_ptr不为null
        if ((node_ptr1->is_odom || node_ptr2->is_odom) && cur_internav_ptr_ != NULL) {
            //如果node1为odom 且 node2在cur_internav_ptr的traj_connect里
            if (node_ptr1->is_odom && FARUtil::IsTypeInStack(node_ptr2, cur_internav_ptr_->trajectory_connects)) {
                //如果满足几何约束
                if (FARUtil::IsInCylinder(cur_internav_ptr_->position, node_ptr2->position, node_ptr1->position, FARUtil::kNearDist)) {
                    is_connect = true;
                }   
            } else if (node_ptr2->is_odom && FARUtil::IsTypeInStack(node_ptr1, cur_internav_ptr_->trajectory_connects)) {
                if (FARUtil::IsInCylinder(cur_internav_ptr_->position, node_ptr1->position, node_ptr2->position, FARUtil::kNearDist)) {
                    is_connect = true;
                }
            }
        }
    }


    /* check for additional contour connection through tight area from current robot position */
    if (!is_connect && (node_ptr1->is_odom || node_ptr2->is_odom) && IsConvexConnect(node_ptr1, node_ptr2) && this->IsInDirectConstraint(node_ptr1, node_ptr2)) {
        //若node1为odom 且 node2对应的contour_connect容器非空
        if (node_ptr1->is_odom && !node_ptr2->contour_connects.empty()) {
            //从node2的contour_connect容器里取出每一个ctnode，
            for (const auto& ctnode_ptr : node_ptr2->contour_connects) {
                // 判断node_ptr1(odom)到线段(ctnode_ptr,node_ptr2)的空间距离是否满足要求
                if (FARUtil::IsInCylinder(ctnode_ptr->position, node_ptr2->position, node_ptr1->position, FARUtil::kNavClearDist)) {
                    is_connect = true;
                }
            }
        } else if (node_ptr2->is_odom && !node_ptr1->contour_connects.empty()) {
            for (const auto& ctnode_ptr : node_ptr1->contour_connects) {
                // 判断node_ptr2(odom)到线段(ctnode_ptr,node_ptr1)的空间距离是否满足要求
                if (FARUtil::IsInCylinder(ctnode_ptr->position, node_ptr1->position, node_ptr2->position, FARUtil::kNavClearDist)) {
                    is_connect = true;
                }
            }
        }
    }
    return is_connect;
}

bool DynamicGraph::IsOnTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_contour) {
        if (!node_ptr1->is_active || !node_ptr2->is_active) return true;
        Point3D mid_p = (node_ptr1->position + node_ptr2->position) / 2.0f;
        const Point3D diff_p = node_ptr2->position - node_ptr1->position;
        if (diff_p.norm() > FARUtil::kMatchDist && abs(diff_p.z) / std::hypotf(diff_p.x, diff_p.y) > 1) {
            if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
            return false; // slope is too steep > 45 degree
        } 
        if (is_contour && node_ptr1->contour_votes.find(node_ptr2->id) != node_ptr1->contour_votes.end()) { // recorded contour terrain connection
            return true;
        }
        bool is_match;
        float minH, maxH;
        const float avg_h = MapHandler::NearestHeightOfRadius(mid_p, FARUtil::kMatchDist, minH, maxH, is_match);
        if (!is_match && (is_contour || !node_ptr1->is_frontier || !node_ptr2->is_frontier)) {
            if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
            return false;
        } 
        if (is_match && (maxH - minH > FARUtil::kMarginHeight || abs(minH + FARUtil::vehicle_height - mid_p.z) > FARUtil::kTolerZ / 2.0f)) {
            if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
            return false;
        }
        if (!is_contour) {
            if (is_match) RecordVaildTerrainConnect(node_ptr1, node_ptr2);
            const auto it = node_ptr1->terrain_votes.find(node_ptr2->id);
            if (it != node_ptr1->terrain_votes.end() && it->second > dg_params_.finalize_thred) {
                return false;
            }
        }
        return true;
    }

//计算near_nav_nodes里的node的is_covered属性
bool DynamicGraph::IsNodeFullyCovered(const NavNodePtr& node_ptr) {
    //如果该node是odom或者是navpoint 或者该node的is_covered属性为true
    if (FARUtil::IsFreeNavNode(node_ptr) || node_ptr->is_covered) return true;
    NodePtrStack check_odom_list = internav_near_nodes_;
    check_odom_list.push_back(odom_node_ptr_);
    //循环check_odom_list的每一个node（internav_near_nodes_本身没有多少个node的）
    //! check_odom_list = [internav_near_nodes_ , odom_node_ptr_]
    for (const auto& near_optr : check_odom_list) {
        //计算node_ptr和near_optr的距离（near_optr是near_odom_ptr的意思)
        
        const float cur_dist = (node_ptr->position - near_optr->position).norm();
        if (cur_dist < FARUtil::kMatchDist) return true;
        //当cur_dist大于1.75时
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            // TODO: concave nodes will not be marked as covered based on current implementation
            const auto it = near_optr->edge_votes.find(node_ptr->id);
            if (it != near_optr->edge_votes.end() && FARUtil::IsVoteTrue(it->second)) {
                const Point3D diff_p = near_optr->position - node_ptr->position;
                if (FARUtil::IsInCoverageDirPairs(diff_p, node_ptr)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynamicGraph::IsFrontierNode(const NavNodePtr& node_ptr) {
    if (node_ptr->is_contour_match) {
        if (node_ptr->is_block_frontier || node_ptr->is_covered || node_ptr->free_direct != NodeFreeDirect::CONVEX ||
            node_ptr->ctnode->poly_ptr->perimeter < dg_params_.frontier_perimeter_thred) 
        {
            node_ptr->frontier_votes.push_back(0); // non convex frontier or too small
        } else {
            node_ptr->frontier_votes.push_back(1); // convex frontier
        }
    } else if (!FARUtil::IsPointInMarginRange(node_ptr->position)) { // if not in margin range, the node won't be deleted
        node_ptr->frontier_votes.push_back(0); // non convex frontier
    }
    if (node_ptr->frontier_votes.size() > dg_params_.finalize_thred) {
        node_ptr->frontier_votes.pop_front();
    }
    bool is_frontier = FARUtil::IsVoteTrue(node_ptr->frontier_votes);
    if (!node_ptr->is_frontier && is_frontier && node_ptr->frontier_votes.size() == dg_params_.finalize_thred) {
        if (!FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
            is_frontier = false;
        }
    }
    return is_frontier;
}

bool DynamicGraph::IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from,
                                              const NavNodePtr& node_ptr_to)
{
    // TODO: check for connection loss
    if (node_ptr_from->is_odom || node_ptr_to->is_odom) return false;
    if (FARUtil::IsTypeInStack(node_ptr_to, node_ptr_from->contour_connects)) { // release for contour connection
        return false;
    }
    // check from to to node connection
    if (this->IsAShorterConnectInDir(node_ptr_from, node_ptr_to)) {
        return true;
    }
    if (this->IsAShorterConnectInDir(node_ptr_to, node_ptr_from)) {
        return true;
    }
    return false;
}

//是否在直接约束中 我删掉照样能运行 那说明它可能只是一个 筛查的？减少输入量的
//! reduced v-graph
bool DynamicGraph::IsInDirectConstraint(const NavNodePtr& node_ptr1,
                                        const NavNodePtr& node_ptr2) 
{
    // check for odom -> frontier connections
    // 如果一个是odom 一个是is_frontier 那么说明是机器人和激光雷达射出去的点（frontier指unknown和known的交界，也就是激光雷达射出去的直线点）
    if ((node_ptr1->is_odom && node_ptr2->is_frontier) || (node_ptr2->is_odom && node_ptr1->is_frontier)) return true;
    // check node1 -> node2
    if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_1to2 = (node_ptr2->position - node_ptr1->position);
        //node1->node2的向量和node1的surf_dir对比
        if (!FARUtil::IsOutReducedDirs(diff_1to2, node_ptr1->surf_dirs)) {
            return false;
        }
    }
    // check node2 -> node1
    if (node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_2to1 = (node_ptr1->position - node_ptr2->position);
        if (!FARUtil::IsOutReducedDirs(diff_2to1, node_ptr2->surf_dirs)) {
            return false;
        }
    }
    return true;
}

bool DynamicGraph::IsInContourDirConstraint(const NavNodePtr& node_ptr1,
                                            const NavNodePtr& node_ptr2) 
{
    if (FARUtil::IsFreeNavNode(node_ptr1) || FARUtil::IsFreeNavNode(node_ptr2)) return false;
    // check node1 -> node2
    if (node_ptr1->is_finalized && node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        const Point3D diff_1to2 = node_ptr2->position - node_ptr1->position;
        if (!FARUtil::IsInContourDirPairs(diff_1to2, node_ptr1->surf_dirs)) {
            if (node_ptr1->contour_connects.size() < 2) {
                this->ResetNodeFilters(node_ptr1);
            } 
            return false;
        }
    }
    // check node1 -> node2
    if (node_ptr2->is_finalized && node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        const Point3D diff_2to1 = node_ptr1->position - node_ptr2->position;
        if (!FARUtil::IsInContourDirPairs(diff_2to1, node_ptr2->surf_dirs)) {
            if (node_ptr2->contour_connects.size() < 2) {
                this->ResetNodeFilters(node_ptr2);
            }
            return false;
        }
    }
    return true;
}

bool DynamicGraph::IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to) {
    bool is_nav_connect = false;
    bool is_cover_connect = false;
    if (node_ptr_from->is_navpoint && node_ptr_to->is_navpoint) is_nav_connect = true;
    if (node_ptr_from->is_covered && node_ptr_to->is_covered) is_cover_connect = true;
    if (node_ptr_from->connect_nodes.empty()) return false;
    Point3D ref_dir, ref_diff;
    const Point3D diff_p = node_ptr_to->position - node_ptr_from->position;
    const Point3D connect_dir = diff_p.normalize();
    const float dist = diff_p.norm();
    for (const auto& cnode : node_ptr_from->connect_nodes) {
        if (is_nav_connect && !cnode->is_navpoint) continue;
        if (is_cover_connect && !cnode->is_covered) continue;
        if (FARUtil::IsTypeInStack(cnode, node_ptr_from->contour_connects)) continue;
        ref_diff = cnode->position - node_ptr_from->position;
        if (cnode->is_odom || ref_diff.norm() < FARUtil::kEpsilon) continue;
        ref_dir = ref_diff.normalize();
        if ((connect_dir * ref_dir) > CONNECT_ANGLE_COS && dist > ref_diff.norm()) {
            return true;
        }
    }
    return false;
}

//用new_pos去更新node_ptr的位置
bool DynamicGraph::UpdateNodePosition(const NavNodePtr& node_ptr,
                                      const Point3D& new_pos) 
{
    //如果是odom或者goal
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        this->InitNodePosition(node_ptr, new_pos);
        return true;
    }
    //如果node属性is_finalized为true
    if (node_ptr->is_finalized) return true; // finalized node 
    //其他node类型
    //往该node的pos_filter_vec里添加新的位置信息
    node_ptr->pos_filter_vec.push_back(new_pos);
    //维护pos_filter_vec在一定长度
    if (node_ptr->pos_filter_vec.size() > dg_params_.pool_size) {
        node_ptr->pos_filter_vec.pop_front();
    }
    // calculate mean nav node position using RANSACS
    std::size_t inlier_size = 0;
    Point3D mean_p = FARUtil::RANSACPoisiton(node_ptr->pos_filter_vec, dg_params_.filter_pos_margin, inlier_size);
    if (node_ptr->pos_filter_vec.size() > 1) mean_p.z = node_ptr->position.z; // keep z value with terrain updates
    //用pos_filter_vec里的点去更新一个mean_p
    node_ptr->position = mean_p;
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;
}

//重新更新Node的位置信息，并把当前的位置push进pos_filter_vec
void DynamicGraph::InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos) {
    node_ptr->pos_filter_vec.clear();
    node_ptr->position = new_pos;
    node_ptr->pos_filter_vec.push_back(new_pos);
}

bool DynamicGraph::UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs)
{
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
        return true;
    }
    if (node_ptr->is_finalized) return true; // finalized node 
    //纠正ctnode的surf_dir方向
    FARUtil::CorrectDirectOrder(node_ptr->surf_dirs, cur_dirs);
    node_ptr->surf_dirs_vec.push_back(cur_dirs);
    if (node_ptr->surf_dirs_vec.size() > dg_params_.pool_size) {
        node_ptr->surf_dirs_vec.pop_front();
    }
    // calculate mean surface corner direction using RANSACS
    std::size_t inlier_size = 0;
    const PointPair mean_dir = FARUtil::RANSACSurfDirs(node_ptr->surf_dirs_vec, dg_params_.filter_dirs_margin, inlier_size);
    if (mean_dir.first == Point3D(0,0,-1) || mean_dir.second == Point3D(0,0,-1)) {
        node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
    } else {
        node_ptr->surf_dirs = mean_dir;
        //重新计算该nav_node的凹凸性
        this->ReEvaluateConvexity(node_ptr);
    }
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;       
}

void DynamicGraph::ReEvaluateConvexity(const NavNodePtr& node_ptr) {
    if (!node_ptr->is_contour_match || node_ptr->ctnode->poly_ptr->is_pillar) return;
    bool is_wall = false;
    const Point3D topo_dir = FARUtil::SurfTopoDirect(node_ptr->surf_dirs, is_wall);
    if (!is_wall) {
        const Point3D ctnode_p = node_ptr->ctnode->position;
        const Point3D ev_p = ctnode_p + topo_dir * FARUtil::kLeafSize;
        if (FARUtil::IsConvexPoint(node_ptr->ctnode->poly_ptr, ev_p)) {
            node_ptr->free_direct = NodeFreeDirect::CONVEX;
        } else {
            node_ptr->free_direct = NodeFreeDirect::CONCAVE;
        }
    }
}

void DynamicGraph::TopTwoContourConnector(const NavNodePtr& node_ptr) {
    //处理contour_votes和potential_contours   他们的node_id是桥梁
    std::vector<int> votesc;
    for (const auto& vote : node_ptr->contour_votes) {
        //vote.first 是连接到的对方的node_id  vote.second是一组0110的deque
        //如果是有效连接
        if (FARUtil::IsVoteTrue(vote.second, false)) {
            //把该vote的deque值累加计入votesc
            votesc.push_back(std::accumulate(vote.second.begin(), vote.second.end(), 0));
        }
    }
    //从大到小排序votesc
    std::sort(votesc.begin(), votesc.end(), std::greater<int>());
    for (const auto& cnode_ptr : node_ptr->potential_contours) {
        //我们通过node_ptr的contour_votes去找cnode的id
        /*
            假设contourA和contourB连接了，那么
            A的potential_contours就有（B,*,*,*) 
            B的potential_contours就有（A,*,*,*)
            A的contour_votes里就会有(<node_id(B),111101>,<node_id(xx),1010011>,....)
            B的contour_votes里就会有(<node_id(A),111001>,<node_id(xx),1110011>,....)
        */
        const auto it = node_ptr->contour_votes.find(cnode_ptr->id);
        // DEBUG
        //  如果node_ptr的contour_votes里没有找到cnode的id，那就报错
        if (it == node_ptr->contour_votes.end()) ROS_ERROR("DG: contour potential node matching error");
        //计算该vote的deque得分
        const int itc = std::accumulate(it->second.begin(), it->second.end(), 0);
        //对deque的数值有一定的要求。（要在所有的votes里的得分最高的两个），而且还要是true vote才行
        if (FARUtil::VoteRankInVotes(itc, votesc) < 2 && FARUtil::IsVoteTrue(it->second, false)) {
            //把两个node互相加到对方的contour_connect里，并把两个node组成NavEdge点对，加入到global_contour_set_中
            DynamicGraph::AddContourConnect(node_ptr, cnode_ptr);
            //把它们互相加到对方的connect_nodes里
            this->AddEdge(node_ptr, cnode_ptr);
        } else if (DynamicGraph::DeleteContourConnect(node_ptr, cnode_ptr) && !FARUtil::IsTypeInStack(cnode_ptr, node_ptr->poly_connects)) {
            this->EraseEdge(node_ptr, cnode_ptr);
        }
    }
}

void DynamicGraph::RecordContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (FARUtil::IsDebug) {
        if ((it1 == node_ptr1->contour_votes.end()) != (it2 == node_ptr2->contour_votes.end())) {
            ROS_ERROR_THROTTLE(1.0, "DG: Critical! Contour edge votes queue error.");
        }
    }
    //如果node1的contour_votes找不到node2的id 或node2的contour_votes找不到node1的id
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
        // init contour connection votes
        // 初始化contour connection
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) {
            node_ptr1->potential_contours.push_back(node_ptr2);
            node_ptr2->potential_contours.push_back(node_ptr1);
        }
    } else {
        if (FARUtil::IsDebug) {
            if (it1->second.size() != it2->second.size()) ROS_ERROR_THROTTLE(1.0, "DG: contour connection votes are not equal.");
        }
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > dg_params_.votes_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }
}

void DynamicGraph::RecordPolygonVote(const NavNodePtr& node_ptr1, 
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size, 
                                     const bool& is_reset) 
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (FARUtil::IsDebug) {
        if ((it1 == node_ptr1->edge_votes.end()) != (it2 == node_ptr2->edge_votes.end())) {
            ROS_ERROR_THROTTLE(1.0, "DG: Critical! Polygon edge votes queue error.");
        }
    }
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
        // init polygon edge votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) {
            node_ptr1->potential_edges.push_back(node_ptr2);
            node_ptr2->potential_edges.push_back(node_ptr1);
        }
    } else {
        if (FARUtil::IsDebug) {
            if (it1->second.size() != it2->second.size()) ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge votes are not equal.");
        }
        if (is_reset) it1->second.clear(), it2->second.clear();
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > queue_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }
}

void DynamicGraph::FillPolygonEdgeConnect(const NavNodePtr& node_ptr1,
                                        const NavNodePtr& node_ptr2,
                                        const int& queue_size)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
        std::deque<int> vote_queue1(queue_size, 1);
        std::deque<int> vote_queue2(queue_size, 1);
        node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && 
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) 
        {
            node_ptr1->potential_edges.push_back(node_ptr2);
            node_ptr2->potential_edges.push_back(node_ptr1);
        }
        // Add connections
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->poly_connects) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->poly_connects)) 
        {
            node_ptr1->poly_connects.push_back(node_ptr2);
            node_ptr2->poly_connects.push_back(node_ptr1);
        }
    }
}

void DynamicGraph::FillContourConnect(const NavNodePtr& node_ptr1,
                                    const NavNodePtr& node_ptr2,
                                    const int& queue_size)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    std::deque<int> vote_queue1(queue_size, 1);
    std::deque<int> vote_queue2(queue_size, 1);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
        // init polygon edge votes
        node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && 
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) 
        {
            node_ptr1->potential_contours.push_back(node_ptr2);
            node_ptr2->potential_contours.push_back(node_ptr1);
        }
        // Add contours
        DynamicGraph::AddContourConnect(node_ptr1, node_ptr2);
    }
}

void DynamicGraph::FillTrajConnect(const NavNodePtr& node_ptr1,
                                 const NavNodePtr& node_ptr2)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->trajectory_votes.end() || it2 == node_ptr2->trajectory_votes.end()) {
        node_ptr1->trajectory_votes.insert({node_ptr2->id, 0});
        node_ptr2->trajectory_votes.insert({node_ptr1->id, 0});
        // Add connection
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->trajectory_connects) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) 
        {   
            node_ptr1->trajectory_connects.push_back(node_ptr2);
            node_ptr2->trajectory_connects.push_back(node_ptr1);
        }
    }
}

void DynamicGraph::DeletePolygonVote(const NavNodePtr& node_ptr1, 
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size,
                                     const bool& is_reset) 
{
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) return;
    if (is_reset) it1->second.clear(), it2->second.clear();
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > queue_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

/* Delete Contour edge for given two navigation nodes */
void DynamicGraph::DeleteContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) return; // no connection (not counter init) in the first place 
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > dg_params_.votes_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

bool DynamicGraph::IsActivateNavNode(const NavNodePtr& node_ptr) {
    if (node_ptr->is_active) return true;
    if (FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
        node_ptr->is_active = true;
        return true;
    }
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        //判断该node是否在odom附近
        const bool is_nearby = (node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kNearDist ? true : false;
        if (is_nearby) {
            node_ptr->is_active = true;
            return true;
        }
        if (FARUtil::IsTypeInStack(node_ptr, odom_node_ptr_->connect_nodes)) {
            node_ptr->is_active = true;
            return true;
        }
        bool is_connects_activate = true;
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            if (!cnode_ptr->is_active) {
                is_connects_activate = false;
                break;
            }
        }
        if ((is_connects_activate && !node_ptr->connect_nodes.empty())) {
            node_ptr->is_active = true;
            return true;
        }
    }
    return false;
}

void DynamicGraph::UpdateGlobalNearNodes() {
    /* update nearby navigation nodes stack --> near_nav_nodes_ */
    //  通过nav_nodes stack 里的 near_nav_nodes来更新
    //  每次都会被clear掉，不会存储之前的内容，所以也是个局部的
    near_nav_nodes_.clear(), wide_near_nodes_.clear(), extend_match_nodes_.clear();
    margin_near_nodes_.clear(); internav_near_nodes_.clear(), surround_internav_nodes_.clear();
    //globalGraphNodes_ 一开始is_near_nodes和is_wide_near都是true
    for (const auto& node_ptr : globalGraphNodes_) {
        node_ptr->is_near_nodes = false;
        node_ptr->is_wide_near  = false;
        // node_ptr高度在一定范围，node_ptr离odom在一定范围，且（node_ptr非active 或 node_ptr的坐标在extend_obs_indices里）
        // 由point构建nav_node的点，is_active都是true 那么IsNavPointOnTerrainNeighbor就比较关键
        if (FARUtil::IsNodeInExtendMatchRange(node_ptr) && (!node_ptr->is_active || MapHandler::IsNavPointOnTerrainNeighbor(node_ptr->position, true))) {
            //判断是否到达了目的地，到达目的地了就直接continue掉
            if (FARUtil::IsOutsideGoal(node_ptr)) continue;
            //目前 is_boundary都是false的，测试过。所以只要是activate的点就都会被push进去
            if (this->IsActivateNavNode(node_ptr) || node_ptr->is_boundary) extend_match_nodes_.push_back(node_ptr);
            // 判断高度在不在范围，然后判断这个点和odom之间的距离有没有超过雷达探测的范围
            if (FARUtil::IsNodeInLocalRange(node_ptr) && IsPointOnTerrain(node_ptr->position)) {
                //! 只要满足上面的if 条件 那它就是 wide_near点
                wide_near_nodes_.push_back(node_ptr);
                node_ptr->is_wide_near = true;

                //! 接着细分
                if (node_ptr->is_active || node_ptr->is_boundary) {
                    //! 在上面的基础上，这个node 是active或者是 boundary 它又可以被分入 nav_nodes
                    //! is_near_nodes 为真
                    near_nav_nodes_.push_back(node_ptr);
                    node_ptr->is_near_nodes = true;

                    //!再接着细分
                    if (node_ptr->is_navpoint) {
                        node_ptr->position.intensity = node_ptr->fgscore;//navpoint的itensity实际上是fgscore
                        internav_near_nodes_.push_back(node_ptr);

                        //!继续细分
                        if ((node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kLocalPlanRange / 2.0f) {
                            surround_internav_nodes_.push_back(node_ptr);
                        }
                    }
                }
            } else if (node_ptr->is_active || node_ptr->is_boundary) {
                //这个margin_near_nodes_一般就是空的
                margin_near_nodes_.push_back(node_ptr);
            }
        }
    }
    //! 完善wide_near_nodes_
    //  把和odom连接的nodes取出来
    for (const auto& cnode_ptr : odom_node_ptr_->connect_nodes) { // add additional odom connections to wide near stack
        if (FARUtil::IsOutsideGoal(cnode_ptr)) continue;
        //如果这个node的is_wide_near为false，就把它加到wide_near_nodes_并把node的is_wide_near属性设置为true
        if (!cnode_ptr->is_wide_near) {
            wide_near_nodes_.push_back(cnode_ptr);
            cnode_ptr->is_wide_near = true;
        }
        //把和nodes连接的其他contour的nodes也取出来
        for (const auto& c2node_ptr : cnode_ptr->connect_nodes) {
            if (!c2node_ptr->is_wide_near && !FARUtil::IsOutsideGoal(c2node_ptr)) {
                wide_near_nodes_.push_back(c2node_ptr);
                c2node_ptr->is_wide_near = true;
            }
        }
    }
    if (!internav_near_nodes_.empty()) { // find the nearest inter_nav node that connect to odom
        std::sort(internav_near_nodes_.begin(), internav_near_nodes_.end(), nodeptr_icomp());   //按intensity从小到大排序
        for (std::size_t i=0; i<internav_near_nodes_.size(); i++) {
            const NavNodePtr temp_internav_ptr = internav_near_nodes_[i];   //取出当前的internav_near_nodes_
            //! 判断temp_internav_ptr在不在odom_node_ptr_->potential_edges这个vector里，判断temp_internav_ptr在不在范围内
            //! 如果这个if不满足，就进行for循环找下一个internav_near_nodes_ ，只需要找到一个满足条件的internav_near_nodes_[i]
            if (FARUtil::IsTypeInStack(temp_internav_ptr, odom_node_ptr_->potential_edges) && this->IsInternavInRange(temp_internav_ptr)) {
                //! 第一次的时候，cur_internav_ptr_ = NULL 满足if的条件判断
                if (cur_internav_ptr_ == NULL || temp_internav_ptr == cur_internav_ptr_ || (temp_internav_ptr->position - cur_internav_ptr_->position).norm() < FARUtil::kNearDist ||
                    FARUtil::IsTypeInStack(temp_internav_ptr, cur_internav_ptr_->connect_nodes)) 
                {   
                    //更新cur_internav_ptr_
                    this->UpdateCurInterNavNode(temp_internav_ptr);  
                } else {
                    //如果都不满足，那么它就是一个bridge_internav
                    is_bridge_internav_ = true;
                }
                break;//跳出for循环
            }
        }
    }
    //test
    bool test = false;
    if(test){
        std::fstream ifile("/home/liqunzhao/far_planner/use_debug/UpdateGlobalNearNodes.txt",std::ios::app);
        ifile<<"global_node\n";
        for(const auto& node_ptr:globalGraphNodes_){
            ifile<<"id:"<<node_ptr->id<<" ";
        }
        ifile<<"\n";
        ifile<<"------------------------------------------------\n";

        ifile<<"near_nav_nodes_\n";
        for(const auto& node_ptr:near_nav_nodes_){
            ifile<<"id:"<<node_ptr->id<<" ";
        }
        ifile<<"\n";
        ifile<<"------------------------------------------------\n";

        ifile<<"wide_near_nodes_\n";
        for(const auto& node_ptr:wide_near_nodes_){
            ifile<<"id:"<<node_ptr->id<<" ";
        }
        ifile<<"\n";
        ifile<<"------------------------------------------------\n";

        ifile<<"extend_match_nodes_\n";
        for(const auto& node_ptr:extend_match_nodes_){ 
            ifile<<"id:"<<node_ptr->id<<" ";
        }
        ifile<<"\n";
        ifile<<"------------------------------------------------\n";
        
        ifile<<"internav_near_nodes_\n";
        if(internav_near_nodes_.empty()) ifile<<"none\n";
        for(const auto& node_ptr:internav_near_nodes_){    
            ifile<<"id:"<<node_ptr->id<<" ";
        }
        ifile<<"\n";
        ifile<<"------------------------------------------------\n";
        
        ifile<<"surround_internav_nodes_\n";
        if(surround_internav_nodes_.empty()) ifile<<"none\n";
        for(const auto& node_ptr:surround_internav_nodes_){
            ifile<<"id:"<<node_ptr->id<<" ";    
        }
        ifile<<"\n";
        ifile<<"------------------------------------------------\n";
        
        ifile<<"margin_near_nodes_\n";
        if(margin_near_nodes_.empty()) ifile<<"none\n";
        for(const auto& node_ptr:margin_near_nodes_){
            ifile<<"id:"<<node_ptr->id<<" ";
        }
        ifile<<"-----------------------done once-------------------------\n";
        ifile.close();
    }
    
}

//重新评估Counter 判断需不需要合并或清除 返回true 不需要评估；false 需要评估
bool DynamicGraph::ReEvaluateCounter(const NavNodePtr node_ptr) {
    if (node_ptr->is_boundary) return true;
    if (node_ptr->is_navpoint) {
        //如果nav_node在surround_internav_nodes_里，且该nav_node也在TerrainOccupy（动态环境） 
        if (FARUtil::IsTypeInStack(node_ptr, surround_internav_nodes_) && this->IsNodeInTerrainOccupy(node_ptr)) {
            return false;
        }
        return true;
    }
    //该nav_node周围新增的obs点云个数如果超过一定值，那么is_near_new_obs_cloud就为true
    const bool is_near_new_obs_cloud = FARUtil::IsPointNearNewPoints(node_ptr->position, false);
    //is_near_new_obs_cloud为true代表环境发生了变化
    if (is_near_new_obs_cloud) { // if nearby env changes;
        this->ResetNodeFilters(node_ptr);
        //如果该nav_node没有match到ctnode， 清除nav_node的contour和poly的connect和votes
        if (!node_ptr->is_contour_match) this->ResetNodeConnectVotes(node_ptr);
    }
    //如果该nav_node没有match到对应的ctnode
    if (!node_ptr->is_contour_match) {
        //如果该点在margin范围内，或is_near_new_obs_cloud为true  需要进行评估
        if (FARUtil::IsPointInMarginRange(node_ptr->position) || is_near_new_obs_cloud) return false;
        //不考虑在marginrange范围之外的点
        return true;
    }
    //如果该nav_node之前已经被校正过
    if (node_ptr->is_finalized) return true;

    bool is_pos_cov  = false;
    bool is_dirs_cov = false;
    //如果该nav_node已经match到对应的ctnode
    if (node_ptr->is_contour_match) {
        //用RANSAC更新一下位置信息，如果更新成功返回true
        is_pos_cov  = this->UpdateNodePosition(node_ptr, node_ptr->ctnode->position);
        //用RANSAC更新一下surf_dir，如果更新成功返回true
        is_dirs_cov = this->UpdateNodeSurfDirs(node_ptr, node_ptr->ctnode->surf_dirs);
        if (FARUtil::IsDebug) ROS_ERROR_COND(node_ptr->free_direct == NodeFreeDirect::UNKNOW, "DG: node free space is unknown.");
    }
    //如果两个bool标识都为true，那么node_ptr的is_finalized为true
    if (is_pos_cov && is_dirs_cov) node_ptr->is_finalized = true;

    return true;
}

bool DynamicGraph::ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2) {
    PointStack terrain_path;
    if (terrain_planner_.PlanPathFromNodeToNode(node_ptr1, node_ptr2, terrain_path)) {
        return true;
    }
    return false;
}
