#ifndef DYNAMIC_GRAPH_H
#define DYNAMIC_GRAPH_H

#include "utility.h"
#include "contour_graph.h"
#include "terrain_planner.h"
#include "map_handler.h"


struct DynamicGraphParams {
    DynamicGraphParams() = default;
    int   dumper_thred;
    int   finalize_thred;
    int   pool_size;
    int   votes_size;
    float kConnectAngleThred;
    float filter_pos_margin;
    float filter_dirs_margin;
    float frontier_perimeter_thred;
};

class DynamicGraph {  
private:
    Point3D robot_pos_;
    NavNodePtr odom_node_ptr_     = NULL;
    NavNodePtr cur_internav_ptr_  = NULL;
    NavNodePtr last_internav_ptr_ = NULL;
    NodePtrStack new_nodes_;
    NodePtrStack near_nav_nodes_, wide_near_nodes_, extend_match_nodes_, margin_near_nodes_;
    NodePtrStack internav_near_nodes_, surround_internav_nodes_;
    NodePtrStack out_contour_nodes_;                //node采取merge操作时，存放具有一定价值的potential_contour_nodes
    float CONNECT_ANGLE_COS, NOISE_ANGLE_COS;
    bool is_bridge_internav_ = false;
    Point3D last_connect_pos_;

    static DynamicGraphParams dg_params_;
    static std::size_t id_tracker_; // Global unique id start from "0" [per robot]
    static NodePtrStack globalGraphNodes_;
    static std::unordered_map<std::size_t, NavNodePtr> idx_node_map_;
    static std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> out_contour_nodes_map_;

    TerrainPlanner terrain_planner_;
    TerrainPlannerParams tp_params_;

    /* Evaluate exist edges */

    /**
     * @brief set a tag "is_connect", when it comes true means the connect is valid
     * 
     * @param node_ptr1 
     * @param node_ptr2 
     * @return true or false
     */
    bool IsValidConnect(const NavNodePtr& node_ptr1, 
                        const NavNodePtr& node_ptr2,
                        const bool& is_check_contour);

    bool NodeLocalPerception(const NavNodePtr& node_ptr,
                             bool& _is_wall_end,
                             const bool& is_nearby_update = true);

    /**
     * @brief 检测node之间的edge是否要被保留（即reduce v graph）
     * 
     * @param node_ptr1 
     * @param node_ptr2 
     * @return true or false
     */
    bool IsInDirectConstraint(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    bool IsInContourDirConstraint(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    bool IsInterNavpointNecessary();

    bool IsFrontierNode(const NavNodePtr& node_ptr);
    
    void ReEvaluateConvexity(const NavNodePtr& node_ptr);

    /* Merge two nodes in Graph into one remain node, (mark one node as merged)*/
    void MergeNodeInGraph(const NavNodePtr& node_ptr1, 
                          const NavNodePtr& node_ptr2);

    /* check whether there is a connection in similar diection */
    bool IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from,
                                    const NavNodePtr& node_ptr_to);

    bool IsActivateNavNode(const NavNodePtr& node_ptr);

    bool IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to);

    bool UpdateNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos);

    static void InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos); 

    bool UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs);

    void ReOrganizeGraphConnect();

    bool ReEvaluateCounter(const NavNodePtr node_ptr);

    void RecordContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    void DeleteContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    void TopTwoContourConnector(const NavNodePtr& node_ptr);

    bool IsNodeFullyCovered(const NavNodePtr& node_ptr);

    bool ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2);


    inline bool IsNodeInTerrainOccupy(const NavNodePtr& node_ptr) {
        if (!FARUtil::IsStaticEnv && terrain_planner_.IsPointOccupy(node_ptr->position)) return true;
        return false;
    }

    /* Defince inline functions*/

    /* Assign ID to new navigation node */
    static inline void AssignGlobalNodeID(const NavNodePtr& node_ptr) {
        node_ptr->id = id_tracker_;
        idx_node_map_.insert({node_ptr->id, node_ptr});
        id_tracker_ ++;
    }

    static inline void RemoveNodeIdFromMap(const NavNodePtr& node_ptr) {
        idx_node_map_.erase(node_ptr->id);
    }

    inline void UpdateCurInterNavNode(const NavNodePtr& internav_node_ptr) {
        if (internav_node_ptr == NULL || !internav_node_ptr->is_navpoint) return;
        //更新cur_internav_ptr_
        cur_internav_ptr_ = internav_node_ptr;
        //使用cur_internav_ptr_去更新terrain_grids_的中心点
        terrain_planner_.UpdateCenterNode(cur_internav_ptr_);
        //如果last_internav_ptr_为空，那就初始化inter navigation nodes
        if (last_internav_ptr_ == NULL) { // init inter navigation nodes
            //更新last_internav_ptr_
            last_internav_ptr_ = cur_internav_ptr_;
        }
        //如果last和cur不相等那么就增加traj_connect
        else if (last_internav_ptr_ != cur_internav_ptr_) {
            this->AddTrajectoryConnect(cur_internav_ptr_, last_internav_ptr_);
            last_internav_ptr_ = cur_internav_ptr_;
        } 
    }

    //将cur和last建立traj
    inline void AddTrajectoryConnect(const NavNodePtr& cur_node, const NavNodePtr& last_node) {
        if (last_node == NULL || cur_node == NULL) return;
        //若last_node不在cur_node的traj_connect里，且cur_node不在last_node的traj_connect里
        if (!FARUtil::IsTypeInStack(last_node, cur_node->trajectory_connects) &&
            !FARUtil::IsTypeInStack(cur_node, last_node->trajectory_connects)) 
        {
            //互相加到对方的trajectory_votes里，
            cur_node->trajectory_votes.insert({last_node->id, 0});
            last_node->trajectory_votes.insert({cur_node->id, 0});
            cur_node->trajectory_connects.push_back(last_node);
            last_node->trajectory_connects.push_back(cur_node);
        }
    }

    inline void RecordValidTrajEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
    
        if (it1->second > 0) {
            it1->second --, it2->second --;
        }
    }

    inline void RemoveInValidTrajEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
        if (FARUtil::IsDebug) {
            if (it1 == node_ptr1->trajectory_votes.end() || it2 == node_ptr2->trajectory_votes.end() || it1->second != it2->second) {
                ROS_ERROR("DG: Trajectory votes queue error.");
                return;
            }
        }
        it1->second ++, it2->second ++;
        if (it1->second > dg_params_.finalize_thred) { // clear trajectory connections and votes
            if (FARUtil::IsDebug) ROS_WARN("DG: Current trajectory edge disconnected, no traversable path found.");
            node_ptr1->trajectory_votes.erase(node_ptr2->id);
            FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->trajectory_connects);

            node_ptr2->trajectory_votes.erase(node_ptr1->id);
            FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->trajectory_connects);

        }
    }

    /* Define inline functions */
    inline bool SetNodeToClear(const NavNodePtr& node_ptr) {
        if (FARUtil::IsStaticNode(node_ptr)) return false;
        node_ptr->clear_dumper_count ++;
        //dumper_thred = 4
        const int N = node_ptr->is_navpoint ? dg_params_.dumper_thred * 2 : dg_params_.dumper_thred;
        if (node_ptr->clear_dumper_count > N) {
            node_ptr->is_merged = true;
            return true;
        }
        return false;
    }
    /**
     * @brief allocate the valid MarginNodes to wide_nodes and near_nodes
     * 
     * @param margin_nodes 
     * @param near_nodes 
     * @param wide_nodes 
     */
    inline void UpdateNearNodesWithMatchedMarginNodes(const NodePtrStack& margin_nodes, NodePtrStack& near_nodes, NodePtrStack& wide_nodes) {
        for (const auto& node_ptr : margin_nodes) {
            if (node_ptr->is_contour_match) {
                node_ptr->is_wide_near = true, wide_nodes.push_back(node_ptr);
                node_ptr->is_near_nodes = true, near_nodes.push_back(node_ptr);
            }
        }
    }

    inline void ReduceDumperCounter(const NavNodePtr& node_ptr) {
        if (FARUtil::IsStaticNode(node_ptr)) return;
        node_ptr->clear_dumper_count --;
        if (node_ptr->clear_dumper_count < 0) {
            node_ptr->clear_dumper_count = 0;
        }
    }

    inline bool IsInternavInRange(const NavNodePtr& cur_inter_ptr) {
        if (cur_inter_ptr == NULL) return false;
        if (cur_inter_ptr->fgscore > FARUtil::kLocalPlanRange || 
            !FARUtil::IsPointInToleratedHeight(cur_inter_ptr->position, FARUtil::kMarginHeight)) 
        {
            return false;
        }
        return true;
    }

    inline bool IsPolyMatchedForConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if ((node_ptr1->is_finalized && !node_ptr1->is_contour_match && !FARUtil::IsFreeNavNode(node_ptr1)) || 
            (node_ptr2->is_finalized && !node_ptr2->is_contour_match && !FARUtil::IsFreeNavNode(node_ptr2))) 
        {
            return false;
        }
        return true;
    }

    inline bool IsPolygonEdgeVoteTrue(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {   // vote有效而且两个node是直连的
        const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
        if (it1 != node_ptr1->edge_votes.end() && it2 != node_ptr2->edge_votes.end()) {
            if (FARUtil::IsVoteTrue(it1->second)) {
                if (FARUtil::IsDebug && !FARUtil::IsVoteTrue(it2->second)) ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge vote result are not matched.");
                if (IsNodeDirectConnect(node_ptr1, node_ptr2) || it1->second.size() > 2) {
                    return true;
                }
            }
        }
        return false;
    }

    inline bool IsNodeDirectConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (FARUtil::IsFreeNavNode(node_ptr1) || FARUtil::IsFreeNavNode(node_ptr2)) return true;
        if (node_ptr1->is_contour_match || node_ptr2->is_contour_match) return true;
        return false;
    }

    inline void AddNodeToOutrangeContourMap(const NavNodePtr& node_ptr) {
        const auto it = out_contour_nodes_map_.find(node_ptr);
        //如果在out_contour_nodes_map_里找到了node_ptr，那么把它的有效计数器重置为finalize_thred
        if (it != out_contour_nodes_map_.end()) it->second.first = dg_params_.finalize_thred;   //dg_params_.finalize_thred = 6
        //如果没找到node_ptr，则创建一个
        else {
            //这个是out_contour_nodes_map_里的second的第二部分
            std::unordered_set<NavNodePtr> empty_set;
            //用dg_params_.finalize_thred和empty_set 制造一个pair
            const auto init_pair = std::make_pair(dg_params_.finalize_thred, empty_set);
            //在out_contour_nodes_map_里创建该node对应的键值对。
            out_contour_nodes_map_.insert({node_ptr, init_pair});
        }   
    }

    /* Create new navigation node, and return a shared pointer to it */
    inline void CreateNewNavNodeFromContour(const CTNodePtr& ctnode_ptr, NavNodePtr& node_ptr) {
        CreateNavNodeFromPoint(ctnode_ptr->position, node_ptr, false);
        node_ptr->is_contour_match = true;
        node_ptr->ctnode = ctnode_ptr;
        node_ptr->free_direct = ctnode_ptr->free_direct;
        UpdateNodeSurfDirs(node_ptr, ctnode_ptr->surf_dirs);
    }

    inline bool IsBoundaryConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
            if (FARUtil::IsTypeInStack(node_ptr2, node_ptr1->contour_connects)) {
                return true;
            }
        }
        return false;
    }

    inline bool IsAValidNewNode(const CTNodePtr ctnode_ptr, bool& is_near_new) {
        is_near_new = FARUtil::IsPointNearNewPoints(ctnode_ptr->position, true);    //单个sub可以存很多个点云，我们搜索看看他是不是孤立的点云，孤立的点云我们就不要了
        if (ctnode_ptr->is_contour_necessary || is_near_new) {
            if (MapHandler::IsNavPointOnTerrainNeighbor(ctnode_ptr->position, false) && IsPointOnTerrain(ctnode_ptr->position)) {
                return true;
            } else if (ctnode_ptr->is_contour_necessary) {
                ctnode_ptr->is_contour_necessary = false;
            }
        }
        return false;
    }

    static inline void AddContourConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects) &&
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->contour_connects))
        {
            node_ptr1->contour_connects.push_back(node_ptr2);
            node_ptr2->contour_connects.push_back(node_ptr1);
            ContourGraph::AddContourToSets(node_ptr1, node_ptr2);
        }
    }

    static inline bool DeleteContourConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects)) return false;
        
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->contour_connects);
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->contour_connects);
        ContourGraph::DeleteContourFromSets(node_ptr1, node_ptr2);
        return true;
    }

    static inline void ClearTrajectoryConnectInGraph(const NavNodePtr& node_ptr) {
        for (const auto& tjnode_ptr : node_ptr->trajectory_connects) {
            tjnode_ptr->trajectory_votes.erase(node_ptr->id);
            FARUtil::EraseNodeFromStack(node_ptr, tjnode_ptr->trajectory_connects);
        }
        node_ptr->trajectory_connects.clear();
        node_ptr->trajectory_votes.clear();
    }

    static inline void ResetBlockedContourPairs(const NavNodePtr& node_ptr) {
        if (node_ptr->contour_connects.empty()) return;
        //获取node_ptr对应的contour_connects的大小
        const std::size_t N = node_ptr->contour_connects.size();
        //循环取出每一个contour_connects的node
        for (std::size_t i=0; i<N; i++) {
            //循环套娃
            for (std::size_t j=0; j<N; j++) {
                //如果i和j相等 即取到了同一个点 或者  j>i 那么就跳过
                if (i == j || j > i) continue;
                const NavNodePtr contour_node1 = node_ptr->contour_connects[i];
                const NavNodePtr contour_node2 = node_ptr->contour_connects[j];
                const auto it1 = contour_node1->contour_votes.find(contour_node2->id);
                //如果contour_node1->contour_votes找到了contour_node2->id
                if (it1 != contour_node1->contour_votes.end()) {
                    //如果node1->node2的vote的连接为fake（即second里的deque里的0的个数很多）
                    if (!FARUtil::IsVoteTrue(it1->second, false)) {
                        //在contour_node2->contour_votes里找contour_node1->id
                        const auto it2 = contour_node2->contour_votes.find(contour_node1->id);
                        //清空second，然后往里填0
                        it1->second.clear(), it1->second.push_back(0);
                        it2->second.clear(), it2->second.push_back(0);
                    }
                }
            }
        }
    }

    inline void ClearContourConnectionInGraph(const NavNodePtr& node_ptr) {
        // reset connected contour pairs if exists
        // 重置已经存在的connected contour pair
        ResetBlockedContourPairs(node_ptr);
        for (const auto& ct_cnode_ptr : node_ptr->contour_connects) { 
            FARUtil::EraseNodeFromStack(node_ptr, ct_cnode_ptr->contour_connects);

            ContourGraph::DeleteContourFromSets(ct_cnode_ptr, node_ptr);
        }
        for (const auto& pt_cnode_ptr : node_ptr->potential_contours) {
            //从node_ptr->potential_contours取出potential_contour_nodes，在这些nodes里找对应node_ptr->id
            const auto it = pt_cnode_ptr->contour_votes.find(node_ptr->id);
            //! 如果 pt_cnode_ptr->is_active 且 pt_cnode_ptr不属于near_nodes 且 vote有效 
            //! 这类型的pt_cnode_ptr可能还有利用的价值，把它放到OutrangeContourMap保存起来
            if (pt_cnode_ptr->is_active && !pt_cnode_ptr->is_near_nodes && FARUtil::IsVoteTrue(it->second, false)) {
                //把该pt_cnode_ptr放入OutrangeContourMap
                AddNodeToOutrangeContourMap(pt_cnode_ptr);
            }
            //从该node的pt_cnode_ptr->potential_contours里删除该node
            FARUtil::EraseNodeFromStack(node_ptr, pt_cnode_ptr->potential_contours);
            //删除对应的votes
            pt_cnode_ptr->contour_votes.erase(it);
        }
        node_ptr->contour_connects.clear();
        node_ptr->contour_votes.clear();
        node_ptr->potential_contours.clear();
    }

    static inline bool IsMergedNode(const NavNodePtr& node_ptr) {
        if (FARUtil::IsStaticNode(node_ptr)) {
            //odom和goal是不可能被merge的
            node_ptr->is_merged = false;
            return false;
        }
        if (node_ptr->is_merged) return true;
        return false;
    }
    //令校正位失效，把用于推测pos的filter_vec和推测surf_dirs的filter_vec清空
    inline void ResetNodeFilters(const NavNodePtr& node_ptr) {
        node_ptr->is_finalized = false;
        node_ptr->pos_filter_vec.clear();
        node_ptr->surf_dirs_vec.clear();
    }

    inline void ResetContourVotes(const NavNodePtr& node_ptr) {
        for (const auto& pcnode : node_ptr->potential_contours) {
            const auto it1 = node_ptr->contour_votes.find(pcnode->id);
            const auto it2 = pcnode->contour_votes.find(node_ptr->id);
            if (FARUtil::IsVoteTrue(it1->second, false)) {
                it1->second.clear(), it1->second.push_back(1);
                it2->second.clear(), it2->second.push_back(1);
            } else {
                it1->second.clear(), it1->second.push_back(0);
                it2->second.clear(), it2->second.push_back(0);
            }
        }
    }

    inline void ResetPolygonVotes(const NavNodePtr& node_ptr) {
        for (const auto& pcnode : node_ptr->potential_edges) {
            const auto it1 = node_ptr->edge_votes.find(pcnode->id);
            const auto it2 = pcnode->edge_votes.find(node_ptr->id);
            if (FARUtil::IsVoteTrue(it1->second)) {
                it1->second.clear(), it1->second.push_back(1);
                it2->second.clear(), it2->second.push_back(1);
            } else {
                it1->second.clear(), it1->second.push_back(0);
                it2->second.clear(), it2->second.push_back(0);
            }
        }
    }

    inline void ResetNodeConnectVotes(const NavNodePtr& node_ptr) {
        // reset contours
        ResetContourVotes(node_ptr);
        // reset polygon connections
        ResetPolygonVotes(node_ptr);
    }

    inline void ClearNodeFromInternalStack(const NavNodePtr& node_ptr) {
        FARUtil::EraseNodeFromStack(node_ptr, near_nav_nodes_);
        FARUtil::EraseNodeFromStack(node_ptr, wide_near_nodes_);
        FARUtil::EraseNodeFromStack(node_ptr, margin_near_nodes_);
        if (node_ptr->is_navpoint) {
            FARUtil::EraseNodeFromStack(node_ptr, internav_near_nodes_);
            FARUtil::EraseNodeFromStack(node_ptr, surround_internav_nodes_);
        }
    }

    /* Clear nodes in global graph which is marked as merge */
    inline void ClearMergedNodesInGraph() {
        // remove nodes
        // it是 NodePtrStack::iterator
        // 遍历globalGraphNodes里的每一个node
        for (auto it = globalGraphNodes_.begin(); it != globalGraphNodes_.end(); it ++) {
            //如果该node的is_merged的tag为true
            if (IsMergedNode(*it)) {
                //清除*it对应的node里所有关联的connect_node、poly_connect_node、potential_edge、edge_votes的部分
                ClearNodeConnectInGraph(*it);
                //处理关于contour的部分  有contour_connect和global_contour_set_还有potential_contour，一些有利用价值但超出距离的点会被存在out_contour_nodes_map_里
                ClearContourConnectionInGraph(*it);
                //处理关于traj的部分
                ClearTrajectoryConnectInGraph(*it);
                //从idx_node_map_根据node_id 擦除对应的value值
                RemoveNodeIdFromMap(*it);
                //清除odom范围内产生的各个internalstack里的该node
                ClearNodeFromInternalStack(*it);
                //在globalgraph里擦除该it指向的node 并将it指向上一个node
                globalGraphNodes_.erase(it--);
            }
        }
        // clean outrange contour nodes 
        out_contour_nodes_.clear();
        //循环out_contour_nodes_map_   <NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>>
        //渐进式的清除out_contour_nodes_map_里的点
        for (auto it = out_contour_nodes_map_.begin(); it != out_contour_nodes_map_.end();) {
            it->second.first --;
            //如果该迭代器指向的node的is_merge为true 或者 指向的node的is_near_nodes属性为true 或 迭代器的计数小于等于0， 就从out_contour_nodes_map_里删除该node
            if (it->second.first <= 0 || it->first->is_near_nodes || IsMergedNode(it->first)) it = out_contour_nodes_map_.erase(it);
            else {
                //把navnodeptr给push进out_contour_nodes_
                out_contour_nodes_.push_back(it->first);
                ++ it;
            }
        }
    }

public:
    DynamicGraph() = default;
    ~DynamicGraph() = default;

    void Init(const ros::NodeHandle& nh, const DynamicGraphParams& params);

    /**
     *  Updtae robot pos and odom node 
     *  @param robot_pos current robot position in world frame
    */
    void UpdateRobotPosition(const Point3D& robot_pos);
    
    /**
     * Extract Navigation Nodes from Vertices Detected -> Update [new_nodes_] internally.
     * @param new_ctnodes new contour vertices without matching global navigation node
    */
    bool ExtractGraphNodes(const CTNodeStack& new_ctnodes);

    /**
     * Update Entire Navigation Graph with given new navigation nodes --
     * 1. clear false positive nodes detection
     * 2. Update Edges between exsiting nodes
     * 3. Adding edges between existing nodes with new extracted nodes
     * @param new_nodes new extracted navigation nodes given for graph updating
     * @param is_freeze_vgraph is stop visibility graph update (except for robot node)
     * @param clear_nodes existing nodes which is now recoginzed as false positive
    */
    void UpdateNavGraph(const NodePtrStack& new_nodes,
                        const bool& is_freeze_vgraph,
                        NodePtrStack& clear_node);


    /**
     *  Updtae local in range nods stack: near nodes, wide near nodes etc.,
    */
    void UpdateGlobalNearNodes();

    /* Static Functions */

    static void DeletePolygonVote(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2, 
                                  const int& queue_size,
                                  const bool& is_reset=false);

    static void RecordPolygonVote(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2, 
                                  const int& queue_size,
                                  const bool& is_reset=false);

    static void FillPolygonEdgeConnect(const NavNodePtr& node_ptr1,
                                       const NavNodePtr& node_ptr2,
                                       const int& queue_size);

    static void FillContourConnect(const NavNodePtr& node_ptr1,
                                   const NavNodePtr& node_ptr2,
                                   const int& queue_size);

    static void FillTrajConnect(const NavNodePtr& node_ptr1,
                                const NavNodePtr& node_ptr2);

    static bool IsOnTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_contour);

    static inline void FillFrontierVotes(const NavNodePtr& node_ptr, const bool& is_frontier) {
        if (is_frontier) {
            std::deque<int> vote_queue(dg_params_.finalize_thred, 1);
            node_ptr->frontier_votes = vote_queue;
        }
    } 

    static inline bool IsPointOnTerrain(const Point3D& p) {
        bool UNUSE_match = false;
        const float terrain_h = MapHandler::TerrainHeightOfPoint(p, UNUSE_match, true);
        if (abs(p.z - terrain_h - FARUtil::vehicle_height) < FARUtil::kTolerZ) {
            return true;
        }
        return false;
    }
    //两个node都不是CONCAVE就为true
    static inline bool IsConvexConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->free_direct != NodeFreeDirect::CONCAVE && node_ptr2->free_direct != NodeFreeDirect::CONCAVE) {
            return true;
        }
        return false;
    }

    static inline void RemoveInvaildTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->terrain_votes.find(node_ptr2->id);
        if (it1 == node_ptr1->terrain_votes.end()) {
            node_ptr1->terrain_votes.insert({node_ptr2->id, 1});
            node_ptr2->terrain_votes.insert({node_ptr1->id, 1});
        } else if (it1->second < dg_params_.finalize_thred * 2) {
            const auto it2 = node_ptr2->terrain_votes.find(node_ptr1->id);
            it1->second ++, it2->second ++;
        }
    }

    static inline void RecordVaildTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->terrain_votes.find(node_ptr2->id);
        if (it1 == node_ptr1->terrain_votes.end()) return;
        if (it1->second > 0) {
            const auto it2 = node_ptr2->terrain_votes.find(node_ptr1->id);
            it1->second --, it2->second --;
        }
    }

    static inline void CreateNavNodeFromPoint(const Point3D& point, NavNodePtr& node_ptr, const bool& is_odom, 
                                              const bool& is_navpoint=false, const bool& is_goal=false, const bool& is_boundary=false) 
    {
        node_ptr = std::make_shared<NavNode>();
        node_ptr->pos_filter_vec.clear();
        node_ptr->surf_dirs_vec.clear();
        node_ptr->ctnode = NULL;
        node_ptr->is_active = true;
        node_ptr->is_block_frontier = false;
        node_ptr->is_contour_match = false;
        node_ptr->is_odom = is_odom;  
        node_ptr->is_near_nodes = true;
        node_ptr->is_wide_near = true;
        node_ptr->is_merged = false;
        node_ptr->is_covered  = (is_odom || is_navpoint || is_goal) ? true : false;
        node_ptr->is_frontier = false;
        node_ptr->is_finalized = is_navpoint ? true : false;
        node_ptr->is_traversable = is_odom;
        node_ptr->is_navpoint = is_navpoint;
        node_ptr->is_boundary = is_boundary;
        node_ptr->is_goal = is_goal;
        node_ptr->clear_dumper_count = 0;
        node_ptr->frontier_votes.clear();
        node_ptr->invalid_boundary.clear();
        node_ptr->connect_nodes.clear();
        node_ptr->poly_connects.clear();
        node_ptr->contour_connects.clear();
        node_ptr->contour_votes.clear();
        node_ptr->potential_contours.clear();
        node_ptr->trajectory_connects.clear();
        node_ptr->trajectory_votes.clear();
        node_ptr->terrain_votes.clear();
        node_ptr->free_direct = (is_odom || is_navpoint) ? NodeFreeDirect::PILLAR : NodeFreeDirect::UNKNOW;
        // planner members
        node_ptr->is_block_to_goal    = false;
        node_ptr->gscore              = FARUtil::kINF;
        node_ptr->fgscore             = FARUtil::kINF;
        node_ptr->is_traversable      = true;
        node_ptr->is_free_traversable = true;
        node_ptr->parent              = NULL;
        node_ptr->free_parent         = NULL;
        InitNodePosition(node_ptr, point);
        // Assign Global Unique ID
        AssignGlobalNodeID(node_ptr);
    }

    //该node的所属关系的子nodes里的孙nodes相应的清除和该node的联系
    static inline void ClearNodeConnectInGraph(const NavNodePtr& node_ptr) {
        if (node_ptr == NULL) return;
        // clear navigation connections
        //  该node_ptr的  每个cnode_ptr->connect_nodes 的connect_nodes 里清除node_ptr
        //  for each node_ptr->connect_nodes->connect_nodes  do  clean node_ptr
        for (const auto& connect_node_ptr: node_ptr->connect_nodes) {
            FARUtil::EraseNodeFromStack(node_ptr, connect_node_ptr->connect_nodes);
        }
        //  该node_ptr的每个node_ptr->poly_connects的poly_connects 里清除node_ptr
        for (const auto& poly_node_ptr: node_ptr->poly_connects) {
            FARUtil::EraseNodeFromStack(node_ptr, poly_node_ptr->poly_connects);
        }
        //  该node_ptr的每个node_ptr->potential_edges的potential_edges里清除node_ptr
        for (const auto& pt_cnode_ptr : node_ptr->potential_edges) {
            FARUtil::EraseNodeFromStack(node_ptr, pt_cnode_ptr->potential_edges);
            //同时还要清除edge_votes里的id
            pt_cnode_ptr->edge_votes.erase(node_ptr->id);
        }
        node_ptr->connect_nodes.clear();
        node_ptr->poly_connects.clear();
        node_ptr->potential_edges.clear();
        node_ptr->edge_votes.clear();
    }

    static inline void ClearGoalNodeInGraph(const NavNodePtr& node_ptr) {
        ClearNodeConnectInGraph(node_ptr);
        //DEBUG
        if (!node_ptr->contour_connects.empty()) ROS_ERROR("DG: Goal node should not have contour connections.");
        FARUtil::EraseNodeFromStack(node_ptr, globalGraphNodes_);
    }

    /* Add new navigation node to global graph */
    static inline void AddNodeToGraph(const NavNodePtr& node_ptr) {
        if (node_ptr != NULL) {
            globalGraphNodes_.push_back(node_ptr);
        } else if (FARUtil::IsDebug) {
            ROS_WARN_THROTTLE(1.0, "DG: exist new node pointer is NULL, fails to add into graph");
        }
    }

    static inline void AddPolyEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1 == node_ptr2) return;
        //如果node2不在node1的poly_connect里 且 node1不在node2的poly_connect里
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->poly_connects) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->poly_connects)) 
        {
            //我们就把它们的node互相加到对方的poly_connects里
            node_ptr1->poly_connects.push_back(node_ptr2);
            node_ptr2->poly_connects.push_back(node_ptr1);
        }
        //如果都存在，那我们就不加了
    }

    static inline void ErasePolyEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        // clear node2 in node1's connection
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->poly_connects);
        // clear node1 in node2's connection 
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->poly_connects);
    }

    /* Add edge for given two navigation nodes */
    static inline void AddEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1 == node_ptr2) return;
        //如果node1不在node2的connect_nodes里 且 node2不在node1的connect_nodes里
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->connect_nodes) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->connect_nodes)) 
        {
            //把它们互相加到对方的connect_nodes里
            node_ptr1->connect_nodes.push_back(node_ptr2);
            node_ptr2->connect_nodes.push_back(node_ptr1);
        }
        //如果都在了，那就不加了
    }

    /* Erase connection between given two nodes */
    static inline void EraseEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        // clear node2 in node1's connection
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->connect_nodes);
        // clear node1 in node2's connection 
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->connect_nodes);
    }

    static inline NavNodePtr MappedNavNodeFromId(const std::size_t id) {
        const auto it = idx_node_map_.find(id);
        if (it != idx_node_map_.end()) {
            return it->second;
        } else {
            return NULL;
        }
    }

    /* Clear Current Graph */
    inline void ResetCurrentGraph() {
        odom_node_ptr_     = NULL; 
        cur_internav_ptr_  = NULL; 
        last_internav_ptr_ = NULL;

        id_tracker_         = 0;
        is_bridge_internav_ = false;
        last_connect_pos_   = Point3D(0,0,0);
        
        idx_node_map_.clear();
        near_nav_nodes_.clear(); 
        wide_near_nodes_.clear(); 
        extend_match_nodes_.clear();
        margin_near_nodes_.clear();
        internav_near_nodes_.clear();
        surround_internav_nodes_.clear();
        out_contour_nodes_.clear();
        out_contour_nodes_map_.clear();
        new_nodes_.clear();
        globalGraphNodes_.clear();
    }

    /* Get Internal Values */
    const NavNodePtr    GetOdomNode()         const { return odom_node_ptr_;};
    const NodePtrStack& GetNavGraph()         const { return globalGraphNodes_;};
    const NodePtrStack& GetExtendLocalNode()  const { return extend_match_nodes_;};
    const NodePtrStack& GetOutContourNodes()  const { return out_contour_nodes_;};
    const NodePtrStack& GetNewNodes()         const { return new_nodes_;};
    const NavNodePtr&   GetLastInterNavNode() const { return last_internav_ptr_;};


};



#endif
