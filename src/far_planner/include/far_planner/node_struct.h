#ifndef NODE_STRUCT_H
#define NODE_STRUCT_H

#include "point_struct.h"

enum NodeType
{
    NOT_DEFINED = 0,
    GROUND = 1,
    AIR = 2
};

enum NodeFreeDirect
{
    UNKNOW = 0,
    CONVEX = 1,
    CONCAVE = 2,
    PILLAR = 3
};

typedef std::pair<Point3D, Point3D> PointPair;

namespace LiDARMODEL
{
    /* array resolution: 1 degree */
    static const int kHorizontalFOV = 360;
    static const int kVerticalFOV = 31;
    static const float kAngleResX = 0.2;
    static const float kAngleResY = 2.0;
}
// NODEPARAMS

struct Polygon
{
    Polygon() = default;
    std::size_t N;                      //构成polygon的点的个数  即vertices.size()
    std::vector<Point3D> vertices;      //构成polygon的点集
    bool is_robot_inside;               //标识位，robot是否在polygon内部
    bool is_pillar;                     //标识位，is_pillar代表一个面积足够小的area，或者是odom 或者是navpoint
    float perimeter;                    //阈值参数
};

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonStack;

struct CTNode
{
    CTNode() = default;
    Point3D position;           //空间位置
    bool is_global_match;      //判断该node是否在global地图上被匹配到
    bool is_contour_necessary; //判断是构成contour必须的点么
    bool is_ground_associate;   //TerrainHeightOfPoint里判断在terrain_grid_traverse_list_是否匹配到该点，默认为false，匹配到为true
    std::size_t nav_node_id;    //id号，和对应的nav_node匹配的
    NodeFreeDirect free_direct;     //node的类型 CONCAVE CONVEX PILLAR UNKNOW

    PointPair surf_dirs;    //该ctnode的front和back方向的方向向量
    PolygonPtr poly_ptr; //类似于poly的质心，存放组成该poly的vertices， 多个ctnodes可以有同一个poly_ptr，因为ctnodes是由poly_ptr的vertices组成的
    std::shared_ptr<CTNode> front;  //该ctnode的front方向的下一个ctnode点
    std::shared_ptr<CTNode> back;   //该ctnode的back方向的下一个ctnode点

    std::vector<std::shared_ptr<CTNode>> connect_nodes;     //跟该ctnode有连接关系的ctnodes的点集
};

typedef std::shared_ptr<CTNode> CTNodePtr;
typedef std::vector<CTNodePtr> CTNodeStack;

struct NavNode
{
    NavNode() = default;
    std::size_t id;                      // nav node的id
    Point3D position;                    // position的位置
    PointPair surf_dirs;                 // dirs的front和back的方向向量
    std::deque<Point3D> pos_filter_vec;  // 使用RANSACS去推算点的时候用的
    std::deque<PointPair> surf_dirs_vec; //存放纠正过的ctnode的surf_dir
    CTNodePtr ctnode;                    // ctnode的node
    bool prime;
    bool is_active;                      //离odom一定范围内的点，可以说是被激活的点（有效点）  由point构建成nav_node的时候，这个都为true
    bool is_block_frontier;
    bool is_contour_match;  //判断该node在contour否被匹配到
    bool is_odom;           //判断是否为odom
    bool is_goal;           //判断是否是目标点
    bool is_near_nodes;     //判断该node是否为odom的near_nodes
    bool is_wide_near;      //判断该node是否为odom的wide_near
    bool is_merged;         //判断该node需不需要被合并
    bool is_covered;        //判断该node是否被发现过
    bool is_frontier;       //判断该node是否在know和unknow的交界区域
    bool is_finalized;      //nav_node的校正位，如果为true 代表这个node的position和surf_dirs已经校正过了
    bool is_navpoint;       //判断这个node是不是一个导航点，也就是轨迹点
    bool is_boundary;       //判断这个node是不是boundary （目前应该都是false）只有3d的情况才会使用
    int clear_dumper_count; //辅助合并node的计数器
    std::deque<int> frontier_votes;
    std::unordered_set<std::size_t> invalid_boundary;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;            //该node连接到的nodes
    std::vector<std::shared_ptr<NavNode>> poly_connects;            //该node连接到的polygon的nodes
    std::vector<std::shared_ptr<NavNode>> contour_connects;         //在potential_contours里通过筛选后，该node连接到的contour的nodes
    std::unordered_map<std::size_t, std::deque<int>> contour_votes; //该node和连接到的contour的nodes的id和计数  (<node_id1,(010101)>,<node_id2,(110101)>,....) node_id是桥梁
    std::unordered_map<std::size_t, std::deque<int>> edge_votes;    //该node和连接到的node的nodes的id和计数  <node_id,(010101之类)>
    std::vector<std::shared_ptr<NavNode>> potential_contours;       //该node连接到的contours对应的node  和contour_votes是对应的     通过node_id可以找到对应的vote （是可能的潜在的需要在global连接的contour node）
    std::vector<std::shared_ptr<NavNode>> potential_edges;          //该node连接到的edges对应的node  和edge_votes是对应的                                       （是可能的潜在的需要在global连接的edge）
    std::vector<std::shared_ptr<NavNode>> trajectory_connects;      //该node的cur_internav_ptr_和last_internav_ptr_连接的nodes
    std::unordered_map<std::size_t, std::size_t> trajectory_votes;  // cur_internav存放last_internav的node_id和0，last_internav存放cur_internav的node_id和0 ;<node_id,0>
    std::unordered_map<std::size_t, std::size_t> terrain_votes;     //
    NodeType node_type;
    NodeFreeDirect free_direct; //该Node的free_direct的类型 UNKNOWN PILLAR CONVEX CONCAVE
    // planner members
    bool is_block_to_goal;
    bool is_traversable;
    bool is_free_traversable;
    float gscore, fgscore, heuristic;
    std::shared_ptr<NavNode> parent;      //该node的nav_node父节点
    std::shared_ptr<NavNode> free_parent; //该node的covered_node父节点 并不是每个node都有covered_node父节点的
};

typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::pair<NavNodePtr, NavNodePtr> NavEdge;

struct nodeptr_equal
{
    bool operator()(const NavNodePtr &n1, const NavNodePtr &n2) const
    {
        return n1->id == n2->id;
    }
};

struct navedge_hash
{
    std::size_t operator()(const NavEdge &nav_edge) const
    {
        return boost::hash<std::pair<std::size_t, std::size_t>>()({nav_edge.first->id, nav_edge.second->id});
    }
};

struct nodeptr_hash
{
    std::size_t operator()(const NavNodePtr &n_ptr) const
    {
        return std::hash<std::size_t>()(n_ptr->id);
    }
};

struct nodeptr_gcomp
{
    bool operator()(const NavNodePtr &n1, const NavNodePtr &n2) const
    {
        return n1->gscore > n2->gscore;
    }
};

struct nodeptr_fgcomp
{
    bool operator()(const NavNodePtr &n1, const NavNodePtr &n2) const
    {
        return n1->fgscore > n2->fgscore;
    }
};

struct nodeptr_icomp
{
    bool operator()(const NavNodePtr &n1, const NavNodePtr &n2) const
    {
        return n1->position.intensity < n2->position.intensity;
    }
};

#endif
