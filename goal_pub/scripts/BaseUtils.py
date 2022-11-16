import copy
# import torch
# from torch_geometric.data import Data
# import matplotlib.pyplot as plt
# import networkx as nx
# from torch_geometric.utils.convert import to_networkx
import math


# node struct in far_planner
class Node:
    def __init__(self, id):
        self.traject_id = {}
        self.contour_id = {}
        self.poly_id = {}
        self.connect_id = {}
        self.boundary = {}
        self.navpoint = {}
        self.frontier = {}
        self.cover = {}
        self.pos = {}
        self.free_direc = {}
        self.surf_dir_f = {}
        self.surf_dir_b = {}
        self.id = id

    def reid(self):
        return self.id

    def position(self, id, x, y, z):
        key = id
        value = (x, y, z)
        self.pos[key] = value

    def see_pos(self):
        return self.pos

    def free_direct(self, id, direct):
        key = id
        value = direct
        self.free_direc[key] = value

    def see_direct(self):
        return self.free_direc

    def surf_dir_front(self, id, x, y, z):
        key = id
        value = (x, y, z)
        self.surf_dir_f[key] = value

    def see_surf_dir_front(self):
        return self.surf_dir_f

    def surf_dir_back(self, id, x, y, z):
        key = id
        value = (x, y, z)
        self.surf_dir_b[key] = value

    def see_surf_dir_back(self):
        return self.surf_dir_b

    def is_covered(self, id, tag):
        key = id
        value = tag
        self.cover[key] = value

    def see_cover(self):
        return self.cover

    def is_frontier(self, id, tag):
        key = id
        value = tag
        self.frontier[key] = value

    def see_frontier(self):
        return self.frontier

    def is_navpoint(self, id, tag):
        key = id
        value = tag
        self.navpoint[key] = value

    def see_navpoint(self):
        return self.navpoint

    def is_boundary(self, id, tag):
        key = id
        value = tag
        self.boundary[key] = value

    def see_boundary(self):
        return self.boundary

    def connect_idx(self, id, vec):
        key = id
        value = vec
        self.connect_id[key] = value

    def see_connect(self):
        return self.connect_id

    def poly_idx(self, id, vec):
        key = id
        value = vec
        self.poly_id[key] = value

    def see_poly(self):
        return self.poly_id

    def contour_idx(self, id, vec):
        key = id
        value = vec
        self.contour_id[key] = value

    def see_contour(self):
        return self.contour_id

    def traject_idx(self, id, vec):
        key = id
        value = vec
        self.traject_id[key] = value

    def see_traj(self):
        return self.traject_id


# Dealling Vgh
def ReadGraph(lis=[], global_graph=[]):
    for unit in lis:
        temp = []
        count = 0
        id = unit[0]
        node = Node(id)
        node.position(id, float(unit[2]), float(unit[3]), float(unit[4]))
        node.free_direct(id, int(unit[1]))
        node.surf_dir_front(float(unit[0]), float(unit[5]), float(unit[6]), float(unit[7]))
        node.surf_dir_back(float(unit[0]), float(unit[8]), float(unit[9]), float(unit[10]))
        node.is_covered(id, int(unit[11]))
        node.is_frontier(id, int(unit[12]))
        node.is_navpoint(id, int(unit[13]))
        node.is_boundary(id, int(unit[14]))
        for connect_id in unit[15:]:
            if connect_id == "|":
                count += 1
                break
            else:
                count += 1
                temp.append(int(connect_id))
        node.connect_idx(id, temp)
        temp = copy.deepcopy(temp)
        temp.clear()
        for poly_id in unit[15 + count:]:
            if poly_id == "|":
                count += 1
                break
            else:
                count += 1
                temp.append(int(poly_id))
        node.poly_idx(id, temp)
        temp = copy.deepcopy(temp)
        temp.clear()
        for contour_id in unit[15 + count:]:
            if contour_id == "|":
                count += 1
                break
            else:
                count += 1
                temp.append(int(contour_id))
        node.contour_idx(id, temp)
        temp = copy.deepcopy(temp)
        temp.clear()
        for traj_id in unit[15 + count:]:
            if traj_id == "|":
                count += 1
                break
            else:
                count += 1
                temp.append(int(traj_id))
        node.traject_idx(id, temp)
        temp = copy.deepcopy(temp)
        temp.clear()
    global_graph.append(node)


# Open vgh file with line and process vgh
def Openvgh(path=""):
    global_graph = []
    lis = []
    with open(path, "r", encoding='UTF-8') as f:
        while True:
            line = f.readline()
            if line:
                lis.append(line.split())
                ReadGraph(lis, global_graph)
            else:
                break
    return global_graph


# edge connect
def Edge(graph=[]):
    graph_edge = []
    for num in range(0, len(graph)):
        # set edge type
        temp = graph[num].see_contour().get(graph[num].reid())
        for node in temp:
            graph_edge.append([int(graph[num].reid()), node])
    # print(graph_edge)
    return graph_edge


# node feature
def NavNode(graph=[]):
    node = []
    for num in range(0, len(graph)):
        node_id = int(graph[num].reid())
        temp_free_direct = graph[num].see_direct().get(graph[num].reid())
        temp_surf_first = graph[num].see_surf_dir_front().get(node_id)
        temp_surf_back = graph[num].see_surf_dir_back().get(node_id)
        temp_cover = graph[num].see_cover().get(graph[num].reid())
        temp_frontier = graph[num].see_frontier().get(graph[num].reid())
        temp_nav = graph[num].see_navpoint().get(graph[num].reid())
        temp_boundary = graph[num].see_boundary().get(graph[num].reid())
        node.append([temp_free_direct, temp_surf_first[0], temp_surf_first[1], temp_surf_first[2], temp_surf_back[0],
                     temp_surf_back[1], temp_surf_back[2], temp_cover, temp_frontier, temp_nav, temp_boundary])
    return node


# node pose
def NodePos(graph=[]):
    pos = []
    for num in range(0, len(graph)):
        temp_pos = graph[num].see_pos().get(graph[num].reid())
        pos.append([temp_pos[0], temp_pos[1], temp_pos[2]])
    return pos


# node pose in nx
def NodePos_nx(graph=[]):
    pos = {}
    for num in range(0, len(graph)):
        temp_pos = graph[num].see_pos().get(graph[num].reid())
        pos_2d = (temp_pos[0], temp_pos[1])
        key = int(graph[num].reid())
        value = pos_2d
        pos[key] = value
    return pos


# def Draw_Contour(graph=[], graph_edge=[]):
#     nx_pos = NodePos_nx(graph)
#     num = len(nx_pos)
#     print(num)
#     G = nx.Graph()
#     for node in graph_edge:
#         # Calculating the distances between the nodes as edge's weight.
#         dist = math.hypot(nx_pos.get(node[0])[0] - nx_pos.get(node[1])[0],
#                           nx_pos.get(node[1])[0] - nx_pos.get(node[1])[1])
#         # print(dist)
#         G.add_edge(node[0], node[1], weight=dist)  # 根据自己的需要为节点添加边
#     nx.draw_networkx(
#         G,
#         nx_pos,
#         with_labels=True,
#         node_size=80,
#         width=3,
#     )
#     # nx.draw(G, pos=nx_pos, with_labels=True,edge_color = 'r',node_size =80)
#     plt.show()


# 取graph里的每一个node的id
def Id_list(graph=[]):
    id_list = []
    for num in range(0, len(graph)):
        temp_id = graph[num].reid()
        id_list.append(temp_id)

    return id_list
