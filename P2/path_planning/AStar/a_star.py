"""
A* grid based planning

author: Atsushi Sakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import math
import csv
import numpy as np
import collections

show_animation = True

STEP = 1.0
MAX_RANGE = 360
LEN_TO_GATE = 15
way_point =[]
TURNING_MAX_RAD = 0.5
WEIGHTS = 0.1
LEN_THRE = 25.0

# start and goal position
sx = 180.0  # [m]
sy = 180.0  # [m]
gx = 120.0  # [m]
gy = 20.0  # [m]
grid_size = 10.0  # [m]
robot_size = 19.0  # [m]



class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_fianl_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_h(ngoal, openset[o].x, openset[o].y))
        current = openset[c_id]
        #  print("current", current)

        # show graph
        # if show_animation:
        #     plt.plot(current.x * reso, current.y * reso, "xc")
        #     if len(closedset.keys()) % 10 == 0:
        #         plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    rx, ry = calc_fianl_path(ngoal, closedset, reso)

    return rx, ry


def calc_h(ngoal, x, y):
    # w = 0.1  # weight of heuristic
    d = WEIGHTS * math.sqrt((ngoal.x - x)**2 + (ngoal.y - y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def dist_cal(point_1, point_2):
    return math.sqrt((point_1[0]-point_2[0])**2 + (point_1[1]-point_2[1])**2)


def create_map(mapfile):
    """
    x_half_width, y_half_width represent the arena

    y
    |
    |--- x
    """
    ox, oy = [], [] # ox, oy stores the non-reachable coords

    
    with open(mapfile, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=" ")
        gates = []
        for row in reader:
            each_gate = []
            for each_num in row:
                each_gate.append(float(each_num))
            gates.append(each_gate)
#     print(g1_lhs_x, g1_lhs_y, g1_rhs_x, g1_rhs_y)

    for i in range(MAX_RANGE):
        ox.append(i)
        oy.append(0)
    for i in range(MAX_RANGE):
        ox.append(i)
        oy.append(MAX_RANGE)
    for i in range(MAX_RANGE+1):
        ox.append(0)
        oy.append(i)
    # for i in range(MAX_RANGE+1):
    #     ox.append(i)
    #     oy.append(MAX_RANGE+1)
    for i in range(MAX_RANGE+1):
        ox.append(MAX_RANGE+1)
        oy.append(i)



    # x_half_width, y_half_width
    for each_gate in gates:
        xlen = each_gate[2] - each_gate[0]
        ylen = each_gate[3] - each_gate[1]
        dis = math.sqrt(xlen**2 + ylen**2)
        N = math.floor(dis/STEP)
        x_stp = xlen/N
        y_stp = ylen/N
        for i in range(N):
            ox.append(each_gate[0] + i*x_stp)
            oy.append(each_gate[1] + i*y_stp)
    return ox, oy, gates

def getTurningPts(rx, ry, start_point, enter_point, count):
    """
    input: full list of waypoints, rx and ry
    output: turing points
    """
    rx = np.asarray(rx)
    ry = np.asarray(ry)
    delta_rx = rx[1:] - rx[:-1]
    delta_ry = ry[1:] - ry[:-1]
    gradient = np.arctan2(delta_ry, delta_rx)

    turningPts = []

    for i in range(1, len(gradient)):
        if abs(gradient[i] - gradient[i-1]) > TURNING_MAX_RAD: # 30 degrees
            turningPts.append([rx[i], ry[i]])

    turningPts = turningPts[::-1]
    if count == 0:
        turningPts = turningPts + [[enter_point[0], enter_point[1]]]
        final_turningPts = [[turningPts[0][0], turningPts[0][1]]]
    else:
        turningPts = [[start_point[0], start_point[1]]] + turningPts + [[enter_point[0], enter_point[1]]]
        final_turningPts = [[start_point[0], start_point[1]]]
    
    point_idx = 0
    while (point_idx < len(turningPts)-1):
        flag = 0
        for point_idx_end in range(point_idx,len(turningPts)):
            if dist_cal(turningPts[point_idx], turningPts[point_idx_end]) > LEN_THRE:
                final_turningPts.append(turningPts[point_idx_end])
                point_idx = point_idx_end
                flag = 1
                break
        if flag == 0:
            break
    # print(turningPts)
    return final_turningPts

def flatten(x):
    if isinstance(x, collections.Iterable):
        return [a for i in x for a in flatten(i)]
    else:
        return [x]

def path_planning_loop(ox, oy, gates):
    # x_half_width, y_half_width
    start_point = np.asarray([sx, sy])
    count = 0
    rx_all = []
    ry_all = []
    for each_gate in gates:
        xlen = each_gate[2] - each_gate[0]
        ylen = each_gate[3] - each_gate[1]
        dis = math.sqrt(xlen**2 + ylen**2)
        N = math.floor(dis/STEP)
        x_stp = xlen/N
        y_stp = ylen/N
        center_gate = np.asarray([(each_gate[0]+each_gate[2])/2, (each_gate[1]+each_gate[3])/2])
        perpen_pt = np.asarray(rotate(center_gate, [each_gate[2],each_gate[3]], math.radians(-90)))

        enter_point = center_gate + (perpen_pt - center_gate)/dist_cal(perpen_pt, center_gate)*LEN_TO_GATE
        rx, ry = a_star_planning(float(start_point[0]), float(start_point[1]), float(enter_point[0]), float(enter_point[1]), ox, oy, grid_size, robot_size)
        turningpoint = getTurningPts(rx, ry,start_point, enter_point, count)
        start_point = center_gate + (perpen_pt - center_gate)/dist_cal(perpen_pt, center_gate)*(-LEN_TO_GATE)
        rx_all = rx_all + rx[::-1]
        ry_all = ry_all + ry[::-1]
        if count == 3:
            turningpoint = turningpoint + [[start_point[0], start_point[1]]]
        
        way_point.append(turningpoint)

        count += 1
        # way_point.append([enter_point[0], enter_point[1]]);

        
        # way_point.append([start_point[0], start_point[1]])
        
        #plot
        if show_animation:
            plt.plot(rx, ry, "-r")
            # plt.show()

    return way_point,rx_all, ry_all


def main():
    print(__file__ + " start!!")


    ox, oy, gates= create_map("mapfile")

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
        # plt.show()
    way_point,rx_all, ry_all = path_planning_loop(ox, oy, gates)


    print(way_point)

    flat_list = flatten(way_point)

    way_x =[]
    way_y =[]
    for idx in range(len(flat_list)):
        if idx % 2 == 0:
            way_x.append(flat_list[idx])
        else:
            way_y.append(flat_list[idx])

    if show_animation:
        plt.plot(way_x, way_y, "*k")

    print('waypoints:', flat_list)

    with open('way_points.dat', 'w') as the_file:
        the_file.write('\n'.join(map(str, flat_list)))

    plt.savefig('final')
    plt.close()

    # for report
    rx_all = rx_all + [way_x[-1]]
    ry_all = ry_all + [way_y[-1]]
    newsx = (np.asarray(sx) - 180)/100
    newsy = (np.asarray(sy) - 180)/100
    ox = (np.asarray(ox) - 180)/100
    oy = (np.asarray(oy) - 180)/100
    rx_all = (np.asarray(rx_all) - 180)/100
    ry_all = (np.asarray(ry_all) - 180)/100
    plt.plot(newsx, newsy, "ob",markersize=8.0, label='start point')
    plt.plot(ox, oy, ".k")
    plt.plot(rx_all, ry_all, "-r",linewidth=3.0, label='final path')
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    plt.legend()
    plt.show()


    print('rx_all:', rx_all)
    print('ry_all:', ry_all)

    plt.plot(ox, oy, ".k")
    plt.plot(rx_all[:13], ry_all[:13], "-r",linewidth=3.0)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    # plt.legend()
    # plt.show()
    plt.savefig('path1')
    plt.close()

    plt.plot(ox, oy, ".k")
    plt.plot(rx_all[12:14], ry_all[12:14], "-r",linewidth=3.0)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    # plt.legend()
    # plt.show()
    plt.savefig('path2')
    plt.close()

    plt.plot(ox, oy, ".k")
    plt.plot(rx_all[13:21], ry_all[13:21], "-r",linewidth=3.0)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    # plt.legend()
    # plt.show()
    plt.savefig('path3')
    plt.close()

    plt.plot(ox, oy, ".k")
    plt.plot(rx_all[20:22], ry_all[20:22], "-r",linewidth=3.0)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    # plt.legend()
    # plt.show()
    plt.savefig('path4')
    plt.close()

    plt.plot(ox, oy, ".k")
    plt.plot(rx_all[21:38], ry_all[21:38], "-r",linewidth=3.0)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    # plt.legend()
    # plt.show()
    plt.savefig('path5')
    plt.close()

    plt.plot(ox, oy, ".k")
    plt.plot(rx_all[37:39], ry_all[37:39], "-r",linewidth=3.0)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    # plt.legend()
    # plt.show()
    plt.savefig('path6')
    plt.close()

    plt.plot(ox, oy, ".k")
    plt.plot(rx_all[38:54], ry_all[38:54], "-r",linewidth=3.0)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    # plt.legend()
    # plt.show()
    plt.savefig('path7')
    plt.close()

    plt.plot(ox, oy, ".k")
    plt.plot(rx_all[53:55], ry_all[53:55], "-r",linewidth=3.0)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel('x[m]',fontsize=18)
    plt.ylabel('y[m]',fontsize=18)
    # plt.legend()
    # plt.show()
    plt.savefig('path8')
    plt.close()


if __name__ == '__main__':
    main()
