#usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from copy import deepcopy
from numba import njit

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose

UNKNOWN = -1
FREE = 0
WALL = 1
PAD = 2
LINE = 3

@njit(cache=True)
def check_space(grid,row,col):
    if grid[row+1,col] == PAD and grid[row-1,col] == PAD and grid[row,col+1] == PAD and grid[row,col-1] == PAD:
        return True
    return False

@njit(cache=True)
def inflate_njit(grid,previous_grid,rows,cols,source=PAD):

    for y in range(0,rows,1):
        for x in range(0,cols,1):
            if previous_grid[y,x] == source:
                rosette = np.array([[y+1,x],[y,x+1],[y-1,x],[y,x-1]])
                for r in rosette:
                    if r[0] >= 0 and r[0] < rows and r[1] >=0 and r[1] < cols:
                        if previous_grid[r[0],r[1]] == FREE:
                            grid[r[0],r[1]] = LINE if check_space(grid,r[0],r[1]) else PAD

    return grid


class CenterLinePublisher(Node):
    def __init__(self):
        super().__init__("center_line_publisher")

        dt = 1

        self.map = None
        self.grid = None
        self.previous_grid = deepcopy(self.grid)

        self.rows = 0
        self.cols = 0

        self.resolution = 1

        self.publishing = False

        self.create_subscription(OccupancyGrid,"/map/file",self.map_cb,10)
        self.create_timer(dt,self.publish_path)
        self.path_publisher = self.create_publisher(Path,"/planning/path",10)

        self.get_logger().info(f'CenterLinePublisher running')

    def map_cb(self,msg):

        if self.map is None:
            self.map = np.array(msg.data)

            self.map = np.reshape(self.map,(msg.info.height,msg.info.width))
            self.map[self.map > 0] = WALL
            
            self.grid = deepcopy(self.map)
            self.previous_grid = deepcopy(self.map)

            self.rows = self.grid.shape[0]
            self.cols = self.grid.shape[1]

            self.resolution = msg.info.resolution

            self.origin = msg.info.origin

            self.get_logger().info(f'Received map and proceeding to inflation')

            self.run()

    def publish_path(self):

        if self.publishing:
            self.path_publisher.publish(self.line)

    def np2path(self,line):


        p = Path()

        poses = []

        for l in line:
            ps = PoseStamped()
            ps.pose.position.x = l[1] * self.resolution
            ps.pose.position.y = l[0] * self.resolution

            poses.append(ps)

        p.header.frame_id = "grid"
        p.poses = poses

        return p


    def get_distance(self,a,b):

        return np.sqrt(np.pow(a[0]-b[0],2)+np.pow(a[1]-b[1],2))      

    
    def check_inflation_end(self):
        c = np.count_nonzero(self.grid==0)
        if c > 0:
            return False
        return True

    def inflate(self,source=PAD):

        t_0 = time.time()

        self.grid = inflate_njit(self.grid,self.previous_grid,self.rows,self.cols,source=source)
        
        return time.time() - t_0
    
    def interpolate_line(self,line,distance=20):

        total_distance = 0

        for i in range(1,len(line)):
            total_distance += np.sqrt(np.pow(line[i-1,0]-line[i,0],2)+np.pow(line[i-1,1]-line[i,1],2))

        points_estimation = np.ceil(1.2 * total_distance / distance).astype(int)

        i_line = np.zeros([points_estimation,2])
        i_index = 0

        running = True
        ref_point = line[0,:]

        i_line[i_index,:] = ref_point
        i_index += 1

        accu_dist = 0

        s_index = 1

        while running:

            prop_point = line[s_index]
            prev_point = line[s_index-1] if accu_dist > 0 else ref_point

            dist = np.sqrt(np.pow(prev_point[0] - prop_point[0],2) + np.pow(prev_point[1] - prop_point[1],2))

            if accu_dist + dist < distance:
                accu_dist += dist
                s_index += 1

            else:

                point_a = prev_point
                point_b = prop_point

                full_len = self.get_distance(point_a,point_b)
                ratio = (distance - accu_dist)/full_len

                new_point = np.round(point_a + ratio * (point_b-point_a)).astype(int)

                i_line[i_index,:] = new_point
                ref_point = new_point
                i_index += 1

                accu_dist = 0
            
            if s_index >= len(line)-1:
                running = False

                if not np.array_equal(i_line[i_index-1,:], line[s_index,:]):

                    i_line[i_index,:] = line[s_index,:]
                    i_index += 1

        return i_line[:i_index,:]
    
    def occupancy2line(self,heuristics_threshold=55):

        t_0 = time.time()

        # sort points + filter really far outliers

        line_points = np.argwhere(self.grid == LINE)
        line = np.zeros_like(line_points)

        ref_point = line_points[0]
        line[0] = ref_point
        line_points = np.delete(line_points,0,0)
        running = True

        line_index = 1

        while running:

            reference_vector = np.array([line[line_index-1][0] - line[line_index-2][0],line[line_index-1][1] - line[line_index-2][1]]) if line_index > 1 else np.array([0,0])
            reference_angle = np.atan2(reference_vector[0],reference_vector[1])            

            angles = np.atan2(line_points[:,0]-ref_point[0],line_points[:,1] - ref_point[1])
            dangles = np.mod(np.abs(reference_angle - angles),np.pi)
            heuristic_angles = 1 + np.pow((dangles / dangles.max()),2)
            distances = np.sqrt(np.pow(ref_point[0] - line_points[:,0],2) + np.pow(ref_point[1] - line_points[:,1],2))
            heuristics = distances * heuristic_angles

            if np.min(heuristics) > heuristics_threshold:
                running = False
                break

            index = np.argmin(heuristics)

            line[line_index] = line_points[index]
            ref_point = line[line_index]
            line_points = np.delete(line_points,index,0)
            line_index += 1

            if line_points.size == 0:
                running = False

        sorted_line = line[:line_index,:]

        t_sorting = time.time()

        self.get_logger().info(f'Line sorting finished in {t_sorting - t_0} s, proceeding to smoothing')

        # smoothing

        tmp_smoothed_line = np.zeros_like(sorted_line)
        tmp_smoothed_line[:2,:] = sorted_line[:2,:]
        index = 2

        for i in range(2,len(sorted_line)-1):
            ref_vector = sorted_line[i-1] - sorted_line[i-2]
            investigated_vector = sorted_line[i] - sorted_line[i-1]
            proposed_vector = sorted_line[i+1] - sorted_line[i-1]

            ref_angle = np.atan2(ref_vector[0],ref_vector[1])
            investigated_angle = np.atan2(investigated_vector[0],investigated_vector[1])
            proposed_angle = np.atan2(proposed_vector[0],proposed_vector[1])

            di = np.mod(np.abs(investigated_angle - ref_angle),np.pi)
            dp = np.mod(np.abs(proposed_angle - ref_angle),np.pi)

            if di <= dp:

                tmp_smoothed_line[index,:] = sorted_line[i,:]
                index += 1

            
        smoothed_line = tmp_smoothed_line[:index,:]

        t_smoothing = time.time()

        self.get_logger().info(f'Line smoothing finished in {t_smoothing - t_sorting} s')

        return smoothed_line
    
    def run(self):

        # init

        inflation_time = 0

        # inflate from wall

        time = self.inflate(source=WALL)
        inflation_time += time

        # inflation from pad

        running = True
        while running:
            time = self.inflate(source=PAD)
            inflation_time += time
            running = not self.check_inflation_end()
            self.previous_grid = deepcopy(self.grid)

        self.get_logger().info(f'Inflation is done, time is {inflation_time} s')

        self.get_logger().info(f'Found {(self.grid == LINE).sum()} line points, proceeding to sorting')

        line = self.occupancy2line()

        self.get_logger().info(f'Received smooth line of {len(line)} points, proceeding to filtration')

        i_line = self.interpolate_line(line,distance=20)

        self.get_logger().info(f'Received filtered line of {len(i_line)} points')

        self.line = self.np2path(i_line)
        self.publishing = True

        #position = np.array([92,106])

        #cp = self.find_closest_point(ref_point=position,line=i_line,show=True)

        #print(cp)



def main(args=None):
    rclpy.init()
    model = CenterLinePublisher()
    rclpy.spin(model)
    rclpy.shutdown()
