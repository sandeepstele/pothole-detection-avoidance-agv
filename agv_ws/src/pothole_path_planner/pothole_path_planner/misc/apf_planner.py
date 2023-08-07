from .vector_2d import Vector2d
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64MultiArray


class APFPlanner():
    def __init__(self,  k_att: float, k_rep: float,
                 step_size: float, max_iters: int, goal_threshold: float):

        self.k_att = k_att
        self.k_rep = k_rep

        self.max_iters = max_iters
        self.iters = 0

        self.step_size = step_size
        self.step_size_ = 2

        self.goal_threshold = goal_threshold
        self.is_path_plan_success = False

        self.delta_t = 0.01

        self.goal_point = None
        self.planner_current_pos = None
        self.car_width = 2.0

        self.isFirst = True
        self.path = []

        self.isPlot = True



    def attractive(self, start_point, goal_point):
        # calculating attractive force
        att = (goal_point - start_point) * self.k_att 
        return att
   

    def repulsion(self, pothole_center, pothole_radius):
        rep1 = Vector2d(0, 0)  

        for i in range(len(pothole_center)):
            obs_dis = self.planner_current_pos - Vector2d(pothole_center[i][0], pothole_center[i][1])
            # obs_dis = self.planner_current_pos - Vector2d(pothole_center[i][0], pothole_center[i][1])

            if (obs_dis.length > pothole_radius[i]):  
                pass
            else:
                rep1 += Vector2d(obs_dis.direction[0], obs_dis.direction[1]) * self.k_rep * (
                        1.0 / obs_dis.length - 1.0 / pothole_radius[i]) / (obs_dis.length ** 2)  
               
        return (rep1)
    
   
    def repulsion_segments(self,segment):
        rep2 = Vector2d(0, 0)  
        for i in range(len(segment)):
            obs_dis = self.planner_current_pos - Vector2d(segment[i][0], segment[i][1])
            if (obs_dis.length > 0.5):  
                pass
            else:
                rep2 += Vector2d(obs_dis.direction[0], obs_dis.direction[1]) * self.k_rep * (
                            1.0 / obs_dis.length - 1.0 / 0.5) / (obs_dis.length ** 2) * 1.0
           
        return (rep2)


   

    # def run_planner(self, car_current_pos):
    #         self.goal_point = [car_current_pos[0]+7.0, car_current_pos[1]]

 
    def apf_path_plan(self, car_current_pos,  goal_point, segment, pothole_center, pothole_radius, isFirst= False ):
        self.path = []
        velocity =[]
        goal_point = Vector2d(goal_point[0],goal_point[1])
        if self.isFirst:
            self.planner_current_pos = car_current_pos
            self.isFirst = False
            self.planner_current_pos = Vector2d(self.planner_current_pos[0], self.planner_current_pos[1])
       

        while (self.iters < self.max_iters and (self.planner_current_pos-goal_point).length > self.goal_threshold):
            f_vec = self.attractive(self.planner_current_pos, goal_point) +(
                    self.repulsion(pothole_center, pothole_radius))+ (
                    self.repulsion_segments(segment))
            self.planner_current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size

            # if self.planner_current_pos.length >= 2/3*(self.goal_point - self.planner_current_pos).length:
            #     self.run_planner(car_current_pos) ##this should be the updated current position
                                                         
            self.iters += 1
            self.path.append([self.planner_current_pos.deltaX, self.planner_current_pos.deltaY])
            velocity.append(1)

            # else:
            #     self.iters += 1
            #     self.path.append([self.planner_current_pos.deltaX, self.planner_current_pos.deltaY])

            plt.plot(np.array(segment)[:,0], np.array(segment)[:,1])
            plt.plot(self.planner_current_pos.deltaX, self.planner_current_pos.deltaY, '.b')
            # plt.show()
            plt.pause(self.delta_t)


        if (self.planner_current_pos - goal_point).length <= self.goal_threshold:
            fig = plt.figure(figsize=(7, 7))
            subplot = fig.add_subplot(111)
            subplot.set_xlabel('X-distance: m')
            subplot.set_ylabel('Y-distance: m')
            subplot.plot(car_current_pos[0], car_current_pos[1], '*r')
            subplot.plot(goal_point.deltaX, goal_point.deltaY, '*r')

            self.path_ = []
            velocity_=[]
            i = 0

            while (i < len(self.path)):
                self.path_.append(self.path[i])
                velocity_.append(velocity[i])
                i += int(self.step_size_ / self.step_size)

            if self.path_[-1] != self.path[-1]:  
                self.path_.append(self.path[-1])
                velocity_.append(velocity[-1])

            path_msg = Float64MultiArray()
            path_msg_data = []
            for point in self.path_:
                path_msg_data.append(point[0])
                path_msg_data.append(point[1])
            path_msg.data = path_msg_data
            return path_msg , velocity_ #need to publish this
        return False, False
       


    def align_center(self, car_current_pos, goal_point, pothole_center, segment, isFirst = False):
        if isFirst:
            self.planner_current_pos = car_current_pos
            self.isFirst = False
            self.planner_current_pos = Vector2d(self.planner_current_pos[0], self.planner_current_pos[1])
        self.path = []
        velocity=[]

 
        goal_point = Vector2d(goal_point[0], goal_point[1])
        pothole_center = Vector2d(pothole_center[0][0], pothole_center[0][1])

        while (self.iters < self.max_iters and (self.planner_current_pos-goal_point).length > self.goal_threshold):
            error=0.15
            f_vec = self.attractive(self.planner_current_pos, goal_point) + self.repulsion_segments(segment)
            #check the distance between the current position and the pothole center, if said distance is less than 2m then we align
            distance= (self.planner_current_pos-pothole_center).length 
            # if self.planner_current_pos.length >= 2/3*(self.goal_point - self.planner_current_pos).length:
            #     self.run_planner(car_current_pos) ##this should be the updated current position  

            if (distance < 2.0) :
                self.planner_current_pos = Vector2d(pothole_center.deltaX, pothole_center.deltaY)
                self.iters += 1
                self.path.append([self.planner_current_pos.deltaX, self.planner_current_pos.deltaY])
                velocity.append(1)

                unitVector = (goal_point - self.planner_current_pos).direction
                unitVector = Vector2d(2.2*unitVector[0],2.2*unitVector[1])

                self.planner_current_pos = Vector2d(pothole_center.deltaX +unitVector.deltaX,
                                                    pothole_center.deltaY+unitVector.deltaY)
                self.path.append([self.planner_current_pos.deltaX ,
                                self.planner_current_pos.deltaY])
                velocity.append(1)
               
            else:
                self.planner_current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
                self.iters += 1
                self.path.append([self.planner_current_pos.deltaX, self.planner_current_pos.deltaY])
                velocity.append(1)
           
            plt.plot(np.array(segment)[:,0], np.array(segment)[:,1])
            plt.plot(self.planner_current_pos.deltaX, self.planner_current_pos.deltaY, '.b')
            # plt.show()
            plt.pause(self.delta_t)

        if (self.planner_current_pos - goal_point).length <= self.goal_threshold:
            self.path_ = []
            velocity_=[]
            i = int(self.step_size_ / self.step_size)
          

            while (i < len(self.path)):
                self.path_.append(self.path[i])
                velocity_.append(velocity[i])
                i += int(self.step_size_ / self.step_size)

            if self.path_[-1] != self.path[-1]:  
                self.path_.append(self.path[-1])
                velocity_.append(velocity[-1])

            # self.path_.insert(0, [0,0])

            path_msg = Float64MultiArray()
            path_msg_data = []

            for point in self.path_:
                path_msg_data.append(float(point[0]))
                path_msg_data.append(float(point[1]))

            path_msg.data = path_msg_data
            return [[self.path_[2*i], self.path_[2*i+1], velocity_[i]] for i in range(len(velocity_))] #need to publish this
       
        return None
   
#    self.distances_right, self.distances_left,
#    self.car_current_pos, self.goal_point, self.pothole_center, self.pothole_depth, self.segment,
#       isFirst = True
   
    def stop_at_pothole(self,distance1,distance2, car_current_pos, goal_point, pothole_center, Depth, segment,  isFirst = True):
        self.path = []
        velocity=[]
        box=[[5.5,5.7,0,4.5,4.3,0]] #define the points for testing
        car_width=2.0

        if isFirst:
            self.planner_current_pos = car_current_pos
            isFirst = False
            self.planner_current_pos = Vector2d(self.planner_current_pos[0], self.planner_current_pos[1])
   
        goal_point = Vector2d(goal_point[0], goal_point[1])
    
        new_position = [1,69]
        pothole_Center=Vector2d(pothole_center[0][0], pothole_center[0][1])

        while (self.iters < self.max_iters and (self.planner_current_pos-goal_point).length > self.goal_threshold):

            f_vec = self.attractive(self.planner_current_pos, goal_point) + self.repulsion_segments(segment)
            distance= (self.planner_current_pos-pothole_Center).length #need this distance to check when will we be stopping
            
            # if self.planner_current_pos.length >= 2/3*(self.goal_point - self.planner_current_pos).length:
            #     self.run_planner(car_current_pos) ##this should be the updated current position
                                                        
            # for i in range(int(len(distance1))):

                # if (distance < 3.0 or distance1[i]< car_width or distance2[i]< car_width) :
            if Depth[0] > 0.1  and Depth[0] < 0.3:

                #change speed to 2km/hour and keep moving forward
                speed = 0.5 #takes it in m/s
                self.planner_current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
                self.iters += 1
                self.path.append([self.planner_current_pos.deltaX,
                                self.planner_current_pos.deltaY])
                velocity.append(speed)

            elif Depth[0]>0.3:
                error=0.15
                self.iters += 1
                speed=0
                new_position=Vector2d(pothole_Center.deltaX - error, pothole_Center.deltaY  - error)
                self.planner_current_pos= new_position
                self.path.append([self.planner_current_pos.deltaX,self.planner_current_pos.deltaY])
                #velocity=0
                velocity.append(speed)

            else:
                self.planner_current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
                self.iters += 1
                self.path.append([self.planner_current_pos.deltaX, self.planner_current_pos.deltaY])
                velocity.append(1)

    

            plt.plot(np.array(segment)[:,0], np.array(segment)[:,1])
            plt.plot(self.planner_current_pos.deltaX, self.planner_current_pos.deltaY, '.b')
            plt.pause(self.delta_t)
        
        if (self.planner_current_pos - goal_point).length <= self.goal_threshold:
            self.path_ = []
            i = int(self.step_size_ / self.step_size)
            velocity_ = []

            while (i < len(self.path)):
                self.path_.append(self.path[i])
                velocity_.append(velocity[i])
                i += int(self.step_size_ / self.step_size)

            if self.path_[-1] != self.path[-1]:  
                self.path_.append(self.path[-1])
                velocity_.append(velocity[-1])

            path_msg = Float64MultiArray()
            path_msg_data = []

            for point in self.path_:
                path_msg_data.append(float(point[0]))
                path_msg_data.append(float(point[1]))

            path_msg.data = path_msg_data
            return path_msg , velocity_ #need to publish this

        return None, velocity
    



    ### iterate over all the potholes, treat each function so that it only takes one pothole at a time
    ## everytime we have a pothole call the desired planner on that pothole alone
    ##to do so:
    ## loop for potholes
    ##      do the condtions required for that pothole
    ##


def main(args=None):
    k_att, k_rep = 1.0, 100.0
    step_size, max_iters, goal_threshold = .5, 500, .5 
    apf_planner = APFPlanner(k_att, k_rep,
                step_size, max_iters, goal_threshold)

if __name__=="__main__":
    main()