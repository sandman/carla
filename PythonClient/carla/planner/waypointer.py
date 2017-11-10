#!/usr/bin/env python3

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB), and the INTEL Visual Computing Lab.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from planner import Planner
from graph import *
from astar import *
import numpy as np
import bezier
from numpy import linalg as LA
import math
import time
import os
import random

class Config(object):

	# GENERAL THINGS

	def __init__(self,city_name):
		# WAYPOINT ----- TRAJECTORY CONFIGURATION

		dir_path = os.path.dirname(__file__)
		self.city_file = dir_path +'/' + city_name + '.txt'
		self.city_map_file = dir_path + '/' + city_name + '.png'

		extra_spacing = (random.randint(0,4)-2)
		self.lane_shift_distance = 13 + extra_spacing   # The amount of shifting from the center the car should go
		self.extra_spacing_rights = -2 - extra_spacing
		self.extra_spacing_lefts = 10 + extra_spacing
		self.way_key_points_predicted = 7
		self.number_of_waypoints = 100



""" This class that specialize the planner to produce waypoints



"""
def angle_between(v1,v2):
    return np.arccos(np.dot(v1,v2) / np.linalg.norm(v1) / np.linalg.norm(v2))


def distance(t1,t2):
	return math.sqrt((t1[0]-t2[0])*(t1[0]-t2[0]) + (t1[1]-t2[1])*(t1[1]-t2[1]))


class Waypointer(Planner):



	def __init__(self,city_name,debug=True):

		config = Config(city_name)
		self.debug = debug
		self.city_file=config.city_file
		self.map_file= config.city_map_file
		# Define here some specific configuration to produce waypoints
		self.last_trajc =[]
		self.lane_shift_distance = config.lane_shift_distance # The amount of shifting from the center the car should go
		self.extra_spacing_rights = config.extra_spacing_rights
		self.extra_spacing_lefts = config.extra_spacing_lefts
		self.way_key_points_predicted = config.way_key_points_predicted
		self.number_of_waypoints = config.number_of_waypoints
		self.previous_map = [0,0]
		Planner.__init__(self,self.city_file,self.map_file) # Initialize the super class
		if debug:

			self.search_image = self.map_image.astype(np.uint8)


	def reset(self):
		self.last_trajc=[]
		self.route=[]
		self.previous_map = [0,0]
		self.previous_source = None
		self.grid = self.make_grid()
		self.walls = self.make_walls()

	def _write_point_on_map(self,position,color,size=4):

		for i in range(0,size):
			self.search_image[int(position[1]),int(position[0])]=color
			self.search_image[int(position[1])+i,int(position[0])]=color
			self.search_image[int(position[1]),int(position[0])+i]=color
			self.search_image[int(position[1])-i,int(position[0])]=color
			self.search_image[int(position[1]),int(position[0])-i]=color
			self.search_image[int(position[1])+i,int(position[0])+i]=color
			self.search_image[int(position[1])-i,int(position[0])-i]=color
			self.search_image[int(position[1])+i,int(position[0])-i]=color
			self.search_image[int(position[1])-i,int(position[0])+i]=color


	def _print_trajectory(self,points,color,size):


		#points = np.transpose(points)
		for point in points:

			self._write_point_on_map(point,color,size)


	def _search_around_square(self,map_point,map_central_2d):

		x = int(map_point[0])
		y = int(map_point[1])

		square_crop=map_central_2d[(y-20):(y+20),(x-20):(x+20)]
		small_distance = 10000
		closest_point = None

		for t in np.transpose(np.nonzero(square_crop)):

			distance = sldist(t,[square_crop.shape[1]/2,square_crop.shape[0]/2])

			if distance < small_distance:
				small_distance = distance

				closest_point = [t[0]-square_crop.shape[1]/2,t[1]-square_crop.shape[0]/2]


		return np.array([x+closest_point[0],y +closest_point[1]])


	def _shift_points(self,distance_to_center,lane_points,inflection_position):





		shifted_lane_vec =[]


		for i in range(len(lane_points[:-1])):
			#if cross > 0:
			# right side
			lane_point = lane_points[i]
			unit_vec =self._get_unit(lane_points[i+1],lane_points[i])


			shifted_lane = [lane_point[0]+unit_vec[0]*distance_to_center[i],lane_point[1]+unit_vec[1]*distance_to_center[i]]

			if i == inflection_position: # One interesting thing is to replicate the point whre the turn happens

				unit_vec =self._get_unit(lane_points[i],lane_points[i-1])

				shifted_lane_vec.append([lane_point[0]+unit_vec[0]*distance_to_center[i],lane_point[1]+unit_vec[1]*distance_to_center[i]])




			shifted_lane_vec.append(shifted_lane)


		last_lane_point = lane_points[-1]
		shifted_lane = [last_lane_point[0]+unit_vec[0]*distance_to_center[i],last_lane_point[1]+unit_vec[1]*distance_to_center[i]]

		shifted_lane_vec.append(shifted_lane)
		return shifted_lane_vec


	# Given a list, find the 3 curve points that this list correspond

	def _find_curve_points(self,points):

		curve_points = None
		first_time = True
		inflection_point = None
		for i in range(len(points)-1):

			unit_vec =self._get_unit(points[i+1],points[i])
			unit_vec = [round(unit_vec[0]),round(unit_vec[1])]

			if not first_time:

				if unit_vec != prev_unit_vec:

					curve_points= [points[i+1],points[i],points[i-1]]
					return curve_points,[i+1,i,i-1],np.cross(unit_vec,prev_unit_vec)

			first_time = False
			prev_unit_vec = unit_vec

		return curve_points,None,None#,inflection_point



	def _get_unit(self,last_pos,first_pos):


		vector_dir = ((last_pos-first_pos)/LA.norm(last_pos-first_pos))
		vector_s_dir = [0,0]
		vector_s_dir[0] = -vector_dir[1]
		vector_s_dir[1]= vector_dir[0]


		return 	vector_s_dir


	def generate_final_trajectory(self,coarse_trajectory):



		total_course_trajectory_distance = 0
		previous_point = coarse_trajectory[0]
		for i in range(1,len(coarse_trajectory)):
			total_course_trajectory_distance += sldist(coarse_trajectory[i],previous_point)



		points = bezier.bezier_curve(coarse_trajectory,max(1,int(total_course_trajectory_distance/10.0)))
		world_points = []
		points = np.transpose(points)
		points_list = []
		for point in points:
			world_points.append(self.make_world_map(point))
			points_list.append(point.tolist())

		return world_points,points_list


	def  get_free_node_direction_target(self,pos,pos_ori,source):

		free_nodes = self.get_adjacent_free_nodes(pos)


		added_walls =set()
		heading_start = np.array([pos_ori[0], pos_ori[1]])
		for adj in free_nodes:

			start_to_goal = np.array([adj[0]  - pos[0],  adj[1] - pos[1] ])
			angle = angle_between(heading_start,start_to_goal)

			if (angle < 2 and adj !=source) :


				added_walls.add((adj[0],adj[1]))


		return added_walls

	def project_source(self,source,source_ori):
		node_source = self.make_node(source)


		source_ori = np.array([source_ori[0],source_ori[1],source_ori[2]])
		source_ori = source_ori.dot(self.worldrotation)


		# Trunkate !
		node_source  =   tuple([ int(x) for x in node_source ])

		# Set to zero if it is less than zero.


		node_source =(max(0,node_source[0]),max(0,node_source[1]))
		node_source =(min(self.resolution[0]-1,node_source[0]),min(self.resolution[1]-1,node_source[1]))
		# is it x or y ? Check to avoid  special corner cases


		if math.fabs(source_ori[0]) > math.fabs(source_ori[1]):
			source_ori = (source_ori[0],0.0,0.0)
		else:
			source_ori = (0.0,source_ori[1],0.0)

		node_source = self.search(node_source[0],node_source[1])


		return node_source

	def project_target(self,target):

		node_target =	self.make_node(target)

		# Trunkate !
		node_target  =   tuple([ int(x) for x in node_target ])
		target_ori   =    self.get_target_ori(target)
		# Set to zero if it is less than zero.



		target_ori = np.array([target_ori[0],target_ori[1],0])
		target_ori = target_ori.dot(self.worldrotation)





		node_target	= self.search(node_target[0],node_target[1])
		return node_target,target_ori

	def get_distance_closest_node_turn(self,pos):
		import collections
		distance  = []
		for node_iter in self.graph.turn_nodes():

			distance.append( sldist(node_iter,pos))

		return sorted(distance)[0]


	def graph_to_waypoints(self, next_route , source_map_position):


		# Take the map with the central lines

		lane_points = []
		for point in next_route:
			map_point = self.make_map_node(point)
			lane_points.append(self._search_around_square(map_point,self.central_path_map_image))


		if self.debug:
			self._print_trajectory(lane_points,[0,0,0,255],7)



		_,points_indexes,curve_direction =  self._find_curve_points(lane_points)


		lan_shift_distance_vec = [self.lane_shift_distance] *len(lane_points)

		if points_indexes != None:
			for i in points_indexes:
				if curve_direction > 0:
					lan_shift_distance_vec[i] +=(self.extra_spacing_lefts*1)
				else:
					lan_shift_distance_vec[i] +=(self.extra_spacing_rights*-1)


			shifted_lane_vec =  self._shift_points(lan_shift_distance_vec,lane_points,points_indexes[1])
		else:
			shifted_lane_vec =  self._shift_points(lan_shift_distance_vec,lane_points,None)



		return shifted_lane_vec


	# Return the route and the necesary walls for computing such a route

	def route_test(self,node_source,source_ori,node_target,target_ori):
		added_walls = self.set_grid_direction(node_source,source_ori,node_target)

		added_walls=added_walls.union(self.set_grid_direction_target(node_target,target_ori,node_source))



		self.a_star =AStar()
		self.init(node_source, node_target)
		route = self.solve()
		for i in added_walls:
			self.walls.remove(i)

			self.grid[i[0],i[1]] = 0.0


		return not route == None


	def route_compute(self,node_source,source_ori,node_target,target_ori):
		added_walls = self.set_grid_direction(node_source,source_ori,node_target)

		added_walls=added_walls.union(self.set_grid_direction_target(node_target,target_ori,node_source))



		self.previous_source = node_source


		self.a_star =AStar()
		self.init(node_source, node_target)
		self.route = self.solve()

		if self.route == None:
			for i in added_walls:
				self.walls.remove(i)

				self.grid[i[0],i[1]] = 0.0
			added_walls = self.set_grid_direction(node_source,source_ori,node_target)
			self.a_star =AStar()
			self.init(node_source, node_target)
			self.route = self.solve()


		for i in added_walls:
			self.walls.remove(i)

			self.grid[i[0],i[1]] = 0.0



		return self.route



	def add_extra_points(self,node_target,target_ori,node_source):

		direction = node_target
		direction_ori =target_ori


		while len(self.route)< 10:# aDD EXTRA POINTS AFTER


			try:
				free_nodes = list(self.get_free_node_direction_target(direction,direction_ori,node_source))

				direction_ori = self._get_unit(np.array(direction),np.array(free_nodes[0]))
				aux = -direction_ori[1]
				direction_ori[1] = direction_ori[0]
				direction_ori[0] = aux

				direction = free_nodes[0]
			except:
				# Repeate some route point, there is no problem.
				direction = [round(self.route[-1][0] + direction_ori[0]),round(self.route[-1][1] + direction_ori[1])]


			self.route.append(direction)




	def get_next_waypoints(self,source,source_ori,  target,target_ori):
		node_source = self.project_source(source,source_ori)

		node_target,target_ori = self.project_target(target)


		if node_source == node_target:

			return self.last_trajc


		# This is to avoid computing a new route when inside the route
		distance_node = self.get_distance_closest_node_turn(node_source)
		if self.debug: # Write the point
			self._write_point_on_map(self.make_map_world(source), [255,0,255,255],size=7)

			self._write_point_on_map(self.make_map_world(target), [255,255,255,255],size=6)

		if distance_node >2 and self.previous_source != node_source:

			self.route_compute(node_source,source_ori,node_target,target_ori)


			# IF needed we add points after the objective
			self.add_extra_points(node_target,target_ori,node_source)


			if self.debug:
				self.search_image = self.map_image.astype(np.uint8)

			self.points = self.graph_to_waypoints(self.route[1:(1+self.way_key_points_predicted)],self.make_map_world(source))
			self.last_trajc,self.last_map_points = self.generate_final_trajectory([np.array(self.make_map_world(source))] +self.points)



			return self.last_trajc #self.generate_final_trajectory([np.array(self.make_map_world(source))] +self.points)


		else:
			if distance(self.previous_map,self.make_map_world(source)) > 3.0:


				# That is because no route was ever computed. This is a problem we should solve.
				if  not self.route:
					self.route_compute(node_source,source_ori,node_target,target_ori)
					self.add_extra_points(node_target,target_ori,node_source)

					if self.debug:
						self.search_image = self.map_image.astype(np.uint8)
					self.points = self.graph_to_waypoints(self.route[1:(1+self.way_key_points_predicted)],self.make_map_world(source))



					self.last_trajc,self.last_map_points = self.generate_final_trajectory([np.array(self.make_map_world(source))] +self.points)



				# We have to find the current node position
				self.previous_map = self.make_map_world(source)

				for point in self.last_map_points:
					point_vec = self._get_unit(np.array(self.make_map_world(source)),point)
					cross_product = np.cross(source_ori[0:2],point_vec)


					if  (cross_product > 0.0 and sldist(point,self.make_map_world(source)) < 50) or sldist(point,self.make_map_world(source)) < 15.0:

						self.last_trajc.remove(self.make_world_map(point))# = [self.make_world_map(point)] + self.last_trajc
						self.last_map_points.remove(point)

				if self.debug:
					self._print_trajectory(self.last_map_points,[255,0,0,255],4)

					self._print_trajectory(self.points,[255,128,0,255],7)



			return self.last_trajc

	# This function uses the map to test if some specific position is too close to intersections

	def test_position(self,source,source_ori):


		node_source = self.project_source(source,source_ori)



		distance_node = self.get_distance_closest_node_turn(node_source)
		if distance_node > 2:
			return True
		else:
			return False

	# Test if distance is larger than a certain limit

	def test_pair(self,source,source_ori,target):

		node_source = self.project_source(source,source_ori)

		node_target,target_ori = self.project_target(target)


		return self.route_test(node_source,source_ori,node_target,target_ori)

    def find_valid_episode_position(self,positions,minimun_distance):
        found_match = False
        while not found_match:
            index_start = np.random.randint(len(positions))
            start_pos =positions[index_start]
            if not self.test_position((start_pos.location.x,start_pos.location.y,22),\
                (start_pos.orientation.x,start_pos.orientation.y,start_pos.orientation.z)):
                continue
            index_goal = np.random.randint(len(positions))
            if index_goal == index_start:
                continue


            goals_pos =positions[index_goal]
            if not self.test_position((goals_pos.location.x,goals_pos.location.y,22),\
                (goals_pos.orientation.x,goals_pos.orientation.y,goals_pos.orientation.z)):
                continue
            if sldist([start_pos.location.x,start_pos.location.y],[goals_pos.location.x,goals_pos.location.y]) < minimun_distance:
                continue

            if self.test_pair((start_pos.location.x,start_pos.location.y,22)\
                ,(start_pos.orientation.x,start_pos.orientation.y,start_pos.orientation.z),\
                (goals_pos.location.x,goals_pos.location.y,22)):
                found_match=True
            self.reset()

        return index_start,index_goal
