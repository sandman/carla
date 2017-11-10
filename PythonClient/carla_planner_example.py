"""
    Sample Agent controlling for carla. Please refer to carla_use_example for a simpler and more
    documented example.

"""
from __future__ import print_function
from carla import CARLA


#from scene_parameters import SceneParams

from PIL import Image
import numpy as np

import os
import random
import time
import sys
import argparse
import logging
from socket import error as socket_error
import scipy
from carla import Waypointer

from carla import Control,Measurements

import math
#from noiser import Noiser


import pygame
from pygame.locals import *


sldist = lambda c1, c2: math.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)

def test_reach_goal(player,goal):
    distance = sldist([player.location.x,player.location.y],[goal.location.x,goal.location.y])
    if distance < 300.0:
        return True
    else:
        return False







class App:
    def __init__(self, port=2000, host='127.0.0.1', config='./CarlaSettings.ini',\
    city= 'town01',verbose=True):

        self._running = True
        self._display_surf = None
        self.port = port
        self.host = host
        self.ini = config
        self.verbose = verbose

        # the waypointer receives a configuration script. This script
        # is placed

        self.waypointer = Waypointer(city)

        # Resolution should be two times the image size
        self.size = self.weight, self.height = [1600,600]



    def on_init(self):
        print (" \n \n \n Welcome to CARLA planner example \n USE ARROWS for control \n Press R for reset \n"\
        +"The goal is represented by a WHITE star \n  STARTING in a few seconds...")
        time.sleep(3)
        pygame.init()

        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._running = True
        self.step = 0
        self.prev_step = 0
        self.prev_time = time.time()

        self.carla =CARLA(self.host, self.port)
        self.positions = self.carla.loadConfigurationFile(self.ini)

        # It is necessary to find valid positions, the planner is not robust
        # to positions that are close to intersections, as they have some ambiguity
        # to be worked on.

        self.index_start,self.index_goal=self.waypointer.find_valid_episode_position(self.positions,self.agent.waypointer)
        print ('Starting on Position %d, Goal is position %d')
        self.carla.newEpisode(self.index_start)

        self.prev_restart_time = time.time()

    def on_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False

    def _parse_keys(self):
        restart = False
        control = Control()

        keys=pygame.key.get_pressed()
        if keys[K_LEFT]:
            control.steer = -1.

        if keys[K_RIGHT]:

            control.steer = 1.
        if keys[K_UP]:

            control.throttle = 1.
        if keys[K_DOWN]:

            control.brake = 1.
        if keys[K_r]:

            if time.time() - self.prev_restart_time > 2.:
                self.prev_restart_time = time.time()
                restart = True

        return control,restart
    def on_loop(self):
        self.step += 1

        control,restart =self._parse_keys()
        measurements = self.carla.getMeasurements()

        player_meas = measurements['PlayerMeasurements']
        self.img_vec = measurements['BGRA']

        target = self.positions[self.index_goal]

        # Here is the main waypoint function. You send the player current positions
        # and orientation, and the target position and orientation

        # it returns a set of 3D in world coordinates that are the next positions
        # client should go in order to reach the goal.

        waypoints = self.waypointer.get_next_waypoints(player_meas.transform.location,\
        player_meas.transform.orientation,  target.location,target.orientation)
        # Here is the main Planner Function. It is a super class of the waypointer
        # you could also call
        #

        # If you reach goal, restart
        if test_reach_goal(pack.transform,self.positions[self.index_goal]):

            self.waypointer.reset()
            self.index_start,self.index_goal =  find_valid_episode_position(self.positions,self.waypointer)

            self.carla.newEpisode(self.index_start)



        self.carla.sendCommand(control)


        if time.time() - self.prev_time > 1.:
            print('Step', self.step, 'FPS', float(self.step - self.prev_step) / (time.time() - self.prev_time))

            print('speed', pack.forward_speed, 'collision', pack.collision_other, \
                'collision_car', pack.collision_vehicles, 'colision_ped', pack.collision_pedestrians)
            self.prev_step = self.step
            self.prev_time = time.time()

        if restart:
            print('\n *** RESTART *** \n')

            player_pos = np.random.randint(self.num_pos)


            print('  Player pos %d' % (player_pos))
            self.carla.newEpisode(player_pos)




    def on_render(self):

        pos_x =0

        if len(self.img_vec) > 0:
            self.img_vec[0] = self.img_vec[0][:,:,:3]
            self.img_vec[0] = self.img_vec[0][:,:,::-1]
            surface = pygame.surfarray.make_surface(np.transpose(self.img_vec[0], (1,0,2)))
            self._display_surf.blit(surface,(pos_x,0))
            pos_x += self.img_vec[0].shape[1]
        if self.plot_map:
            #print (self.waypointer.search_image)
            search_image =self.waypointer.search_image[:,:,:3]

            # get correct size
            search_image= scipy.misc.imresize(search_image,[800,600])

            surface = pygame.surfarray.make_surface(np.transpose(search_image, (1,0,2)))
            self._display_surf.blit(surface,(pos_x,0))



        pygame.display.flip()


    def on_cleanup(self):
        self.carla.closeConections()
        pygame.quit()

    def on_execute(self):
        if self.on_init() == False:
            self._running = False

        while( self._running ):
            #try:

            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
            self.on_render()

            #except Exception:
                #   self._running = False
                #break


        self.on_cleanup()





if __name__ == "__main__" :
    parser = argparse.ArgumentParser(description='Run multiple servers on multiple GPUs')
    parser.add_argument('host', metavar='HOST', type=str, help='host to connect to')
    parser.add_argument('port', metavar='PORT', type=int, help='port to connect to')


    parser.add_argument("-c", "--config", help="the path for the server config file that the client sends",type=str,default="./CarlaSettings.ini")



    parser.add_argument("-l", "--log", help="activate the log file",action="store_true")
    parser.add_argument("-lv", "--log_verbose", help="put the log file to screen",action="store_true")

    parser.add_argument("-cy", "--city",default="town01", help="activate the log file",action="store_true")

    args = parser.parse_args()

    print(args)

    if args.log or args.log_verbose:
        LOG_FILENAME = 'log_manual_control.log'
        logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)
        if args.log_verbose:  # set of functions to put the logging to screen


            root = logging.getLogger()
            root.setLevel(logging.DEBUG)
            ch = logging.StreamHandler(sys.stdout)
            ch.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            ch.setFormatter(formatter)
            root.addHandler(ch)



    theApp = App(port=args.port, host=args.host, config=args.config,city = args.city)
    theApp.on_execute()
