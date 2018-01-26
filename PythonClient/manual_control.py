#!/usr/bin/env python3

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB), and the INTEL Visual Computing Lab.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Keyboard controlling for carla. Please refer to client_example for a simpler
# and more documented example.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake

    R            : restart level

STARTING in a moment...
"""

from __future__ import print_function

import argparse
import logging
import random
import sys
import time
import logging
import scipy.spatial.distance

try:
    import pygame
    from pygame.locals import *
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

from carla import image_converter
from carla import sensor
from carla.client import make_carla_client, VehicleControl
from carla.planner.map import CarlaMap
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line


WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
MINI_WINDOW_WIDTH = 320
MINI_WINDOW_HEIGHT = 180

autopilot = False
mixed_reality = False
hud_active = True
show_post_processing = False
ul_time_delay = 0.2 # Time delay for uplink in seconds

#logging.basicConfig(filename='carlaClient.log', filemode='w', level=logging.DEBUG)


def make_carla_settings():
    """Make a CarlaSettings object with the settings we need."""
    settings = CarlaSettings()
    settings.set(
        SynchronousMode=False,
        SendNonPlayerAgentsInfo=True,
        NumberOfVehicles=15,
        NumberOfPedestrians=30,
        WeatherId=random.choice([1, 3, 7, 8, 14]))
    settings.randomize_seeds()
    camera0 = sensor.Camera('CameraRGB')
    camera0.set_image_size(WINDOW_WIDTH, WINDOW_HEIGHT)
    camera0.set_position(200, 0, 140)
    camera0.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera0)
    camera1 = sensor.Camera('CameraDepth', PostProcessing='Depth')
    camera1.set_image_size(MINI_WINDOW_WIDTH, MINI_WINDOW_HEIGHT)
    camera1.set_position(200, 0, 140)
    camera1.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera1)
    camera2 = sensor.Camera('CameraSemSeg', PostProcessing='SemanticSegmentation')
    if mixed_reality:
        camera2.set_image_size(WINDOW_WIDTH, WINDOW_HEIGHT)
    else:
        camera2.set_image_size(MINI_WINDOW_WIDTH, MINI_WINDOW_HEIGHT)
    camera2.set_position(200, 0, 140)
    camera2.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera2)
    return settings


class Timer(object):
    def __init__(self):
        self.step = 0
        self._lap_step = 0
        self._lap_time = time.time()

    def tick(self):
        self.step += 1

    def lap(self):
        self._lap_step = self.step
        self._lap_time = time.time()

    def ticks_per_second(self):
        return float(self.step - self._lap_step) / self.elapsed_seconds_since_lap()

    def elapsed_seconds_since_lap(self):
        return time.time() - self._lap_time


class CarlaGame(object):
    def __init__(self, carla_client, city_name=None):
        self.client = carla_client
        self._timer = None
        self._display = None
        self._main_image = None
        self._mini_view_image1 = None
        self._mini_view_image2 = None
        self._map_view = None
        self._is_on_reverse = False
        self._city_name = city_name
        self._map = CarlaMap(city_name) if city_name is not None else None
        self._map_shape = self._map.map_image.shape if city_name is not None else None
        self._map_view = self._map.get_map(WINDOW_HEIGHT) if city_name is not None else None

    def execute(self):
        """Launch the PyGame."""
        pygame.init()

        # Adding Joystick controls here
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        axis = self.js.get_axis(1)
        jsInit = self.js.get_init()
        jsId = self.js.get_id()
        print("Joystick ID: %d Init status: %s Axis(1): %d" % (jsId, jsInit, axis))

        self._initialize_game()
        DISPLAYSURF = pygame.display.set_mode((0, 0), pygame.RESIZABLE)

        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                meas = self._on_loop()
                self._on_render(meas)
        finally:
            pygame.quit()

    def _initialize_game(self):
        if self._city_name is not None:
            self._display = pygame.display.set_mode(
                (WINDOW_WIDTH + int((WINDOW_HEIGHT/float(self._map.map_image.shape[0]))*self._map.map_image.shape[1]), WINDOW_HEIGHT),
                pygame.HWSURFACE | pygame.DOUBLEBUF)
        else:
            self._display = pygame.display.set_mode(
                (WINDOW_WIDTH, WINDOW_HEIGHT),
                pygame.HWSURFACE | pygame.DOUBLEBUF)

        logging.debug('pygame started')
        self._on_new_episode()

    def _on_new_episode(self):
        scene = self.client.load_settings(make_carla_settings())
        number_of_player_starts = len(scene.player_start_spots)
        player_start = np.random.randint(number_of_player_starts)
        print('Starting new episode...')
        self.client.start_episode(player_start)
        self._timer = Timer()
        self._is_on_reverse = False

    def _on_loop(self):
        self._timer.tick()
        time.sleep(ul_time_delay)

        measurements, sensor_data = self.client.read_data()

        self._main_image = sensor_data['CameraRGB']
        self._mini_view_image1 = sensor_data['CameraDepth']
        self._mini_view_image2 = sensor_data['CameraSemSeg']




        # Print measurements every second.
        if self._timer.elapsed_seconds_since_lap() > 3.0:
            if self._city_name is not None:
                # Function to get car position on map.
                map_position = self._map.get_position_on_map([
                    measurements.player_measurements.transform.location.x,
                    measurements.player_measurements.transform.location.y,
                    measurements.player_measurements.transform.location.z])
                # Function to get orientation of the road car is in.
                lane_orientation = self._map.get_lane_orientation([
                    measurements.player_measurements.transform.location.x,
                    measurements.player_measurements.transform.location.y,
                    measurements.player_measurements.transform.location.z])

                self._print_player_measurements_map(
                    measurements.player_measurements,
                    map_position,
                    lane_orientation)
            else:
                self._print_player_measurements(measurements.player_measurements)

            # Plot position on the map as well.

            self._timer.lap()

        control = self._get_keyboard_control(pygame.key.get_pressed())

        numAxes = self.js.get_numaxes()
        jsInputs = [ float(self.js.get_axis(i)) for i in range(numAxes)]

        #if time.time() - self.prev_restart_time < 2.:
        #control.throttle = 0.0
        #control.steer = 0.0

        control.steer = jsInputs[0]

        brakeCmd = (((jsInputs[1] - (-1)) * (1.0 - 0)) / (1.0 - (-1.0))) + 0
        throttleCmd = (((jsInputs[2] - (-1)) * (1.0 - 0)) / (1.0 - (-1.0))) + 0

        control.brake = brakeCmd
        control.throttle = throttleCmd
        #logging.info("Control: Brake: %.2f Throttle: %.2f Steer: %.2f", brakeCmd, throttleCmd, control.steer)

        # Set the player position
        if self._city_name is not None:
            self._position = self._map.get_position_on_map([
                        measurements.player_measurements.transform.location.x,
                        measurements.player_measurements.transform.location.y,
                        measurements.player_measurements.transform.location.z])
            self._agent_positions = measurements.non_player_agents

        #logging.info(measurements.non_player_agents)
        #perception_200 = [agent for neighbor in measurements.non_player_agents
        #                  if scipy.spatial.distance.cdist([[measurements.player_measurements.transform.location.x,
        #                  measurements.player_measurements.transform.location.y ]],
        #                  [[neighbor.vehicle.transform.location.x, neighbor.vehicle.transform.location.y]]) < 200]
        #res = [f.name for f in message.DESCRIPTOR.fields]
        #perception = [f.name for f in measurements.non_player_agents.DESCRIPTOR.fields]
        #neighborAgents = []
        #for agent in measurements.non_player_agents:
            #if agent.HasField('vehicle') or agent.HasField('pedestrian'):
                #print(agent.vehicle.transform.location)
                #print (measurements.player_measurements.transform.location)
                #tmpDist =
                #if tmpDist < 100e2:
                #    neighborAgents.append(agent)

        # neighborAgents = [agent for agent in measurements.non_player_agents if scipy.spatial.distance.cdist([[measurements.player_measurements.transform.location.x, \
        #                                           measurements.player_measurements.transform.location.y]], \
        #             [[agent.vehicle.transform.location.x,agent.vehicle.transform.location.y]]) < 50e2]
        #print (neighborAgents.__len__())
        #logging.info(perception)
        if autopilot:
            control = measurements.player_measurements.autopilot_control

        if control is None:
            self._on_new_episode()
        else:
            self.client.send_control(control)

        return measurements

    def _get_keyboard_control(self, keys):
        """
        Return a VehicleControl message based on the pressed keys. Return None
        if a new episode was requested.
        """
        if keys[K_r]:
            return None
        control = VehicleControl()
        if keys[K_LEFT] or keys[K_a]:
            control.steer = -1.0
        if keys[K_RIGHT] or keys[K_d]:
            control.steer = 1.0
        if keys[K_UP] or keys[K_w]:
            control.throttle = 1.0
        if keys[K_DOWN] or keys[K_s]:
            control.brake = 1.0
        if keys[K_SPACE]:
            control.hand_brake = True
        if keys[K_q]:
            self._is_on_reverse = not self._is_on_reverse
        control.reverse = self._is_on_reverse
        return control

    def _print_player_measurements_map(
            self,
            player_measurements,
            map_position,
            lane_orientation):
        message = 'Step {step} ({fps:.1f} FPS): '
        message += 'Map Position ({map_x:.1f},{map_y:.1f}) Lane Orientation ({ori_x:.1f},{ori_y:.1f})  '
        message += '{speed:.2f} km/h, '
        message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road'
        message = message.format(
            map_x=map_position[0],
            map_y=map_position[1],
            ori_x=lane_orientation[0],
            ori_y=lane_orientation[1],
            step=self._timer.step,
            fps=self._timer.ticks_per_second(),
            speed=player_measurements.forward_speed,
            other_lane=100 * player_measurements.intersection_otherlane,
            offroad=100 * player_measurements.intersection_offroad)
        print_over_same_line(message)
        #logging.debug(message)

    def _print_player_measurements(self, player_measurements):
        message = 'Step {step} ({fps:.1f} FPS): '
        message += '{speed:.2f} km/h, '
        message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road'
        message = message.format(
            step=self._timer.step,
            fps=self._timer.ticks_per_second(),
            speed=player_measurements.forward_speed,
            other_lane=100 * player_measurements.intersection_otherlane,
            offroad=100 * player_measurements.intersection_offroad)
        print_over_same_line(message)
        logging.debug(message)

    def _on_render(self, meas):
        gap_x = (WINDOW_WIDTH - 2 * MINI_WINDOW_WIDTH) / 3
        mini_image_y = WINDOW_HEIGHT - MINI_WINDOW_HEIGHT - gap_x

        #meas.pl
        if self._main_image is not None:
            array = image_converter.to_rgb_array(self._main_image)
            #print("RGB Array shape: %s" % (array.shape,))
            mask = np.random.randint(1, size=array.shape)
            array = np.bitwise_xor(array,mask)
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self._display.blit(surface, (0, 0))


        if mixed_reality:
            if self._mini_view_image2 is not None:
                array1 = image_converter.labels_to_cityscapes_palette(
                    self._mini_view_image2)
                surface1 = pygame.surfarray.make_surface(array1.swapaxes(0, 1))
                surface1.set_alpha(128)
                self._display.blit(surface1, (0, 0))
        else:

            if show_post_processing:
                if self._mini_view_image1 is not None:
                    array = image_converter.depth_to_logarithmic_grayscale(self._mini_view_image1)
                    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                    self._display.blit(surface, (gap_x, mini_image_y))

                if self._mini_view_image2 is not None:
                    array = image_converter.labels_to_cityscapes_palette(
                        self._mini_view_image2)
                    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                    self._display.blit(
                        surface, (2 * gap_x + MINI_WINDOW_WIDTH, mini_image_y))


        myfont = pygame.font.SysFont("sans", 25, bold=True)
        label = myfont.render(("Current Speed: %f km/h" % meas.player_measurements.forward_speed), 1, (255, 255, 0))
        self._display.blit(label, (WINDOW_WIDTH/2 , WINDOW_HEIGHT - 40))

        neighborAgents = [agent for agent in meas.non_player_agents if scipy.spatial.distance.cdist([[meas.player_measurements.transform.location.x, \
                                                  meas.player_measurements.transform.location.y]], \
                    [[agent.vehicle.transform.location.x,agent.vehicle.transform.location.y]]) < 50e2]

        myfont = pygame.font.SysFont("sans", 20, bold=True)

        for agent in neighborAgents:
            if agent.HasField('vehicle'):
                # print(agent.vehicle.transform.location)
                label = myfont.render(("Vehicle nearby" ), 1, (255, 0, 255))
                self._display.blit(label, (WINDOW_WIDTH / 2, 40))
            # else:
            #
            #
            # if agent.HasField('pedestrian'):
            #     # print(agent.vehicle.transform.location)
            #     label = myfont.render(("Pedestrian nearby" ), 1, (255, 0, 255))
            #     self._display.blit(label, (WINDOW_WIDTH / 2, 80))


        if self._map_view is not None:
            array = self._map_view
            array = array[:, :, :3]
            new_window_width =(float(WINDOW_HEIGHT)/float(self._map_shape[0]))*float(self._map_shape[1])
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

            w_pos = int(self._position[0]*(float(WINDOW_HEIGHT)/float(self._map_shape[0])))
            h_pos =int(self._position[1] *(new_window_width/float(self._map_shape[1])))

            pygame.draw.circle(surface, [255, 0, 0, 255], (w_pos,h_pos), 6, 0)
            for agent in self._agent_positions:
                if agent.HasField('vehicle'):
                    print("found non-player agent")
                    agent_position = self._map.get_position_on_map([
                        agent.vehicle.transform.location.x,
                        agent.vehicle.transform.location.y,
                        agent.vehicle.transform.location.z])
                    w_pos = int(agent_position[0]*(float(WINDOW_HEIGHT)/float(self._map_shape[0])))
                    h_pos =int(agent_position[1] *(new_window_width/float(self._map_shape[1])))         
                    pygame.draw.circle(surface, [255, 0, 255, 255], (w_pos,h_pos), 4, 0)

            self._display.blit(surface, (WINDOW_WIDTH, 0))

        pygame.display.flip()


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-m', '--map-name',
        metavar='M',
        default=None,
        help='plot the map of the current city (needs to match active map in server, options: Town01 or Town02)')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(filename='carlaClient.log', filemode='w', format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    while True:
        try:

            with make_carla_client(args.host, args.port) as client:
                game = CarlaGame(client, args.map_name)
                game.execute()
                break

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)
        except Exception as exception:
            logging.exception(exception)
            sys.exit(1)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
