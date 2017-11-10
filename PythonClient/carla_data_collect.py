import traceback

import sys


sys.path.append('drive_interfaces')
sys.path.append('drive_interfaces/carla')
sys.path.append('drive_interfaces/carla/carla_client')
sys.path.append('drive_interfaces/carla/comercial_cars')

sys.path.append('drive_interfaces/carla/virtual_elektra')
sys.path.append('drive_interfaces/gta_interface')
sys.path.append('drive_interfaces/deeprc_interface')
sys.path.append('drive_interfaces/carla/carla_client/testing')
sys.path.append('test_interfaces')
sys.path.append('utils')
sys.path.append('dataset_manipulation')
sys.path.append('configuration')
sys.path.append('structures')
sys.path.append('evaluation')


import math
import argparse
from noiser import Noiser
import configparser
import datetime

from screen_manager import ScreenManager

import numpy as np
import os
import time


#from config import *
#from eConfig import *
from drawing_tools import *
from extra import *
pygame.init()
clock = pygame.time.Clock()
def frame2numpy(frame, frameSize):
	return np.resize(np.fromstring(frame, dtype='uint8'), (frameSize[1], frameSize[0], 3))


def get_camera_dict(ini_file):
	config = configparser.ConfigParser()
	config.read(ini_file)
	cameras =  config['CARLA/SceneCapture']['Cameras']
	camera_dict = {}
	cameras = cameras.split(',')
	print cameras
	for i in range(len(cameras)):

		angle = config['CARLA/SceneCapture/' + cameras[i]]['CameraRotationYaw']
		camera_dict.update({i:(cameras[i],angle)})

	return camera_dict

# TODO: TURN this into A FACTORY CLASS
"""
def get_instance(drive_config,experiment_name,drivers_name,memory_use):

	if drive_config.interface == "Carla":

		from carla_recorder import Recorder

		if drive_config.type_of_driver == "Human":
			from carla_human import CarlaHuman
			driver = CarlaHuman(drive_config)
		else:
			from carla_machine import CarlaMachine
			driver = CarlaMachine("0",experiment_name,drive_config,memory_use)

	if drive_config.interface == "VirtualElektra":

		from carla_recorder import Recorder

		if drive_config.type_of_driver == "Human":
			from virtual_elektra_human import VirtualElektraHuman
			driver = VirtualElektraHuman(drive_config)
		else:
			from virtual_elektra_machine import VirtualElektraMachine
			driver = VirtualElektraMachine("0",experiment_name,drive_config,memory_use)

	elif drive_config.interface == 'GTA':
		
		
		from gta_recorder import Recorder
		if drive_config.type_of_driver == "Human":
			from gta_human import GTAHuman
			driver = GTAHuman()
		else:
			from gta_machine import GTAMachine
			driver = GTAMachine("0",experiment_name)

	elif  drive_config.interface == 'DeepRC':

		from deeprc_recorder import Recorder
		if drive_config.type_of_driver == "Human":
			from deeprc_human import DeepRCHuman
			driver = DeepRCHuman(drive_config)
		else:
			from deeprc_machine import DeepRCMachine
			driver = DeepRCMachine("0",experiment_name,drive_config,memory_use)	

	else:
		print " Not valid interface is set "


	camera_dict = get_camera_dict(drive_config.carla_config)
	print " Camera Dict "
	print camera_dict

	folder_name = str(datetime.datetime.today().year) + str(datetime.datetime.today().month) + str(datetime.datetime.today().day)
	
	if drivers_name is not None:
		folder_name += '_' + drivers_name
	folder_name += '_'+ str(get_latest_file_number(drive_config.path,folder_name))

	recorder = Recorder(drive_config.path + folder_name +'/', drive_config.resolution,\
		image_cut= drive_config.image_cut,camera_dict=camera_dict,record_waypoints=True)

	return driver,recorder


"""

def drive(experiment_name,drive_config,name = None,memory_use=1.0):
	#host,port,gpu_number,path,show_screen,resolution,noise_type,config_path,type_of_driver,experiment_name,city_name,game,drivers_name

		



	driver,recorder = get_instance(drive_config,experiment_name,name,memory_use)

	noiser = Noiser(drive_config.noise)

	print 'before starting'
	driver.start()
	first_time = True


	try:
		while direction != -1:
			capture_time  = time.time()
			measurements,direction = driver.get_sensor_data() # Later it would return more image like [rewards,images,segmentation]
			
			#sensor_data = frame2numpy(image,[800,600])


			
			# Compute now the direction
			if drive_config.show_screen:
				for event in pygame.event.get(): # User did something
					if event.type == pygame.QUIT: # If user clicked close
						done=True # Flag that we are done so we exit this loop


			recording = driver.get_recording()
			driver.get_reset()
			speed = measurements['PlayerMeasurements'].forward_speed
			#actions = driver.compute_action(images.rgb[drive_config.middle_camera],measurements.forward_speed,\
			#driver.compute_direction((measurements.transform.location.x,measurements.transform.location.y,22),\
			#(measurements.transform.orientation.x,measurements.transform.orientation.y,measurements.transform.orientation.z))) #rewards.speed
			#actions = driver.compute_action(images.rgb[drive_config.middle_camera],measurements.forward_speed) #rewards.speed
			actions = driver.compute_action([measurements['BGRA'][drive_config.middle_camera],measurements['Labels'][drive_config.middle_camera]],speed) #rewards.speed

			action_noisy,drifting_time,will_drift = noiser.compute_noise(actions,speed)
			
			#print actions
			if recording:
				
				recorder.record(measurements,actions,action_noisy,direction,driver.get_waypoints())



			if drive_config.show_screen:
				if drive_config.interface == "Carla" or drive_config.interface == "VirtualElektra":
					#for i in range(drive_config.aspect_ratio[0]*drive_config.aspect_ratio[1]):
					print 'fps',1.0/(time.time() - capture_time)
					#print measurements['BGRA'][drive_config.middle_camera].shape
					image = measurements['Labels'][0]
					image = image[:,:,2]*30
					image = image[:,:,np.newaxis]
					#print image.shape
					#image = image[:, :, ::-1]
					image.setflags(write=1)
					screen_manager.plot_camera_steer(image,actions.steer,[0,0])
					#print 'fps',1.0/(time.time() - capture_time)
					#print measurements['BGRA'][drive_config.middle_camera].shape
					image = measurements['BGRA'][0]
					image = image[:,:,:3]
					image = image[:, :, ::-1]
					#print image.shape
					#image = image[:, :, ::-1]
					image.setflags(write=1)
					screen_manager.plot_camera_steer(image,actions.steer,[1,0])

					mid_rep = mid_rep*255
					print mid_rep.shape
					print mid_rep

					screen_manager.plot_camera_steer(mid_rep,actions.steer,[2,0])

				elif drive_config.interface == "GTA":

					dist_to_goal = math.sqrt(( rewards.goal[0]- measurements.position[0]) *(measurements.goal[0] - measurements.position[0]) + (measurements.goal[1] - measurements.position[1]) *(measurements.goal[1] - measurements.position[1]))
					
					image = image[:, :, ::-1]
					screen_manager.plot_driving_interface( capture_time,np.copy(images),	action,action_noisy,\
						direction,recording and (drifting_time == 0.0 or  will_drift),drifting_time,will_drift\
						,measurements.speed,0,0,None,measurements.reseted,driver.get_number_completions(),dist_to_goal,0) #

				elif drive_config.interface == "DeepRC":
					for key,value in drive_config.cameras_to_plot.iteritems():
						screen_manager.plot_driving_interface( capture_time,np.copy(images[key]),\
							actions[key],action_noisy,measurements.direction,recording and (drifting_time == 0.0 or  will_drift),\
							drifting_time,will_drift,measurements.speed,0,0,value) #
				else:
					print "Not supported interface"
					pass

			
			if drive_config.type_of_driver == "Machine" and drive_config.show_screen and drive_config.plot_vbp:

				image_vbp =driver.compute_perception_activations(measurements['BGRA'][drive_config.middle_camera],speed)

				screen_manager.plot_camera(image_vbp,[1,0])


			iteration +=1
			driver.act(action_noisy)

	except:
		traceback.print_exc()

	finally:

		#driver.write_performance_file(path,folder_name,iteration)
		pygame.quit()

def parse_drive_arguments(args,driver_conf):


  # Carla Config
  if args.carla_config is not None:
    driver_conf.carla_config = args.carla_config


  if args.host is not None:
    driver_conf.host = args.host

  if args.port is not None:
    driver_conf.port = int(args.port)

  if args.path is not None:
    driver_conf.path = args.path

  if args.noise is not None:
    driver_conf.noise = args.noise
  if args.driver  is not None:
    driver_conf.type_of_driver = args.driver
  if args.interface is not None:
    driver_conf.interface = args.interface
  if args.number_screens is not None:
    driver_conf.number_screens = args.number_screens
  if args.scale_factor is not None:
    driver_conf.scale_factor = args.scale_factor

  if args.resolution is not None:
    res_string = args.resolution.split(',')
    resolution = []
    resolution.append(int(res_string[0]))
    resolution.append(int(res_string[1]))
    driver_conf.resolution = resolution



  if args.image_cut is not None:
    cut_string = args.image_cut.split(',')
    image_cut = []
    image_cut.append(int(cut_string[0]))
    image_cut.append(int(cut_string[1]))
    driver_conf.image_cut = image_cut



  return driver_conf

  #def main():
  #  parser = ap.ArgumentParser(description="My Script")
  #  parser.add_argument("--myArg")
  #  args, leftovers = parser.parse_known_args()

  #  if args.myArg is not None:
  #      print "myArg has been set (value is %s)" % args.myArg


if __name__ == '__main__':


  parser = argparse.ArgumentParser(description='Chauffeur')
  # General
  parser.add_argument('mode', metavar='mode',default='train', type=str, help='what kind of mode you are running')



  parser.add_argument('-g','--gpu', type=str,default="0", help='GPU NUMBER')
  parser.add_argument('-lg', '--log', help="activate the log file",action="store_true") 
  parser.add_argument('-db', '--debug', help="put the log file to screen",action="store_true") 
  parser.add_argument('-e', '--experiment-name', help="The experiment name (NAME.py file should be in configuration folder, and the results will be saved to models/NAME)", default="")
  parser.add_argument('-pt','--path', type=str,default="../Desktop/", help='Path to Store or read outputs')

  # Train 

  parser.add_argument('-m','--memory', default=0.9, help='The amount of memory this process is going to use')



  
  


  # Drive

  parser.add_argument('-dc', '--driver-config',type=str ,help="The configuration of the driving file")
  parser.add_argument('-cc', '--carla-config', help="Carla config file used for driving")
  parser.add_argument('-l', '--host', type=str, help='The IP where DeepGTAV is running')
  parser.add_argument('-p', '--port', help='The port where Any server to be connected is running')  


  parser.add_argument('-nm','--name', type=str,default="Felipe", help='Name of the person who is going to drive')
  parser.add_argument('-sc', '--show_screen',default=True, action="store_true", help='If we are showing the screen of the player')
  parser.add_argument('-res', '--resolution',  help='If we are showing the screen of the player')
  parser.add_argument('-n', '--noise',  help='Set the types of noise:  Spike or None')
  parser.add_argument('--driver',  help='Select who is driving, a human or a machine')
  parser.add_argument('-in','--interface', help='The environment being used as interface')
  parser.add_argument('-cy','--city',type=str,  help='select the graph from the city being used')
  parser.add_argument('-nc','--number_screens', help='Set The number of screens that are being ploted')
  parser.add_argument('-sf','--scale_factor',  help='Set the scale of the ploted screens')
  parser.add_argument('-up','--use_planner',  help='Check if the planner is going to be used')
  parser.add_argument('-imc','--image_cut',  help='Set the positions where the image is cut')


  args = parser.parse_args()
  know_args = parser.parse_known_args()


  
  if args.log or args.debug:
    LOG_FILENAME = 'log_manual_control.log'
    logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)
    if args.debug:  # set of functions to put the logging to screen


      root = logging.getLogger()
      root.setLevel(logging.DEBUG)
      ch = logging.StreamHandler(sys.stdout)
      ch.setLevel(logging.DEBUG)
      formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
      ch.setFormatter(formatter)
      root.addHandler(ch)





  try:

    if args.mode == 'drive':
      from drive import drive
      driver_conf_module  = __import__(args.driver_config)
      driver_conf= driver_conf_module.configDrive()

      driver_conf = parse_drive_arguments(args,driver_conf)
      
      drive(args.experiment_name,driver_conf,args.name,float(args.memory))

    elif args.mode == 'train':

      from train import train
      train(args.gpu, args.experiment_name,args.path,args.memory,int(args.port))
    elif args.mode == 'evaluate':
      from evaluate import evaluate 
      #General evaluation algorithm ( train again for a while and check network stile)
      evaluate(args.gpu, args.experiment_name)
    elif args.mode == 'predict':
      from predict import predict
      driver_conf_module  = __import__(args.driver_config)
      driver_conf= driver_conf_module.configDrive()

      driver_conf = parse_drive_arguments(args,driver_conf)

      predict(args.experiment_name,driver_conf,args.name,float(args.memory))

    elif args.mode == 'test_input':
      from test_input import test_input
      test_input(args.gpu)
    elif args.mode == 'test_train':
      from test_train import test_train
      test_train(args.gpu)
    else: # mode == evaluate
      evaluate.evaluate(args.gpu)

  except KeyboardInterrupt:
    os._exit(1)
    exitapp = True
    raise
