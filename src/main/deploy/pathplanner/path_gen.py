# How to use:
#   1. Using the naming convention we already have, name a file with the desired two coordinates.
#	Example Name:
#   	- A2-D1.path
#		- A2.path
#		- A2-D1 Greg.path
#		- A2 Greg.path
#	
#	All of these names should work just fine because It will just read the first part for cords
#
#	2. After writing the file name, be sure to go into the file and add and empty {} inside,
#	this is because it will not be read as a .json file otherwise
#
# This slideshow shows what each point is supposed to be:
# https://docs.google.com/presentation/d/1zT4bFxyZKDe3ofrhip2B1Tj3Z3is9n5AYjpofgDgosI/edit?usp=sharing

import os
import json

settings_file_path = 'src/main/deploy/pathplanner/settings.json'
commands_file_path = 'src/main/deploy/pathplanner/Commands.json'

lables = ['Intake', 'Shoot', 'Amp', 'Drop']

with open(settings_file_path, 'r') as f:
	settings = json.load(f)
	setpoints = settings['Setpoints']
	path_folder = settings['Path Folder']
	auto_folder = settings['Auto Folder']

with open(commands_file_path, 'r') as f:
	commands = json.load(f)
	command_names = [key for key in commands]


def get_file_labels(file_name):
	name_labels = file_name.split('-')
	name_labels = [*name_labels[0].split(' '), *name_labels[1:]]
	for i in range(1, len(name_labels)-1):
		name_labels = [*name_labels[:i], *name_labels[i].split(' '), *name_labels[i+1:]]
	name_labels = [*name_labels[0:-1], *name_labels[-1].split(' ')]
	name_labels = [*name_labels[0:-1], *name_labels[-1].split('.')]
	name_labels = name_labels[:-1]
	return name_labels

def create_path_file(file_name):
	name_labels = get_file_labels(file_name)

	x1 = setpoints[name_labels[0]]['x']
	y1 = setpoints[name_labels[0]]['y']
	r1 = setpoints[name_labels[0]]['r']

	if name_labels[1] in setpoints:
		x2 = setpoints[name_labels[1]]['x']
		y2 = setpoints[name_labels[1]]['y']
		r2 = setpoints[name_labels[1]]['r']
	else:
		x2=x1
		y2=y1
		r2=r1


	cpX1 = x1
	cpY1 = round(y2 + ((y1-y2)/2) if y1 > y2 else y1 + ((y2-y1)/2), 3)

	cpX2 = round(x2 + ((x1-x2)/2) if x1 > x2 else x1 + ((x2-x1)/2), 3)
	cpY2 = y2

	file = {
		'version' : 1.0,
		"waypoints" : [
			{
				"anchor": {
					"x": x1,
					"y": y1
				},
				"prevControl": None,
				"nextControl": {
					"x": cpX1,
					"y": cpY1
				},
				"isLocked": False,
				"linkedName": None
			},
			{
				"anchor": {
					"x": x2,
					"y": y2
				},
				"prevControl": {
					"x": cpX2,
					"y": cpY2
				},
				"nextControl": None,
				"isLocked": False,
				"linkedName": None
			}
		],
		"rotationTargets" : [],
		"constraintZones" : [],
		"eventMarkers" : [],
		"globalConstraints" : {
			"maxVelocity": 3.0,
			"maxAcceleration": 3.0,
			"maxAngularVelocity": 540.0,
			"maxAngularAcceleration": 720.0
		},
		"goalEndState" : {
			"velocity": 0,
			"rotation": r2,
			"rotateFast": False
		},
		"reversed" : False,
		"folder" : None,
		"previewStartingState" : {
			"rotation": r1,
			"velocity": 0
		},
		"useDefaultConstraints" : True
	}
	
	return file

def get_path_json(path):
	path_json = {
		"type": "path",
		"data": {
			"pathName": f'{path[0]}-{path[1]}' + ' '.join(path[2:])
		}
	}
	return path_json

def create_auto_file(nickname, path_list):
	start_pose = setpoints[path_list[0][0]]
	auto = {
		"version": 1.0,
		"startingPose": {
			"position": {
				"x": start_pose['x'],
				"y": start_pose['y']
			},
			"rotation": start_pose['r']
		},
		"command": {
			"type": "sequential",
			"data": {
				"commands": []
			}
		},
		"folder": None,
		"choreoAuto": False
	}

	for path in path_list:
		auto['command']['data']['commands'].append(get_path_json(path))
		auto['command']['data']['commands'].append(command_names[path[2]])
	
	with open(f'{nickname}.auto', 'w') as file:
		json.dump(auto)



not_format_files_num = 0
not_format_files_list = []

path_name_list = []
for filename in os.listdir(path_folder):
	current_file_path = path_folder+'/'+filename
	path_name_list.append(filename)
	try:
		with open(current_file_path, 'r') as file:
			current_file = json.load(file)
		generated_path = create_path_file(filename)
		if str(current_file) == '{}':
			with open(current_file_path, 'w') as file:
				print(f'{current_file_path} was generated')
				json.dump(generated_path, file)
	except KeyError:
		not_format_files_num += 1
		not_format_files_list.append(filename)
if not_format_files_num > 0:
	print(f"The names of {not_format_files_num} files were not formatted properly.")
	for file in not_format_files_list:
		print(f'  -{file}', end='\n\n')

auto_name_list = []
for filename in os.listdir(auto_folder):
	file_name = get_file_labels(filename)
	auto_name_list.append(file_name)
auto_nicknames = [name[0] for name in auto_name_list]
print(auto_nicknames)
	
autos_to_make = {}
for path in (path_name_list):
	current_label = get_file_labels(path)
	if len(current_label) >= 5:
		if current_label[3] not in auto_nicknames:
			if current_label[3] in autos_to_make:
				autos_to_make[current_label[3]].append(current_label)
			else: 
				autos_to_make[current_label[3]] = [current_label]

for key in autos_to_make:
	create_auto_file(key, autos_to_make[key])

print('Done!')