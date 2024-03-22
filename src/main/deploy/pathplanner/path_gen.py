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

lables = ['Intake', 'Shoot', 'Amp', 'Drop']

with open(settings_file_path, 'r') as f:
	settings = json.load(f)
	setpoints = settings['Setpoints']
	path_folder = settings['Path Folder']

def create_path_file(file_name):
	name_labels = file_name.split('-')
	name_labels = [*name_labels[0:-1], *name_labels[-1].split(' ')]
	name_labels = [*name_labels[0:-1], *name_labels[-1].split('.')]

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

not_format_files_num = 0
not_format_files_list = []

for filename in os.listdir(path_folder):
	current_file_path = path_folder+'/'+filename
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
		print(f'  -{file}')
print('Done!')