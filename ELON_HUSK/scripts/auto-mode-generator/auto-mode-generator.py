#!/usr/bin/python

import sys
import os

MODE_DIR = '../../src/main/java/frc/robot/autonomous/'
PATH_TEMPLATE = 'path.txt'
COORD_TEMPLATE = 'coord.txt'
SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))
os.chdir(SCRIPT_PATH)

className = ''
modeType = ''

if len(sys.argv) == 3:
	className = sys.argv[1].strip()
	modeType = sys.argv[2].lower().strip()
elif len(sys.argv) == 2:
	# Check if user may have only provided mode and no class name
	arg1 = sys.argv[1].lower().strip()
	if arg1 == 'path' or arg1 == 'coord':
		modeType = arg1
	else:
		className = arg1

# Input verification
while len(className) < 1:
	className = input('Enter a name for the new mode: ').strip()
while modeType != 'path' and modeType != 'coord':
	modeType = input('Choose which type of mode you want to generate (path|coord): ').lower().strip()

modePath = os.path.join(MODE_DIR, className + '.java')
if modeType == 'path':
	print('Generating path mode ' + className + '...')
	with open(PATH_TEMPLATE, 'r') as templateFile:
		templateData = templateFile.read()
elif modeType == 'coord':
	print('Generating coordinate mode ' + className + '...')
	with open(COORD_TEMPLATE, 'r') as templateFile:
		templateData = templateFile.read()

templateData = templateData.replace('#CLASSNAME', className)

with open(modePath, 'w') as modeFile:
	modeFile.write(templateData)

print('File generated as ' + modePath)