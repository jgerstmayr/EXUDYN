#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  The utilities contains general helper functions for the robotics module
#
# Authors:  Johannes Gerstmayr
# Date:     2023-04-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
from math import sin, cos
import exudyn
from exudyn.utilities import CreateDistanceSensor

#**function: Function to add many distance sensors to represent Lidar; sensors can be either placed on absolute position or attached to rigid body marker
#**input:
#  generalContactIndex: the number of the GeneralContact object in mbs; the index of the GeneralContact object which has been added with last AddGeneralContact(...) command is generalContactIndex=mbs.NumberOfGeneralContacts()-1
#  positionOrMarker: either a 3D position as list or np.array, or a MarkerIndex with according rigid body marker
#  minDistance: the minimum distance which is accepted; smaller distance will be ignored
#  maxDistance: the maximum distance which is accepted; items being at maxDistance or futher are ignored; if no items are found, the function returns maxDistance
#  cylinderRadius: in case of spheres (selectedTypeIndex=ContactTypeIndex.IndexSpheresMarkerBased), a cylinder can be used which measures the shortest distance at a certain radius (geometrically interpreted as cylinder)
#  lineLength: length of line to be drawn; note that this length is drawn from obstacle towards sensor if drawDisplaced=True, but the length is always constant
#  numberOfSensors: number of sensors arranged between angleStart and angleEnd; higher numbers give finer resolution (but requires more CPU time); must be larger than 1
#  angleStart: starting rangle of angles to be used (in radiant); angle of lidar beam is relative to X-axis, using positive rotation sense about Z-axis
#  angleEnd: end of range for angle to be used (in radiant); angle of lidar beam is relative to X-axis, using positive rotation sense about Z-axis
#  inclination: angle of inclination (radiant), positive values showing upwards (Z-direction) if rotation is the identity matrix
#  rotation: a 3x3 rotation matrix (numpy); the sensor is placed in the X-Y plane of the marker where it is added to; however, you can use this rotation matrix to change the orientation
#  selectedTypeIndex: either this type has default value, meaning that all items in GeneralContact are measured, or there is a specific type index, which is the only type that is considered during measurement
#  storeInternal: like with any SensorUserFunction, setting to True stores sensor data internally
#  fileName: if defined, recorded data of SensorUserFunction is written to specified file
#  measureVelocity: if True, the sensor measures additionally the velocity (component 0=distance, component 1=velocity); velocity is the velocity in direction 'dirSensor' and does not account for changes in geometry, thus it may be different from the time derivative of the distance!
#  addGraphicsObject: if True, the distance sensor is also visualized graphically in a simplified manner with a red line having the length of dirSensor; NOTE that updates are ONLY performed during computation, not in visualization; for this reason, solutionSettings.sensorsWritePeriod should be accordingly small
#  drawDisplaced: if True, the red line is drawn backwards such that it moves along the measured surface; if False, the beam is fixed to marker or position
#  color: optional color for 'laser beam' to be drawn
#**output: creates sensor and returns list of sensor numbers for all laser sensors
#**notes: use generalContactIndex = CreateDistanceSensorGeometry(...) before to create GeneralContact module containing geometry
def AddLidar(mbs, generalContactIndex,
            positionOrMarker, minDistance=0,
            maxDistance=1e7, cylinderRadius=0, lineLength=1,
            numberOfSensors=100, angleStart=0, angleEnd=2*np.pi,
            inclination=0, rotation=np.eye(3),
            selectedTypeIndex=exudyn.ContactTypeIndex.IndexEndOfEnumList,
            storeInternal = False, fileName = '', measureVelocity = False,
            addGraphicsObject=True, drawDisplaced=True, color=[1.0, 0.0, 0.0, 1.0]):

    if numberOfSensors <= 1: raise ValueError('AddLidar: numberOfSensors must be > 1')
    a=inclination
    L=lineLength
    phiRange=angleEnd-angleStart
    sensorList = []
    for i in range(numberOfSensors):
        phi = i/(numberOfSensors-1)*phiRange+angleStart
        dirSensor = rotation @ np.array([L*cos(phi)*cos(a), L*sin(phi)*cos(a),L*sin(a)])
        #print(dirSensor, positionOrMarker)
        sensorList += [CreateDistanceSensor(mbs, generalContactIndex, 
                                            positionOrMarker=positionOrMarker, 
                                            dirSensor=dirSensor,
                                            minDistance=minDistance, maxDistance=maxDistance, 
                                            cylinderRadius=cylinderRadius, 
                                            selectedTypeIndex=selectedTypeIndex,
                                            storeInternal=storeInternal, 
                                            fileName=fileName, 
                                            measureVelocity=measureVelocity, 
                                            addGraphicsObject=addGraphicsObject,
                                            drawDisplaced=drawDisplaced,
                                            color=color)]

    return sensorList

