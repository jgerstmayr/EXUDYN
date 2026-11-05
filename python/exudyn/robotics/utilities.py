#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  The utilities contains general helper functions for the robotics module: the lidar sensors
#
# Authors:  Johannes Gerstmayr
# Date:     2023-04-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
from exudyn.cMath import sin, cos
import exudyn
from exudyn.misc.mainSystemExtensions import CreateDistanceSensor
from exudyn.misc.deprecation import Deprecated #the deprecations of the library (#2807)
import exudyn.robotics.urdfImport as urdfImport

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'AddLidar', 'GetRoboticsToolboxInternalModel', 'LoadURDFrobot', 'GetURDFrobotData',
    ]

def AddLidar(mbs, generalContactIndex,
            positionOrMarker, minDistance=0,
            maxDistance=1e7, cylinderRadius=0, lineLength=1,
            numberOfSensors=100, angleStart=0, angleEnd=2*np.pi,
            inclination=0, rotation=np.eye(3),
            selectedTypeIndex=exudyn.ContactTypeIndex.IndexEndOfEnumList,
            storeInternal = False, fileName = '', measureVelocity = False,
            addGraphicsObject=True, drawDisplaced=True, color=[1.0, 0.0, 0.0, 1.0]):
    """Function to add many distance sensors to represent Lidar; sensors can be either placed on absolute position or attached to rigid body marker

    Args:
        generalContactIndex: the number of the GeneralContact object in mbs; the index of the GeneralContact object which has been added with last AddGeneralContact(...) command is generalContactIndex=mbs.NumberOfGeneralContacts()-1
        positionOrMarker: either a 3D position as list or np.array, or a MarkerIndex with according rigid body marker
        minDistance: the minimum distance which is accepted; smaller distance will be ignored
        maxDistance: the maximum distance which is accepted; items being at maxDistance or futher are ignored; if no items are found, the function returns maxDistance
        cylinderRadius: in case of spheres (selectedTypeIndex=ContactTypeIndex.IndexSpheresMarkerBased), a cylinder can be used which measures the shortest distance at a certain radius (geometrically interpreted as cylinder)
        lineLength: length of line to be drawn; note that this length is drawn from obstacle towards sensor if drawDisplaced=True, but the length is always constant
        numberOfSensors: number of sensors arranged between angleStart and angleEnd; higher numbers give finer resolution (but requires more CPU time); must be larger than 1
        angleStart: starting rangle of angles to be used (in radiant); angle of lidar beam is relative to X-axis, using positive rotation sense about Z-axis
        angleEnd: end of range for angle to be used (in radiant); angle of lidar beam is relative to X-axis, using positive rotation sense about Z-axis
        inclination: angle of inclination (radiant), positive values showing upwards (Z-direction) if rotation is the identity matrix
        rotation: a 3x3 rotation matrix (numpy); the sensor is placed in the X-Y plane of the marker where it is added to; however, you can use this rotation matrix to change the orientation
        selectedTypeIndex: either this type has default value, meaning that all items in GeneralContact are measured, or there is a specific type index, which is the only type that is considered during measurement
        storeInternal: like with any SensorUserFunction, setting to True stores sensor data internally
        fileName: if defined, recorded data of SensorUserFunction is written to specified file
        measureVelocity: if True, the sensor measures additionally the velocity (component 0=distance, component 1=velocity); velocity is the velocity in direction 'dirSensor' and does not account for changes in geometry, thus it may be different from the time derivative of the distance!
        addGraphicsObject: if True, the distance sensor is also visualized graphically in a simplified manner with a red line having the length of dirSensor; NOTE that updates are ONLY performed during computation, not in visualization; for this reason, solution.sensors.writePeriod should be accordingly small
        drawDisplaced: if True, the red line is drawn backwards such that it moves along the measured surface; if False, the beam is fixed to marker or position
        color: optional color for 'laser beam' to be drawn

    Returns:
        creates sensor and returns list of sensor numbers for all laser sensors

    Note:
        use generalContactIndex = CreateDistanceSensorGeometry(...) before to create GeneralContact module containing geometry
    """

    if numberOfSensors <= 1: raise ValueError('AddLidar: numberOfSensors must be > 1')
    a=inclination
    L=lineLength
    phiRange=angleEnd-angleStart
    sensorList = []
    for i in range(numberOfSensors):
        phi = i/(numberOfSensors-1)*phiRange+angleStart
        dirSensor = rotation @ np.array([L*cos(phi)*cos(a), L*sin(phi)*cos(a),L*sin(a)])

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


#the URDF import is in exudyn.robotics.urdfImport (#3256)
@Deprecated('1.13.0', 2029, use='robotics.urdfImport.GetRoboticsToolboxInternalModel')
def GetRoboticsToolboxInternalModel(*args, **kwargs):
    """DEPRECATED: moved to exudyn.robotics.urdfImport"""
    return urdfImport.GetRoboticsToolboxInternalModel(*args, **kwargs)

@Deprecated('1.13.0', 2029, use='robotics.urdfImport.LoadURDFrobot')
def LoadURDFrobot(*args, **kwargs):
    """DEPRECATED: moved to exudyn.robotics.urdfImport"""
    return urdfImport.LoadURDFrobot(*args, **kwargs)

@Deprecated('1.13.0', 2029, use='robotics.urdfImport.GetURDFrobotData')
def GetURDFrobotData(*args, **kwargs):
    """DEPRECATED: moved to exudyn.robotics.urdfImport"""
    return urdfImport.GetURDFrobotData(*args, **kwargs)
