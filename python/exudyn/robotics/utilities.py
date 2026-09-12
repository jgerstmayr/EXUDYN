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
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.rigidBodyUtilities import HT2translation, HT2rotationMatrix

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


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Interface to roboticstoolbox (RTB) for loading internal robot models. Function retrieves internal model available from roboticstoolbox.models.URDF, usually stored in 
#  site-packages/rtbdata/xacro/. See the github project of roboticstoolbox-python of P. Corke and J. Haviland for more details.
#  The model name is the short name used internally in the RTB. For available names, see the list roboticstoolbox.models.URDF.\_\_all\_\_ !
#**input:
#  modelName: string for model, such as UR5, Puma560, Panda or LBR
#  ignoreURDFerrors: if set True, urdf errors are ignored and only the model is loaded
#**output: returns dictionary with 'robot' (RTB Robot class), 'urdf' which is the RTB representation of the URDF file for loading mesh files
#**notes: requires installation (pip install) of roboticstoolbox-python; in our tests we had problems with installers on newer Python and therefore tested with Python 3.9! Note that some models doe not include mass and inertia and therefore will not run as dynamic models!
def GetRoboticsToolboxInternalModel(modelName='', ignoreURDFerrors=True):
    from pathlib import Path
    #import spatialgeometry as sg
    try:
        import roboticstoolbox as rtb
        from roboticstoolbox.tools.data import rtb_path_to_datafile
        from roboticstoolbox.tools import xacro
    except:
        raise ImportError('GetRoboticsToolboxInternalModel: import of roboticstoolbox failed. You have to do "pip install roboticstoolbox-python" in order to use this function!')
    
    try:
        robotFunc = getattr(rtb.models.URDF, modelName)
        robot = robotFunc() #this reads out the main robot structure
    except:
        raise ValueError('GetRoboticsToolboxInternalModel: could not retrieve robot model "'+modelName+'"')
        
    urdf = None
    urdfBasePath = None
    urdfFilePath = None

    try:
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
        #now try to find the path in the robotFunc (this is a dirty hack, 
        #  but RTB does not deliver the urdf file, nor the paths!!!)    
        import inspect, re
        functionSourceCode = inspect.getsource(robotFunc)
        
        pattern = r'URDF_read\(\s*"([^"]+)"\s*\)'
        
        # Search for the pattern
        match = re.search(pattern, functionSourceCode)
        
        # Extract the result if found
        if match:
            urdfFilePath = match.group(1)
        else:
            raise ValueError("GetRoboticsToolboxInternalModel: URDF path not found. Try to load urdf/xacro files manually")
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
        
        urdfBasePath = rtb_path_to_datafile("xacro")
        #urdfBasePath = 'Anaconda/envs/myenv/Lib/site-packages/rtbdata/xacro'
        
        if str(urdfFilePath).endswith('.xacro'):
            # xacro_tld = None
            urdfString = xacro.main(Path(urdfBasePath)/urdfFilePath, None)
        else:
            urdfString = open(urdfFilePath).read()
        
        urdf = rtb.tools.urdf.urdf.URDF.loadstr(urdfString,  
                                                Path(urdfBasePath) / urdfFilePath,
                                                urdfBasePath)
    except: #except Exception as e:
        if int(ignoreURDFerrors) <= 1:  #if somebody gets annoyed of warning!
            exudyn.Print('WARNING: GetRoboticsToolboxInternalModel could not retreive urdf file!')
        if not ignoreURDFerrors:
            raise

    return {'robot':robot, 'urdf':urdf, 
            'urdfBasePath':str(urdfBasePath), 
            'urdfFilePath':str(urdfFilePath)}

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland. Use this function for loading urdf/xacro files.
#**input:
#  urdfFilePath: string relative to urdfBasePath, representing xacro or urdf file
#  urdfBasePath: string representing the base path of the urdf or xacro directory for the respective robot; the urdfBasePath is used to load further files which are given internally in urdf files, such as collision or mesh files 
#  gripperLinks: list of link numbers representing gripper (as used internally in RTB Robot class)
#**output: returns dictionary with 'robot' (RTB Robot class), 'urdf' which is the RTB representation of the URDF file for loading mesh files
#**notes: requires installation (pip install) of roboticstoolbox-python; in our tests we had problems with installers on newer Python and therefore tested with Python 3.9! Note that some models doe not include mass and inertia and therefore will not run as dynamic models!
def LoadURDFrobot(urdfFilePath, urdfBasePath,
                  gripperLinks = None, manufacturer=''):
    from pathlib import Path
    try:
        import roboticstoolbox as rtb
    except:
        raise ImportError('LoadURDFrobot: import of roboticstoolbox failed. You have to do "pip install roboticstoolbox-python" in order to use this function!')
    
    # urdfFilePath = 'ur_description/urdf/ur5_joint_limited_robot.urdf.xacro'
    # urdfBasePath = '[installation-specific]/Anaconda/[depends-on-envs]/site-packages/rtbdata/xacro'
    
    links, name, urdfString, urdfFilePath_URDF_read = rtb.robot.Robot.URDF_read(urdfFilePath, urdfBasePath)
    #create robot:
    robot = rtb.robot.Robot(
        links,
        name=name.upper(),
        #manufacturer='', #not needed!
        gripper_links=gripperLinks, #links[7], #adjusted per robot
        urdf_string=urdfString,
        urdf_filepath=urdfFilePath_URDF_read,
    )
    #also load urdf file, needed for visualization
    urdf = rtb.tools.urdf.urdf.URDF.loadstr(urdfString,
                                            Path(urdfBasePath) / urdfFilePath, 
                                            urdfBasePath)

    return {'robot':robot, 'urdf':urdf, 
            'urdfBasePath':str(urdfBasePath), 
            'urdfFilePath':str(urdfFilePath)}


#%%+++++++++++++++++++++++++++++++

#**function: Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland and Pymeshlab to import robot model and visualization into a struture readable by Exudyn. NOTE that this function is to be seen as a starting point for import, while some models have to be imported differently, in particular for joints that are not revolute or prismatic (in this case, copy function into local file and modify)! 
#**input:
#  robot: a RTB Robot (class) model, as returned e.g. by LoadURDFrobot
#  urdf: a RTB URDF (class) representing the URDF data, as returned e.g. by LoadURDFrobot
#  linkColorList: if not None, this can contain a list of RGBA color lists for each link to prescribe colors instead of using internally stored colors or general color information (.obj files); set linkColorList=[graphics.color.red]*8 to set 8 link colors red; links are counted as in the urdf file and may be different from the number of joints
#  staticJointValues: if not None, has to be a list of joint angles (or displacements) for computing the static graphics list
#  returnStaticGraphicsList: return a list of GraphicsData which can be put into a ground to check visualization for zero joints, using: mbs.CreateGround(graphicsDataList=staticGraphicsList)
#  exportMesh: if True, the returned dict also contains a meshSetList which refers to the MeshSet in pymeshlab, which can be used for debugging purposes
#  verbose: 0 .. no output printed (only exceptions), 1 .. warnings, 2 .. further information
#**output: returns dictionary with items linkList, graphicsBaseList, graphicsToolList
#**notes: requires installation (pip install) of roboticstoolbox-python and pymeshlab; if pymeshlab is not installed, a warning is raised and graphics is ignored
def GetURDFrobotData(robot, 
                     urdf=None, 
                     linkColorList = None,
                     staticJointValues=None,
                     returnStaticGraphicsList = False,
                     exportMesh = False,
                     verbose=1):
    hasPymeshlab = False
    try:
        import pymeshlab #pip install pymeshlab
        hasPymeshlab = True
    except:
        if verbose > 0: 
            exudyn.Print('WARNING: GetURDFrobotData: import of pymeshlab failed. You have to do "pip install pymeshlab" in order to enable visualization!')
    
    linkName2Index = {}
    index2linkName = [None]*len(robot.links)
    for i, link in enumerate(robot.links):
        linkName2Index[link.name] = i
        index2linkName[i] = link.name
    
    numberOfJoints = robot.n
    if verbose > 1: 
        exudyn.Print('load robot "'+robot.name+'": #joints=',numberOfJoints,', #links=',len(robot.links),
              ', #vlinks=',len(urdf.links))
    
    graphicsBaseList = []
    graphicsToolList = []
    linkList = []
    staticGraphicsList = []
    
    meshSetList = []
    
    rtbAxis2exuJointType = {'Rx':'Rx','Ry':'Ry','Rz':'Rz',
                            'tx':'Px','ty':'Py','tz':'Pz',}
    
    if staticJointValues is None:
        staticJointValues = np.zeros(numberOfJoints)
    elif len(staticJointValues) != numberOfJoints:
        raise ValueError('GetURDFrobotData: staticJointValues are inconsistent: robot has '+str(numberOfJoints)+', but len(staticJointValues)='+str(len(staticJointValues)) )

    robotFkine = robot.fkine_all(staticJointValues)
    
    staticHTlinks = []
    for HT in robotFkine: #convert into numpy arrays
        staticHTlinks.append(np.array(HT))
    
    jointFound = False #first links without joints added to base graphics
    for i, link in enumerate(robot.links):
        offsetLink = 1 #+1 as 0 is the base link; 
        
        #offsetLink: no offset for UR5 ?WHY?
        if any(substring in robot.name for substring in ['UR3','UR5','UR10']):
            offsetLink = 0
            
        staticHT = staticHTlinks[i+offsetLink]
        
        jointIndex = link.jindex

        linkDict = {} #filled later
        if link.isjoint:
            if jointIndex is None: raise ValueError('inconsistent joint index / isjoint in robot structure')
            jointFound = True

            parentNumber = -1 if link.parent is None else link.parent.number
            parentName = 'None' if link.parent is None else link.parent.name
            #required fields:
            linkDict['name'] = link.name
            linkDict['jointNumber'] = jointIndex
            linkDict['parentNumber'] = parentNumber
            linkDict['parentName'] = parentName
            linkDict['preHT'] = np.array(link.Ts) #A(0)
            linkDict['inertiaCOM'] = np.array(link.I)
            linkDict['com'] = np.array(link.r)
            linkDict['mass'] = link.m
            linkDict['jointType'] = rtbAxis2exuJointType[link.v.axis] #link.v.axis #Rx,Ry,Rz,tx,ty,tz
            #optional fields:
            linkDict['motorViscousFriction'] = link.B if hasattr(link, "B") else 0
            linkDict['motorCoulombFriction'] = np.array(link.Tc) if hasattr(link, "Tc") else np.zeros(2)
            linkDict['motorGearRatio'] = link.G if hasattr(link, "G") else 0
            linkDict['motorInertia'] = link.Jm if hasattr(link, "Jm") else 0
            linkDict['jointLimits'] = list(link.qlim) if (hasattr(link, "qlim") and link.qlim is not None) else np.zeros(2)

            linkDict['staticHT'] = staticHT
            linkDict['graphicsDataList'] = []
            
            if verbose > 1: exudyn.Print('define link',i,', joint',jointIndex, ', type=', linkDict['jointType'],#link.v.axis, 
                  #'Ts=\n',np.array(link.Ts).round(3),
                  #'staticHT=\n',staticHT.round(3)
                  )
            
            linkList += [linkDict]
            if link.isflip and verbose > 0:
                exudyn.Print('WARNING: joint',i,'is flipped, but this is not implemented!')
            
        sceneGroup = link.geometry
        nScenes = len(sceneGroup)
        for j, scene in enumerate(sceneGroup):
            if hasPymeshlab and scene.stype == 'mesh' and scene.filename != '':
                ms = pymeshlab.MeshSet()
                #ms.load_new_mesh(fileDir+'ur5_description/visual/base.dae')
                ms.load_new_mesh(scene.filename)
                mesh = ms.current_mesh()
                meshSetList.append(ms) #for debugging
                
                
                if np.linalg.norm(mesh.transform_matrix()-np.eye(4)) != 0:
                    if verbose > 1: exudyn.Print('mesh',i,'has additional transformation')
                # if mesh.has_face_color(): #only in case of .obj files! use: convert3d.org/dae-to-obj
                #     if verbose > 0: exudyn.Print('link',i,'scene',j,'has colored faces')
                color = graphics.colorList[i%16]
                if linkColorList is not None:
                    color = linkColorList[i]
                elif hasattr(scene, 'color'):
                    color = scene.color
                
                gData = graphics.FromPointsAndTrigs(points = mesh.vertex_matrix() * scene.scale,
                                                    triangles = mesh.face_matrix(),
                                                    normals = mesh.vertex_normal_matrix(),
                                                    color = color,
                                                    )
                # normals = mesh.vertex_normal_matrix()
                # gData['normals'] = np.array(normals).flatten()

                #if mesh has face colors (.obj files), read them and store in gData:
                if mesh.has_face_color() and (linkColorList is None):
                    faceColors = mesh.face_color_matrix()
                    triangles = mesh.face_matrix()
                    #convert face colors to vertex colors: (only available for .obj files)
                    vertexColors = np.zeros((mesh.vertex_matrix().shape[0],4))
                    for it, trig in enumerate(triangles):
                        color = faceColors[it, :]
                        for vertex in trig:
                            vertexColors[vertex,:] = color
                    gData['colors'] = np.array(vertexColors).flatten()

                #simple search to check if indices match (use names):
                k = None
                for index, visLink in enumerate(urdf.links):
                    if visLink.name == link.name:
                        k = index
                        #exudyn.Print('visuals for link',i,'(visual',k,')=',link.name)
    
                if k is None:
                    if verbose > 0: exudyn.Print('WARNING: inconsistent urdf and robot model detected!')
                else:
                    if len(urdf.links[k].visuals) != nScenes:
                        if verbose > 0: exudyn.Print('WARNING: inconsistent urdf and robot model: number of visual objects is different!')
                        continue
                    
                    visualHT = urdf.links[k].visuals[j].origin
    
                    if link.isjoint:
                        linkDict['graphicsDataList'] += [graphics.Move(gData, HT2translation(visualHT), HT2rotationMatrix(visualHT))]
                        # lg = linkDict['graphicsDataList'][-1]
                        # lg['normals'] = np.array(lg['normals'])
                        # lg['points'] = np.array(lg['points'])
                        # lg['triangles'] = np.array(lg['triangles'])
                        # lg['colors'] = np.array(lg['colors'])
    
                    totalHT = staticHT@visualHT
    
                    if jointIndex is None and not jointFound:
                        #add to base
                        graphicsBaseList += [graphics.Move(gData, HT2translation(totalHT), HT2rotationMatrix(totalHT))]
                    elif jointIndex is None and jointFound:
                        #here we probably need to store some additional transformations, starting with last link
                        graphicsToolList += [graphics.Move(gData, HT2translation(visualHT), HT2rotationMatrix(visualHT))]
                        #check robot.ee_links for tool links?
                    
                    if returnStaticGraphicsList :
                        staticGraphicsList += [graphics.Move(gData, HT2translation(totalHT), HT2rotationMatrix(totalHT) )]
                
    robotData = {'linkList': linkList,
            'numberOfJoints': numberOfJoints,
            'graphicsBaseList': graphicsBaseList,
            'graphicsToolList': graphicsToolList,
            'staticJointValues': staticJointValues,
            'staticGraphicsList': staticGraphicsList}
    if exportMesh: robotData['meshSetList'] = meshSetList

    return robotData


