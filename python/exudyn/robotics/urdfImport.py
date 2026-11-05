#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library for robotics
#
# Details:  The import of robots from URDF files, through the Robotics Toolbox for Python (RTB) of P. Corke and
#           J. Haviland: the internal models of the RTB, a URDF file, and the data of a robot for Robot and its graphics;
#           MakeCorkeRobot, an RTB robot of the DH parameters of a robot dictionary of exudyn.robotics.models
#
# Authors:  Johannes Gerstmayr
# Date:     2023-04-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from exudyn.misc.docmeta import docmeta
import numpy as np
import exudyn
import exudyn.graphics as graphics
from exudyn.rigidBodyUtilities import HT2translation, HT2rotationMatrix
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform (#2962)

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'GetRoboticsToolboxInternalModel', 'LoadURDFrobot', 'GetURDFrobotData', 'MakeCorkeRobot',
    ]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def GetRoboticsToolboxInternalModel(modelName='', ignoreURDFerrors=True):
    """Interface to roboticstoolbox (RTB) for loading internal robot models. Function retrieves internal model available from roboticstoolbox.models.URDF, usually stored in
    site-packages/rtbdata/xacro/. See the github project of roboticstoolbox-python of P. Corke and J. Haviland for more details.
    The model name is the short name used internally in the RTB. For available names, see the list roboticstoolbox.models.URDF.__all__ !

    Args:
        modelName: string for model, such as UR5, Puma560, Panda or LBR
        ignoreURDFerrors: if set True, urdf errors are ignored and only the model is loaded

    Returns:
        returns dictionary with 'robot' (RTB Robot class), 'urdf' which is the RTB representation of the URDF file for loading mesh files

    Note:
        requires installation (pip install) of roboticstoolbox-python; in our tests we had problems with installers on newer Python and therefore tested with Python 3.9! Note that some models doe not include mass and inertia and therefore will not run as dynamic models!
    """
    from pathlib import Path
    try:
        import roboticstoolbox as rtb
        from roboticstoolbox.tools.data import rtb_path_to_datafile
        from roboticstoolbox.tools import xacro
    except ImportError:
        raise ImportError('GetRoboticsToolboxInternalModel: import of roboticstoolbox failed. You have to do "pip install roboticstoolbox-python" in order to use this function!')
    
    try:
        robotFunc = getattr(rtb.models.URDF, modelName)
        robot = robotFunc() #this reads out the main robot structure
    except Exception:
        raise ValueError('GetRoboticsToolboxInternalModel: could not retrieve robot model "'+modelName+'"')
        
    urdf = None
    urdfBasePath = None
    urdfFilePath = None

    try:
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
        #now try to find the path in the robotFunc (this is a dirty hack, 
        #  but RTB does not deliver the urdf file, nor the paths!!!)    
        import inspect
        import re
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
        
        if str(urdfFilePath).endswith('.xacro'):
            urdfString = xacro.main(Path(urdfBasePath)/urdfFilePath, None)
        else:
            urdfString = open(urdfFilePath).read()
        
        urdf = rtb.tools.urdf.urdf.URDF.loadstr(urdfString,  
                                                Path(urdfBasePath) / urdfFilePath,
                                                urdfBasePath)
    except Exception: #except Exception as e:
        if int(ignoreURDFerrors) <= 1:  #if somebody gets annoyed of warning!
            exudyn.Print('WARNING: GetRoboticsToolboxInternalModel could not retreive urdf file!')
        if not ignoreURDFerrors:
            raise

    return {'robot':robot, 'urdf':urdf, 
            'urdfBasePath':str(urdfBasePath), 
            'urdfFilePath':str(urdfFilePath)}

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def LoadURDFrobot(urdfFilePath, urdfBasePath,
                  gripperLinks = None, manufacturer=''):
    """Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland. Use this function for loading urdf/xacro files.

    Args:
        urdfFilePath: string relative to urdfBasePath, representing xacro or urdf file
        urdfBasePath: string representing the base path of the urdf or xacro directory for the respective robot; the urdfBasePath is used to load further files which are given internally in urdf files, such as collision or mesh files
        gripperLinks: list of link numbers representing gripper (as used internally in RTB Robot class)
        manufacturer: not used

    Returns:
        returns dictionary with 'robot' (RTB Robot class), 'urdf' which is the RTB representation of the URDF file for loading mesh files

    Note:
        requires installation (pip install) of roboticstoolbox-python; in our tests we had problems with installers on newer Python and therefore tested with Python 3.9! Note that some models doe not include mass and inertia and therefore will not run as dynamic models!
    """
    from pathlib import Path
    try:
        import roboticstoolbox as rtb
    except ImportError:
        raise ImportError('LoadURDFrobot: import of roboticstoolbox failed. You have to do "pip install roboticstoolbox-python" in order to use this function!')
    
    links, name, urdfString, urdfFilePath_URDF_read = rtb.robot.Robot.URDF_read(urdfFilePath, urdfBasePath)
    #create robot:
    robot = rtb.robot.Robot(
        links,
        name=name.upper(),
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

def GetURDFrobotData(robot, 
                     urdf=None, 
                     linkColorList = None,
                     staticJointValues=None,
                     returnStaticGraphicsList = False,
                     exportMesh = False,
                     verbose=1):
    """Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland and Pymeshlab to import robot model and visualization into a struture readable by Exudyn. NOTE that this function is to be seen as a starting point for import, while some models have to be imported differently, in particular for joints that are not revolute or prismatic (in this case, copy function into local file and modify)!

    Args:
        robot: a RTB Robot (class) model, as returned e.g. by LoadURDFrobot
        urdf: a RTB URDF (class) representing the URDF data, as returned e.g. by LoadURDFrobot
        linkColorList: if not None, this can contain a list of RGBA color lists for each link to prescribe colors instead of using internally stored colors or general color information (.obj files); set linkColorList=[graphics.color.red]*8 to set 8 link colors red; links are counted as in the urdf file and may be different from the number of joints
        staticJointValues: if not None, has to be a list of joint angles (or displacements) for computing the static graphics list
        returnStaticGraphicsList: return a list of GraphicsData which can be put into a ground to check visualization for zero joints, using: mbs.CreateGround(graphicsDataList=staticGraphicsList)
        exportMesh: if True, the returned dict also contains a meshSetList which refers to the MeshSet in pymeshlab, which can be used for debugging purposes
        verbose: 0 .. no output printed (only exceptions), 1 .. warnings, 2 .. further information

    Returns:
        returns dictionary with items linkList, graphicsBaseList, graphicsToolList

    Note:
        requires installation (pip install) of roboticstoolbox-python and pymeshlab; if pymeshlab is not installed, a warning is raised and graphics is ignored
    """
    hasPymeshlab = False
    try:
        import pymeshlab #pip install pymeshlab
        hasPymeshlab = True
    except ImportError:
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
                ms.load_new_mesh(scene.filename)
                mesh = ms.current_mesh()
                meshSetList.append(ms) #for debugging
                
                
                if cMath.norm(mesh.transform_matrix()-np.eye(4)) != 0:
                    if verbose > 1: exudyn.Print('mesh',i,'has additional transformation')
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
    
                if k is None:
                    if verbose > 0: exudyn.Print('WARNING: inconsistent urdf and robot model detected!')
                else:
                    if len(urdf.links[k].visuals) != nScenes:
                        if verbose > 0: exudyn.Print('WARNING: inconsistent urdf and robot model: number of visual objects is different!')
                        continue
                    
                    visualHT = urdf.links[k].visuals[j].origin
    
                    if link.isjoint:
                        linkDict['graphicsDataList'] += [graphics.Move(gData, HT2translation(visualHT), HT2rotationMatrix(visualHT))]
    
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


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@docmeta(author='Martin Sereinig')
def MakeCorkeRobot(robotDic):
    """makeCorkeRobot, creates robot using the peter corke toolbox using standard (stdDH) or modified (modKKDH) Denavid Hartenberg parameters

    Args:
        robotDic: robot dictionary by exudyn robotic models, its 'dhMode' stdDH for standard DH parameters, modKKDH for modified ones

    Returns:
        serial robot object by corke

    Note:
        DH Parameter Information:
        stdH = [theta, d, a, alpha] with Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
        modDH = [alpha, dx, theta, rz] with
        used by Corke and Lynch: Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
        used by Khali:           Rx(alpha) * Tx(d) * Rz(theta) * Tz(r)
        Important note:  d(khali)=a(corke)  and r(khali)=d(corke)
    """
    
    try:
        import roboticstoolbox as rtb

        nLinks = len(robotDic['links'])
        dhpara = robotDic['dhMode']
        Links = []
        if dhpara == 'stdDH':
            for i in range(nLinks):
                Links += [rtb.RevoluteDH(d = robotDic['links'][i][dhpara][1], a = robotDic['links'][i][dhpara][2], alpha = robotDic['links'][i][dhpara][3])]
                
        elif dhpara == 'modKKDH':
            for i in range(nLinks):   
                Links += [rtb.RevoluteMDH(d = robotDic['links'][i][dhpara][3], a = robotDic['links'][i][dhpara][1], alpha = robotDic['links'][i][dhpara][0])]
        
        robotCorke = rtb.robot.DHRobot ( Links )
        
    except Exception as e:
        #printing and continuing left robotCorke undefined, so the next line raised
        #UnboundLocalError instead of saying what was wrong, #2489
        raise ImportError('MakeCorkeRobot: check the installation of roboticstoolbox from Peter '
                          'Corke ("pip install roboticstoolbox-python"); the original error was: '+str(e)) from e

    return robotCorke
