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
import os
import numpy as np
import exudyn
import exudyn.graphics as graphics
from exudyn.rigidBodyUtilities import HT2translation, HT2rotationMatrix
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform (#2962)

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'GetRoboticsToolboxInternalModel', 'LoadURDFrobot', 'GetURDFrobotData', 'MakeCorkeRobot', 'LoadURDFgraphics',
    'AddURDFgraphics',
    ]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def GetRoboticsToolboxInternalModel(modelName='', ignoreURDFerrors=True):
    """Interface to roboticstoolbox (RTB) for loading internal robot models: the model of roboticstoolbox.models.URDF of
    this name, read from its URDF or xacro file. See the github project of roboticstoolbox-python of P. Corke and J.
    Haviland for more details. The model name is the short name used internally in the RTB. For available names, see the
    list roboticstoolbox.models.URDF.__all__ !

    Args:
        modelName: string for model, such as UR5, Puma560, Panda or LBR
        ignoreURDFerrors: not used; kept for the scripts that pass it

    Returns:
        returns dictionary with 'robot' (RTB Robot class), 'urdf' (None; the visuals are in the links of the robot),
        'urdfBasePath' (the xacro directory of the rtb-data package) and 'urdfFilePath' (the file that was read)

    Note:
        requires the Robotics Toolbox for Python 1.5 (pip install roboticstoolbox-python); its models Panda, UR3, UR5,
        UR10, KinovaGen3, Jaco, Fetch and PR2 are read through the package robot_descriptions, which downloads them on
        first use; Puma560, LBR, AL5D and the Interbotix robots (px150, vx300, wx250, ...) are part of the RTB's data.
        Some models do not include mass and inertia and therefore will not run as dynamic models (#3280)
    """
    try:
        import roboticstoolbox as rtb
        import rtbdata
    except ImportError:
        raise ImportError('GetRoboticsToolboxInternalModel: import of roboticstoolbox failed. You have to do "pip install roboticstoolbox-python" in order to use this function!')

    if modelName not in rtb.models.URDF.__all__:
        raise ValueError('GetRoboticsToolboxInternalModel: unknown robot model "'+modelName+'"; available are '+str(rtb.models.URDF.__all__))
    robot = getattr(rtb.models.URDF, modelName)() #reads the URDF or xacro file
    from pathlib import Path
    return {'robot':robot, 'urdf':None,
            'urdfBasePath':str(Path(rtbdata.__file__).parent / 'xacro'),
            'urdfFilePath':str(robot.urdf_filepath)}

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def LoadURDFrobot(urdfFilePath, urdfBasePath,
                  gripperLinks = None, manufacturer=''):
    """Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland. Use this function for loading urdf/xacro files.

    Args:
        urdfFilePath: string relative to urdfBasePath, representing xacro or urdf file
        urdfBasePath: string representing the base path of the urdf or xacro directory for the respective robot, the
            directory of its ROS package; meshes given as package://<name>/... are found there and in the directories
            next to it
        gripperLinks: list of link numbers representing gripper (as used internally in RTB Robot class)
        manufacturer: the manufacturer, stored in the RTB robot

    Returns:
        returns dictionary with 'robot' (RTB Robot class), 'urdf' (None; the visuals are in the links of the robot),
        'urdfBasePath' and 'urdfFilePath'

    Note:
        requires the Robotics Toolbox for Python 1.5 (pip install roboticstoolbox-python), which reads the file by
        xacrodoc (#3280). Note that some models do not include mass and inertia and therefore will not run as dynamic
        models!
    """
    try:
        import roboticstoolbox as rtb
        from roboticstoolbox.models.URDF.URDFRobot import URDF_file
        from xacrodoc import packages
    except ImportError:
        raise ImportError('LoadURDFrobot: import of roboticstoolbox failed. You have to do "pip install roboticstoolbox-python" in order to use this function!')

    #the package of the robot and the packages next to it, for package:// and $(find ...)
    basePath = os.path.abspath(urdfBasePath)
    parent = os.path.dirname(basePath)
    packageMap = {os.path.basename(basePath): basePath}
    for directory in [parent, basePath]: #also package://meshes/... as some URDF files use
        for d in os.listdir(directory):
            if os.path.isdir(os.path.join(directory, d)) and d not in packageMap:
                packageMap[d] = os.path.join(directory, d)
    packages.update_package_cache(packageMap)

    links, name, filePath = URDF_file(os.path.join(basePath, urdfFilePath))
    if gripperLinks is not None:
        gripperLinks = [links[i] if isinstance(i, int) else i for i in gripperLinks]
    robot = rtb.Robot(links, name=name, manufacturer=manufacturer, gripper_links=gripperLinks)

    return {'robot':robot, 'urdf':None,
            'urdfBasePath':str(urdfBasePath),
            'urdfFilePath':str(urdfFilePath)}


#%%+++++++++++++++++++++++++++++++
def _InertiaProblems(mass, inertia):
    """the problems of the mass and the inertia tensor (at the COM) of a link, as a list of strings; empty if it is
    physical: mass > 0, the tensor symmetric and positive definite, its principal moments fulfilling the triangle
    inequality (#3282)"""
    if mass <= 0:
        return ['mass ' + str(mass)]
    inertia = np.array(inertia, dtype=float)
    if np.max(np.abs(inertia - inertia.T)) > 1e-12*max(1e-300, np.max(np.abs(inertia))):
        return ['inertia not symmetric']
    moments = np.linalg.eigvalsh(inertia)
    if moments[0] <= 0:
        return ['inertia not positive definite, principal moments ' + str(moments)]
    if moments[0] + moments[1] < moments[2]*(1 - 1e-9):
        return ['principal moments ' + str(moments) + ' violate the triangle inequality']
    return []

def _CorrectedInertia(inertia):
    """the inertia tensor with its principal moments raised as little as possible to be positive and to fulfill the
    triangle inequality, in the same principal axes (#3282)"""
    inertia = 0.5*(np.array(inertia, dtype=float) + np.array(inertia, dtype=float).T)
    (moments, axes) = np.linalg.eigh(inertia)
    moments = np.maximum(moments, 1e-6*max(1e-12, moments[2]))
    deficit = moments[2] - moments[0] - moments[1]
    if deficit > 0:
        moments[0] += 0.5*deficit
        moments[1] += 0.5*deficit
    return axes @ np.diag(moments) @ axes.T

def _Steiner(mass, d):
    """the inertia of a point mass at the distance vector d"""
    return mass*(np.dot(d, d)*np.eye(3) - np.outer(d, d))

def GetURDFrobotData(robot, 
                     urdf=None, 
                     linkColorList = None,
                     staticJointValues=None,
                     returnStaticGraphicsList = False,
                     exportMesh = False,
                     verbose=1,
                     meshDensity = 0.,
                     correctInertia = False):
    """Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland and Pymeshlab to import robot model and visualization into a struture readable by Exudyn. NOTE that this function is to be seen as a starting point for import, while some models have to be imported differently, in particular for joints that are not revolute or prismatic (in this case, copy function into local file and modify)!

    Args:
        robot: a RTB Robot (class) model, as returned e.g. by LoadURDFrobot
        urdf: not used: the visuals and their origins are those of the links of robot (#3280)
        linkColorList: if not None, this can contain a list of RGBA color lists for each link to prescribe colors instead of using internally stored colors or general color information (.obj files); set linkColorList=[graphics.color.red]*8 to set 8 link colors red; links are counted as in the urdf file and may be different from the number of joints
        staticJointValues: if not None, has to be a list of joint angles (or displacements) for computing the static graphics list
        returnStaticGraphicsList: return a list of GraphicsData which can be put into a ground to check visualization for zero joints, using: mbs.CreateGround(graphicsDataList=staticGraphicsList)
        exportMesh: if True, the returned dict also contains a meshSetList which refers to the MeshSet in pymeshlab, which can be used for debugging purposes
        verbose: 0 .. no output printed (only exceptions), 1 .. warnings, 2 .. further information
        meshDensity: if > 0, a joint link without mass gets the mass, center of mass and inertia of the solid its meshes
            enclose, with this density (kg/m^3); a mesh that does not close gives an estimate only (#3282)
        correctInertia: if True, an inertia tensor that is not positive definite or whose principal moments violate the
            triangle inequality gets its principal moments raised as little as possible, and a zero one that of the shape
            of the meshes with the mass of the link; every link reports the problems
            of its mass and inertia in 'inertiaProblems', and they are printed with verbose > 0 (#3282)

    Returns:
        returns dictionary with items linkList (the joint links, each with its preHT relative to the joint link above,
        its parentName, mass, com, inertiaCOM, jointType, jointLimits and graphicsDataList), numberOfJoints,
        graphicsBaseList (the links fixed to the base), graphicsToolList (empty: the links fixed after the last joint
        are in the graphics of that link), staticJointValues and staticGraphicsList; a link of the URDF without joint
        (a fixed joint) moves with the joint link above it, which gets its graphics and its mass (#3280); a flipped joint
        (axis -1 in the URDF) has the same joint coordinate in a link frame turned by pi, flipHT of the link (#3282)

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
        exudyn.Print('load robot "'+robot.name+'": #joints=',numberOfJoints,', #links=',len(robot.links))
    
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

    #a link without joint (a fixed joint of the URDF) moves with the nearest joint link above it, or with the base: its
    #frame relative to that one, its mass and its graphics are added there (#3280)
    def LinkTs(link): #the frame of the link relative to its parent at zero joint values; None for a link at the root
        return np.eye(4) if link.Ts is None else np.array(link.Ts)

    #a flipped joint turns its link by -q: the link of Exudyn turns by q in a frame turned by pi about an axis normal to
    #the joint axis, F; the frame of the RTB link is that frame times F, and F also transforms what is given in it
    def FlipHT(link):
        HT = np.eye(4)
        if link is not None and link.isjoint and link.isflip:
            HT[:3, :3] = np.diag({'x': [-1., 1., -1.], 'y': [-1., -1., 1.], 'z': [1., -1., -1.]}[link.v.axis[-1].lower()])
        return HT

    def JointAncestor(link): #the joint link above, and the frame relative to the frame of its Exudyn link
        HT = np.eye(4)
        while link is not None and not link.isjoint:
            HT = LinkTs(link) @ HT
            link = link.parent
        return link, FlipHT(link) @ HT

    linkDictOfName = {}
    texturesAdded = []
    for i, link in enumerate(robot.links):
        #the frame of the link at staticJointValues
        if link.parent is None:
            staticHT = np.array(robot.base) @ LinkTs(link)
        else:
            staticHT = np.array(robot.fkine(staticJointValues, end=link))

        jointIndex = link.jindex
        (ancestor, HTancestor) = (link, FlipHT(link)) if link.isjoint else JointAncestor(link)
        linkDict = None if ancestor is None else linkDictOfName.get(ancestor.name) #None: the base
        if link.isjoint:
            if jointIndex is None: raise ValueError('inconsistent joint index / isjoint in robot structure')
            (parent, HTparent) = JointAncestor(link.parent)

            linkDict = {}
            #required fields:
            linkDict['name'] = link.name
            linkDict['jointNumber'] = jointIndex
            linkDict['parentNumber'] = -1 if parent is None else parent.number
            linkDict['parentName'] = 'None' if parent is None else parent.name
            F = FlipHT(link)
            linkDict['preHT'] = HTparent @ LinkTs(link) @ F #A(0), relative to the joint link above
            linkDict['inertiaCOM'] = F[:3, :3] @ np.array(link.I) @ F[:3, :3]
            linkDict['com'] = F[:3, :3] @ np.array(link.r)
            linkDict['flipHT'] = F #the frame of the URDF link is the frame of the Exudyn link times flipHT
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

            if verbose > 1: exudyn.Print('define link',i,', joint',jointIndex, ', type=', linkDict['jointType'])

            linkList += [linkDict]
            linkDictOfName[link.name] = linkDict
        elif linkDict is not None and link.m > 0: #the mass of a fixed link, added to its joint link
            R = HTancestor[:3, :3]
            (m0, c0, I0) = (linkDict['mass'], linkDict['com'], linkDict['inertiaCOM'])
            (m1, c1, I1) = (link.m, HTancestor[:3, :3] @ np.array(link.r) + HTancestor[:3, 3], R @ np.array(link.I) @ R.T)
            m = m0 + m1
            c = (m0*c0 + m1*c1)/m
            linkDict['inertiaCOM'] = I0 + _Steiner(m0, c0 - c) + I1 + _Steiner(m1, c1 - c)
            linkDict['com'] = c
            linkDict['mass'] = m
            
        sceneGroup = link.geometry
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
                
                textureFile = ''
                if mesh.has_wedge_tex_coord() and mesh.texture_number() > 0 and linkColorList is None:
                    textureFile = os.path.join(os.path.dirname(scene.filename), list(mesh.textures().keys())[0])
                if textureFile != '' and os.path.isfile(textureFile):
                    #a textured mesh (.dae, .obj): a point per corner of each triangle, as the texture coordinates are
                    #given per corner; the first texture of the mesh, added under the name of its file (#3282)
                    faces = mesh.face_matrix()
                    gData = graphics.FromPointsAndTrigs(points = (mesh.vertex_matrix() * scene.scale)[faces].reshape(-1, 3),
                                                        triangles = np.arange(3*len(faces)).reshape(-1, 3),
                                                        normals = mesh.vertex_normal_matrix()[faces].reshape(-1, 3),
                                                        color = graphics.color.white)
                    textureName = os.path.basename(textureFile)
                    if textureName not in texturesAdded:
                        image = graphics.ImageFromFile(textureFile)
                        if image.ndim == 2:
                            image = np.stack([image]*3, axis=-1)
                        if image.shape[2] == 3:
                            image = np.concatenate([image, np.full(image.shape[:2] + (1,), 255, dtype=np.uint8)], axis=2)
                        exudyn._compiledModule._SetTexture({'name': textureName, 'image': image, 'interpretation': 'RGBA', 'flags': 0})
                        texturesAdded.append(textureName)
                    gData['texture'] = textureName
                    gData['textureCoordinates'] = np.array(mesh.wedge_tex_coord_matrix(), dtype=float).flatten()
                else:
                    gData = graphics.FromPointsAndTrigs(points = mesh.vertex_matrix() * scene.scale,
                                                        triangles = mesh.face_matrix(),
                                                        normals = mesh.vertex_normal_matrix(),
                                                        color = color,
                                                        )

                #if mesh has face colors (.obj files), read them and store in gData:
                if mesh.has_face_color() and (linkColorList is None) and 'texture' not in gData:
                    faceColors = mesh.face_color_matrix()
                    triangles = mesh.face_matrix()
                    #convert face colors to vertex colors: (only available for .obj files)
                    vertexColors = np.zeros((mesh.vertex_matrix().shape[0],4))
                    for it, trig in enumerate(triangles):
                        color = faceColors[it, :]
                        for vertex in trig:
                            vertexColors[vertex,:] = color
                    gData['colors'] = np.array(vertexColors).flatten()

                visualHT = np.array(scene.T) #the origin of the visual in the link frame
                totalHT = staticHT@visualHT
                if linkDict is not None: #in the frame of the joint link it moves with
                    localHT = HTancestor@visualHT
                    linkDict['graphicsDataList'] += [graphics.Move(gData, HT2translation(localHT), HT2rotationMatrix(localHT))]
                else: #fixed to the base
                    graphicsBaseList += [graphics.Move(gData, HT2translation(totalHT), HT2rotationMatrix(totalHT))]
                
                if returnStaticGraphicsList :
                    staticGraphicsList += [graphics.Move(gData, HT2translation(totalHT), HT2rotationMatrix(totalHT) )]

    def MeshMassProperties(linkDict, density): #[mass, COM, inertia at the COM] of the meshes of a link; mass 0 without meshes
        parts = []
        for g in linkDict['graphicsDataList']:
            if g['type'] == 'TriangleList':
                points = np.array(g['points'], dtype=float).reshape(-1, 3)
                parts.append(graphics._MassPropertiesOfTriangles(points[np.array(g['triangles'], dtype=int).reshape(-1, 3)], density))
        m = sum(part[1] for part in parts)
        if m <= 0:
            return [0., np.zeros(3), np.zeros((3, 3))]
        c = sum(part[1]*np.array(part[2]) for part in parts)/m
        return [m, c, sum(np.array(part[3]) + _Steiner(part[1], np.array(part[2]) - c) for part in parts)]

    for linkDict in linkList:
        if linkDict['mass'] <= 0 and meshDensity > 0:
            [m, c, inertia] = MeshMassProperties(linkDict, meshDensity)
            if m > 0:
                (linkDict['mass'], linkDict['com'], linkDict['inertiaCOM']) = (m, c, inertia)
        problems = _InertiaProblems(linkDict['mass'], linkDict['inertiaCOM'])
        if correctInertia and len(problems) != 0 and linkDict['mass'] > 0:
            if np.max(np.abs(linkDict['inertiaCOM'])) == 0: #no inertia at all: that of the shape of the meshes, with the mass
                [m, c, inertia] = MeshMassProperties(linkDict, 1.)
                if m > 0:
                    linkDict['inertiaCOM'] = inertia*(linkDict['mass']/m)
            linkDict['inertiaCOM'] = _CorrectedInertia(linkDict['inertiaCOM'])
            problems = [problem + ' (corrected)' for problem in problems]
        linkDict['inertiaProblems'] = problems
        if len(problems) != 0 and verbose > 0:
            exudyn.Print('WARNING: GetURDFrobotData: link', linkDict['name'] + ':', '; '.join(problems))

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


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the visual meshes of a URDF file without the RTB (#3263)

def _URDForigin(element):
    """the HT of an <origin xyz rpy> element (None: identity); rpy as in URDF, R = Rz(yaw) Ry(pitch) Rx(roll)"""
    H = exudyn.HT()
    if element is None:
        return H
    xyz = [float(x) for x in element.get('xyz', '0 0 0').split()]
    (r, p, y) = [float(x) for x in element.get('rpy', '0 0 0').split()]
    R = exudyn.HT().SetRotationZ(y)*exudyn.HT().SetRotationY(p)*exudyn.HT().SetRotationX(r)
    return exudyn.HT(rotation=np.array(R.rotation), translation=xyz)


def LoadURDFgraphics(urdfFile, packagePath=None, color=None, invertTriangles=False, orientTriangles=True,
                     smoothNormals=True, smoothingAngle=0.25*np.pi, addEdges=False):
    """the visual meshes of the links of a URDF file as Exudyn graphics, without the Robotics Toolbox (#3263): for each
    <link> its <visual> elements with a mesh of an STL file (binary or text, read by graphics.FromSTLfile), their <origin> and scale,
    and the color of their <material>; meshes of other formats and the shapes box, cylinder, sphere are skipped with a
    warning. The triangles of STL files are often not oriented consistently, and their normals are those of the
    triangles: the options invert them, orient them (graphics.OrientTriangles) and smoothen the normals
    (graphics.AddEdgesAndSmoothenNormals), as the meshes of a URDF need

    Args:
        urdfFile: the path of the URDF file
        packagePath: the directory that contains the packages of the mesh file names package://package/...; None: the
            directory above the package of the URDF file (the layout package/urdf/robot.urdf of ROS packages)
        color: None, the colors of the URDF materials (grey without one); or one RGBA color for all meshes
        invertTriangles: if True, the orientation of the triangles of the STL files inverted, for meshes that come
            inside out; STL files and Exudyn have the corners counterclockwise seen from outside (#3275)
        orientTriangles: if True, then the triangles of each mesh oriented consistently, closed parts and open parts
            that enclose a volume (the meshes of many URDFs have gaps) outward (graphics.OrientTriangles); another
            open surface keeps the orientation that most of its triangles have
        smoothNormals: if True, the normals smoothened at the points, except at edges sharper than smoothingAngle;
            else the normals of the triangles
        smoothingAngle: the angle between two triangles (in radians) above which their common edge stays sharp
        addEdges: if True, the sharp edges added as lines

    Returns:
        a dict of the link names to lists of graphics data in the frame of the link
    """
    import xml.etree.ElementTree as ElementTree
    import os
    if packagePath is None:
        packagePath = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(urdfFile))))
    root = ElementTree.parse(urdfFile).getroot()
    materials = {m.get('name'): m.find('color') for m in root.findall('material')}
    linkGraphics = {}
    for link in root.findall('link'):
        graphicsList = []
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                exudyn.Print('WARNING: LoadURDFgraphics: link ' + str(link.get('name')) + ': only meshes are loaded')
                continue
            fileName = mesh.get('filename')
            if fileName.startswith('package://'):
                fileName = os.path.join(packagePath, fileName[len('package://'):])
            elif not os.path.isabs(fileName):
                fileName = os.path.join(os.path.dirname(os.path.abspath(urdfFile)), fileName)
            if not fileName.lower().endswith('.stl'):
                exudyn.Print('WARNING: LoadURDFgraphics: ' + fileName + ' skipped; only STL files are read')
                continue
            rgba = [0.6, 0.6, 0.6, 1.]
            material = visual.find('material')
            if material is not None:
                colorElement = material.find('color')
                if colorElement is None:
                    colorElement = materials.get(material.get('name'))
                if colorElement is not None:
                    rgba = [float(x) for x in colorElement.get('rgba').split()]
            if color is not None:
                rgba = list(color)
            scale = [float(x) for x in mesh.get('scale', '1 1 1').split()]
            H = _URDForigin(visual.find('origin'))
            gData = graphics.FromSTLfile(fileName, color=rgba, scale=np.array(scale), Aoff=np.array(H.rotation),
                                         pOff=np.array(H.translation), invertTriangles=invertTriangles, invertNormals=invertTriangles,
                                         checkOrientation=not orientTriangles)
            if orientTriangles:
                gData = graphics.OrientTriangles(gData, outward=True, normals='remove')
            if smoothNormals or addEdges:
                gData = graphics.AddEdgesAndSmoothenNormals(gData, edgeAngle=smoothingAngle, addEdges=addEdges,
                                                            smoothNormals=smoothNormals)
            graphicsList += [gData]
        linkGraphics[link.get('name')] = graphicsList
    return linkGraphics


def AddURDFgraphics(robot, urdfFile, linkNames, baseLinkName=None, packagePath=None, color=None, **options):
    """the graphics of a Robot from the visual meshes of a URDF file (#3263), for a robot whose link frames are the
    frames of the URDF links, as the models of exudyn.robotics.models with dhMode 'jointFrames' (e.g.
    ManipulatorStaubliTX2_90L, whose dict names the URDF and its links in 'urdfGraphics'); the joint and COM graphics of
    the links switched off; further keyword arguments go to LoadURDFgraphics (invertTriangles, orientTriangles,
    smoothNormals, smoothingAngle, addEdges)

    Args:
        robot: the exudyn.robotics.Robot
        urdfFile: the path of the URDF file
        linkNames: the names of the URDF links of the robot links, one per link
        baseLinkName: None, or the name of the URDF link of the base
        packagePath: as LoadURDFgraphics
        color: as LoadURDFgraphics
    """
    if len(linkNames) != len(robot.links):
        raise ValueError('AddURDFgraphics: linkNames must have one name per link (' + str(len(robot.links)) + ')')
    linkGraphics = LoadURDFgraphics(urdfFile, packagePath, color, **options)
    from copy import copy
    from exudyn.robotics.roboticsCore import VRobotBase
    for (link, name) in zip(robot.links, linkNames):
        if name not in linkGraphics:
            raise ValueError('AddURDFgraphics: the URDF file has no link ' + str(name))
        link.visualization = copy(link.visualization) #a default VRobotLink may be shared by links
        link.visualization.graphicsData = linkGraphics[name]
        link.visualization.showMBSjoint = False
        link.visualization.showCOM = False
    if baseLinkName is not None:
        robot.base.visualization = VRobotBase(graphicsData=linkGraphics[baseLinkName])
