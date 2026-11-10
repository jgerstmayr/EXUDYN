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
    'GetRoboticsToolboxInternalModel', 'LoadURDFrobot', 'CompleteInertiasFromMeshes',
    'ReadColladaMesh', 'GetURDFrobotData', 'ReduceURDFrobotData', 'RobotModelDictFromData',
    'FitCapsuleToPoints', 'FitCapsulesToMesh', 'CollisionShapesFromGeometry', 'ConvexHullOfMesh',
    'RobotFromURDFdata', 'MakeCorkeRobot', 'LoadURDFgraphics', 'AddURDFgraphics',
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
        requires the Robotics Toolbox for Python 1.5 (pip install roboticstoolbox-python); its models UR3, UR5, UR10,
        KinovaGen3, Jaco, Fetch and PR2 are read through the package robot_descriptions, which downloads them on first
        use; Puma560, Panda, LBR, AL5D and the Interbotix robots (px150, vx300, wx250, ...) are part of the RTB's data.
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

def _MeshMassProperties(linkDict, density):
    """[mass, COM, inertia at the COM] of the solid enclosed by the meshes of a link; mass 0 without meshes"""
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

def CompleteInertiasFromMeshes(robotData, density=1000., verbose=1):
    """An estimate for the links of GetURDFrobotData whose mass or inertia is missing or not physical (#3357). A mesh gives the surface, not the interior of a link with its
    hollow arms, drives and cables, so that only one overall density of the enclosed volume makes sense, around 1000 kg/m^3; the inertials of a URDF which are correct always remain.
    A link with a mass but without a positive definite inertia gets the inertia of the solid of its meshes, scaled to the mass; a link without mass gets the mass, center of mass and
    inertia of that solid with the density. The principal moments are then raised as little as possible to fulfill the triangle inequality.

    Args:
        robotData: the dict of GetURDFrobotData
        density: the overall density of the volume enclosed by the meshes of a link, in kg/m^3, used for the links without mass
        verbose: 0 .. no output, 1 .. the links which remain without a physical mass or inertia

    Returns:
        a new dict of the same form; 'inertiaProblems' of each link tells what was changed, with ' (corrected)' at each problem, or what remains

    Note:
        the result is an estimate; store it with the Robot (Robot.SaveToHDF5), where it can be read and corrected
    """
    result = dict(robotData)
    result['linkList'] = [dict(link) for link in robotData['linkList']]
    for link in result['linkList']:
        problems = _InertiaProblems(link['mass'], link['inertiaCOM'])
        if len(problems) == 0:
            link['inertiaProblems'] = []
            continue
        [m, c, inertia] = _MeshMassProperties(link, 1.)
        if link['mass'] <= 0 and m > 0:
            (link['mass'], link['com'], link['inertiaCOM']) = (m*density, c, inertia*density)
        elif link['mass'] > 0 and m > 0 and np.linalg.eigvalsh(0.5*(np.array(link['inertiaCOM']) + np.array(link['inertiaCOM']).T))[0] <= 0:
            link['inertiaCOM'] = inertia*(link['mass']/m)
        if link['mass'] > 0:
            link['inertiaCOM'] = _CorrectedInertia(link['inertiaCOM'])
            problems = [problem + ' (corrected)' for problem in problems]
        elif verbose > 0:
            exudyn.Print('WARNING: CompleteInertiasFromMeshes: link', link['name'], 'has neither a mass nor meshes')
        link['inertiaProblems'] = problems
    return result

def ReadColladaMesh(fileName):
    """The triangles of a Collada (.dae) file with the diffuse colors of their materials, which pymeshlab does not keep (#3398); the transformations of the nodes, the unit and the up axis are applied, and the polygons of a polylist are split into triangles.

    Args:
        fileName: the .dae file

    Returns:
        a dict with 'points', 'normals' and 'colors' (RGBA), one per row, and 'triangles', the indices of their points, or None if a material has a texture or the file has no triangles
    """
    import xml.etree.ElementTree as ET
    root = ET.parse(fileName).getroot()
    ns = root.tag[:root.tag.index('}') + 1] if root.tag.startswith('{') else ''
    def All(element, path):
        return element.findall('/'.join(ns + part for part in path.split('/')))
    def One(element, path):
        return element.find('/'.join(ns + part for part in path.split('/')))

    (scale, upAxis) = (1., 'Z_UP')
    asset = One(root, 'asset')
    if asset is not None:
        unit = One(asset, 'unit')
        if unit is not None and unit.get('meter') is not None:
            scale = float(unit.get('meter'))
        up = One(asset, 'up_axis')
        if up is not None and up.text:
            upAxis = up.text.strip()

    effectColor = {} #the diffuse color of each effect, and of each material by its effect
    for effect in All(root, 'library_effects/effect'):
        diffuse = next(effect.iter(ns + 'diffuse'), None)
        if diffuse is None:
            continue
        if One(diffuse, 'texture') is not None:
            return None
        color = One(diffuse, 'color')
        if color is not None:
            effectColor[effect.get('id')] = [float(v) for v in color.text.split()][:4]
    materialColor = {}
    for material in All(root, 'library_materials/material'):
        instance = One(material, 'instance_effect')
        if instance is not None:
            materialColor[material.get('id')] = effectColor.get(instance.get('url', '#')[1:])

    geometries = {g.get('id'): g for g in All(root, 'library_geometries/geometry')}
    (points, normals, colors) = ([], [], [])

    def AddGeometry(geometry, HT, bindings):
        mesh = One(geometry, 'mesh')
        if mesh is None:
            return
        sources = {}
        for source in All(mesh, 'source'):
            (array, accessor) = (One(source, 'float_array'), One(source, 'technique_common/accessor'))
            if array is not None and accessor is not None and array.text:
                sources[source.get('id')] = np.array(array.text.split(), dtype=float).reshape(-1, int(accessor.get('stride', '1')))
        vertices = {}
        for v in All(mesh, 'vertices'):
            for inp in All(v, 'input'):
                if inp.get('semantic') == 'POSITION':
                    vertices[v.get('id')] = sources[inp.get('source')[1:]]
        for kind in ('triangles', 'polylist'):
            for primitive in All(mesh, kind):
                inputs = All(primitive, 'input')
                p = One(primitive, 'p')
                if p is None or not p.text:
                    continue
                indices = np.array(p.text.split(), dtype=int).reshape(-1, 1 + max(int(inp.get('offset', '0')) for inp in inputs))
                if kind == 'polylist': #a fan of triangles per polygon
                    (corners, start) = ([], 0)
                    for n in np.array(One(primitive, 'vcount').text.split(), dtype=int):
                        for k in range(1, n - 1):
                            corners += [start, start + k, start + k + 1]
                        start += n
                    indices = indices[corners]
                (position, normal) = (None, None)
                for inp in inputs:
                    offset = int(inp.get('offset', '0'))
                    if inp.get('semantic') == 'VERTEX':
                        position = vertices[inp.get('source')[1:]][indices[:, offset]]
                    elif inp.get('semantic') == 'NORMAL':
                        normal = sources[inp.get('source')[1:]][indices[:, offset]]
                if position is None:
                    continue
                position = position @ HT[:3, :3].T + HT[:3, 3]
                if normal is None: #the normals of the triangles
                    corners3 = position.reshape(-1, 3, 3)
                    normal = np.repeat(np.cross(corners3[:, 1] - corners3[:, 0], corners3[:, 2] - corners3[:, 0]), 3, axis=0)
                else:
                    normal = normal @ HT[:3, :3].T
                normal = normal/np.maximum(np.linalg.norm(normal, axis=1), 1e-30)[:, None]
                color = materialColor.get(bindings.get(primitive.get('material'), primitive.get('material')))
                color = [0.7, 0.7, 0.7, 1.] if color is None else (color + [1.])[:4]
                points.append(position)
                normals.append(normal)
                colors.append(np.tile(color, (len(position), 1)))

    def Visit(node, HT):
        matrix = One(node, 'matrix')
        if matrix is not None:
            HT = HT @ np.array(matrix.text.split(), dtype=float).reshape(4, 4)
        for instance in All(node, 'instance_geometry'):
            bindings = {m.get('symbol'): m.get('target', '#')[1:] for m in instance.iter(ns + 'instance_material')}
            geometry = geometries.get(instance.get('url', '#')[1:])
            if geometry is not None:
                AddGeometry(geometry, HT, bindings)
        for child in All(node, 'node'):
            Visit(child, HT)

    HT0 = np.diag([scale, scale, scale, 1.])
    if upAxis == 'Y_UP':
        HT0 = np.array([[1., 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]) @ HT0
    elif upAxis == 'X_UP':
        HT0 = np.array([[0., -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ HT0
    for scene in All(root, 'library_visual_scenes/visual_scene'):
        for node in All(scene, 'node'):
            Visit(node, HT0)
    if len(points) == 0:
        return None
    corners = np.hstack([np.vstack(points), np.vstack(normals), np.vstack(colors)]) #corners of the same point, normal and color are one point
    (unique, index) = np.unique(corners, axis=0, return_inverse=True)
    return {'points': unique[:, :3], 'triangles': np.array(index).reshape(-1, 3), 'normals': unique[:, 3:6], 'colors': unique[:, 6:]}


def GetURDFrobotData(robot, 
                     urdf=None, 
                     linkColorList = None,
                     staticJointValues=None,
                     returnStaticGraphicsList = False,
                     exportMesh = False,
                     verbose=1,
                     densityForInertiaEstimation = None,
                     toolLinkName = None,
                     collision = False):
    """Interface to roboticstoolbox (RTB) of P. Corke and J. Haviland and Pymeshlab to import robot model and visualization into a struture readable by Exudyn. NOTE that this function is to be seen as a starting point for import, while some models have to be imported differently, in particular for joints that are not revolute or prismatic (in this case, copy function into local file and modify)!

    Args:
        robot: a RTB Robot (class) model, as returned e.g. by LoadURDFrobot
        urdf: not used: the visuals and their origins are those of the links of robot (#3280)
        linkColorList: if not None, this can contain a list of RGBA color lists for each link to prescribe colors instead of using internally stored colors or general color information (.obj files, the materials of .dae files, see ReadColladaMesh); set linkColorList=[graphics.color.red]*8 to set 8 link colors red; links are counted as in the urdf file and may be different from the number of joints
        staticJointValues: if not None, has to be a list of joint angles (or displacements) for computing the static graphics list
        returnStaticGraphicsList: return a list of GraphicsData which can be put into a ground to check visualization for zero joints, using: mbs.CreateGround(graphicsDataList=staticGraphicsList)
        exportMesh: if True, the returned dict also contains a meshSetList which refers to the MeshSet in pymeshlab, which can be used for debugging purposes
        verbose: 0 .. no output printed (only exceptions), 1 .. warnings, 2 .. further information
        densityForInertiaEstimation: None, or the overall density of the volume enclosed by the meshes in kg/m^3, around 1000: a joint link without any inertial data gets the mass, center of
            mass and inertia of that solid; a mesh that does not close gives an estimate only; for partly given data see CompleteInertiasFromMeshes (#3357). Every link reports the problems
            of its mass and inertia in 'inertiaProblems', and they are printed with verbose > 0 (#3282)
        toolLinkName: if not None, the name of the link of the tool frame; the frame of this link relative to the last
            joint link is returned in 'toolHT', for RobotFromURDFdata (#3356)
        collision: if True, each joint link gets 'collision' and the dict the key 'collisionBase' for the links fixed to the base: a list of the
            collision geometries of the URDF in the frame of the link, also those of the links fixed to it: {'type': 'box', 'size': ...,
            'HT': ...}, {'type': 'cylinder', 'radius': ..., 'length': ..., 'HT': ...}, {'type': 'sphere', 'radius': ..., 'HT': ...} and
            {'type': 'mesh', 'points': ..., 'triangles': ...}; the box and the axis of the cylinder are centered in HT, a 4x4 matrix;
            RobotFromURDFdata puts them into the collision shapes of the robot; a mesh needs pymeshlab (#3382)

    Returns:
        returns dictionary with items linkList (the joint links, each with its name, jointName, jointNumber (the index of
        the joint in the toolbox, as in staticJointValues), preHT relative to the joint link above, its parentName, mass,
        com, inertiaCOM, jointType, jointLimits and graphicsDataList; a parent before its children, else in the order of
        the links of the toolbox), numberOfJoints, baseLinkNames (the names of the links fixed to the base),
        graphicsBaseList (the graphics of the links fixed to the base), graphicsToolList (empty: the links fixed after the last joint
        are in the graphics of that link), staticJointValues and staticGraphicsList; a link of the URDF without joint
        (a fixed joint) moves with the joint link above it, which gets its graphics and its mass (#3280); a flipped joint
        (axis -1 in the URDF) has the same joint coordinate in a link frame turned by pi, flipHT of the link (#3282); the
        visuals of the URDF are its meshes and its boxes, cylinders and spheres (#3393)

    Note:
        requires installation (pip install) of roboticstoolbox-python and pymeshlab; if pymeshlab is not installed, a warning is raised and the meshes are ignored
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
            HT[:3, :3] = np.diag({'x': [-1., 1., -1.], 'y': [-1., -1., 1.], 'z': [1., -1., -1.]}[link.v.kind[-1].lower()])
        return HT

    def JointAncestor(link): #the joint link above, and the frame relative to the frame of its Exudyn link
        HT = np.eye(4)
        while link is not None and not link.isjoint:
            HT = LinkTs(link) @ HT
            link = link.parent
        return link, FlipHT(link) @ HT

    #the links with each parent before its children, else in the order of the toolbox; the fixed links below need the
    #joint link above them (#3393)
    indexOfLink = {id(link): i for (i, link) in enumerate(robot.links)}
    orderedLinks = []
    placed = set()
    remaining = list(robot.links)
    while len(remaining) != 0:
        rest = []
        for link in remaining:
            if link.parent is None or id(link.parent) not in indexOfLink or id(link.parent) in placed:
                orderedLinks.append(link)
                placed.add(id(link))
            else:
                rest.append(link)
        if len(rest) == len(remaining):
            raise ValueError('GetURDFrobotData: the parents of the links ' + str([link.name for link in rest]) + ' form a loop')
        remaining = rest

    linkDictOfName = {}
    baseLinkNames = [] #the links fixed to the base
    texturesAdded = []
    for link in orderedLinks:
        i = indexOfLink[id(link)]
        #the frame of the link at staticJointValues
        if link.parent is None:
            staticHT = np.array(robot.base) @ LinkTs(link)
        else:
            staticHT = np.array(robot.fkine(staticJointValues, end=link))

        jointIndex = link.jindex
        (ancestor, HTancestor) = (link, FlipHT(link)) if link.isjoint else JointAncestor(link)
        linkDict = None if ancestor is None else linkDictOfName.get(ancestor.name) #None: the base
        if ancestor is None:
            baseLinkNames.append(link.name)
        if link.isjoint:
            if jointIndex is None: raise ValueError('inconsistent joint index / isjoint in robot structure')
            (parent, HTparent) = JointAncestor(link.parent)

            linkDict = {}
            #required fields:
            linkDict['name'] = link.name
            linkDict['jointName'] = getattr(link, '_joint_name', None) #the name of the joint in the URDF
            linkDict['jointNumber'] = jointIndex
            linkDict['parentNumber'] = -1 if parent is None else parent.number
            linkDict['parentName'] = 'None' if parent is None else parent.name
            F = FlipHT(link)
            linkDict['preHT'] = HTparent @ LinkTs(link) @ F #A(0), relative to the joint link above
            linkDict['inertiaCOM'] = F[:3, :3] @ np.array(link.I) @ F[:3, :3]
            linkDict['com'] = F[:3, :3] @ np.array(link.r)
            linkDict['flipHT'] = F #the frame of the URDF link is the frame of the Exudyn link times flipHT
            linkDict['mass'] = link.m
            linkDict['jointType'] = rtbAxis2exuJointType[link.v.kind] #Rx,Ry,Rz,tx,ty,tz; ET.axis is deprecated since RTB 1.4 (#3285)
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
                    collada = None
                    if scene.filename.lower().endswith('.dae') and linkColorList is None:
                        collada = ReadColladaMesh(scene.filename) #the colors of its materials, which pymeshlab drops (#3398)
                    if collada is not None:
                        gData = graphics.FromPointsAndTrigs(points = collada['points'] * scene.scale,
                                                            triangles = collada['triangles'],
                                                            normals = collada['normals'],
                                                            color = collada['colors'], orientTriangles = False)
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

            elif scene.stype in ['cuboid', 'cylinder', 'sphere']: #a primitive as visual (#3393)
                color = graphics.colorList[i%16]
                if linkColorList is not None:
                    color = linkColorList[i]
                elif getattr(scene, 'color', None) is not None:
                    color = list(scene.color)
                if scene.stype == 'cuboid':
                    gData = graphics.Brick(size=list(scene.scale), color=color)
                elif scene.stype == 'cylinder': #its axis is z, centered at the origin
                    gData = graphics.Cylinder(pAxis=[0, 0, -0.5*scene.length], vAxis=[0, 0, scene.length], radius=scene.radius,
                                              color=color, nTiles=32)
                else:
                    gData = graphics.Sphere(radius=scene.radius, color=color, nTiles=16)
            else:
                continue #a mesh without pymeshlab, or a shape that the URDF does not define

            visualHT = np.array(scene.T) #the origin of the visual in the link frame
            totalHT = staticHT@visualHT
            if linkDict is not None: #in the frame of the joint link it moves with
                localHT = HTancestor@visualHT
                linkDict['graphicsDataList'] += [graphics.Move(gData, HT2translation(localHT), HT2rotationMatrix(localHT))]
            else: #fixed to the base
                graphicsBaseList += [graphics.Move(gData, HT2translation(totalHT), HT2rotationMatrix(totalHT))]
            
            if returnStaticGraphicsList :
                staticGraphicsList += [graphics.Move(gData, HT2translation(totalHT), HT2rotationMatrix(totalHT) )]

    collisionBase = []
    if collision:
        for link in robot.links:
            (ancestor, HTancestor) = (link, FlipHT(link)) if link.isjoint else JointAncestor(link)
            linkDict = None if ancestor is None else linkDictOfName.get(ancestor.name)
            target = collisionBase if linkDict is None else linkDict.setdefault('collision', [])
            for scene in link.collision:
                T = HTancestor @ np.array(scene.T)
                if scene.stype == 'cuboid':
                    target += [{'type': 'box', 'size': np.array(scene.scale, dtype=float), 'HT': T}]
                elif scene.stype == 'cylinder':
                    target += [{'type': 'cylinder', 'radius': float(scene.radius), 'length': float(scene.length), 'HT': T}]
                elif scene.stype == 'sphere':
                    target += [{'type': 'sphere', 'radius': float(scene.radius), 'HT': T}]
                elif scene.stype == 'mesh':
                    if not hasPymeshlab:
                        raise ImportError('GetURDFrobotData: a collision mesh needs pymeshlab')
                    ms = pymeshlab.MeshSet()
                    ms.load_new_mesh(scene.filename)
                    mesh = ms.current_mesh()
                    points = (T[:3, :3] @ (mesh.vertex_matrix() * scene.scale).T).T + T[:3, 3]
                    target += [{'type': 'mesh', 'points': points, 'triangles': np.array(mesh.face_matrix())}]
                elif verbose > 0:
                    exudyn.Print('WARNING: GetURDFrobotData: collision geometry', scene.stype, 'of link', link.name, 'is ignored')

    for linkDict in linkList:
        if densityForInertiaEstimation is not None and linkDict['mass'] <= 0 and np.max(np.abs(linkDict['inertiaCOM'])) == 0:
            [m, c, inertia] = _MeshMassProperties(linkDict, densityForInertiaEstimation)
            if m > 0:
                (linkDict['mass'], linkDict['com'], linkDict['inertiaCOM']) = (m, c, inertia)
        problems = _InertiaProblems(linkDict['mass'], linkDict['inertiaCOM'])
        linkDict['inertiaProblems'] = problems
        if len(problems) != 0 and verbose > 0:
            exudyn.Print('WARNING: GetURDFrobotData: link', linkDict['name'] + ':', '; '.join(problems))

    robotData = {'linkList': linkList,
            'baseLinkNames': baseLinkNames,
            'numberOfJoints': numberOfJoints,
            'graphicsBaseList': graphicsBaseList,
            'graphicsToolList': graphicsToolList,
            'staticJointValues': staticJointValues,
            'staticGraphicsList': staticGraphicsList}
    if exportMesh: robotData['meshSetList'] = meshSetList
    if collision: robotData['collisionBase'] = collisionBase
    if toolLinkName is not None:
        toolLinks = [link for link in robot.links if link.name == toolLinkName]
        if len(toolLinks) != 1:
            raise ValueError('GetURDFrobotData: toolLinkName "' + str(toolLinkName) + '" is not a link of the robot')
        zero = np.zeros(numberOfJoints)
        robotData['toolHT'] = linkList[-1]['flipHT'] @ np.linalg.inv(np.array(robot.fkine(zero, end=linkList[-1]['name']))) @ np.array(robot.fkine(zero, end=toolLinks[0]))

    return robotData


def _MovedCollisionGeometry(geometry, R, d):
    """a collision geometry of a link given in a frame with rotation R and position d of the new frame of the link"""
    moved = dict(geometry)
    if geometry['type'] == 'mesh':
        moved['points'] = d + (R @ np.array(geometry['points']).T).T
    else:
        T = np.eye(4)
        (T[:3, :3], T[:3, 3]) = (R, d)
        moved['HT'] = T @ np.array(geometry['HT'])
    return moved


def _JointHT(jointType, q):
    """the homogeneous transformation of a joint of type Rx, Ry, Rz, Px, Py or Pz at the joint value q"""
    T = np.eye(4)
    axis = 'xyz'.index(jointType[1])
    if jointType[0] == 'P':
        T[axis, 3] = q
    else:
        (i, j) = ((axis + 1) % 3, (axis + 2) % 3)
        (c, s) = (cMath.cos(q), cMath.sin(q))
        T[i, i], T[i, j], T[j, i], T[j, j] = c, -s, s, c
    return T


def ReduceURDFrobotData(robotData, lockedJoints=None, removedLinks=None):
    """Leave out joints and links in the data of GetURDFrobotData, for example the fingers of a gripper (#3372); needs
    neither the Robotics Toolbox nor a file.

    Args:
        robotData: the dict of GetURDFrobotData; it is not changed
        lockedJoints: None or a dict {name: value}, name the joint of the URDF or its link, value the joint value at which
            it is fixed; the link is merged into the joint link above it: mass, center of mass and inertia added by the
            parallel axis theorem, the graphics moved, its children attached to that link. A locked link at the root
            has its graphics in the base, and its mass is lost as the base has none
        removedLinks: None or a list of names of links (or their joints) which are left out with all links below them,
            with their mass and graphics; only the first n links of a serial robot: the name of link n+1

    Returns:
        a new dict of the same form, with numberOfJoints, staticJointValues and inertiaProblems updated; the 'toolHT' relative
        to the last remaining link, for the tool frame at zero joint values (a tool on a locked link at a value other than
        zero is not moved with it); staticGraphicsList is empty, as it shows the robot before the reduction

    Example:
        data = GetURDFrobotData(robot, toolLinkName='panda_hand')
        data = ReduceURDFrobotData(data, lockedJoints={'panda_finger_joint1':0.04, 'panda_finger_joint2':0.04})
        robot = RobotFromURDFdata(data, PDcontrol=(1e4, 1e2))
    """
    lockedJoints = {} if lockedJoints is None else lockedJoints
    removedLinks = [] if removedLinks is None else removedLinks
    oldLinks = robotData['linkList']
    def Index(name):
        for (i, link) in enumerate(oldLinks):
            if name in (link['name'], link.get('jointName')):
                return i
        raise ValueError('ReduceURDFrobotData: "' + str(name) + '" is neither a joint nor a link with a joint of the robot, it has the links '
                         + str([link['name'] for link in oldLinks]))
    locked = {Index(name): value for (name, value) in lockedJoints.items()}
    numberOfName = {link['name']: i for (i, link) in enumerate(oldLinks)}
    numberOfName['None'] = -1

    #the links left out: a parent comes before its children in linkList
    removed = set(Index(name) for name in removedLinks)
    for (i, link) in enumerate(oldLinks):
        if numberOfName[link['parentName']] in removed:
            removed.add(i)
    if len(removed.intersection(locked)) != 0:
        raise ValueError('ReduceURDFrobotData: a joint cannot be locked and removed, ' + str([oldLinks[i]['name'] for i in removed.intersection(locked)]))

    #the frames of all links at zero joint values, for the tool frame
    frames = []
    for link in oldLinks:
        parent = numberOfName[link['parentName']]
        frames.append((np.eye(4) if parent < 0 else frames[parent]) @ np.array(link['preHT']))

    links = {i: dict(link) for (i, link) in enumerate(oldLinks) if i not in removed}
    graphicsBaseList = list(robotData.get('graphicsBaseList', []))
    parentOf = {i: numberOfName[link['parentName']] for (i, link) in links.items()} #changes as links are merged
    for i in sorted(locked):
        link = links.pop(i)
        A = np.array(link['preHT']) @ _JointHT(link['jointType'], locked[i]) #the merged link in the frame of its parent
        (R, d) = (A[:3, :3], A[:3, 3])
        parent = parentOf[i]
        moved = [graphics.Move(g, list(d), R.tolist()) for g in link['graphicsDataList']]
        if parent < 0:
            graphicsBaseList += moved
        else:
            target = links[parent]
            (m0, c0, I0) = (target['mass'], np.array(target['com'], dtype=float), np.array(target['inertiaCOM'], dtype=float))
            (m1, c1, I1) = (link['mass'], R @ np.array(link['com'], dtype=float) + d, R @ np.array(link['inertiaCOM'], dtype=float) @ R.T)
            m = m0 + m1
            c = c0 if m <= 0 else (m0*c0 + m1*c1)/m
            target['inertiaCOM'] = I0 + _Steiner(m0, c0 - c) + I1 + _Steiner(m1, c1 - c)
            target['com'] = c
            target['mass'] = m
            target['graphicsDataList'] = list(target['graphicsDataList']) + moved
            if 'collision' in link:
                target['collision'] = target.get('collision', []) + [_MovedCollisionGeometry(g, R, d) for g in link['collision']]
            target['inertiaProblems'] = _InertiaProblems(m, target['inertiaCOM'])
        for (j, child) in links.items():
            if parentOf[j] == i:
                child['preHT'] = A @ np.array(child['preHT'])
                child['parentName'] = 'None' if parent < 0 else links[parent]['name']
                parentOf[j] = parent

    keep = sorted(links)
    if len(keep) == 0:
        raise ValueError('ReduceURDFrobotData: no link with a joint remains')
    linkList = [links[i] for i in keep]
    newNumber = {i: n for (n, i) in enumerate(keep)}
    for (n, i) in enumerate(keep):
        linkList[n]['jointNumber'] = n
        linkList[n]['parentNumber'] = -1 if parentOf[i] < 0 else newNumber[parentOf[i]]

    result = dict(robotData)
    result.update({'linkList': linkList, 'numberOfJoints': len(linkList), 'graphicsBaseList': graphicsBaseList, 'staticGraphicsList': [],
                   'staticJointValues': np.array(robotData['staticJointValues'])[keep] if 'staticJointValues' in robotData else np.zeros(len(linkList))})
    if 'toolHT' in robotData:
        result['toolHT'] = np.linalg.inv(frames[keep[-1]]) @ frames[-1] @ np.array(robotData['toolHT'])
    return result


def RobotModelDictFromData(robotData):
    """The part of a model of robotics.models given by a URDF, as Python text, from the data of GetURDFrobotData (#3373): the
    links in the joint frames of the URDF, for dhMode 'jointFrames'; what a URDF does not give is added by hand to the dict.

    Args:
        robotData: the dict of GetURDFrobotData, possibly reduced by ReduceURDFrobotData

    Returns:
        a string with the Python text of the list of the links, each with name, parent, jointType, jointLimits, preHT, mass, COM, inertia and, if the link has them, collisionShapes
    """
    def Text(x):
        x = np.array(x, dtype=float)
        if x.ndim > 1:
            return '[' + ', '.join(Text(row) for row in x) + ']'
        return '[' + ', '.join('%.12g' % (0. if abs(v) < 1e-10 else v) for v in x) + ']'
    def TextHT(T):
        T = np.array(T, dtype=float)
        if np.max(np.abs(T[:3, :3] - np.eye(3))) < 1e-10:
            return 'exudyn.HT(translation=%s)' % Text(T[:3, 3])
        return 'exudyn.HT(rotation=%s, translation=%s)' % (Text(T[:3, :3]), Text(T[:3, 3]))
    numberOfName = {link['name']: i for (i, link) in enumerate(robotData['linkList'])}
    numberOfName['None'] = -1
    lines = ['[']
    for link in robotData['linkList']:
        limits = link['jointLimits']
        lines += ["    {'name': '%s', 'parent': %d, 'jointType': '%s', 'jointLimits': %s," % (link['name'], numberOfName[link['parentName']], link['jointType'], 'None' if limits[1] <= limits[0] else Text(limits)),
                  "     'preHT': %s," % TextHT(link['preHT']),
                  "     'mass': %.12g, 'COM': %s, 'inertia': np.array(%s)," % (link['mass'], Text(link['com']), Text(link['inertiaCOM']))]
        if 'collisionShapes' in link:
            lines += ["     'collisionShapes': %s," % repr(link['collisionShapes'])]
        lines[-1] = lines[-1][:-1] + '},'
    return '\n'.join(lines + [']'])


def _EnclosingCircle(points):
    """center and radius of a small circle containing all 2D points, by the iteration of Badoiu and Clarkson"""
    center = points.mean(axis=0)
    for i in range(1, 300):
        center = center + (points[np.argmax(np.sum((points - center)**2, axis=1))] - center)/(i + 1)
    return center, np.sqrt(np.max(np.sum((points - center)**2, axis=1)))


def FitCapsuleToPoints(points):
    """A small capsule that contains all given points, the simple collision shape of a link (#3374).

    Args:
        points: the points, an array of shape (n, 3), for example of a collision mesh

    Returns:
        dict with type 'capsule' and pAxis, vAxis, radius as for graphics.Capsule: pAxis and pAxis+vAxis are the centers of its two spheres; the
        principal axis of the points giving the smallest volume is taken; vAxis is zero for a sphere
    """
    points = np.array(points, dtype=float)
    mean = points.mean(axis=0)
    axes = np.linalg.svd(points - mean, full_matrices=False)[2]
    best = None
    for k in range(3):
        (axis, t1, t2) = (axes[k], axes[(k + 1) % 3], axes[(k + 2) % 3])
        local = points - mean
        (center, r) = _EnclosingCircle(np.stack([local @ t1, local @ t2], axis=1))
        s = local @ axis
        d2 = (local @ t1 - center[0])**2 + (local @ t2 - center[1])**2
        h = np.sqrt(np.maximum(r*r - d2, 0.))
        (a0, a1) = (np.min(s + h), np.max(s - h)) #the centers of the spheres must reach all points beyond them
        if a1 < a0: #a sphere: the center between them, the radius such that it holds all points
            a0 = a1 = 0.5*(a0 + a1)
            r = np.sqrt(np.max(d2 + (s - a0)**2))
        volume = np.pi*r*r*(a1 - a0) + 4/3*np.pi*r**3
        if best is None or volume < best[0]:
            best = (volume, mean + axis*a0 + t1*center[0] + t2*center[1], axis*(a1 - a0), r)
    return {'type': 'capsule', 'pAxis': best[1].tolist(), 'vAxis': best[2].tolist(), 'radius': float(best[3])}


def _PointsOnMesh(points, triangles, spacing):
    """the points of a mesh and points on its triangles, so that long triangles are represented at the distance spacing"""
    points = np.array(points, dtype=float)
    result = [points]
    for (a, b, c) in points[np.array(triangles, dtype=int)]:
        n = int(np.ceil(max(cMath.norm(b - a), cMath.norm(c - b), cMath.norm(a - c))/spacing))
        if n > 1:
            result += [np.array([a + (b - a)*(i/n) + (c - a)*(j/n) for i in range(n + 1) for j in range(n + 1 - i)])]
    return np.vstack(result)


def _UnionVolume(capsules, numberOfGridPoints=40):
    """the volume of the union of capsules, on a regular grid of points"""
    ends = np.array([c['pAxis'] for c in capsules] + [np.array(c['pAxis']) + np.array(c['vAxis']) for c in capsules])
    radius = max(c['radius'] for c in capsules)
    (low, high) = (ends.min(axis=0) - radius, ends.max(axis=0) + radius)
    h = (high - low)/numberOfGridPoints
    axes = [low[i] + h[i]*(np.arange(numberOfGridPoints) + 0.5) for i in range(3)]
    grid = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1).reshape(-1, 3)
    inside = np.any(_DistanceToSegments(grid, capsules) <= np.array([c['radius'] for c in capsules]), axis=1)
    return np.count_nonzero(inside)*np.prod(h)


def _DistanceToSegments(points, capsules):
    """the distances of the points to the axes of the capsules, one column per capsule"""
    result = []
    for capsule in capsules:
        (p, v) = (np.array(capsule['pAxis']), np.array(capsule['vAxis']))
        s = np.clip((points - p) @ v/max(v @ v, 1e-30), 0., 1.)
        d = points - p - np.outer(s, v)
        result += [np.sqrt(np.sum(d*d, axis=1))]
    return np.array(result).T


def FitCapsulesToMesh(points, triangles=None, numberOfCapsules=2, minimumGain=1.):
    """Small capsules that together contain a mesh, for the collision shapes of a link: for a link that is bent, several capsules are
    much closer to it than one (#3382). The points are divided by their distance to the axes of the capsules, which are fitted again;
    all points on the surface are in one of the capsules.

    Args:
        points: the points of the mesh, an array of shape (n, 3)
        triangles: None or the triangles of the mesh, an array of point indices: points are added on long triangles, as the capsules must also contain these
        numberOfCapsules: the number of capsules
        minimumGain: the capsules are only returned if the volume of their union is below minimumGain times the volume of a single capsule; else one capsule is returned

    Returns:
        the list of dicts as returned by FitCapsuleToPoints
    """
    points = np.array(points, dtype=float)
    size = cMath.norm(np.max(points, axis=0) - np.min(points, axis=0))
    samples = points if triangles is None else _PointsOnMesh(points, triangles, 0.03*size)
    single = FitCapsuleToPoints(samples)
    if numberOfCapsules < 2:
        return [single]
    def Divided(assignment, n):
        return [FitCapsuleToPoints(samples[assignment == i]) for i in range(n) if np.any(assignment == i)]
    best = None
    axes = np.linalg.svd(samples - samples.mean(axis=0), full_matrices=False)[2]
    for axis in axes[:2]: #the start: slices along the principal axes
        s = samples @ axis
        assignment = np.minimum(((s - s.min())/max(s.max() - s.min(), 1e-30)*numberOfCapsules).astype(int), numberOfCapsules - 1)
        for iteration in range(12):
            capsules = Divided(assignment, numberOfCapsules)
            volume = _UnionVolume(capsules)
            if best is None or volume < best[0]:
                best = (volume, capsules)
            next = np.argmin(_DistanceToSegments(samples, capsules) - np.array([c['radius'] for c in capsules]), axis=1)
            if len(capsules) < numberOfCapsules or np.array_equal(next, assignment):
                break
            assignment = next
    return best[1] if best[0] < minimumGain*_UnionVolume([single]) else [single]


def CollisionShapesFromGeometry(geometry, meshesAs='capsules', numberOfCapsules=2):
    """The collision shapes of a RobotLink from the collision geometries of GetURDFrobotData (#3382).

    Args:
        geometry: the list of the key 'collision' of a link, or 'collisionBase', of GetURDFrobotData
        meshesAs: 'capsules': all meshes of the list together by FitCapsulesToMesh, for a fast check; 'convex': each mesh as the points of its convex hull, which requires scipy
        numberOfCapsules: the number of capsules for the meshes of the list

    Returns:
        the list of shapes, the box, cylinder and sphere as they are
    """
    if meshesAs not in ['capsules', 'convex']:
        raise ValueError("CollisionShapesFromGeometry: meshesAs must be 'capsules' or 'convex', not " + str(meshesAs))
    shapes = [g for g in geometry if g['type'] != 'mesh']
    meshes = [g for g in geometry if g['type'] == 'mesh']
    if meshesAs == 'convex':
        return shapes + [{'type': 'points', 'points': ConvexHullOfMesh(g['points'], g['triangles'], verbose=0)[0]} for g in meshes]
    if len(meshes) != 0:
        offsets = np.cumsum([0] + [len(g['points']) for g in meshes])
        points = np.vstack([g['points'] for g in meshes])
        triangles = np.vstack([np.array(g['triangles']) + offsets[i] for (i, g) in enumerate(meshes)])
        shapes = shapes + FitCapsulesToMesh(points, triangles, numberOfCapsules)
    return shapes


def ConvexHullOfMesh(points, triangles, name='', verbose=1):
    """The convex hull of a triangle mesh, as the contacts of polytopes need it, and how convex the mesh is (#3374).

    Args:
        points: the points of the mesh, an array of shape (n, 3)
        triangles: the triangles of the closed mesh, an array of shape (m, 3) of point indices
        name: the name written in the warning
        verbose: if larger than 0, a warning is printed where the volume of the mesh is less than 0.98 of its hull

    Returns:
        [hullPoints, hullTriangles, ratio], the triangles oriented outward and ratio the volume of the mesh divided by that of the hull

    Note:
        requires scipy
    """
    try:
        from scipy.spatial import ConvexHull
    except ImportError:
        raise ImportError('ConvexHullOfMesh: requires scipy (pip install scipy)')
    points = np.array(points, dtype=float)
    triangles = np.array(triangles, dtype=int)
    hull = ConvexHull(points)
    tri = points[triangles]
    ratio = abs(np.sum(np.einsum('ij,ij->i', tri[:, 0], np.cross(tri[:, 1], tri[:, 2])))/6)/hull.volume
    if ratio < 0.98 and verbose > 0:
        exudyn.Print('WARNING: the collision mesh', name, 'is not convex: volume/hull =', round(ratio, 3), '; its convex hull is used')
    indices = np.zeros(len(points), dtype=int)
    indices[hull.vertices] = np.arange(len(hull.vertices))
    hullTriangles = []
    for (simplex, equation) in zip(hull.simplices, hull.equations):
        (a, b, c) = points[simplex]
        hullTriangles += [simplex[[0, 1, 2]] if np.cross(b - a, c - a) @ equation[:3] > 0 else simplex[[0, 2, 1]]]
    return [points[hull.vertices], indices[np.array(hullTriangles)], ratio]


def _URDFjointDynamics(urdfFile):
    """{name of the child link: (damping, friction)} of the joints of a URDF file with an element dynamics"""
    import xml.etree.ElementTree as ElementTree
    result = {}
    for joint in ElementTree.parse(urdfFile).getroot().findall('joint'):
        dynamics = joint.find('dynamics')
        child = joint.find('child')
        if dynamics is not None and child is not None:
            result[child.get('link')] = (float(dynamics.get('damping', 0.)), float(dynamics.get('friction', 0.)))
    return result


def RobotFromURDFdata(robotData, gravity=[0, 0, -9.81], baseHT=None, toolHT=None, PDcontrol=(None, None), urdfFile=None,
                      enableEndStops=False, collisionMeshesAs='capsules'):
    """Create a Robot from the data of GetURDFrobotData (or of a dict of it loaded from a file): the links with their
    parents, masses, inertias, joint limits and graphics, the base graphics and the tool frame (#3356). A Robot is saved
    with its graphics by Robot.SaveToHDF5, and loaded without the packages that read the URDF.

    Args:
        robotData: the dict of GetURDFrobotData
        gravity: the gravity vector of the robot
        baseHT: None or the homogeneous transformation of the base
        toolHT: None or the frame of the tool relative to the last link; None: the 'toolHT' of robotData (argument
            toolLinkName of GetURDFrobotData), else the frame of the last link
        PDcontrol: (P, D), the same for all joints, or a list of one (P, D) per joint; (None, None): no control
        urdfFile: None or a URDF file (not xacro) from which the elements dynamics of the joints are read, as viscous
            friction (damping) and Coulomb friction (friction) of the links, since release 1.5 of the Robotics Toolbox does
            not read them; this may change with a later release
        enableEndStops: enableEndStops of Robot; the limits of the joints are stored in the links in any case
        collisionMeshesAs: how the collision meshes of robotData (argument collision of GetURDFrobotData) become collision shapes, see CollisionShapesFromGeometry

    Returns:
        the Robot; a joint without limits (upper not larger than lower) has jointLimits=None

    Example:
        data = GetURDFrobotData(LoadURDFrobot('urdf/robot.urdf', packagePath)['robot'], toolLinkName='tool0')
        robot = RobotFromURDFdata(data, PDcontrol=(1e4, 1e2))
        robot.SaveToHDF5('robot.h5')
    """
    from exudyn.robotics.roboticsCore import Robot, RobotBase, RobotTool, RobotLink, VRobotBase, VRobotTool, VRobotLink
    linkList = robotData['linkList']
    #one (P, D) for all joints is a tuple or list of two numbers or None
    perLink = len(PDcontrol) == len(linkList) and isinstance(PDcontrol[0], (list, tuple, np.ndarray))
    if not perLink and len(PDcontrol) != 2:
        raise ValueError('RobotFromURDFdata: PDcontrol must be (P, D) or a list of one (P, D) per joint, the robot has ' + str(len(linkList)) + ' joints')
    if toolHT is None:
        toolHT = robotData.get('toolHT', np.eye(4))
    baseArguments = {} if baseHT is None else {'HT': baseHT}
    robot = Robot(gravity=gravity, enableEndStops=enableEndStops,
                  base=RobotBase(visualization=VRobotBase(graphicsData=list(robotData.get('graphicsBaseList', []))),
                                 collisionShapes=CollisionShapesFromGeometry(robotData.get('collisionBase', []), collisionMeshesAs), **baseArguments),
                  tool=RobotTool(HT=toolHT, visualization=VRobotTool(graphicsData=list(robotData.get('graphicsToolList', [])))))
    dynamics = {} if urdfFile is None else _URDFjointDynamics(urdfFile)
    linkNumbers = {'None': -1}
    for (i, link) in enumerate(linkList):
        linkNumbers[link['name']] = i
        if link['parentName'] not in linkNumbers:
            raise ValueError('RobotFromURDFdata: link ' + link['name'] + ': the parent "' + link['parentName'] + '" is not a joint link before it')
        limits = link['jointLimits']
        if len(link['graphicsDataList']) != 0:
            visualization = VRobotLink(graphicsData=link['graphicsDataList'], showMBSjoint=False)
        else:
            visualization = VRobotLink(linkColor=graphics.colorList[i%16])
        (damping, friction) = dynamics.get(link['name'], (0., 0.))
        robot.AddLink(RobotLink(mass=link['mass'], COM=link['com'], inertia=link['inertiaCOM'], preHT=link['preHT'],
                                jointType=link['jointType'], parent=linkNumbers[link['parentName']],
                                PDcontrol=tuple(PDcontrol[i]) if perLink else tuple(PDcontrol),
                                jointLimits=list(limits) if limits[1] > limits[0] else None,
                                viscousFriction=damping, coulombFriction=friction, visualization=visualization,
                                collisionShapes=CollisionShapesFromGeometry(link.get('collision', []), collisionMeshesAs)))
    return robot


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
