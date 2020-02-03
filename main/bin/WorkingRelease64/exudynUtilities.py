# Utility functions and structures for Exudyn
"""
Created on Fri Jul 26 10:53:30 2019

@author: Johannes Gerstmayr

goal: support functions, which simplify the generation of models
"""
#constants and fixed structures:
import numpy as np #LoadSolutionFile
import time        #AnimateSolution
import copy as copy #to be able to copy e.g. lists

#pi = 3.1415926535897932 #define pi in order to avoid importing large libraries
#sqrt2 = 1.41421356237309505

pi = np.pi
sqrt2 = np.sqrt(2.)
g = 9.81 #gravity constant

eye2D = np.eye(2) #2x2 identity matrix
eye3D = np.eye(3) #3x3 identity matrix
eye4D = np.eye(4) #4x4 identity matrix

eulerParameters0 = [1.,0.,0.,0.] #Euler parameters for case where rotation angle is zero (rotation axis arbitrary)

#compute L2 norm for vectors without switching to numpy or math module
def NormL2(vector):
    value = 0
    for x in vector:
        value += x**2
    return value**0.5

#add two vectors instead using numpy
def VAdd(v0, v1):
    if len(v0) != len(v1): print("ERROR in VAdd: incompatible vectors!")
    n = len(v0)
    v = [0]*n
    for i in range(n):
        v[i] = v0[i]+v1[i]
    return v

#subtract two vectors instead using numpy: result = v0-v1
def VSub(v0, v1):
    if len(v0) != len(v1): print("ERROR in VSub: incompatible vectors!")
    n = len(v0)
    v = [0]*n
    for i in range(n):
        v[i] = v0[i]-v1[i]
    return v

#scalar multiplication of two vectors instead using numpy: result = v0'*v1
def VMult(v0, v1):
    if len(v0) != len(v1): print("ERROR in VMult: incompatible vectors!")
    r = 0
    for i in range(len(v0)):
        r += v0[i]*v1[i]
    return r

#multiplication vectors with scalar: result = s*v
def ScalarMult(scalar, v):
    res=[0]*len(v)
    for i in range(len(v)):
        res[i] += scalar*v[i]
    return res

# compute orthogonal basis vectors (normal1, normal2) for given vector0 (non-unique solution!); if vector0 == [0,0,0], then any normal basis is returned
def ComputeOrthonormalBasis(vector0):

    v = np.array(vector0)
    L0 = np.linalg.norm(v)
    if L0 == 0:
        n1 = np.array([1,0,0])
        n2 = np.array([0,1,0])
    else:
        v = (1. / L0)*v;
    
        if (abs(v[0]) > 0.5) and (abs(v[1]) < 0.1) and (abs(v[2]) < 0.1):
            n1 = np.array([0., 1., 0.])
        else:
            n1 = np.array([1., 0., 0.])
    
        h = np.dot(n1, v);
        n1 -= h * v;
        n1 = (1/np.linalg.norm(n1))*n1;
        n2 = np.cross(v,n1)
    #print("basis=", v,n1,n2)
    return [v, n1, n2]

#normalize a 3D vector (set length to 1)
def Normalize(vector):
    v=copy.deepcopy(vector) #copy, such that vector is not changed

    fact = NormL2(v)
    fact = 1./fact
    for i in range(len(v)): 
        v[i]*=fact
    return v
    
#apply tilde operator (skew) to R3-vector
def Vec2Tilde(v):
    return [[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]]

#convert skew symmetric matrix to vector
def Tilde2Vec(m):
    return [-m[1][2], m[0][2], -m[0][1]]

    
# graphics functions
color4steelblue = [0.4,0.4,0.9,1.]
color4lightred = [0.9,0.4,0.4,1.]
color4lightgreen = [0.3,0.9,0.3,1.]
color4darkgrey = [0.25,0.25,0.25,1.]
color4grey = [0.5,0.5,0.5,1.]
color4lightgrey = [0.75,0.75,0.75,1.]

#************************************************
#generate graphics dictionary data for rectangle
def GraphicsDataRectangle(xMin, yMin, xMax, yMax, color=[0.,0.,0.,1.]): 

    rect = [xMin, yMin,xMax,yMax]
    dataRect = {'type':'Line', 'color': color, 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]}

    return dataRect

#************************************************
#generate graphics dictionary data for rectangle
def GraphicsDataOrthoCubeLines(xMin, yMin, zMin, xMax, yMax, zMax, color=[0.,0.,0.,1.]): 

    dataRect = {'type':'Line', 'color': color, 'data':[xMin,yMin,zMin, xMin,yMax,zMin, xMin,yMin,zMin, xMax,yMin,zMin, xMax,yMax,zMin, xMax,yMin,zMin, 
                                                       xMax,yMin,zMax, xMax,yMax,zMax, xMax,yMin,zMax, xMin,yMin,zMax, xMin,yMax,zMax, xMin,yMin,zMax, 
                                                       xMin,yMin,zMin, xMin,yMax,zMin, xMax,yMax,zMin, xMax,yMax,zMax, xMin,yMax,zMax, xMin,yMax,zMin]}

    return dataRect

def GraphicsDataOrthoCube(xMin, yMin, zMin, xMax, yMax, zMax, color=[0.,0.,0.,1.]): 
#    colors=[]
#    for i in range(8):
#        colors=colors+color
#
#    points = [xMin,yMin,zMin, xMax,yMin,zMin, xMax,yMax,zMin, xMin,yMax,zMin,
#              xMin,yMin,zMax, xMax,yMin,zMax, xMax,yMax,zMax, xMin,yMax,zMax]
#
#    #1-based ... triangles = [1,3,2, 1,4,3, 5,6,7, 5,7,8, 1,2,5, 2,6,5, 2,3,6, 3,7,6, 3,4,7, 4,8,7, 4,1,8, 1,5,8 ]
#    #triangles = [0,2,1, 0,3,2, 6,4,5, 6,7,4, 0,1,4, 1,5,4, 1,2,5, 2,6,5, 2,3,6, 3,7,6, 3,0,7, 0,4,7]
#    triangles = [0,1,2, 0,2,3, 6,5,4, 6,4,7, 0,4,1, 1,4,5, 1,5,2, 2,5,6, 2,6,3, 3,6,7, 3,7,0, 0,7,4]
#    #data = {'type':'TriangleList', 'colors': colors, 'points':points, 'triangles':triangles}
#    data = {'type':'TriangleList', 'points':points, 'triangles':triangles}
#
#    return data
    
    pList = [[xMin,yMin,zMin], [xMax,yMin,zMin], [xMax,yMax,zMin], [xMin,yMax,zMin],
             [xMin,yMin,zMax], [xMax,yMin,zMax], [xMax,yMax,zMax], [xMin,yMax,zMax]]
    return GraphicsDataCube(pList, color)

#draw general cube with endpoints, according to given vertex definition
#pList is a list of points [[x0,y0,z0],[x1,y11,z1],...]
#faces includes the list with active faces (1); set index to zero, if face is not shown
def GraphicsDataCube(pList, color=[0.,0.,0.,1.], faces=[1,1,1,1,1,1]): 
# bottom: (z goes upwards from node 1 to node 5)
# ^y
# |
# 3---2
# |   |
# |   |
# 0---1-->x
#
# top:
# ^y
# |
# 7---6
# |   |
# |   |
# 4---5-->x
#
# faces: bottom, top, sideface0, sideface1, sideface2, sideface3 (sideface0 has nodes 0,1,4,5)

    colors=[]
    for i in range(8):
        colors=colors+color

    points = []
    for p in pList:
        points += p
#    points = [xMin,yMin,zMin, xMax,yMin,zMin, xMax,yMax,zMin, xMin,yMax,zMin,
#              xMin,yMin,zMax, xMax,yMin,zMax, xMax,yMax,zMax, xMin,yMax,zMax]

    #1-based ... triangles = [1,3,2, 1,4,3, 5,6,7, 5,7,8, 1,2,5, 2,6,5, 2,3,6, 3,7,6, 3,4,7, 4,8,7, 4,1,8, 1,5,8 ]
    #triangles = [0,2,1, 0,3,2, 6,4,5, 6,7,4, 0,1,4, 1,5,4, 1,2,5, 2,6,5, 2,3,6, 3,7,6, 3,0,7, 0,4,7]

#    triangles = [0,1,2, 0,2,3, 6,5,4, 6,4,7, 0,4,1, 1,4,5, 1,5,2, 2,5,6, 2,6,3, 3,6,7, 3,7,0, 0,7,4]
    trigList = [[0,1,2, 0,2,3], [6,5,4, 6,4,7], [0,4,1, 1,4,5], [1,5,2, 2,5,6], [2,6,3, 3,6,7], [3,7,0, 0,7,4]]
    triangles = []
    for i in range(6):
        if faces[i]:
            triangles += trigList[i]
    
    data = {'type':'TriangleList', 'colors': colors, 'points':points, 'triangles':triangles}

    return data

#switch order of three items in a list; mostly used for reverting normals in triangles
def SwitchTripletOrder(vector):
    v=copy.deepcopy(vector) #copy, such that vector is not changed
    a = v[2]
    v[2] = v[1]
    v[1] = a
    return v

#draw a sphere with point p and radius
def GraphicsDataSphere(point, radius, color=[0.,0.,0.,1.], nTiles = 8):
    if nTiles < 3: print("WARNING: GraphicsDataSphere: nTiles < 3: set nTiles=3")
    
    p = copy.deepcopy(point)
    r = radius
    #orthonormal basis:
    e0=np.array([1,0,0])
    e1=np.array([0,1,0])
    e2=np.array([0,0,1])
    
    points = []
    normals = []
    colors = []
    triangles = []
    
    #create points for circles around z-axis with tiling
    for i0 in range(nTiles+1):
        for iphi in range(nTiles):
            z = -r*np.cos(np.pi*i0/nTiles)    #runs from -r .. r (this is the coordinate of the axis of circles)
            phi = 2*np.pi*iphi/nTiles #angle
            
            fact = np.sin(np.pi*i0/nTiles)
            x = fact*r*np.sin(phi)
            y = fact*r*np.cos(phi)

            vv = x*e0 + y*e1 + z*e2
            points += list(p + vv)
            
            n = Normalize(list(-vv))
            normals += n
            
            colors += color

    
    for i0 in range(nTiles):
        for iphi in range(nTiles):
            p0 = i0*nTiles+iphi
            p1 = (i0+1)*nTiles+iphi
            iphi1 = iphi + 1
            if iphi1 >= nTiles: 
                iphi1 = 0
            p2 = i0*nTiles+iphi1
            p3 = (i0+1)*nTiles+iphi1

            triangles += [p0,p3,p1, p0,p2,p3]
            
    data = {'type':'TriangleList', 'colors':colors, 'normals':normals, 'points':points, 'triangles':triangles}
    return data
            
#draw a meshed cylinder with given axis, radius and color; nFaces gives the number of tiles (minimum=3)
#the range of angles is given in rad [0..2 * pi]
#lastFace: if angleRange != [0,2*pi], then the open cylinder is shown with lastFace = True
def GraphicsDataCylinder(pAxis, vAxis, radius, color=[0.,0.,0.,1.], nTiles = 16, 
                         angleRange=[0,2*np.pi], #generate just a part of cylinder, using range
                         lastFace = True, #add closing face (otherwise cylinder is open)
                         cutPlain = True):  #if true, a plain cut through cylinder is made; otherwise it is the cake shape ...

    if nTiles < 3: print("WARNING: GraphicsDataCylinder: nTiles < 3: set nTiles=3")
    
    #create points at left and right face
    points0=copy.deepcopy(pAxis) #[pAxis[0],pAxis[1],pAxis[2]] #avoid change of pAxis
    pAxis1=[pAxis[0]+vAxis[0],pAxis[1]+vAxis[1],pAxis[2]+vAxis[2]]
    points1=copy.deepcopy(pAxis1) #[pAxis[0]+vAxis[0],pAxis[1]+vAxis[1],pAxis[2]+vAxis[2]] #copy in order to avoid change of pAxis1 for use lateron
    vz = np.array(vAxis)
    p0 = np.array(pAxis)
    p1 = np.array(pAxis) + np.array(vAxis)
    
    basis = ComputeOrthonormalBasis(vAxis)
    #v0 = basis[0]
    n1 = basis[1]
    n2 = basis[2]
    r=radius
    
    #create normals at left and right face (pointing inwards)
    normals0 = Normalize(vAxis)
    normals1 = Normalize([-vAxis[0],-vAxis[1],-vAxis[2]])

    points2 = []
    points3 = []
    
    alpha = angleRange[1]-angleRange[0] #angular range
    alpha0 = angleRange[0]

    fact = nTiles #create correct part of cylinder
    if alpha < 2.*np.pi: 
        fact = nTiles-1
    
    for i in range(nTiles):
        phi = alpha0 + i*alpha/fact
        x = r*np.sin(phi)
        y = r*np.cos(phi)
        vv = x*n1 + y*n2
        pz0 = p0 + vv
        pz1 = p1 + vv
        points0 += list(pz0)
        points1 += list(pz1)
        points2 += list(pz0) #other points for side faces (different normals)
        points3 += list(pz1) #other points for side faces (different normals)
        n = Normalize(list(-vv))
        normals0 = normals0 + n
        normals1 = normals1 + n
    
    points0 += points1+points2+points3
    normals0 += normals1

    for i in range(nTiles):
        normals0 += Normalize(vAxis)
    for i in range(nTiles):
        normals0 += Normalize([-vAxis[0],-vAxis[1],-vAxis[2]])

    n = nTiles+1 #number of points of one ring+midpoint
    colors=[]
    for i in range(2*n+2*nTiles):
        colors += color

    triangles = []
    #circumference:
    for i in range(nTiles):
        if i != nTiles-1:
            triangles += [1+i,n+1+i+1,n+1+i]
            triangles += [1+i,1+i+1,n+1+i+1]
        else:
            if lastFace and cutPlain:
                triangles += [1+i,n+1,n+1+i]
                triangles += [1+i,1,n+1]
            

    #sides faces left and right:
    nn=2*n #offset
    for i in range(nTiles):
        if i != nTiles-1:
            triangles += [0,nn+i+1,nn+i]
            triangles += [n,nn+nTiles+i,nn+nTiles+i+1]
        else:
            if cutPlain:
                triangles += [0,nn,nn+i]
                triangles += [n,nn+nTiles+i,nn+nTiles]

    #if angles are not 2*pi, add closing face
    if lastFace and not(cutPlain):
        s = int(len(points0)/3) #starting index for side triangles
        p2 = points2[0:3]
        p3 = points3[0:3]
        p4 = points2[len(points2)-3:len(points2)]
        p5 = points3[len(points3)-3:len(points3)]
        points0 += pAxis + pAxis1 + p2 + p3 + pAxis + pAxis1 + p4 + p5
        n1=np.cross(VSub(pAxis,pAxis1),VSub(p3,pAxis))
        n1=list(Normalize(n1))
        n2=np.cross(VSub(pAxis1,pAxis),VSub(p4,pAxis))
        n2=list(Normalize(n2))
        normals0 += n1+n1+n1+n1+n2+n2+n2+n2  #8 additional normals
        triangles += [s+0,s+3,s+1, s+0,s+2,s+3, 
                      s+5,s+6,s+4, s+5,s+7,s+6]
        for i in range(8): #8 additional colors
            colors += color

    #triangle normals point inwards to object ...
    data = {'type':'TriangleList', 'colors':colors, 'normals':normals0, 'points':points0, 'triangles':triangles}

    return data

#create a planar Link, oriented in x-direction, 
#joint0 center position = p0
#joint1 center position = p1
#axis0 = rotation axis of p0, if drawn as a cylinder; [0,0,0] otherwise
#axis1 = rotation axis of p1, if drawn as a cylinder; [0,0,0] otherwise
#radius = [radius0, radius1] .. the two radii of the joints to draw by a cylinder or sphere    
#width = [width0, width1] .. the two width of the joints to draw by a cylinder; ignored for sphere    
#thickness ... the thickness of the link in z-direction or diameter (cylinder)
def GraphicsDataRigidLink(p0,p1,axis0, axis1, radius, thickness, width, color=[0.,0.,0.,1.], nTiles = 16):
    linkAxis = VSub(p1,p0)
    linkAxis0 = Normalize(linkAxis)
    a0=copy.deepcopy(axis0)
    a1=copy.deepcopy(axis1)
    a0=Normalize(a0)
    a1=Normalize(a1)
    
    data0 = GraphicsDataCylinder(p0, linkAxis, 0.5*thickness, color, nTiles)
    data1 = {}
    data2 = {}

    if NormL2(axis0) == 0:
        data1 = GraphicsDataSphere(p0, radius[0], color, nTiles)
    else:
        data1 = GraphicsDataCylinder(list(np.array(p0)-0.5*width[0]*np.array(a0)), 
                                     list(width[0]*np.array(a0)), 
                                     radius[0], color, nTiles)
        
    if NormL2(axis1) == 0:
        data2 = GraphicsDataSphere(p1, radius[1], color, nTiles)
    else:
        data2 = GraphicsDataCylinder(list(np.array(p1)-0.5*width[1]*np.array(a1)), 
                                     list(width[1]*np.array(a1)), radius[1], color, nTiles)

    #now merge lists, including appropriate indices of triangle points!
    np0 = int(len(data0['points'])/3) #number of points of first point list ==> this is the offset for next list
    np1 = np0 + int(len(data1['points'])/3) #number of points of first point list ==> this is the offset for next list

    triangles = data0['triangles']
    trigs1 = np.array(data1['triangles'])
    trigs1 += np0
    triangles += list(trigs1)
    
    trigs2 = np.array(data2['triangles'])
    trigs2 += np1
    triangles += list(trigs2)
    
    points = data0['points'] + data1['points'] + data2['points']
    normals = data0['normals'] + data1['normals'] + data2['normals']
    colors = data0['colors'] + data1['colors'] + data2['colors']
    
    data = {'type':'TriangleList', 'colors':colors, 'normals':normals, 'points':points, 'triangles':triangles}
    return data

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++   LOAD SOLUTION AND ANIMATION   ++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#++++++++++++++++++++++++++++++++++++++++++++
#read Solution:
def LoadSolutionFile(fileName):
    data = np.loadtxt(fileName, comments='#', delimiter=',')

    fileRead=open(fileName,'r') 
    fileLines = fileRead.readlines()
    fileRead.close()
    leftStr=fileLines[4].split('=')[0]
    if leftStr[0:30] != '#number of written coordinates': 
        print('ERROR in LoadSolution: file header corrupted')

    columnsExported = eval(fileLines[4].split('=')[1]) #load according column information into vector: [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData]
    print('columns imported =', columnsExported)
    nColumns = sum(columnsExported)
    print('total columns =', nColumns, ', array size =', np.size(data,1))

    if (nColumns + 1) != np.size(data,1): #one additional column for time!
        print('ERROR in LoadSolution: number of columns is inconsistent')

    nRows = np.size(data,0)

    return dict({'data': data, 'columnsExported': columnsExported,'nColumns': nColumns,'nRows': nRows})
    
#++++++++++++++++++++++++++++++++++++++++++++
#load selected row of solution into specific state
def SetSolutionState(exu, mbs, solution, row, configuration):
    if row < solution['nRows']:
        rowData = solution['data'][row]
        #cols = solution['columnsExported']
        [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = solution['columnsExported']

        #note that these visualization updates are not threading safe!
        mbs.systemData.SetODE2Coordinates(rowData[1:1+nODE2], configuration)
        if (nVel2): mbs.systemData.SetODE2Coordinates_t(rowData[1+nODE2:1+nODE2+nVel2], configuration)
        if (nAlgebraic): mbs.systemData.SetAECoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic], configuration)
        if (nData): mbs.systemData.SetDataCoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic+nData], configuration)

        if configuration == exu.ConfigurationType.Visualization:
            mbs.systemData.SetVisualizationTime(rowData[0])
            mbs.SendRedrawSignal()
    else:
        print("ERROR in SetVisualizationState: invalid row (out of range)")

#++++++++++++++++++++++++++++++++++++++++++++
#load selected row of solution into visualization state
def SetVisualizationState(exu, mbs, solution, row):
    SetSolutionState(exu, mbs, solution, row, exu.ConfigurationType.Visualization)

#++++++++++++++++++++++++++++++++++++++++++++
#consecutively load the rows of a solution file and visualize the result
#timeout (in seconds) is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
#rowIncrement can be set larger than one in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
def AnimateSolution(exu, SC, mbs, solution, rowIncrement = 1, timeout=0.04, createImages = False):
    nRows = solution['nRows']
    if (rowIncrement < 1) | (rowIncrement > nRows):
        print('ERROR in AnimateSolution: rowIncrement must be at least 1 and must not be larger than the number of rows in the solution file')
    oldUpdateInterval = SC.visualizationSettings.general.graphicsUpdateInterval
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.5*min(timeout, 2e-3) #avoid too small values to run multithreading properly

    for i in range(0,nRows,rowIncrement):
        if not(mbs.GetRenderEngineStopFlag()):
            SetVisualizationState(exu, mbs, solution, i)
            if createImages:
                SC.RedrawAndSaveImage() #create images for animation
            time.sleep(timeout)

    SC.visualizationSettings.general.graphicsUpdateInterval = oldUpdateInterval #set values back to original




#++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++
#helper functions for RIGID BODY KINEMATICS:

#convert Euler parameters (ep) to G-matrix (=\partial(omega) / \partial(ep_t))
def EulerParameters2G(eulerParameters):
    ep = eulerParameters
    return np.array([[-2.*ep[1], 2.*ep[0],-2.*ep[3], 2.*ep[2]],
                     [-2.*ep[2], 2.*ep[3], 2.*ep[0],-2.*ep[1]],
                     [-2.*ep[3],-2.*ep[2], 2.*ep[1], 2.*ep[0]] ])

#convert Euler parameters (ep) to local G-matrix (=\partial(omegaLocal) / \partial(ep_t))
def EulerParameters2GLocal(eulerParameters):
    ep = eulerParameters
    return np.array([[-2.*ep[1], 2.*ep[0], 2.*ep[3],-2.*ep[2]],
                     [-2.*ep[2],-2.*ep[3], 2.*ep[0], 2.*ep[1]],
                     [-2.*ep[3], 2.*ep[2],-2.*ep[1], 2.*ep[0]] ])

#compute rotation matrix from eulerParameters    
def EulerParameters2RotationMatrix(eulerParameters):
    ep = eulerParameters
    return np.array([[-2.0*ep[3]*ep[3] - 2.0*ep[2]*ep[2] + 1.0, -2.0*ep[3]*ep[0] + 2.0*ep[2]*ep[1], 2.0*ep[3]*ep[1] + 2.0*ep[2]*ep[0]],
                     [ 2.0*ep[3]*ep[0] + 2.0*ep[2]*ep[1], -2.0*ep[3]*ep[3] - 2.0*ep[1]*ep[1] + 1.0, 2.0*ep[3]*ep[2] - 2.0*ep[1]*ep[0]],
                     [-2.0*ep[2]*ep[0] + 2.0*ep[3]*ep[1], 2.0*ep[3]*ep[2] + 2.0*ep[1]*ep[0], -2.0*ep[2]*ep[2] - 2.0*ep[1]*ep[1] + 1.0] ])

#compute Euler parameters from given rotation matrix
def RotationMatrix2EulerParameters(rotationMatrix):
    A=rotationMatrix
    trace = A[0][0] + A[1][1] + A[2][2] + 1.0
    M_EPSILON = 1e-15 #small number to avoid division by zero

    if (abs(trace) > M_EPSILON):
        s = 0.5 / np.sqrt(abs(trace))
        ep0 = 0.25 / s
        ep1 = (A[2][1] - A[1][2]) * s
        ep2 = (A[0][2] - A[2][0]) * s
        ep3 = (A[1][0] - A[0][1]) * s
    else:
        if (A[0][0] > A[1][1]) and (A[0][0] > A[2][2]):
            s = 2.0 * np.sqrt(abs(1.0 + A[0][0] - A[1][1] - A[2][2]))
            ep1 = 0.25 * s
            ep2 = (A[0][1] + A[1][0]) / s
            ep3 = (A[0][2] + A[2][0]) / s
            ep0 = (A[1][2] - A[2][1]) / s
        elif A[1][1] > A[2][2]:
            s = 2.0 * np.sqrt(abs(1.0 + A[1][1] - A[0][0] - A[2][2]))
            ep1 = (A[0][1] + A[1][0]) / s
            ep2 = 0.25 * s
            ep3 = (A[1][2] + A[2][1]) / s
            ep0 = (A[0][2] - A[2][0]) / s
        else:
            s = 2.0 * np.sqrt(abs(1.0 + A[2][2] - A[0][0] - A[1][1]));
            ep1 = (A[0][2] + A[2][0]) / s
            ep2 = (A[1][2] + A[2][1]) / s
            ep3 = 0.25 * s
            ep0 = (A[0][1] - A[1][0]) / s

    return np.array([ep0,ep1,ep2,ep3])

#compute time derivative of Euler parameters from (global) angular velocity vector
#note that omega=G*ep_t ==> G^T*omega = G^T*G*ep_t ==> G^T*G=4*(I4 - ep*ep^T)*ep_t = 4*(I4)*ep_t
def AngularVelocity2EulerParameters_t(angularVelocity, eulerParameters):
    
    GT = np.transpose(EulerParameters2G(eulerParameters))
    return 0.25*(GT.dot(angularVelocity))
    
#compute rotation matrix w.r.t. X-axis (first axis)
def RotationMatrixX(angleRad):
    return np.array([[1, 0, 0],
                     [0, np.cos(angleRad),-np.sin(angleRad)],
                     [0, np.sin(angleRad), np.cos(angleRad)] ])

#compute rotation matrix w.r.t. Y-axis (second axis)
def RotationMatrixY(angleRad):
    return np.array([ [ np.cos(angleRad), 0, np.sin(angleRad)],
                      [0,        1, 0],
                      [-np.sin(angleRad),0, np.cos(angleRad)] ])

#compute rotation matrix w.r.t. Z-axis (third axis)
def RotationMatrixZ(angleRad):
    return np.array([ [np.cos(angleRad),-np.sin(angleRad), 0],
                      [np.sin(angleRad), np.cos(angleRad), 0],
                      [0,	    0,        1] ]);

    
#def AngularVelocity2EulerParameters_t(angularVelocity):


    
from itemInterface import *    
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#inputs:
# mbs: exudyn.MainSystem()
# positionOfNode0: 
#
# cableTemplate: e.g. ObjectANCFCable2D object with according properties; physicsLength and nodeNumbers will be filled automatically
#output:
#[...]
def GenerateStraightLineANCFCable2D(mbs, positionOfNode0, positionOfNode1, numberOfElements, cableTemplate,
                                massProportionalLoad=[0,0,0], fixedConstraintsNode0=[0,0,0,0], fixedConstraintsNode1=[0,0,0,0],
                                vALE=0, ConstrainAleCoordinate=True):
    
    cableNodeList=[]
    cableNodePositionList=[positionOfNode0]
    cableObjectList=[]
    loadList=[]

    # length of one element, calculated from first and last node position:
    cableLength = NormL2(VSub(positionOfNode1, positionOfNode0))/numberOfElements
    
    # slope of elements in reference position, calculated from first and last node position:
    cableSlopeVec = Normalize(VSub(positionOfNode1, positionOfNode0))
   
    # add first ANCF node (straight reference configuration):
    nCable0 = mbs.AddNode(Point2DS1(referenceCoordinates=[positionOfNode0[0],positionOfNode0[1],cableSlopeVec[0],cableSlopeVec[1]])) 
    cableNodeList+=[nCable0]
    
    
    cableTemplate.physicsLength = cableLength
    
#    #ALE node: 
#    if vALE !=0:
#        nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], initialCoordinates=[0], initialCoordinates_t=[vALE]))
#        mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nALE, coordinate=0)) #ALE velocity  marker

    # add all other ANCF nodes (straight reference configuration) and attach Gravity marker to them:
    for i in range(numberOfElements): 
        
        positionOfCurrentNode=[positionOfNode0[0]+cableLength*cableSlopeVec[0]*(i+1),positionOfNode0[1]+cableLength*cableSlopeVec[1]*(i+1)]
        cableNodePositionList+=[positionOfCurrentNode]
        
        nCableLast = mbs.AddNode(Point2DS1(referenceCoordinates=[positionOfCurrentNode[0],positionOfCurrentNode[1],cableSlopeVec[0],cableSlopeVec[1]]))
        cableNodeList+=[nCableLast]
        

#        if vALE ==0:
#            cableTemplate.nodeNumbers=[cableNodeList[i],cableNodeList[i+1]]
#        else:
        cableTemplate.nodeNumbers[0:2]=[cableNodeList[i],cableNodeList[i+1]]
            
        oCable=mbs.AddObject(cableTemplate)
        cableObjectList+=[oCable]

        if NormL2(massProportionalLoad) != 0:
            mBodyMassLast = mbs.AddMarker(MarkerBodyMass(bodyNumber=oCable))
            lLoadLast=mbs.AddLoad(Gravity(markerNumber=mBodyMassLast,loadVector=massProportionalLoad))
            loadList+=[lLoadLast]
        
    
    if (NormL2(fixedConstraintsNode0+fixedConstraintsNode1)) != 0:
        # ground "node" at 0,0,0:
        nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) 
        # add marker to ground "node": 
        mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0))
    

        for j in range(4):            
            if fixedConstraintsNode0[j] != 0:            
                #fix ANCF coordinates of first node
                mCableCoordinateConstraint0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nCable0, coordinate=j)) #add marker
                cBoundaryCondition=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mCableCoordinateConstraint0])) #add constraint
            
            if fixedConstraintsNode1[j] != 0:                 
                # fix right end position coordinates, i.e., add markers and constraints:
                mCableCoordinateConstraint1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nCableLast, coordinate=j))#add marker
                cBoundaryCondition=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mCableCoordinateConstraint1])) #add constraint  
        
        
#        if vALE !=0 and ConstrainAleCoordinate:
#            cConstrainAle=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mALE]))
            
    
    return [cableNodeList, cableObjectList, loadList, cableNodePositionList]



def GenerateSlidingJoint(mbs,cableObjectList,markerBodyPositionOfSlidingBody,localMarkerIndexOfStartCable=0,slidingCoordinateStartPosition=0):

    cableMarkerList = []#list of Cable2DCoordinates markers
    offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
    offset = 0          #first cable element has offset 0
    
    for item in cableObjectList: #create markers for cable elements
        m = mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber = item))
        cableMarkerList += [m]
        offsetList += [offset]  
        offset += mbs.GetObjectParameter(item,'physicsLength')
    
    nodeDataSlidingJoint = mbs.AddNode(NodeGenericData(initialCoordinates=[localMarkerIndexOfStartCable,slidingCoordinateStartPosition],numberOfDataCoordinates=2)) #initial index in cable list
    
    oSlidingJoint = mbs.AddObject(ObjectJointSliding2D(markerNumbers=[markerBodyPositionOfSlidingBody,cableMarkerList[localMarkerIndexOfStartCable]], 
                                                      slidingMarkerNumbers=cableMarkerList, 
                                                      slidingMarkerOffsets=offsetList, 
                                                      nodeNumber=nodeDataSlidingJoint))

    return [oSlidingJoint]




def GenerateAleSlidingJoint(mbs,cableObjectList,markerBodyPositionOfSlidingBody,AleNode,localMarkerIndexOfStartCable=0,AleSlidingOffset=0):

    cableMarkerList = []#list of Cable2DCoordinates markers
    offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
    offset = 0          #first cable element has offset 0
    
    for item in cableObjectList: #create markers for cable elements
        m = mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber = item))
        cableMarkerList += [m]
        offsetList += [offset]  
        offset += mbs.GetObjectParameter(item,'physicsLength')
    
    nodeDataAleSlidingJoint = mbs.AddNode(NodeGenericData(initialCoordinates=[localMarkerIndexOfStartCable],numberOfDataCoordinates=1)) #initial index in cable list   
    oAleSlidingJoint = mbs.AddObject(ObjectJointALEMoving2D(markerNumbers=[markerBodyPositionOfSlidingBody,cableMarkerList[localMarkerIndexOfStartCable]], 
                                                      slidingMarkerNumbers=cableMarkerList, slidingMarkerOffsets=offsetList,
                                                      nodeNumbers=[nodeDataAleSlidingJoint, AleNode], slidingOffset=AleSlidingOffset))


    return [oAleSlidingJoint]
