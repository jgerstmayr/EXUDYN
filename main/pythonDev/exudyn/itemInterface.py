#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#automatically generated file for conversion of item (node, object, marker, ...) data to dictionaries
#author: Johannes Gerstmayr
#created: 2019-07-01
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#item interface diagonal matrix creator

import exudyn #for exudyn.InvalidIndex() and other exudyn native structures needed in RigidBodySpringDamper

#helper function diagonal matrices, not needing numpy
def IIDiagMatrix(rowsColumns, value):
    m = []
    for i in range(rowsColumns):
        m += [rowsColumns*[0]]
        m[i][i] = value
    return m

#helper function to check valid range
def CheckForValidUInt(value, parameterName, objectName):
    if value < 0:
        raise ValueError("Error in "+objectName+": (int) parameter "+parameterName + " may not be negative, but received "+str(value))
        return 0
    return value
#helper function to check valid range
def CheckForValidPInt(value, parameterName, objectName):
    if value <= 0:
        raise ValueError("Error in "+objectName+": (int) parameter "+parameterName + " must be positive (> 0), but received "+str(value))
        return 1 #this position is usually not reached
    return value
#helper function to check valid range
def CheckForValidUReal(value, parameterName, objectName):
    if value < 0:
        raise ValueError("Error in "+objectName+": (float) parameter "+parameterName + " may not be negative, but received "+str(value))
        return 0.
    return value
#helper function to check valid range
def CheckForValidPReal(value, parameterName, objectName):
    if value <= 0:
        raise ValueError("Error in "+objectName+": (float) parameter "+parameterName + " must be positive (> 0), but received "+str(value))
        return 1. #this position is usually not reached
    return value


#+++++++++++++++++++++++++++++++
#NODE
class VNodePoint:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodePoint:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.], initialCoordinates = [0.,0.,0.], initialVelocities = [0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'Point'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Point = NodePoint
VPoint = VNodePoint

class VNodePoint2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodePoint2D:
    def __init__(self, name = '', referenceCoordinates = [0.,0.], initialCoordinates = [0.,0.], initialVelocities = [0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'Point2D'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Point2D = NodePoint2D
VPoint2D = VNodePoint2D

class VNodeRigidBodyEP:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodeRigidBodyEP:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.,0.], initialCoordinates = [0.,0.,0., 0.,0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.,0.], addConstraintEquation = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.addConstraintEquation = addConstraintEquation
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBodyEP'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'addConstraintEquation', self.addConstraintEquation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidEP = NodeRigidBodyEP
VRigidEP = VNodeRigidBodyEP

class VNodeRigidBodyRxyz:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodeRigidBodyRxyz:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.], initialCoordinates = [0.,0.,0., 0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBodyRxyz'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidRxyz = NodeRigidBodyRxyz
VRigidRxyz = VNodeRigidBodyRxyz

class VNodeRigidBodyRotVecLG:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodeRigidBodyRotVecLG:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.], initialCoordinates = [0.,0.,0., 0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBodyRotVecLG'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidRotVecLG = NodeRigidBodyRotVecLG
VRigidRotVecLG = VNodeRigidBodyRotVecLG

class VNodeRigidBodyRotVecDataLG:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodeRigidBodyRotVecDataLG:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.], initialCoordinates = [0.,0.,0., 0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBodyRotVecDataLG'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidRotVecLG = NodeRigidBodyRotVecDataLG
VRigidRotVecLG = VNodeRigidBodyRotVecDataLG

class VNodeRigidBody2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodeRigidBody2D:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.], initialCoordinates = [0.,0.,0.], initialVelocities = [0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBody2D'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Rigid2D = NodeRigidBody2D
VRigid2D = VNodeRigidBody2D

class VNode1D:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class Node1D:
    def __init__(self, name = '', referenceCoordinates = [0.], initialCoordinates = [0.], initialVelocities = [0.], visualization = {'show': False}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', '1D'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodePoint2DSlope1:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodePoint2DSlope1:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,1.,0.], initialCoordinates = [0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'Point2DSlope1'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Point2DS1 = NodePoint2DSlope1
VPoint2DS1 = VNodePoint2DSlope1

class VNodePoint3DSlope1:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodePoint3DSlope1:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.,1.,0.,0.], initialCoordinates = [0.,0.,0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'Point3DSlope1'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Point3DS1 = NodePoint3DSlope1
VPoint3DS1 = VNodePoint3DSlope1

class VNodePoint3DSlope23:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodePoint3DSlope23:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.,1.,0.,0.,1.,0.,0.], initialCoordinates = [0.,0.,0.,0.,0.,0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.,0.,0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'Point3DSlope23'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Point3DS23 = NodePoint3DSlope23
VPoint3DS23 = VNodePoint3DSlope23

class VNodeGenericODE2:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class NodeGenericODE2:
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], initialCoordinates_t = [], numberOfODE2Coordinates = 0, visualization = {'show': False}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialCoordinates_t = initialCoordinates_t
        self.numberOfODE2Coordinates = CheckForValidPInt(numberOfODE2Coordinates,"numberOfODE2Coordinates","NodeGenericODE2")
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'GenericODE2'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialCoordinates_t', self.initialCoordinates_t
        yield 'numberOfODE2Coordinates', self.numberOfODE2Coordinates
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodeGenericODE1:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class NodeGenericODE1:
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], numberOfODE1Coordinates = 0, visualization = {'show': False}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.numberOfODE1Coordinates = CheckForValidPInt(numberOfODE1Coordinates,"numberOfODE1Coordinates","NodeGenericODE1")
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'GenericODE1'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfODE1Coordinates', self.numberOfODE1Coordinates
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodeGenericAE:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class NodeGenericAE:
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], numberOfAECoordinates = 0, visualization = {'show': False}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.numberOfAECoordinates = CheckForValidPInt(numberOfAECoordinates,"numberOfAECoordinates","NodeGenericAE")
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'GenericAE'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfAECoordinates', self.numberOfAECoordinates
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodeGenericData:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class NodeGenericData:
    def __init__(self, name = '', initialCoordinates = [], numberOfDataCoordinates = 0, visualization = {'show': False}):
        self.name = name
        self.initialCoordinates = initialCoordinates
        self.numberOfDataCoordinates = CheckForValidUInt(numberOfDataCoordinates,"numberOfDataCoordinates","NodeGenericData")
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'GenericData'
        yield 'name', self.name
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfDataCoordinates', self.numberOfDataCoordinates
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VNodePointGround:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class NodePointGround:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'PointGround'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
PointGround = NodePointGround
VPointGround = VNodePointGround

#+++++++++++++++++++++++++++++++
#OBJECT
class VObjectGround:
    def __init__(self, show = True, graphicsDataUserFunction = 0, color = [-1.,-1.,-1.,-1.], graphicsData = []):
        self.show = show
        self.graphicsDataUserFunction = graphicsDataUserFunction
        self.color = color
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsDataUserFunction', self.graphicsDataUserFunction
        yield 'color', self.color
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectGround:
    def __init__(self, name = '', referencePosition = [0.,0.,0.], visualization = {'show': True, 'graphicsDataUserFunction': 0, 'color': [-1.,-1.,-1.,-1.], 'graphicsData': []}):
        self.name = name
        self.referencePosition = referencePosition
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'Ground'
        yield 'name', self.name
        yield 'referencePosition', self.referencePosition
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
class VObjectMassPoint:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectMassPoint:
    def __init__(self, name = '', physicsMass = 0., nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectMassPoint")
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'MassPoint'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
MassPoint = ObjectMassPoint
VMassPoint = VObjectMassPoint

class VObjectMassPoint2D:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectMassPoint2D:
    def __init__(self, name = '', physicsMass = 0., nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectMassPoint2D")
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'MassPoint2D'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
MassPoint2D = ObjectMassPoint2D
VMassPoint2D = VObjectMassPoint2D

class VObjectMass1D:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectMass1D:
    def __init__(self, name = '', physicsMass = 0., nodeNumber = exudyn.InvalidIndex(), referencePosition = [0.,0.,0.], referenceRotation = IIDiagMatrix(rowsColumns=3,value=1), visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectMass1D")
        self.nodeNumber = nodeNumber
        self.referencePosition = referencePosition
        self.referenceRotation = referenceRotation
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'Mass1D'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'nodeNumber', self.nodeNumber
        yield 'referencePosition', self.referencePosition
        yield 'referenceRotation', self.referenceRotation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Mass1D = ObjectMass1D
VMass1D = VObjectMass1D

class VObjectRotationalMass1D:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectRotationalMass1D:
    def __init__(self, name = '', physicsInertia = 0., nodeNumber = exudyn.InvalidIndex(), referencePosition = [0.,0.,0.], referenceRotation = IIDiagMatrix(rowsColumns=3,value=1), visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsInertia = CheckForValidUReal(physicsInertia,"physicsInertia","ObjectRotationalMass1D")
        self.nodeNumber = nodeNumber
        self.referencePosition = referencePosition
        self.referenceRotation = referenceRotation
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'RotationalMass1D'
        yield 'name', self.name
        yield 'physicsInertia', self.physicsInertia
        yield 'nodeNumber', self.nodeNumber
        yield 'referencePosition', self.referencePosition
        yield 'referenceRotation', self.referenceRotation
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Rotor1D = ObjectRotationalMass1D
VRotor1D = VObjectRotationalMass1D

class VObjectRigidBody:
    def __init__(self, show = True, graphicsDataUserFunction = 0, graphicsData = []):
        self.show = show
        self.graphicsDataUserFunction = graphicsDataUserFunction
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsDataUserFunction', self.graphicsDataUserFunction
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectRigidBody:
    def __init__(self, name = '', physicsMass = 0., physicsInertia = [0.,0.,0., 0.,0.,0.], physicsCenterOfMass = [0.,0.,0.], nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectRigidBody")
        self.physicsInertia = physicsInertia
        self.physicsCenterOfMass = physicsCenterOfMass
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'RigidBody'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'physicsInertia', self.physicsInertia
        yield 'physicsCenterOfMass', self.physicsCenterOfMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidBody = ObjectRigidBody
VRigidBody = VObjectRigidBody

class VObjectRigidBody2D:
    def __init__(self, show = True, graphicsDataUserFunction = 0, graphicsData = []):
        self.show = show
        self.graphicsDataUserFunction = graphicsDataUserFunction
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsDataUserFunction', self.graphicsDataUserFunction
        yield 'graphicsData', self.graphicsData

    def __repr__(self):
        return str(dict(self))
class ObjectRigidBody2D:
    def __init__(self, name = '', physicsMass = 0., physicsInertia = 0., nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True, 'graphicsDataUserFunction': 0, 'graphicsData': []}):
        self.name = name
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectRigidBody2D")
        self.physicsInertia = CheckForValidUReal(physicsInertia,"physicsInertia","ObjectRigidBody2D")
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'RigidBody2D'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'physicsInertia', self.physicsInertia
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidBody2D = ObjectRigidBody2D
VRigidBody2D = VObjectRigidBody2D

class VObjectGenericODE2:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.], triangleMesh = [], showNodes = False, graphicsDataUserFunction = 0):
        self.show = show
        self.color = color
        self.triangleMesh = triangleMesh
        self.showNodes = showNodes
        self.graphicsDataUserFunction = graphicsDataUserFunction

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color
        yield 'triangleMesh', self.triangleMesh
        yield 'showNodes', self.showNodes
        yield 'graphicsDataUserFunction', self.graphicsDataUserFunction

    def __repr__(self):
        return str(dict(self))
class ObjectGenericODE2:
    def __init__(self, name = '', nodeNumbers = [], massMatrix = [], stiffnessMatrix = [], dampingMatrix = [], forceVector = [], forceUserFunction = 0, massMatrixUserFunction = 0, jacobianUserFunction = 0, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False, 'graphicsDataUserFunction': 0}):
        self.name = name
        self.nodeNumbers = nodeNumbers
        self.massMatrix = massMatrix
        self.stiffnessMatrix = stiffnessMatrix
        self.dampingMatrix = dampingMatrix
        self.forceVector = forceVector
        self.forceUserFunction = forceUserFunction
        self.massMatrixUserFunction = massMatrixUserFunction
        self.jacobianUserFunction = jacobianUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'GenericODE2'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'massMatrix', self.massMatrix
        yield 'stiffnessMatrix', self.stiffnessMatrix
        yield 'dampingMatrix', self.dampingMatrix
        yield 'forceVector', self.forceVector
        yield 'forceUserFunction', self.forceUserFunction
        yield 'massMatrixUserFunction', self.massMatrixUserFunction
        yield 'jacobianUserFunction', self.jacobianUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VtriangleMesh', dict(self.visualization)["triangleMesh"]
        yield 'VshowNodes', dict(self.visualization)["showNodes"]
        yield 'VgraphicsDataUserFunction', dict(self.visualization)["graphicsDataUserFunction"]

    def __repr__(self):
        return str(dict(self))
class VObjectGenericODE1:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class ObjectGenericODE1:
    def __init__(self, name = '', nodeNumbers = [], systemMatrix = [], rhsVector = [], rhsUserFunction = 0, visualization = {'show': True}):
        self.name = name
        self.nodeNumbers = nodeNumbers
        self.systemMatrix = systemMatrix
        self.rhsVector = rhsVector
        self.rhsUserFunction = rhsUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'GenericODE1'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'systemMatrix', self.systemMatrix
        yield 'rhsVector', self.rhsVector
        yield 'rhsUserFunction', self.rhsUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VObjectKinematicTree:
    def __init__(self, show = True, showLinks = True, showJoints = True, color = [-1.,-1.,-1.,-1.], graphicsDataList = []):
        self.show = show
        self.showLinks = showLinks
        self.showJoints = showJoints
        self.color = color
        self.graphicsDataList = graphicsDataList

    def __iter__(self):
        yield 'show', self.show
        yield 'showLinks', self.showLinks
        yield 'showJoints', self.showJoints
        yield 'color', self.color
        yield 'graphicsDataList', self.graphicsDataList

    def __repr__(self):
        return str(dict(self))
class ObjectKinematicTree:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), gravity = [0.,0.,0.], baseOffset = [0.,0.,0.], jointTypes = [], linkParents = [], jointTransformations = [], jointOffsets = [], linkInertiasCOM = [], linkCOMs = [], linkMasses = [], linkForces = [], linkTorques = [], jointForceVector = [], jointPositionOffsetVector = [], jointVelocityOffsetVector = [], jointPControlVector = [], jointDControlVector = [], forceUserFunction = 0, visualization = {'show': True, 'showLinks': True, 'showJoints': True, 'color': [-1.,-1.,-1.,-1.], 'graphicsDataList': []}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.gravity = gravity
        self.baseOffset = baseOffset
        self.jointTypes = jointTypes
        self.linkParents = linkParents
        self.jointTransformations = jointTransformations
        self.jointOffsets = jointOffsets
        self.linkInertiasCOM = linkInertiasCOM
        self.linkCOMs = linkCOMs
        self.linkMasses = linkMasses
        self.linkForces = linkForces
        self.linkTorques = linkTorques
        self.jointForceVector = jointForceVector
        self.jointPositionOffsetVector = jointPositionOffsetVector
        self.jointVelocityOffsetVector = jointVelocityOffsetVector
        self.jointPControlVector = jointPControlVector
        self.jointDControlVector = jointDControlVector
        self.forceUserFunction = forceUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'KinematicTree'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'gravity', self.gravity
        yield 'baseOffset', self.baseOffset
        yield 'jointTypes', self.jointTypes
        yield 'linkParents', self.linkParents
        yield 'jointTransformations', self.jointTransformations
        yield 'jointOffsets', self.jointOffsets
        yield 'linkInertiasCOM', self.linkInertiasCOM
        yield 'linkCOMs', self.linkCOMs
        yield 'linkMasses', self.linkMasses
        yield 'linkForces', self.linkForces
        yield 'linkTorques', self.linkTorques
        yield 'jointForceVector', self.jointForceVector
        yield 'jointPositionOffsetVector', self.jointPositionOffsetVector
        yield 'jointVelocityOffsetVector', self.jointVelocityOffsetVector
        yield 'jointPControlVector', self.jointPControlVector
        yield 'jointDControlVector', self.jointDControlVector
        yield 'forceUserFunction', self.forceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowLinks', dict(self.visualization)["showLinks"]
        yield 'VshowJoints', dict(self.visualization)["showJoints"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VgraphicsDataList', dict(self.visualization)["graphicsDataList"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
KinematicTree = ObjectKinematicTree
VKinematicTree = VObjectKinematicTree

class VObjectFFRF:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.], triangleMesh = [], showNodes = False):
        self.show = show
        self.color = color
        self.triangleMesh = triangleMesh
        self.showNodes = showNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color
        yield 'triangleMesh', self.triangleMesh
        yield 'showNodes', self.showNodes

    def __repr__(self):
        return str(dict(self))
class ObjectFFRF:
    def __init__(self, name = '', nodeNumbers = [], massMatrixFF = [], stiffnessMatrixFF = [], dampingMatrixFF = [], forceVector = [], forceUserFunction = 0, massMatrixUserFunction = 0, computeFFRFterms = True, objectIsInitialized = False, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False}):
        self.name = name
        self.nodeNumbers = nodeNumbers
        self.massMatrixFF = massMatrixFF
        self.stiffnessMatrixFF = stiffnessMatrixFF
        self.dampingMatrixFF = dampingMatrixFF
        self.forceVector = forceVector
        self.forceUserFunction = forceUserFunction
        self.massMatrixUserFunction = massMatrixUserFunction
        self.computeFFRFterms = computeFFRFterms
        self.objectIsInitialized = objectIsInitialized
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'FFRF'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'massMatrixFF', self.massMatrixFF
        yield 'stiffnessMatrixFF', self.stiffnessMatrixFF
        yield 'dampingMatrixFF', self.dampingMatrixFF
        yield 'forceVector', self.forceVector
        yield 'forceUserFunction', self.forceUserFunction
        yield 'massMatrixUserFunction', self.massMatrixUserFunction
        yield 'computeFFRFterms', self.computeFFRFterms
        yield 'objectIsInitialized', self.objectIsInitialized
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VtriangleMesh', dict(self.visualization)["triangleMesh"]
        yield 'VshowNodes', dict(self.visualization)["showNodes"]

    def __repr__(self):
        return str(dict(self))
class VObjectFFRFreducedOrder:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.], triangleMesh = [], showNodes = False):
        self.show = show
        self.color = color
        self.triangleMesh = triangleMesh
        self.showNodes = showNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color
        yield 'triangleMesh', self.triangleMesh
        yield 'showNodes', self.showNodes

    def __repr__(self):
        return str(dict(self))
class ObjectFFRFreducedOrder:
    def __init__(self, name = '', nodeNumbers = [], massMatrixReduced = [], stiffnessMatrixReduced = [], dampingMatrixReduced = [], forceUserFunction = 0, massMatrixUserFunction = 0, computeFFRFterms = True, modeBasis = [], outputVariableModeBasis = [], outputVariableTypeModeBasis = 0, referencePositions = [], objectIsInitialized = False, physicsMass = 0., physicsInertia = IIDiagMatrix(rowsColumns=3,value=1), physicsCenterOfMass = [0.,0.,0.], mPsiTildePsi = [], mPsiTildePsiTilde = [], mPhitTPsi = [], mPhitTPsiTilde = [], mXRefTildePsi = [], mXRefTildePsiTilde = [], physicsCenterOfMassTilde = IIDiagMatrix(rowsColumns=3,value=0), visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'triangleMesh': [], 'showNodes': False}):
        self.name = name
        self.nodeNumbers = nodeNumbers
        self.massMatrixReduced = massMatrixReduced
        self.stiffnessMatrixReduced = stiffnessMatrixReduced
        self.dampingMatrixReduced = dampingMatrixReduced
        self.forceUserFunction = forceUserFunction
        self.massMatrixUserFunction = massMatrixUserFunction
        self.computeFFRFterms = computeFFRFterms
        self.modeBasis = modeBasis
        self.outputVariableModeBasis = outputVariableModeBasis
        self.outputVariableTypeModeBasis = outputVariableTypeModeBasis
        self.referencePositions = referencePositions
        self.objectIsInitialized = objectIsInitialized
        self.physicsMass = CheckForValidUReal(physicsMass,"physicsMass","ObjectFFRFreducedOrder")
        self.physicsInertia = physicsInertia
        self.physicsCenterOfMass = physicsCenterOfMass
        self.mPsiTildePsi = mPsiTildePsi
        self.mPsiTildePsiTilde = mPsiTildePsiTilde
        self.mPhitTPsi = mPhitTPsi
        self.mPhitTPsiTilde = mPhitTPsiTilde
        self.mXRefTildePsi = mXRefTildePsi
        self.mXRefTildePsiTilde = mXRefTildePsiTilde
        self.physicsCenterOfMassTilde = physicsCenterOfMassTilde
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'FFRFreducedOrder'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'massMatrixReduced', self.massMatrixReduced
        yield 'stiffnessMatrixReduced', self.stiffnessMatrixReduced
        yield 'dampingMatrixReduced', self.dampingMatrixReduced
        yield 'forceUserFunction', self.forceUserFunction
        yield 'massMatrixUserFunction', self.massMatrixUserFunction
        yield 'computeFFRFterms', self.computeFFRFterms
        yield 'modeBasis', self.modeBasis
        yield 'outputVariableModeBasis', self.outputVariableModeBasis
        yield 'outputVariableTypeModeBasis', self.outputVariableTypeModeBasis
        yield 'referencePositions', self.referencePositions
        yield 'objectIsInitialized', self.objectIsInitialized
        yield 'physicsMass', self.physicsMass
        yield 'physicsInertia', self.physicsInertia
        yield 'physicsCenterOfMass', self.physicsCenterOfMass
        yield 'mPsiTildePsi', self.mPsiTildePsi
        yield 'mPsiTildePsiTilde', self.mPsiTildePsiTilde
        yield 'mPhitTPsi', self.mPhitTPsi
        yield 'mPhitTPsiTilde', self.mPhitTPsiTilde
        yield 'mXRefTildePsi', self.mXRefTildePsi
        yield 'mXRefTildePsiTilde', self.mXRefTildePsiTilde
        yield 'physicsCenterOfMassTilde', self.physicsCenterOfMassTilde
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VtriangleMesh', dict(self.visualization)["triangleMesh"]
        yield 'VshowNodes', dict(self.visualization)["showNodes"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CMSobject = ObjectFFRFreducedOrder
VCMSobject = VObjectFFRFreducedOrder

class VObjectANCFCable2D:
    def __init__(self, show = True, drawHeight = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawHeight = drawHeight
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawHeight', self.drawHeight
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectANCFCable2D:
    def __init__(self, name = '', physicsLength = 0., physicsMassPerLength = 0., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsReferenceAxialStrain = 0., physicsReferenceCurvature = 0., strainIsRelativeToReference = 0., nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsLength = CheckForValidUReal(physicsLength,"physicsLength","ObjectANCFCable2D")
        self.physicsMassPerLength = CheckForValidUReal(physicsMassPerLength,"physicsMassPerLength","ObjectANCFCable2D")
        self.physicsBendingStiffness = CheckForValidUReal(physicsBendingStiffness,"physicsBendingStiffness","ObjectANCFCable2D")
        self.physicsAxialStiffness = CheckForValidUReal(physicsAxialStiffness,"physicsAxialStiffness","ObjectANCFCable2D")
        self.physicsBendingDamping = CheckForValidUReal(physicsBendingDamping,"physicsBendingDamping","ObjectANCFCable2D")
        self.physicsAxialDamping = CheckForValidUReal(physicsAxialDamping,"physicsAxialDamping","ObjectANCFCable2D")
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.nodeNumbers = nodeNumbers
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ANCFCable2D'
        yield 'name', self.name
        yield 'physicsLength', self.physicsLength
        yield 'physicsMassPerLength', self.physicsMassPerLength
        yield 'physicsBendingStiffness', self.physicsBendingStiffness
        yield 'physicsAxialStiffness', self.physicsAxialStiffness
        yield 'physicsBendingDamping', self.physicsBendingDamping
        yield 'physicsAxialDamping', self.physicsAxialDamping
        yield 'physicsReferenceAxialStrain', self.physicsReferenceAxialStrain
        yield 'physicsReferenceCurvature', self.physicsReferenceCurvature
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Cable2D = ObjectANCFCable2D
VCable2D = VObjectANCFCable2D

class VObjectALEANCFCable2D:
    def __init__(self, show = True, drawHeight = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawHeight = drawHeight
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawHeight', self.drawHeight
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectALEANCFCable2D:
    def __init__(self, name = '', physicsLength = 0., physicsMassPerLength = 0., physicsMovingMassFactor = 1., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsReferenceAxialStrain = 0., physicsReferenceCurvature = 0., physicsUseCouplingTerms = True, physicsAddALEvariation = True, nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex(), exudyn.InvalidIndex()], useReducedOrderIntegration = 0, strainIsRelativeToReference = 0., visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsLength = CheckForValidUReal(physicsLength,"physicsLength","ObjectALEANCFCable2D")
        self.physicsMassPerLength = CheckForValidUReal(physicsMassPerLength,"physicsMassPerLength","ObjectALEANCFCable2D")
        self.physicsMovingMassFactor = CheckForValidUReal(physicsMovingMassFactor,"physicsMovingMassFactor","ObjectALEANCFCable2D")
        self.physicsBendingStiffness = CheckForValidUReal(physicsBendingStiffness,"physicsBendingStiffness","ObjectALEANCFCable2D")
        self.physicsAxialStiffness = CheckForValidUReal(physicsAxialStiffness,"physicsAxialStiffness","ObjectALEANCFCable2D")
        self.physicsBendingDamping = CheckForValidUReal(physicsBendingDamping,"physicsBendingDamping","ObjectALEANCFCable2D")
        self.physicsAxialDamping = CheckForValidUReal(physicsAxialDamping,"physicsAxialDamping","ObjectALEANCFCable2D")
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.physicsUseCouplingTerms = physicsUseCouplingTerms
        self.physicsAddALEvariation = physicsAddALEvariation
        self.nodeNumbers = nodeNumbers
        self.useReducedOrderIntegration = useReducedOrderIntegration
        self.strainIsRelativeToReference = strainIsRelativeToReference
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ALEANCFCable2D'
        yield 'name', self.name
        yield 'physicsLength', self.physicsLength
        yield 'physicsMassPerLength', self.physicsMassPerLength
        yield 'physicsMovingMassFactor', self.physicsMovingMassFactor
        yield 'physicsBendingStiffness', self.physicsBendingStiffness
        yield 'physicsAxialStiffness', self.physicsAxialStiffness
        yield 'physicsBendingDamping', self.physicsBendingDamping
        yield 'physicsAxialDamping', self.physicsAxialDamping
        yield 'physicsReferenceAxialStrain', self.physicsReferenceAxialStrain
        yield 'physicsReferenceCurvature', self.physicsReferenceCurvature
        yield 'physicsUseCouplingTerms', self.physicsUseCouplingTerms
        yield 'physicsAddALEvariation', self.physicsAddALEvariation
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'strainIsRelativeToReference', self.strainIsRelativeToReference
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
ALECable2D = ObjectALEANCFCable2D
VALECable2D = VObjectALEANCFCable2D

class VObjectANCFBeam3D:
    def __init__(self, show = True, sectionGeometry = exudyn.BeamSectionGeometry(), color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.sectionGeometry = sectionGeometry
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'sectionGeometry', self.sectionGeometry
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectANCFBeam3D:
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], physicsLength = 0., sectionData = exudyn.BeamSection(), crossSectionPenaltyFactor = [1.,1.,1.], testBeamRectangularSize = [-1.,-1.], visualization = {'show': True, 'sectionGeometry': exudyn.BeamSectionGeometry(), 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = nodeNumbers
        self.physicsLength = CheckForValidPReal(physicsLength,"physicsLength","ObjectANCFBeam3D")
        self.sectionData = sectionData
        self.crossSectionPenaltyFactor = crossSectionPenaltyFactor
        self.testBeamRectangularSize = testBeamRectangularSize
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ANCFBeam3D'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'physicsLength', self.physicsLength
        yield 'sectionData', self.sectionData
        yield 'crossSectionPenaltyFactor', self.crossSectionPenaltyFactor
        yield 'testBeamRectangularSize', self.testBeamRectangularSize
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VsectionGeometry', dict(self.visualization)["sectionGeometry"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Beam3D = ObjectANCFBeam3D
VBeam3D = VObjectANCFBeam3D

class VObjectBeamGeometricallyExact2D:
    def __init__(self, show = True, drawHeight = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawHeight = drawHeight
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawHeight', self.drawHeight
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectBeamGeometricallyExact2D:
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], physicsLength = 0., physicsMassPerLength = 0., physicsCrossSectionInertia = 0., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsShearStiffness = 0., visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = nodeNumbers
        self.physicsLength = CheckForValidUReal(physicsLength,"physicsLength","ObjectBeamGeometricallyExact2D")
        self.physicsMassPerLength = CheckForValidUReal(physicsMassPerLength,"physicsMassPerLength","ObjectBeamGeometricallyExact2D")
        self.physicsCrossSectionInertia = CheckForValidUReal(physicsCrossSectionInertia,"physicsCrossSectionInertia","ObjectBeamGeometricallyExact2D")
        self.physicsBendingStiffness = CheckForValidUReal(physicsBendingStiffness,"physicsBendingStiffness","ObjectBeamGeometricallyExact2D")
        self.physicsAxialStiffness = CheckForValidUReal(physicsAxialStiffness,"physicsAxialStiffness","ObjectBeamGeometricallyExact2D")
        self.physicsShearStiffness = CheckForValidUReal(physicsShearStiffness,"physicsShearStiffness","ObjectBeamGeometricallyExact2D")
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'BeamGeometricallyExact2D'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'physicsLength', self.physicsLength
        yield 'physicsMassPerLength', self.physicsMassPerLength
        yield 'physicsCrossSectionInertia', self.physicsCrossSectionInertia
        yield 'physicsBendingStiffness', self.physicsBendingStiffness
        yield 'physicsAxialStiffness', self.physicsAxialStiffness
        yield 'physicsShearStiffness', self.physicsShearStiffness
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Beam2D = ObjectBeamGeometricallyExact2D
VBeam2D = VObjectBeamGeometricallyExact2D

class VObjectBeamGeometricallyExact3D:
    def __init__(self, show = True, sectionGeometry = exudyn.BeamSectionGeometry(), color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.sectionGeometry = sectionGeometry
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'sectionGeometry', self.sectionGeometry
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectBeamGeometricallyExact3D:
    def __init__(self, name = '', nodeNumbers = [exudyn.InvalidIndex(), exudyn.InvalidIndex()], physicsLength = 0., sectionData = exudyn.BeamSection(), visualization = {'show': True, 'sectionGeometry': exudyn.BeamSectionGeometry(), 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.nodeNumbers = nodeNumbers
        self.physicsLength = CheckForValidPReal(physicsLength,"physicsLength","ObjectBeamGeometricallyExact3D")
        self.sectionData = sectionData
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'BeamGeometricallyExact3D'
        yield 'name', self.name
        yield 'nodeNumbers', self.nodeNumbers
        yield 'physicsLength', self.physicsLength
        yield 'sectionData', self.sectionData
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VsectionGeometry', dict(self.visualization)["sectionGeometry"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Beam3D = ObjectBeamGeometricallyExact3D
VBeam3D = VObjectBeamGeometricallyExact3D

class VObjectConnectorSpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorSpringDamper:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], referenceLength = 0., stiffness = 0., damping = 0., force = 0., velocityOffset = 0., activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.referenceLength = CheckForValidPReal(referenceLength,"referenceLength","ObjectConnectorSpringDamper")
        self.stiffness = CheckForValidUReal(stiffness,"stiffness","ObjectConnectorSpringDamper")
        self.damping = CheckForValidUReal(damping,"damping","ObjectConnectorSpringDamper")
        self.force = force
        self.velocityOffset = velocityOffset
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'referenceLength', self.referenceLength
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'force', self.force
        yield 'velocityOffset', self.velocityOffset
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
SpringDamper = ObjectConnectorSpringDamper
VSpringDamper = VObjectConnectorSpringDamper

class VObjectConnectorCartesianSpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorCartesianSpringDamper:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = [0.,0.,0.], damping = [0.,0.,0.], offset = [0.,0.,0.], springForceUserFunction = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.stiffness = stiffness
        self.damping = damping
        self.offset = offset
        self.springForceUserFunction = springForceUserFunction
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorCartesianSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'offset', self.offset
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CartesianSpringDamper = ObjectConnectorCartesianSpringDamper
VCartesianSpringDamper = VObjectConnectorCartesianSpringDamper

class VObjectConnectorRigidBodySpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorRigidBodySpringDamper:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = IIDiagMatrix(rowsColumns=6,value=0.), damping = IIDiagMatrix(rowsColumns=6,value=0.), rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), offset = [0.,0.,0.,0.,0.,0.], activeConnector = True, springForceTorqueUserFunction = 0, postNewtonStepUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.stiffness = stiffness
        self.damping = damping
        self.rotationMarker0 = rotationMarker0
        self.rotationMarker1 = rotationMarker1
        self.offset = offset
        self.activeConnector = activeConnector
        self.springForceTorqueUserFunction = springForceTorqueUserFunction
        self.postNewtonStepUserFunction = postNewtonStepUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorRigidBodySpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'springForceTorqueUserFunction', self.springForceTorqueUserFunction
        yield 'postNewtonStepUserFunction', self.postNewtonStepUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RigidBodySpringDamper = ObjectConnectorRigidBodySpringDamper
VRigidBodySpringDamper = VObjectConnectorRigidBodySpringDamper

class VObjectConnectorLinearSpringDamper:
    def __init__(self, show = True, drawSize = -1., drawAsCylinder = False, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.drawAsCylinder = drawAsCylinder
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'drawAsCylinder', self.drawAsCylinder
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorLinearSpringDamper:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = 0., damping = 0., axisMarker0 = [1,0,0], offset = 0., velocityOffset = 0., force = 0., activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'drawAsCylinder': False, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.stiffness = stiffness
        self.damping = damping
        self.axisMarker0 = axisMarker0
        self.offset = offset
        self.velocityOffset = velocityOffset
        self.force = force
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorLinearSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'axisMarker0', self.axisMarker0
        yield 'offset', self.offset
        yield 'velocityOffset', self.velocityOffset
        yield 'force', self.force
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'VdrawAsCylinder', dict(self.visualization)["drawAsCylinder"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
LinearSpringDamper = ObjectConnectorLinearSpringDamper
VLinearSpringDamper = VObjectConnectorLinearSpringDamper

class VObjectConnectorTorsionalSpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorTorsionalSpringDamper:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = 0., damping = 0., rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), offset = 0., velocityOffset = 0., torque = 0., activeConnector = True, springTorqueUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.stiffness = stiffness
        self.damping = damping
        self.rotationMarker0 = rotationMarker0
        self.rotationMarker1 = rotationMarker1
        self.offset = offset
        self.velocityOffset = velocityOffset
        self.torque = torque
        self.activeConnector = activeConnector
        self.springTorqueUserFunction = springTorqueUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorTorsionalSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'offset', self.offset
        yield 'velocityOffset', self.velocityOffset
        yield 'torque', self.torque
        yield 'activeConnector', self.activeConnector
        yield 'springTorqueUserFunction', self.springTorqueUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
TorsionalSpringDamper = ObjectConnectorTorsionalSpringDamper
VTorsionalSpringDamper = VObjectConnectorTorsionalSpringDamper

class VObjectConnectorCoordinateSpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorCoordinateSpringDamper:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], stiffness = 0., damping = 0., offset = 0., dryFriction = 0., dryFrictionProportionalZone = 0., activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.stiffness = stiffness
        self.damping = damping
        self.offset = offset
        self.dryFriction = dryFriction
        self.dryFrictionProportionalZone = dryFrictionProportionalZone
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinateSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'offset', self.offset
        yield 'dryFriction', self.dryFriction
        yield 'dryFrictionProportionalZone', self.dryFrictionProportionalZone
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CoordinateSpringDamper = ObjectConnectorCoordinateSpringDamper
VCoordinateSpringDamper = VObjectConnectorCoordinateSpringDamper

class VObjectConnectorCoordinateSpringDamperExt:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorCoordinateSpringDamperExt:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), stiffness = 0., damping = 0., offset = 0., velocityOffset = 0., dynamicFriction = 0., staticFrictionOffset = 0., stickingStiffness = 0., stickingDamping = 0., frictionExpVelocity = 0., frictionViscous = 0., limitStopsUpper = 0., limitStopsLower = 0., limitStopsStiffness = 0., limitStopsDamping = 0., useLimitStops = False, activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.stiffness = stiffness
        self.damping = damping
        self.offset = offset
        self.velocityOffset = velocityOffset
        self.dynamicFriction = dynamicFriction
        self.staticFrictionOffset = staticFrictionOffset
        self.stickingStiffness = stickingStiffness
        self.stickingDamping = stickingDamping
        self.frictionExpVelocity = frictionExpVelocity
        self.frictionViscous = frictionViscous
        self.limitStopsUpper = limitStopsUpper
        self.limitStopsLower = limitStopsLower
        self.limitStopsStiffness = limitStopsStiffness
        self.limitStopsDamping = limitStopsDamping
        self.useLimitStops = useLimitStops
        self.activeConnector = activeConnector
        self.springForceUserFunction = springForceUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinateSpringDamperExt'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'offset', self.offset
        yield 'velocityOffset', self.velocityOffset
        yield 'dynamicFriction', self.dynamicFriction
        yield 'staticFrictionOffset', self.staticFrictionOffset
        yield 'stickingStiffness', self.stickingStiffness
        yield 'stickingDamping', self.stickingDamping
        yield 'frictionExpVelocity', self.frictionExpVelocity
        yield 'frictionViscous', self.frictionViscous
        yield 'limitStopsUpper', self.limitStopsUpper
        yield 'limitStopsLower', self.limitStopsLower
        yield 'limitStopsStiffness', self.limitStopsStiffness
        yield 'limitStopsDamping', self.limitStopsDamping
        yield 'useLimitStops', self.useLimitStops
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CoordinateSpringDamperExt = ObjectConnectorCoordinateSpringDamperExt
VCoordinateSpringDamperExt = VObjectConnectorCoordinateSpringDamperExt

class VObjectConnectorGravity:
    def __init__(self, show = False, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorGravity:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], gravitationalConstant = 6.67430e-11, mass0 = 0., mass1 = 0., minDistanceRegularization = 0., activeConnector = True, visualization = {'show': False, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.gravitationalConstant = gravitationalConstant
        self.mass0 = CheckForValidUReal(mass0,"mass0","ObjectConnectorGravity")
        self.mass1 = CheckForValidUReal(mass1,"mass1","ObjectConnectorGravity")
        self.minDistanceRegularization = CheckForValidUReal(minDistanceRegularization,"minDistanceRegularization","ObjectConnectorGravity")
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorGravity'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'gravitationalConstant', self.gravitationalConstant
        yield 'mass0', self.mass0
        yield 'mass1', self.mass1
        yield 'minDistanceRegularization', self.minDistanceRegularization
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
ConnectorGravity = ObjectConnectorGravity
VConnectorGravity = VObjectConnectorGravity

class VObjectConnectorHydraulicActuatorSimple:
    def __init__(self, show = True, cylinderRadius = 0.05, rodRadius = 0.03, pistonRadius = 0.04, pistonLength = 0.001, rodMountRadius = 0.0, baseMountRadius = 0.0, baseMountLength = 0.0, colorCylinder = [-1.,-1.,-1.,-1.], colorPiston = [0.8,0.8,0.8,1.]):
        self.show = show
        self.cylinderRadius = cylinderRadius
        self.rodRadius = rodRadius
        self.pistonRadius = pistonRadius
        self.pistonLength = pistonLength
        self.rodMountRadius = rodMountRadius
        self.baseMountRadius = baseMountRadius
        self.baseMountLength = baseMountLength
        self.colorCylinder = colorCylinder
        self.colorPiston = colorPiston

    def __iter__(self):
        yield 'show', self.show
        yield 'cylinderRadius', self.cylinderRadius
        yield 'rodRadius', self.rodRadius
        yield 'pistonRadius', self.pistonRadius
        yield 'pistonLength', self.pistonLength
        yield 'rodMountRadius', self.rodMountRadius
        yield 'baseMountRadius', self.baseMountRadius
        yield 'baseMountLength', self.baseMountLength
        yield 'colorCylinder', self.colorCylinder
        yield 'colorPiston', self.colorPiston

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorHydraulicActuatorSimple:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumbers = [], offsetLength = 0., strokeLength = 0., chamberCrossSection0 = 0., chamberCrossSection1 = 0., referenceVolume0 = 0., referenceVolume1 = 0., valveOpening0 = 0., valveOpening1 = 0., actuatorDamping = 0., oilBulkModulus = 0., nominalFlow = 0., systemPressure = 0., tankPressure = 0., useChamberVolumeChange = False, activeConnector = True, visualization = {'show': True, 'cylinderRadius': 0.05, 'rodRadius': 0.03, 'pistonRadius': 0.04, 'pistonLength': 0.001, 'rodMountRadius': 0.0, 'baseMountRadius': 0.0, 'baseMountLength': 0.0, 'colorCylinder': [-1.,-1.,-1.,-1.], 'colorPiston': [0.8,0.8,0.8,1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumbers = nodeNumbers
        self.offsetLength = CheckForValidUReal(offsetLength,"offsetLength","ObjectConnectorHydraulicActuatorSimple")
        self.strokeLength = CheckForValidPReal(strokeLength,"strokeLength","ObjectConnectorHydraulicActuatorSimple")
        self.chamberCrossSection0 = CheckForValidPReal(chamberCrossSection0,"chamberCrossSection0","ObjectConnectorHydraulicActuatorSimple")
        self.chamberCrossSection1 = CheckForValidPReal(chamberCrossSection1,"chamberCrossSection1","ObjectConnectorHydraulicActuatorSimple")
        self.referenceVolume0 = CheckForValidPReal(referenceVolume0,"referenceVolume0","ObjectConnectorHydraulicActuatorSimple")
        self.referenceVolume1 = CheckForValidPReal(referenceVolume1,"referenceVolume1","ObjectConnectorHydraulicActuatorSimple")
        self.valveOpening0 = valveOpening0
        self.valveOpening1 = valveOpening1
        self.actuatorDamping = CheckForValidUReal(actuatorDamping,"actuatorDamping","ObjectConnectorHydraulicActuatorSimple")
        self.oilBulkModulus = CheckForValidPReal(oilBulkModulus,"oilBulkModulus","ObjectConnectorHydraulicActuatorSimple")
        self.nominalFlow = CheckForValidPReal(nominalFlow,"nominalFlow","ObjectConnectorHydraulicActuatorSimple")
        self.systemPressure = systemPressure
        self.tankPressure = tankPressure
        self.useChamberVolumeChange = useChamberVolumeChange
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorHydraulicActuatorSimple'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumbers', self.nodeNumbers
        yield 'offsetLength', self.offsetLength
        yield 'strokeLength', self.strokeLength
        yield 'chamberCrossSection0', self.chamberCrossSection0
        yield 'chamberCrossSection1', self.chamberCrossSection1
        yield 'referenceVolume0', self.referenceVolume0
        yield 'referenceVolume1', self.referenceVolume1
        yield 'valveOpening0', self.valveOpening0
        yield 'valveOpening1', self.valveOpening1
        yield 'actuatorDamping', self.actuatorDamping
        yield 'oilBulkModulus', self.oilBulkModulus
        yield 'nominalFlow', self.nominalFlow
        yield 'systemPressure', self.systemPressure
        yield 'tankPressure', self.tankPressure
        yield 'useChamberVolumeChange', self.useChamberVolumeChange
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VcylinderRadius', dict(self.visualization)["cylinderRadius"]
        yield 'VrodRadius', dict(self.visualization)["rodRadius"]
        yield 'VpistonRadius', dict(self.visualization)["pistonRadius"]
        yield 'VpistonLength', dict(self.visualization)["pistonLength"]
        yield 'VrodMountRadius', dict(self.visualization)["rodMountRadius"]
        yield 'VbaseMountRadius', dict(self.visualization)["baseMountRadius"]
        yield 'VbaseMountLength', dict(self.visualization)["baseMountLength"]
        yield 'VcolorCylinder', dict(self.visualization)["colorCylinder"]
        yield 'VcolorPiston', dict(self.visualization)["colorPiston"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
HydraulicActuatorSimple = ObjectConnectorHydraulicActuatorSimple
VHydraulicActuatorSimple = VObjectConnectorHydraulicActuatorSimple

class VObjectConnectorDistance:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorDistance:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], distance = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.distance = CheckForValidUReal(distance,"distance","ObjectConnectorDistance")
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorDistance'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'distance', self.distance
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
DistanceConstraint = ObjectConnectorDistance
VDistanceConstraint = VObjectConnectorDistance

class VObjectConnectorCoordinate:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorCoordinate:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], offset = 0., factorValue1 = 1., velocityLevel = False, offsetUserFunction = 0, offsetUserFunction_t = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.offset = offset
        self.factorValue1 = factorValue1
        self.velocityLevel = velocityLevel
        self.offsetUserFunction = offsetUserFunction
        self.offsetUserFunction_t = offsetUserFunction_t
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinate'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'offset', self.offset
        yield 'factorValue1', self.factorValue1
        yield 'velocityLevel', self.velocityLevel
        yield 'offsetUserFunction', self.offsetUserFunction
        yield 'offsetUserFunction_t', self.offsetUserFunction_t
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CoordinateConstraint = ObjectConnectorCoordinate
VCoordinateConstraint = VObjectConnectorCoordinate

class VObjectConnectorCoordinateVector:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorCoordinateVector:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], scalingMarker0 = [], scalingMarker1 = [], quadraticTermMarker0 = [], quadraticTermMarker1 = [], offset = [], velocityLevel = False, constraintUserFunction = 0, jacobianUserFunction = 0, activeConnector = True, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.scalingMarker0 = scalingMarker0
        self.scalingMarker1 = scalingMarker1
        self.quadraticTermMarker0 = quadraticTermMarker0
        self.quadraticTermMarker1 = quadraticTermMarker1
        self.offset = offset
        self.velocityLevel = velocityLevel
        self.constraintUserFunction = constraintUserFunction
        self.jacobianUserFunction = jacobianUserFunction
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorCoordinateVector'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'scalingMarker0', self.scalingMarker0
        yield 'scalingMarker1', self.scalingMarker1
        yield 'quadraticTermMarker0', self.quadraticTermMarker0
        yield 'quadraticTermMarker1', self.quadraticTermMarker1
        yield 'offset', self.offset
        yield 'velocityLevel', self.velocityLevel
        yield 'constraintUserFunction', self.constraintUserFunction
        yield 'jacobianUserFunction', self.jacobianUserFunction
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
CoordinateVectorConstraint = ObjectConnectorCoordinateVector
VCoordinateVectorConstraint = VObjectConnectorCoordinateVector

class VObjectConnectorRollingDiscPenalty:
    def __init__(self, show = True, discWidth = 0.1, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.discWidth = discWidth
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'discWidth', self.discWidth
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectConnectorRollingDiscPenalty:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), dryFrictionAngle = 0., contactStiffness = 0., contactDamping = 0., dryFriction = [0,0], dryFrictionProportionalZone = 0., rollingFrictionViscous = 0., activeConnector = True, discRadius = 0, planeNormal = [0,0,1], visualization = {'show': True, 'discWidth': 0.1, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.dryFrictionAngle = dryFrictionAngle
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.dryFriction = dryFriction
        self.dryFrictionProportionalZone = dryFrictionProportionalZone
        self.rollingFrictionViscous = rollingFrictionViscous
        self.activeConnector = activeConnector
        self.discRadius = discRadius
        self.planeNormal = planeNormal
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorRollingDiscPenalty'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'dryFrictionAngle', self.dryFrictionAngle
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'dryFriction', self.dryFriction
        yield 'dryFrictionProportionalZone', self.dryFrictionProportionalZone
        yield 'rollingFrictionViscous', self.rollingFrictionViscous
        yield 'activeConnector', self.activeConnector
        yield 'discRadius', self.discRadius
        yield 'planeNormal', self.planeNormal
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdiscWidth', dict(self.visualization)["discWidth"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RollingDiscPenalty = ObjectConnectorRollingDiscPenalty
VRollingDiscPenalty = VObjectConnectorRollingDiscPenalty

class VObjectContactConvexRoll:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectContactConvexRoll:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., staticFrictionOffset = 0., viscousFriction = 0., exponentialDecayStatic = 1e-3, frictionProportionalZone = 1e-3, rollLength = 0., coefficientsHull =  [], rBoundingSphere = 0, activeConnector = True, visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.dynamicFriction = CheckForValidUReal(dynamicFriction,"dynamicFriction","ObjectContactConvexRoll")
        self.staticFrictionOffset = CheckForValidUReal(staticFrictionOffset,"staticFrictionOffset","ObjectContactConvexRoll")
        self.viscousFriction = CheckForValidUReal(viscousFriction,"viscousFriction","ObjectContactConvexRoll")
        self.exponentialDecayStatic = CheckForValidPReal(exponentialDecayStatic,"exponentialDecayStatic","ObjectContactConvexRoll")
        self.frictionProportionalZone = CheckForValidUReal(frictionProportionalZone,"frictionProportionalZone","ObjectContactConvexRoll")
        self.rollLength = CheckForValidUReal(rollLength,"rollLength","ObjectContactConvexRoll")
        self.coefficientsHull = coefficientsHull
        self.rBoundingSphere = CheckForValidUReal(rBoundingSphere,"rBoundingSphere","ObjectContactConvexRoll")
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ContactConvexRoll'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'dynamicFriction', self.dynamicFriction
        yield 'staticFrictionOffset', self.staticFrictionOffset
        yield 'viscousFriction', self.viscousFriction
        yield 'exponentialDecayStatic', self.exponentialDecayStatic
        yield 'frictionProportionalZone', self.frictionProportionalZone
        yield 'rollLength', self.rollLength
        yield 'coefficientsHull', self.coefficientsHull
        yield 'rBoundingSphere', self.rBoundingSphere
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
class VObjectContactCoordinate:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectContactCoordinate:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), contactStiffness = 0., contactDamping = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactCoordinate")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactCoordinate")
        self.offset = offset
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ContactCoordinate'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
class VObjectContactCircleCable2D:
    def __init__(self, show = True, showContactCircle = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.showContactCircle = showContactCircle
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'showContactCircle', self.showContactCircle
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectContactCircleCable2D:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., circleRadius = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'showContactCircle': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = numberOfContactSegments
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactCircleCable2D")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactCircleCable2D")
        self.circleRadius = CheckForValidUReal(circleRadius,"circleRadius","ObjectContactCircleCable2D")
        self.offset = offset
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ContactCircleCable2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'numberOfContactSegments', self.numberOfContactSegments
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'circleRadius', self.circleRadius
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowContactCircle', dict(self.visualization)["showContactCircle"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
class VObjectContactFrictionCircleCable2D:
    def __init__(self, show = True, showContactCircle = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.showContactCircle = showContactCircle
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'showContactCircle', self.showContactCircle
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectContactFrictionCircleCable2D:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., frictionVelocityPenalty = 0., frictionStiffness = 0., frictionCoefficient = 0., circleRadius = 0., useSegmentNormals = True, activeConnector = True, visualization = {'show': True, 'showContactCircle': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = CheckForValidPInt(numberOfContactSegments,"numberOfContactSegments","ObjectContactFrictionCircleCable2D")
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactFrictionCircleCable2D")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactFrictionCircleCable2D")
        self.frictionVelocityPenalty = CheckForValidUReal(frictionVelocityPenalty,"frictionVelocityPenalty","ObjectContactFrictionCircleCable2D")
        self.frictionStiffness = CheckForValidUReal(frictionStiffness,"frictionStiffness","ObjectContactFrictionCircleCable2D")
        self.frictionCoefficient = CheckForValidUReal(frictionCoefficient,"frictionCoefficient","ObjectContactFrictionCircleCable2D")
        self.circleRadius = CheckForValidUReal(circleRadius,"circleRadius","ObjectContactFrictionCircleCable2D")
        self.useSegmentNormals = useSegmentNormals
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ContactFrictionCircleCable2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'numberOfContactSegments', self.numberOfContactSegments
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'frictionVelocityPenalty', self.frictionVelocityPenalty
        yield 'frictionStiffness', self.frictionStiffness
        yield 'frictionCoefficient', self.frictionCoefficient
        yield 'circleRadius', self.circleRadius
        yield 'useSegmentNormals', self.useSegmentNormals
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowContactCircle', dict(self.visualization)["showContactCircle"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
class VObjectJointGeneric:
    def __init__(self, show = True, axesRadius = 0.1, axesLength = 0.4, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.axesRadius = axesRadius
        self.axesLength = axesLength
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'axesRadius', self.axesRadius
        yield 'axesLength', self.axesLength
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointGeneric:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], constrainedAxes = [1,1,1,1,1,1], rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), activeConnector = True, offsetUserFunctionParameters = [0.,0.,0.,0.,0.,0.], offsetUserFunction = 0, offsetUserFunction_t = 0, visualization = {'show': True, 'axesRadius': 0.1, 'axesLength': 0.4, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.constrainedAxes = constrainedAxes
        self.rotationMarker0 = rotationMarker0
        self.rotationMarker1 = rotationMarker1
        self.activeConnector = activeConnector
        self.offsetUserFunctionParameters = offsetUserFunctionParameters
        self.offsetUserFunction = offsetUserFunction
        self.offsetUserFunction_t = offsetUserFunction_t
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointGeneric'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'constrainedAxes', self.constrainedAxes
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'activeConnector', self.activeConnector
        yield 'offsetUserFunctionParameters', self.offsetUserFunctionParameters
        yield 'offsetUserFunction', self.offsetUserFunction
        yield 'offsetUserFunction_t', self.offsetUserFunction_t
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VaxesRadius', dict(self.visualization)["axesRadius"]
        yield 'VaxesLength', dict(self.visualization)["axesLength"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
GenericJoint = ObjectJointGeneric
VGenericJoint = VObjectJointGeneric

class VObjectJointRevoluteZ:
    def __init__(self, show = True, axisRadius = 0.1, axisLength = 0.4, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.axisRadius = axisRadius
        self.axisLength = axisLength
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'axisRadius', self.axisRadius
        yield 'axisLength', self.axisLength
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointRevoluteZ:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), activeConnector = True, visualization = {'show': True, 'axisRadius': 0.1, 'axisLength': 0.4, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.rotationMarker0 = rotationMarker0
        self.rotationMarker1 = rotationMarker1
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointRevoluteZ'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VaxisRadius', dict(self.visualization)["axisRadius"]
        yield 'VaxisLength', dict(self.visualization)["axisLength"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RevoluteJointZ = ObjectJointRevoluteZ
VRevoluteJointZ = VObjectJointRevoluteZ

class VObjectJointPrismaticX:
    def __init__(self, show = True, axisRadius = 0.1, axisLength = 0.4, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.axisRadius = axisRadius
        self.axisLength = axisLength
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'axisRadius', self.axisRadius
        yield 'axisLength', self.axisLength
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointPrismaticX:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), activeConnector = True, visualization = {'show': True, 'axisRadius': 0.1, 'axisLength': 0.4, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.rotationMarker0 = rotationMarker0
        self.rotationMarker1 = rotationMarker1
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointPrismaticX'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VaxisRadius', dict(self.visualization)["axisRadius"]
        yield 'VaxisLength', dict(self.visualization)["axisLength"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
PrismaticJointX = ObjectJointPrismaticX
VPrismaticJointX = VObjectJointPrismaticX

class VObjectJointSpherical:
    def __init__(self, show = True, jointRadius = 0.1, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.jointRadius = jointRadius
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'jointRadius', self.jointRadius
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointSpherical:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], constrainedAxes = [1,1,1], activeConnector = True, visualization = {'show': True, 'jointRadius': 0.1, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.constrainedAxes = constrainedAxes
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointSpherical'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'constrainedAxes', self.constrainedAxes
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VjointRadius', dict(self.visualization)["jointRadius"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
SphericalJoint = ObjectJointSpherical
VSphericalJoint = VObjectJointSpherical

class VObjectJointRollingDisc:
    def __init__(self, show = True, discWidth = 0.1, color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.discWidth = discWidth
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'discWidth', self.discWidth
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointRollingDisc:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], constrainedAxes = [1,1,1], activeConnector = True, discRadius = 0, planeNormal = [0,0,1], visualization = {'show': True, 'discWidth': 0.1, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.constrainedAxes = constrainedAxes
        self.activeConnector = activeConnector
        self.discRadius = CheckForValidPReal(discRadius,"discRadius","ObjectJointRollingDisc")
        self.planeNormal = planeNormal
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointRollingDisc'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'constrainedAxes', self.constrainedAxes
        yield 'activeConnector', self.activeConnector
        yield 'discRadius', self.discRadius
        yield 'planeNormal', self.planeNormal
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdiscWidth', dict(self.visualization)["discWidth"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RollingDiscJoint = ObjectJointRollingDisc
VRollingDiscJoint = VObjectJointRollingDisc

class VObjectJointRevolute2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointRevolute2D:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointRevolute2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
RevoluteJoint2D = ObjectJointRevolute2D
VRevoluteJoint2D = VObjectJointRevolute2D

class VObjectJointPrismatic2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointPrismatic2D:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], axisMarker0 = [1.,0.,0.], normalMarker1 = [0.,1.,0.], constrainRotation = True, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.axisMarker0 = axisMarker0
        self.normalMarker1 = normalMarker1
        self.constrainRotation = constrainRotation
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointPrismatic2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'axisMarker0', self.axisMarker0
        yield 'normalMarker1', self.normalMarker1
        yield 'constrainRotation', self.constrainRotation
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
PrismaticJoint2D = ObjectJointPrismatic2D
VPrismaticJoint2D = VObjectJointPrismatic2D

class VObjectJointSliding2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointSliding2D:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], slidingMarkerNumbers = [], slidingMarkerOffsets = [], nodeNumber = exudyn.InvalidIndex(), classicalFormulation = True, constrainRotation = False, axialForce = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.slidingMarkerNumbers = slidingMarkerNumbers
        self.slidingMarkerOffsets = slidingMarkerOffsets
        self.nodeNumber = nodeNumber
        self.classicalFormulation = classicalFormulation
        self.constrainRotation = constrainRotation
        self.axialForce = axialForce
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointSliding2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'slidingMarkerNumbers', self.slidingMarkerNumbers
        yield 'slidingMarkerOffsets', self.slidingMarkerOffsets
        yield 'nodeNumber', self.nodeNumber
        yield 'classicalFormulation', self.classicalFormulation
        yield 'constrainRotation', self.constrainRotation
        yield 'axialForce', self.axialForce
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
SlidingJoint2D = ObjectJointSliding2D
VSlidingJoint2D = VObjectJointSliding2D

class VObjectJointALEMoving2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectJointALEMoving2D:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], slidingMarkerNumbers = [], slidingMarkerOffsets = [], slidingOffset = 0., nodeNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], usePenaltyFormulation = False, penaltyStiffness = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.slidingMarkerNumbers = slidingMarkerNumbers
        self.slidingMarkerOffsets = slidingMarkerOffsets
        self.slidingOffset = slidingOffset
        self.nodeNumbers = nodeNumbers
        self.usePenaltyFormulation = usePenaltyFormulation
        self.penaltyStiffness = penaltyStiffness
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointALEMoving2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'slidingMarkerNumbers', self.slidingMarkerNumbers
        yield 'slidingMarkerOffsets', self.slidingMarkerOffsets
        yield 'slidingOffset', self.slidingOffset
        yield 'nodeNumbers', self.nodeNumbers
        yield 'usePenaltyFormulation', self.usePenaltyFormulation
        yield 'penaltyStiffness', self.penaltyStiffness
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
ALEMovingJoint2D = ObjectJointALEMoving2D
VALEMovingJoint2D = VObjectJointALEMoving2D

class VObjectContactFrictionCircleCable2DOld:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

    def __repr__(self):
        return str(dict(self))
class ObjectContactFrictionCircleCable2DOld:
    def __init__(self, name = '', markerNumbers = [ exudyn.InvalidIndex(), exudyn.InvalidIndex() ], nodeNumber = exudyn.InvalidIndex(), numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., frictionVelocityPenalty = 0., frictionStiffness = 0., frictionCoefficient = 0., circleRadius = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = CheckForValidPInt(numberOfContactSegments,"numberOfContactSegments","ObjectContactFrictionCircleCable2DOld")
        self.contactStiffness = CheckForValidUReal(contactStiffness,"contactStiffness","ObjectContactFrictionCircleCable2DOld")
        self.contactDamping = CheckForValidUReal(contactDamping,"contactDamping","ObjectContactFrictionCircleCable2DOld")
        self.frictionVelocityPenalty = CheckForValidUReal(frictionVelocityPenalty,"frictionVelocityPenalty","ObjectContactFrictionCircleCable2DOld")
        self.frictionStiffness = CheckForValidUReal(frictionStiffness,"frictionStiffness","ObjectContactFrictionCircleCable2DOld")
        self.frictionCoefficient = CheckForValidUReal(frictionCoefficient,"frictionCoefficient","ObjectContactFrictionCircleCable2DOld")
        self.circleRadius = CheckForValidUReal(circleRadius,"circleRadius","ObjectContactFrictionCircleCable2DOld")
        self.offset = offset
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ContactFrictionCircleCable2DOld'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'nodeNumber', self.nodeNumber
        yield 'numberOfContactSegments', self.numberOfContactSegments
        yield 'contactStiffness', self.contactStiffness
        yield 'contactDamping', self.contactDamping
        yield 'frictionVelocityPenalty', self.frictionVelocityPenalty
        yield 'frictionStiffness', self.frictionStiffness
        yield 'frictionCoefficient', self.frictionCoefficient
        yield 'circleRadius', self.circleRadius
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

    def __repr__(self):
        return str(dict(self))
#+++++++++++++++++++++++++++++++
#MARKER
class VMarkerBodyMass:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyMass:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyMass'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerBodyPosition:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyPosition:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = localPosition
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyPosition'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerBodyRigid:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyRigid:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = localPosition
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyRigid'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodePosition:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodePosition:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodePosition'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeRigid:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeRigid:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodeRigid'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeCoordinate:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeCoordinate:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), coordinate = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.coordinate = CheckForValidUInt(coordinate,"coordinate","MarkerNodeCoordinate")
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodeCoordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'coordinate', self.coordinate
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeCoordinates:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeCoordinates:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodeCoordinates'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeODE1Coordinate:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeODE1Coordinate:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), coordinate = exudyn.InvalidIndex(), visualization = {'show': False}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.coordinate = CheckForValidUInt(coordinate,"coordinate","MarkerNodeODE1Coordinate")
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodeODE1Coordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'coordinate', self.coordinate
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerNodeRotationCoordinate:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerNodeRotationCoordinate:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), rotationCoordinate = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.rotationCoordinate = CheckForValidUInt(rotationCoordinate,"rotationCoordinate","MarkerNodeRotationCoordinate")
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodeRotationCoordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'rotationCoordinate', self.rotationCoordinate
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerSuperElementPosition:
    def __init__(self, show = True, showMarkerNodes = True):
        self.show = show
        self.showMarkerNodes = showMarkerNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'showMarkerNodes', self.showMarkerNodes

    def __repr__(self):
        return str(dict(self))
class MarkerSuperElementPosition:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), meshNodeNumbers = [], weightingFactors = [], visualization = {'show': True, 'showMarkerNodes': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.meshNodeNumbers = meshNodeNumbers
        self.weightingFactors = weightingFactors
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'SuperElementPosition'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'meshNodeNumbers', self.meshNodeNumbers
        yield 'weightingFactors', self.weightingFactors
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowMarkerNodes', dict(self.visualization)["showMarkerNodes"]

    def __repr__(self):
        return str(dict(self))
class VMarkerSuperElementRigid:
    def __init__(self, show = True, showMarkerNodes = True):
        self.show = show
        self.showMarkerNodes = showMarkerNodes

    def __iter__(self):
        yield 'show', self.show
        yield 'showMarkerNodes', self.showMarkerNodes

    def __repr__(self):
        return str(dict(self))
class MarkerSuperElementRigid:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), offset = [0.,0.,0.], meshNodeNumbers = [], weightingFactors = [], useAlternativeApproach = True, visualization = {'show': True, 'showMarkerNodes': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.offset = offset
        self.meshNodeNumbers = meshNodeNumbers
        self.weightingFactors = weightingFactors
        self.useAlternativeApproach = useAlternativeApproach
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'SuperElementRigid'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'offset', self.offset
        yield 'meshNodeNumbers', self.meshNodeNumbers
        yield 'weightingFactors', self.weightingFactors
        yield 'useAlternativeApproach', self.useAlternativeApproach
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VshowMarkerNodes', dict(self.visualization)["showMarkerNodes"]

    def __repr__(self):
        return str(dict(self))
class VMarkerKinematicTreeRigid:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerKinematicTreeRigid:
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), linkNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.linkNumber = CheckForValidUInt(linkNumber,"linkNumber","MarkerKinematicTreeRigid")
        self.localPosition = localPosition
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'KinematicTreeRigid'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'linkNumber', self.linkNumber
        yield 'localPosition', self.localPosition
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerObjectODE2Coordinates:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerObjectODE2Coordinates:
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'ObjectODE2Coordinates'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerBodyCable2DShape:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyCable2DShape:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), numberOfSegments = 3, verticalOffset = 0., visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.numberOfSegments = CheckForValidPInt(numberOfSegments,"numberOfSegments","MarkerBodyCable2DShape")
        self.verticalOffset = verticalOffset
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyCable2DShape'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'numberOfSegments', self.numberOfSegments
        yield 'verticalOffset', self.verticalOffset
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VMarkerBodyCable2DCoordinates:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class MarkerBodyCable2DCoordinates:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyCable2DCoordinates'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
#+++++++++++++++++++++++++++++++
#LOAD
class VLoadForceVector:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class LoadForceVector:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], bodyFixed = False, loadVectorUserFunction = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.loadVector = loadVector
        self.bodyFixed = bodyFixed
        self.loadVectorUserFunction = loadVectorUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'loadType', 'ForceVector'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'loadVector', self.loadVector
        yield 'bodyFixed', self.bodyFixed
        yield 'loadVectorUserFunction', self.loadVectorUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Force = LoadForceVector
VForce = VLoadForceVector

class VLoadTorqueVector:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class LoadTorqueVector:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], bodyFixed = False, loadVectorUserFunction = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.loadVector = loadVector
        self.bodyFixed = bodyFixed
        self.loadVectorUserFunction = loadVectorUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'loadType', 'TorqueVector'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'loadVector', self.loadVector
        yield 'bodyFixed', self.bodyFixed
        yield 'loadVectorUserFunction', self.loadVectorUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Torque = LoadTorqueVector
VTorque = VLoadTorqueVector

class VLoadMassProportional:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class LoadMassProportional:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), loadVector = [0.,0.,0.], loadVectorUserFunction = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.loadVector = loadVector
        self.loadVectorUserFunction = loadVectorUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'loadType', 'MassProportional'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'loadVector', self.loadVector
        yield 'loadVectorUserFunction', self.loadVectorUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
#add typedef for short usage:
Gravity = LoadMassProportional
VGravity = VLoadMassProportional

class VLoadCoordinate:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class LoadCoordinate:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), load = 0., loadUserFunction = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.load = load
        self.loadUserFunction = loadUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'loadType', 'Coordinate'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'load', self.load
        yield 'loadUserFunction', self.loadUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
#+++++++++++++++++++++++++++++++
#SENSOR
class VSensorNode:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorNode:
    def __init__(self, name = '', nodeNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'Node'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VSensorObject:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorObject:
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'Object'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VSensorBody:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorBody:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = localPosition
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'Body'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VSensorSuperElement:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorSuperElement:
    def __init__(self, name = '', bodyNumber = exudyn.InvalidIndex(), meshNodeNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.meshNodeNumber = CheckForValidUInt(meshNodeNumber,"meshNodeNumber","SensorSuperElement")
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'SuperElement'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'meshNodeNumber', self.meshNodeNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VSensorKinematicTree:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorKinematicTree:
    def __init__(self, name = '', objectNumber = exudyn.InvalidIndex(), linkNumber = exudyn.InvalidIndex(), localPosition = [0.,0.,0.], writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.objectNumber = objectNumber
        self.linkNumber = CheckForValidUInt(linkNumber,"linkNumber","SensorKinematicTree")
        self.localPosition = localPosition
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'KinematicTree'
        yield 'name', self.name
        yield 'objectNumber', self.objectNumber
        yield 'linkNumber', self.linkNumber
        yield 'localPosition', self.localPosition
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VSensorMarker:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorMarker:
    def __init__(self, name = '', markerNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', outputVariableType = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.storeInternal = storeInternal
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'Marker'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VSensorLoad:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorLoad:
    def __init__(self, name = '', loadNumber = exudyn.InvalidIndex(), writeToFile = True, fileName = '', storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.loadNumber = loadNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.storeInternal = storeInternal
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'Load'
        yield 'name', self.name
        yield 'loadNumber', self.loadNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
class VSensorUserFunction:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

    def __repr__(self):
        return str(dict(self))
class SensorUserFunction:
    def __init__(self, name = '', sensorNumbers = [], factors = [], writeToFile = True, fileName = '', sensorUserFunction = 0, storeInternal = False, visualization = {'show': True}):
        self.name = name
        self.sensorNumbers = sensorNumbers
        self.factors = factors
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.sensorUserFunction = sensorUserFunction
        self.storeInternal = storeInternal
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'UserFunction'
        yield 'name', self.name
        yield 'sensorNumbers', self.sensorNumbers
        yield 'factors', self.factors
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'sensorUserFunction', self.sensorUserFunction
        yield 'storeInternal', self.storeInternal
        yield 'Vshow', dict(self.visualization)["show"]

    def __repr__(self):
        return str(dict(self))
