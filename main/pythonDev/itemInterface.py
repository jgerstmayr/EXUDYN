#item interface diagonal matrix creator
def IIDiagMatrix(rowsColumns, value):
    m = []
    for i in range(rowsColumns):
        m += [rowsColumns*[0]]
        m[i][i] = value
    return m

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

class NodePoint:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.], initialDisplacements = [0.,0.,0.], initialVelocities = [0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialDisplacements = initialDisplacements
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'Point'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialDisplacements', self.initialDisplacements
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
Point = NodePoint

class VNodePoint2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class NodePoint2D:
    def __init__(self, name = '', referenceCoordinates = [0.,0.], initialDisplacements = [0.,0.], initialVelocities = [0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialDisplacements = initialDisplacements
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'Point2D'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialDisplacements', self.initialDisplacements
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
Point2D = NodePoint2D

class VNodeRigidBodyEP:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class NodeRigidBodyEP:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.,0.], initialDisplacements = [0.,0.,0., 0.,0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialDisplacements = initialDisplacements
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBodyEP'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialDisplacements', self.initialDisplacements
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
RigidEP = NodeRigidBodyEP

class VNodeRigidBodyRxyz:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class NodeRigidBodyRxyz:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.], initialDisplacements = [0.,0.,0., 0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialDisplacements = initialDisplacements
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBodyRxyz'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialDisplacements', self.initialDisplacements
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
RigidRxyz = NodeRigidBodyRxyz

class VNodeRigidBodyRotVecLG:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class NodeRigidBodyRotVecLG:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0., 0.,0.,0.], initialDisplacements = [0.,0.,0., 0.,0.,0.], initialVelocities = [0.,0.,0., 0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialDisplacements = initialDisplacements
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBodyRotVecLG'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialDisplacements', self.initialDisplacements
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
RigidRotVecLG = NodeRigidBodyRotVecLG

class VNodeRigidBody2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class NodeRigidBody2D:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,0.], initialDisplacements = [0.,0.,0.], initialVelocities = [0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialDisplacements = initialDisplacements
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'RigidBody2D'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialDisplacements', self.initialDisplacements
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
Rigid2D = NodeRigidBody2D

class VNodePoint2DSlope1:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class NodePoint2DSlope1:
    def __init__(self, name = '', referenceCoordinates = [0.,0.,1.,0.], initialDisplacements = [0.,0.,0.,0.], initialVelocities = [0.,0.,0.,0.], visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialDisplacements = initialDisplacements
        self.initialVelocities = initialVelocities
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'Point2DSlope1'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialDisplacements', self.initialDisplacements
        yield 'initialVelocities', self.initialVelocities
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
Point2DS1 = NodePoint2DSlope1

class VNodeGenericODE2:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class NodeGenericODE2:
    def __init__(self, name = '', referenceCoordinates = [], initialCoordinates = [], initialCoordinates_t = [], numberOfODE2Coordinates = 0, visualization = {'show': False}):
        self.name = name
        self.referenceCoordinates = referenceCoordinates
        self.initialCoordinates = initialCoordinates
        self.initialCoordinates_t = initialCoordinates_t
        self.numberOfODE2Coordinates = numberOfODE2Coordinates
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'GenericODE2'
        yield 'name', self.name
        yield 'referenceCoordinates', self.referenceCoordinates
        yield 'initialCoordinates', self.initialCoordinates
        yield 'initialCoordinates_t', self.initialCoordinates_t
        yield 'numberOfODE2Coordinates', self.numberOfODE2Coordinates
        yield 'Vshow', dict(self.visualization)["show"]

class VNodeGenericData:
    def __init__(self, show = False):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class NodeGenericData:
    def __init__(self, name = '', initialCoordinates = [], numberOfDataCoordinates = 0, visualization = {'show': False}):
        self.name = name
        self.initialCoordinates = initialCoordinates
        self.numberOfDataCoordinates = numberOfDataCoordinates
        self.visualization = visualization

    def __iter__(self):
        yield 'nodeType', 'GenericData'
        yield 'name', self.name
        yield 'initialCoordinates', self.initialCoordinates
        yield 'numberOfDataCoordinates', self.numberOfDataCoordinates
        yield 'Vshow', dict(self.visualization)["show"]

class VNodePointGround:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

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

#add typedef for short usage:
PointGround = NodePointGround

#+++++++++++++++++++++++++++++++
#OBJECT
class VObjectMassPoint:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

class ObjectMassPoint:
    def __init__(self, name = '', physicsMass = 0., nodeNumber = -1, visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = physicsMass
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'MassPoint'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

#add typedef for short usage:
MassPoint = ObjectMassPoint

class VObjectMassPoint2D:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

class ObjectMassPoint2D:
    def __init__(self, name = '', physicsMass = 0., nodeNumber = -1, visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = physicsMass
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'MassPoint2D'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

#add typedef for short usage:
MassPoint2D = ObjectMassPoint2D

class VObjectRigidBody:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

class ObjectRigidBody:
    def __init__(self, name = '', physicsMass = 0., physicsInertia = [0.,0.,0., 0.,0.,0.], nodeNumber = -1, visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = physicsMass
        self.physicsInertia = physicsInertia
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'RigidBody'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'physicsInertia', self.physicsInertia
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

#add typedef for short usage:
RigidBody = ObjectRigidBody

class VObjectRigidBody2D:
    def __init__(self, show = True, graphicsData = []):
        self.show = show
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'graphicsData', self.graphicsData

class ObjectRigidBody2D:
    def __init__(self, name = '', physicsMass = 0., physicsInertia = 0., nodeNumber = -1, visualization = {'show': True, 'graphicsData': []}):
        self.name = name
        self.physicsMass = physicsMass
        self.physicsInertia = physicsInertia
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'RigidBody2D'
        yield 'name', self.name
        yield 'physicsMass', self.physicsMass
        yield 'physicsInertia', self.physicsInertia
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

#add typedef for short usage:
RigidBody2D = ObjectRigidBody2D

class VObjectANCFCable2D:
    def __init__(self, show = True, drawHeight = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawHeight = drawHeight
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawHeight', self.drawHeight
        yield 'color', self.color

class ObjectANCFCable2D:
    def __init__(self, name = '', physicsLength = 0., physicsMassPerLength = 0., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsReferenceAxialStrain = 0., physicsReferenceCurvature = 0., nodeNumbers = [-1, -1], useReducedOrderIntegration = False, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsLength = physicsLength
        self.physicsMassPerLength = physicsMassPerLength
        self.physicsBendingStiffness = physicsBendingStiffness
        self.physicsAxialStiffness = physicsAxialStiffness
        self.physicsBendingDamping = physicsBendingDamping
        self.physicsAxialDamping = physicsAxialDamping
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.physicsReferenceCurvature = physicsReferenceCurvature
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
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
Cable2D = ObjectANCFCable2D

class VObjectALEANCFCable2D:
    def __init__(self, show = True, drawHeight = 0., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawHeight = drawHeight
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawHeight', self.drawHeight
        yield 'color', self.color

class ObjectALEANCFCable2D:
    def __init__(self, name = '', physicsLength = 0., physicsMassPerLength = 0., physicsMovingMassFactor = 1., physicsBendingStiffness = 0., physicsAxialStiffness = 0., physicsBendingDamping = 0., physicsAxialDamping = 0., physicsReferenceAxialStrain = 0., physicsReferenceCurvature = 0., physicsUseCouplingTerms = True, nodeNumbers = [-1, -1, -1], useReducedOrderIntegration = False, visualization = {'show': True, 'drawHeight': 0., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.physicsLength = physicsLength
        self.physicsMassPerLength = physicsMassPerLength
        self.physicsMovingMassFactor = physicsMovingMassFactor
        self.physicsBendingStiffness = physicsBendingStiffness
        self.physicsAxialStiffness = physicsAxialStiffness
        self.physicsBendingDamping = physicsBendingDamping
        self.physicsAxialDamping = physicsAxialDamping
        self.physicsReferenceAxialStrain = physicsReferenceAxialStrain
        self.physicsReferenceCurvature = physicsReferenceCurvature
        self.physicsUseCouplingTerms = physicsUseCouplingTerms
        self.nodeNumbers = nodeNumbers
        self.useReducedOrderIntegration = useReducedOrderIntegration
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
        yield 'nodeNumbers', self.nodeNumbers
        yield 'useReducedOrderIntegration', self.useReducedOrderIntegration
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawHeight', dict(self.visualization)["drawHeight"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
ALECable2D = ObjectALEANCFCable2D

class VObjectGround:
    def __init__(self, show = True, color = [-1.,-1.,-1.,-1.], graphicsData = []):
        self.show = show
        self.color = color
        self.graphicsData = graphicsData

    def __iter__(self):
        yield 'show', self.show
        yield 'color', self.color
        yield 'graphicsData', self.graphicsData

class ObjectGround:
    def __init__(self, name = '', referencePosition = [0.,0.,0.], visualization = {'show': True, 'color': [-1.,-1.,-1.,-1.], 'graphicsData': []}):
        self.name = name
        self.referencePosition = referencePosition
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'Ground'
        yield 'name', self.name
        yield 'referencePosition', self.referencePosition
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'Vcolor', dict(self.visualization)["color"]
        yield 'VgraphicsData', dict(self.visualization)["graphicsData"]

class VObjectConnectorSpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectConnectorSpringDamper:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], referenceLength = 0., stiffness = 0., damping = 0., force = 0., activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.referenceLength = referenceLength
        self.stiffness = stiffness
        self.damping = damping
        self.force = force
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
        yield 'activeConnector', self.activeConnector
        yield 'springForceUserFunction', self.springForceUserFunction
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
SpringDamper = ObjectConnectorSpringDamper

class VObjectConnectorCartesianSpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectConnectorCartesianSpringDamper:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], stiffness = [0.,0.,0.], damping = [0.,0.,0.], offset = [0.,0.,0.], activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.stiffness = stiffness
        self.damping = damping
        self.offset = offset
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorCartesianSpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
CartesianSpringDamper = ObjectConnectorCartesianSpringDamper

class VObjectConnectorRigidBodySpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectConnectorRigidBodySpringDamper:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], stiffness = IIDiagMatrix(rowsColumns=6,value=0.), damping = IIDiagMatrix(rowsColumns=6,value=0.), rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), offset = [0.,0.,0.,0.,0.,0.], activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.stiffness = stiffness
        self.damping = damping
        self.rotationMarker0 = rotationMarker0
        self.rotationMarker1 = rotationMarker1
        self.offset = offset
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'ConnectorRigidBodySpringDamper'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'stiffness', self.stiffness
        yield 'damping', self.damping
        yield 'rotationMarker0', self.rotationMarker0
        yield 'rotationMarker1', self.rotationMarker1
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
RigidBodySpringDamper = ObjectConnectorRigidBodySpringDamper

class VObjectConnectorCoordinateSpringDamper:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectConnectorCoordinateSpringDamper:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], stiffness = 0., damping = 0., offset = 0., dryFriction = 0., dryFrictionProportionalZone = 0., activeConnector = True, springForceUserFunction = 0, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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

#add typedef for short usage:
CoordinateSpringDamper = ObjectConnectorCoordinateSpringDamper

class VObjectConnectorDistance:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectConnectorDistance:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], distance = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.distance = distance
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

#add typedef for short usage:
DistanceConstraint = ObjectConnectorDistance

class VObjectConnectorCoordinate:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectConnectorCoordinate:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], offset = 0., factorValue1 = 1., velocityLevel = False, offsetUserFunction = 0, offsetUserFunction_t = 0, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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

#add typedef for short usage:
CoordinateConstraint = ObjectConnectorCoordinate

class VObjectContactCoordinate:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectContactCoordinate:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], nodeNumber = -1, contactStiffness = 0., contactDamping = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
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

class VObjectContactCircleCable2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectContactCircleCable2D:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], nodeNumber = -1, numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., circleRadius = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = numberOfContactSegments
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.circleRadius = circleRadius
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
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

class VObjectContactFrictionCircleCable2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectContactFrictionCircleCable2D:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], nodeNumber = -1, numberOfContactSegments = 3, contactStiffness = 0., contactDamping = 0., frictionVelocityPenalty = 0., frictionStiffness = 0., frictionCoefficient = 0., circleRadius = 0., offset = 0., activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.nodeNumber = nodeNumber
        self.numberOfContactSegments = numberOfContactSegments
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.frictionVelocityPenalty = frictionVelocityPenalty
        self.frictionStiffness = frictionStiffness
        self.frictionCoefficient = frictionCoefficient
        self.circleRadius = circleRadius
        self.offset = offset
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
        yield 'offset', self.offset
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

class VObjectJointSliding2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectJointSliding2D:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], slidingMarkerNumbers = [], slidingMarkerOffsets = [], nodeNumber = -1, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.slidingMarkerNumbers = slidingMarkerNumbers
        self.slidingMarkerOffsets = slidingMarkerOffsets
        self.nodeNumber = nodeNumber
        self.activeConnector = activeConnector
        self.visualization = visualization

    def __iter__(self):
        yield 'objectType', 'JointSliding2D'
        yield 'name', self.name
        yield 'markerNumbers', self.markerNumbers
        yield 'slidingMarkerNumbers', self.slidingMarkerNumbers
        yield 'slidingMarkerOffsets', self.slidingMarkerOffsets
        yield 'nodeNumber', self.nodeNumber
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
SlidingJoint2D = ObjectJointSliding2D

class VObjectJointALEMoving2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectJointALEMoving2D:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], slidingMarkerNumbers = [], slidingMarkerOffsets = [], slidingOffset = 0., nodeNumbers = [ -1, -1 ], activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.slidingMarkerNumbers = slidingMarkerNumbers
        self.slidingMarkerOffsets = slidingMarkerOffsets
        self.slidingOffset = slidingOffset
        self.nodeNumbers = nodeNumbers
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
        yield 'activeConnector', self.activeConnector
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VdrawSize', dict(self.visualization)["drawSize"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
ALEMovingJoint2D = ObjectJointALEMoving2D

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

class ObjectJointGeneric:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], constrainedAxes = [1,1,1,1,1,1], rotationMarker0 = IIDiagMatrix(rowsColumns=3,value=1), rotationMarker1 = IIDiagMatrix(rowsColumns=3,value=1), activeConnector = True, forceTorqueUserFunctionParameters = [0.,0.,0.,0.,0.,0.], offsetUserFunctionParameters = [0.,0.,0.,0.,0.,0.], forceTorqueUserFunction = 0, offsetUserFunction = 0, offsetUserFunction_t = 0, visualization = {'show': True, 'axesRadius': 0.1, 'axesLength': 0.4, 'color': [-1.,-1.,-1.,-1.]}):
        self.name = name
        self.markerNumbers = markerNumbers
        self.constrainedAxes = constrainedAxes
        self.rotationMarker0 = rotationMarker0
        self.rotationMarker1 = rotationMarker1
        self.activeConnector = activeConnector
        self.forceTorqueUserFunctionParameters = forceTorqueUserFunctionParameters
        self.offsetUserFunctionParameters = offsetUserFunctionParameters
        self.forceTorqueUserFunction = forceTorqueUserFunction
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
        yield 'forceTorqueUserFunctionParameters', self.forceTorqueUserFunctionParameters
        yield 'offsetUserFunctionParameters', self.offsetUserFunctionParameters
        yield 'forceTorqueUserFunction', self.forceTorqueUserFunction
        yield 'offsetUserFunction', self.offsetUserFunction
        yield 'offsetUserFunction_t', self.offsetUserFunction_t
        yield 'Vshow', dict(self.visualization)["show"]
        yield 'VaxesRadius', dict(self.visualization)["axesRadius"]
        yield 'VaxesLength', dict(self.visualization)["axesLength"]
        yield 'Vcolor', dict(self.visualization)["color"]

#add typedef for short usage:
GenericJoint = ObjectJointGeneric

class VObjectJointRevolute2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectJointRevolute2D:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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

#add typedef for short usage:
RevoluteJoint2D = ObjectJointRevolute2D

class VObjectJointPrismatic2D:
    def __init__(self, show = True, drawSize = -1., color = [-1.,-1.,-1.,-1.]):
        self.show = show
        self.drawSize = drawSize
        self.color = color

    def __iter__(self):
        yield 'show', self.show
        yield 'drawSize', self.drawSize
        yield 'color', self.color

class ObjectJointPrismatic2D:
    def __init__(self, name = '', markerNumbers = [ -1, -1 ], axisMarker0 = [1.,0.,0.], normalMarker1 = [0.,1.,0.], constrainRotation = True, activeConnector = True, visualization = {'show': True, 'drawSize': -1., 'color': [-1.,-1.,-1.,-1.]}):
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

#add typedef for short usage:
PrismaticJoint2D = ObjectJointPrismatic2D

#+++++++++++++++++++++++++++++++
#MARKER
class VMarkerBodyMass:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class MarkerBodyMass:
    def __init__(self, name = '', bodyNumber = -1, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyMass'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'Vshow', dict(self.visualization)["show"]

class VMarkerBodyPosition:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class MarkerBodyPosition:
    def __init__(self, name = '', bodyNumber = -1, localPosition = [0.,0.,0.], bodyFixed = False, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = localPosition
        self.bodyFixed = bodyFixed
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyPosition'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'bodyFixed', self.bodyFixed
        yield 'Vshow', dict(self.visualization)["show"]

class VMarkerBodyRigid:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class MarkerBodyRigid:
    def __init__(self, name = '', bodyNumber = -1, localPosition = [0.,0.,0.], bodyFixed = False, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = localPosition
        self.bodyFixed = bodyFixed
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyRigid'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'bodyFixed', self.bodyFixed
        yield 'Vshow', dict(self.visualization)["show"]

class VMarkerNodePosition:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class MarkerNodePosition:
    def __init__(self, name = '', nodeNumber = -1, visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodePosition'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]

class VMarkerNodeRigid:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class MarkerNodeRigid:
    def __init__(self, name = '', nodeNumber = -1, visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodeRigid'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'Vshow', dict(self.visualization)["show"]

class VMarkerNodeCoordinate:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class MarkerNodeCoordinate:
    def __init__(self, name = '', nodeNumber = -1, coordinate = -1, visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.coordinate = coordinate
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'NodeCoordinate'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'coordinate', self.coordinate
        yield 'Vshow', dict(self.visualization)["show"]

class VMarkerBodyCable2DShape:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class MarkerBodyCable2DShape:
    def __init__(self, name = '', bodyNumber = -1, numberOfSegments = 3, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.numberOfSegments = numberOfSegments
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyCable2DShape'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'numberOfSegments', self.numberOfSegments
        yield 'Vshow', dict(self.visualization)["show"]

class VMarkerBodyCable2DCoordinates:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class MarkerBodyCable2DCoordinates:
    def __init__(self, name = '', bodyNumber = -1, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.visualization = visualization

    def __iter__(self):
        yield 'markerType', 'BodyCable2DCoordinates'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'Vshow', dict(self.visualization)["show"]

#+++++++++++++++++++++++++++++++
#LOAD
class VLoadForceVector:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class LoadForceVector:
    def __init__(self, name = '', markerNumber = -1, loadVector = [0.,0.,0.], loadVectorUserFunction = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.loadVector = loadVector
        self.loadVectorUserFunction = loadVectorUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'loadType', 'ForceVector'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'loadVector', self.loadVector
        yield 'loadVectorUserFunction', self.loadVectorUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

#add typedef for short usage:
Force = LoadForceVector

class VLoadTorqueVector:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class LoadTorqueVector:
    def __init__(self, name = '', markerNumber = -1, loadVector = [0.,0.,0.], loadVectorUserFunction = 0, visualization = {'show': True}):
        self.name = name
        self.markerNumber = markerNumber
        self.loadVector = loadVector
        self.loadVectorUserFunction = loadVectorUserFunction
        self.visualization = visualization

    def __iter__(self):
        yield 'loadType', 'TorqueVector'
        yield 'name', self.name
        yield 'markerNumber', self.markerNumber
        yield 'loadVector', self.loadVector
        yield 'loadVectorUserFunction', self.loadVectorUserFunction
        yield 'Vshow', dict(self.visualization)["show"]

#add typedef for short usage:
Torque = LoadTorqueVector

class VLoadMassProportional:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class LoadMassProportional:
    def __init__(self, name = '', markerNumber = -1, loadVector = [0.,0.,0.], loadVectorUserFunction = 0, visualization = {'show': True}):
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

#add typedef for short usage:
Gravity = LoadMassProportional

class VLoadCoordinate:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class LoadCoordinate:
    def __init__(self, name = '', markerNumber = -1, load = 0., loadUserFunction = 0, visualization = {'show': True}):
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

#+++++++++++++++++++++++++++++++
#SENSOR
class VSensorNode:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class SensorNode:
    def __init__(self, name = '', nodeNumber = -1, writeToFile = True, fileName = '', outputVariableType = 0, visualization = {'show': True}):
        self.name = name
        self.nodeNumber = nodeNumber
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'Node'
        yield 'name', self.name
        yield 'nodeNumber', self.nodeNumber
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'Vshow', dict(self.visualization)["show"]

class VSensorBody:
    def __init__(self, show = True):
        self.show = show

    def __iter__(self):
        yield 'show', self.show

class SensorBody:
    def __init__(self, name = '', bodyNumber = -1, localPosition = [0.,0.,0.], writeToFile = True, fileName = '', outputVariableType = 0, visualization = {'show': True}):
        self.name = name
        self.bodyNumber = bodyNumber
        self.localPosition = localPosition
        self.writeToFile = writeToFile
        self.fileName = fileName
        self.outputVariableType = outputVariableType
        self.visualization = visualization

    def __iter__(self):
        yield 'sensorType', 'Body'
        yield 'name', self.name
        yield 'bodyNumber', self.bodyNumber
        yield 'localPosition', self.localPosition
        yield 'writeToFile', self.writeToFile
        yield 'fileName', self.fileName
        yield 'outputVariableType', self.outputVariableType
        yield 'Vshow', dict(self.visualization)["show"]

