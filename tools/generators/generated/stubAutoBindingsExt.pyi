from exudyn.interactive import SolutionViewer as _SolutionViewer
from exudyn.misc.mainSystemExtensions import MainSystemCreateGround as _MainSystemCreateGround
from exudyn.misc.mainSystemExtensions import MainSystemCreateMassPoint as _MainSystemCreateMassPoint
from exudyn.misc.mainSystemExtensions import MainSystemCreateRigidBody as _MainSystemCreateRigidBody
from exudyn.misc.mainSystemExtensions import MainSystemCreateSpringDamper as _MainSystemCreateSpringDamper
from exudyn.misc.mainSystemExtensions import MainSystemCreateCartesianSpringDamper as _MainSystemCreateCartesianSpringDamper
from exudyn.misc.mainSystemExtensions import MainSystemCreateRigidBodySpringDamper as _MainSystemCreateRigidBodySpringDamper
from exudyn.misc.mainSystemExtensions import MainSystemCreateTorsionalSpringDamper as _MainSystemCreateTorsionalSpringDamper
from exudyn.misc.mainSystemExtensions import MainSystemCreateRevoluteJoint as _MainSystemCreateRevoluteJoint
from exudyn.misc.mainSystemExtensions import MainSystemCreatePrismaticJoint as _MainSystemCreatePrismaticJoint
from exudyn.misc.mainSystemExtensions import MainSystemCreateSphericalJoint as _MainSystemCreateSphericalJoint
from exudyn.misc.mainSystemExtensions import MainSystemCreateGenericJoint as _MainSystemCreateGenericJoint
from exudyn.misc.mainSystemExtensions import MainSystemCreateDistanceConstraint as _MainSystemCreateDistanceConstraint
from exudyn.misc.mainSystemExtensions import MainSystemCreateCoordinateConstraint as _MainSystemCreateCoordinateConstraint
from exudyn.misc.mainSystemExtensions import MainSystemCreateRollingDisc as _MainSystemCreateRollingDisc
from exudyn.misc.mainSystemExtensions import MainSystemCreateRollingDiscPenalty as _MainSystemCreateRollingDiscPenalty
from exudyn.misc.mainSystemExtensions import MainSystemCreateSphereSphereContact as _MainSystemCreateSphereSphereContact
from exudyn.misc.mainSystemExtensions import MainSystemCreateSphereQuadContact as _MainSystemCreateSphereQuadContact
from exudyn.misc.mainSystemExtensions import MainSystemCreateSphereTriangleContact as _MainSystemCreateSphereTriangleContact
from exudyn.misc.mainSystemExtensions import MainSystemCreateKinematicTree as _MainSystemCreateKinematicTree
from exudyn.misc.mainSystemExtensions import MainSystemCreateFFRFReducedOrderObject as _MainSystemCreateFFRFReducedOrderObject
from exudyn.misc.mainSystemExtensions import MainSystemCreateForce as _MainSystemCreateForce
from exudyn.misc.mainSystemExtensions import MainSystemCreateTorque as _MainSystemCreateTorque
from exudyn.misc.mainSystemExtensions import CreateDistanceSensorGeometry as _CreateDistanceSensorGeometry
from exudyn.misc.mainSystemExtensions import CreateDistanceSensor as _CreateDistanceSensor
from exudyn.misc.mainSystemExtensions import DrawSystemGraph as _DrawSystemGraph
from exudyn.plot import PlotSensor as _PlotSensor
from exudyn.solver import SolveStatic as _SolveStatic
from exudyn.solver import SolveDynamic as _SolveDynamic
from exudyn.solver import ComputeLinearizedSystem as _ComputeLinearizedSystem
from exudyn.solver import ComputeODE2Eigenvalues as _ComputeODE2Eigenvalues
from exudyn.solver import ComputeSystemDegreeOfFreedom as _ComputeSystemDegreeOfFreedom

class MainSystem:
    SolutionViewer = _SolutionViewer
    CreateGround = _MainSystemCreateGround
    CreateMassPoint = _MainSystemCreateMassPoint
    CreateRigidBody = _MainSystemCreateRigidBody
    CreateSpringDamper = _MainSystemCreateSpringDamper
    CreateCartesianSpringDamper = _MainSystemCreateCartesianSpringDamper
    CreateRigidBodySpringDamper = _MainSystemCreateRigidBodySpringDamper
    CreateTorsionalSpringDamper = _MainSystemCreateTorsionalSpringDamper
    CreateRevoluteJoint = _MainSystemCreateRevoluteJoint
    CreatePrismaticJoint = _MainSystemCreatePrismaticJoint
    CreateSphericalJoint = _MainSystemCreateSphericalJoint
    CreateGenericJoint = _MainSystemCreateGenericJoint
    CreateDistanceConstraint = _MainSystemCreateDistanceConstraint
    CreateCoordinateConstraint = _MainSystemCreateCoordinateConstraint
    CreateRollingDisc = _MainSystemCreateRollingDisc
    CreateRollingDiscPenalty = _MainSystemCreateRollingDiscPenalty
    CreateSphereSphereContact = _MainSystemCreateSphereSphereContact
    CreateSphereQuadContact = _MainSystemCreateSphereQuadContact
    CreateSphereTriangleContact = _MainSystemCreateSphereTriangleContact
    CreateKinematicTree = _MainSystemCreateKinematicTree
    CreateFFRFReducedOrderObject = _MainSystemCreateFFRFReducedOrderObject
    CreateForce = _MainSystemCreateForce
    CreateTorque = _MainSystemCreateTorque
    CreateDistanceSensorGeometry = _CreateDistanceSensorGeometry
    CreateDistanceSensor = _CreateDistanceSensor
    DrawSystemGraph = _DrawSystemGraph
    PlotSensor = _PlotSensor
    SolveStatic = _SolveStatic
    SolveDynamic = _SolveDynamic
    ComputeLinearizedSystem = _ComputeLinearizedSystem
    ComputeODE2Eigenvalues = _ComputeODE2Eigenvalues
    ComputeSystemDegreeOfFreedom = _ComputeSystemDegreeOfFreedom
