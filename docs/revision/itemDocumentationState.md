<!-- written by tools/itemDocumentationReport.py - run it again rather than editing this -->

| kind | items | no equations text | no figure | no MiniExample | parameters without a real description | output variables without one | used in no script |
|---|---|---|---|---|---|---|---|
| Node | 16 | 9 | 16 | 16 | 0 of 101 | 0 of 137 | 3 |
| Object (Body) | 7 | 0 | 6 | 2 | 0 of 46 | 0 of 59 | 0 |
| Object (SuperElement) | 4 | 0 | 3 | 2 | 2 of 122 | 0 of 16 | 0 |
| Object (Object) | 1 | 0 | 1 | 0 | 0 of 9 | 0 of 3 | 0 |
| Object (FiniteElement) | 7 | 1 | 7 | 4 | 0 of 104 | 0 of 66 | 2 |
| Object (Connector) | 19 | 1 | 14 | 12 | 0 of 309 | 0 of 74 | 1 |
| Object (Constraint) | 3 | 0 | 3 | 1 | 0 of 31 | 0 of 12 | 0 |
| Object (Joint) | 10 | 1 | 5 | 9 | 0 of 100 | 0 of 46 | 0 |
| Marker | 18 | 10 | 17 | 17 | 1 of 81 | 0 of 0 | 1 |
| Load | 4 | 0 | 4 | 3 | 0 of 22 | 0 of 0 | 0 |
| Sensor | 8 | 7 | 8 | 8 | 0 of 60 | 0 of 0 | 0 |
| **all** | 97 | 29 | 84 | 74 | 3 of 985 | 0 of 413 | 7 |

| item | kind | class description (words) | equations (words, sections) | figure | parameters: without a real description | output variables (undescribed) | MiniExample | used in scripts |
|---|---|---|---|---|---|---|---|---|
| NodePoint | Node | 18 | 88, 0 | - | 0 of 7 | 12 | - | 40 |
| NodePoint2D | Node | 18 | 73, 0 | - | 0 of 7 | 12 | - | 19 |
| NodeRigidBodyEP | Node | 30 | 165, 0 | - | 0 of 8 | 13 | - | 108 |
| NodeRigidBodyRxyz | Node | 35 | 121, 0 | - | 0 of 7 | 13 | - | 6 |
| NodeRigidBodyRotVecLG | Node | 36 | 236, 0 | - | 0 of 7 | 11 | - | 3 |
| NodeRigidBody2D | Node | 33 | 77, 0 | - | 0 of 7 | 13 | - | 51 |
| Node1D | Node | 58 | 38, 0 | - | 0 of 5 | 4 | - | 10 |
| NodePoint2DSlope1 | Node | 66 | - | - | 0 of 7 | 8 | - | 18 |
| NodePointSlope1 | Node | 66 | - | - | 0 of 7 | 8 | - | 0 |
| NodePointSlope12 | Node | 65 | - | - | 0 of 7 | 12 | - | 0 |
| NodePointSlope23 | Node | 65 | - | - | 0 of 7 | 12 | - | 5 |
| NodeGenericODE2 | Node | 39 | - | - | 0 of 6 | 4 | - | 15 |
| NodeGenericODE1 | Node | 27 | - | - | 0 of 5 | 3 | - | 8 |
| NodeGenericAE | Node | 24 | - | - | 0 of 5 | 1 | - | 0 |
| NodeGenericData | Node | 19 | - | - | 0 of 4 | 1 | - | 54 |
| NodePointGround | Node | 49 | - | - | 0 of 5 | 10 | - | 129 |
| ObjectGround | Object (Body) | 27 | 93, 1 | - | 0 of 6 | 5 | - | 191 |
| ObjectMassPoint | Object (Body) | 11 | 132, 2 | - | 0 of 5 | 8 | yes | 35 |
| ObjectMassPoint2D | Object (Body) | 9 | 134, 2 | - | 0 of 5 | 8 | yes | 19 |
| ObjectMass1D | Object (Body) | 21 | 151, 2 | - | 0 of 7 | 7 | yes | 9 |
| ObjectRotationalMass1D | Object (Body) | 8 | 172, 2 | - | 0 of 7 | 7 | yes | 3 |
| ObjectRigidBody | Object (Body) | 101 | 850, 5 | yes | 0 of 8 | 12 | - | 109 |
| ObjectRigidBody2D | Object (Body) | 20 | 272, 2 | - | 0 of 8 | 12 | yes | 46 |
| ObjectGenericODE2 | Object (SuperElement) | 71 | 243, 3 | - | 0 of 18 | 5 | yes | 12 |
| ObjectGenericODE1 | Object (Object) | 48 | 250, 2 | - | 0 of 9 | 3 | yes | 5 |
| ObjectKinematicTree | Object (SuperElement) | 154 | 1090, 5 | - | 0 of 40 | 4 | yes | 5 |
| ObjectFFRF | Object (SuperElement) | 63 | 849, 6 | - | 2 of 27: `tempVector`, `tempMatrix` | 4 | - | 2 |
| ObjectFFRFreducedOrder | Object (SuperElement) | 88 | 1224, 6 | yes | 0 of 37 | 3 | - | 6 |
| ObjectANCFCable | Object (FiniteElement) | 64 | - | - | 0 of 14 | 9 | yes | 0 |
| ObjectANCFCable2D | Object (FiniteElement) | 39 | 991, 6 | - | 0 of 17 | 13 | yes | 22 |
| ObjectALEANCFCable2D | Object (FiniteElement) | 66 | 225, 0 | - | 0 of 18 | 10 | - | 5 |
| ObjectANCFBeam | Object (FiniteElement) | 57 | 4, 0 | - | 0 of 15 | 9 | - | 5 |
| ObjectBeamGeometricallyExact2D | Object (FiniteElement) | 107 | 11, 0 | - | 0 of 16 | 8 | - | 5 |
| ObjectBeamGeometricallyExact | Object (FiniteElement) | 44 | 4, 0 | - | 0 of 11 | 6 | - | 2 |
| ObjectANCFThinPlate | Object (FiniteElement) | 57 | 15, 0 | - | 0 of 13 | 11 | yes | 0 |
| ObjectConnectorSpringDamper | Object (Connector) | 13 | 329, 3 | - | 0 of 12 | 5 | yes | 27 |
| ObjectConnectorCartesianSpringDamper | Object (Connector) | 21 | 292, 2 | - | 0 of 10 | 4 | yes | 48 |
| ObjectConnectorRigidBodySpringDamper | Object (Connector) | 28 | 204, 2 | - | 0 of 15 | 6 | yes | 9 |
| ObjectConnectorLinearSpringDamper | Object (Connector) | 67 | 160, 2 | - | 0 of 14 | 3 | yes | 1 |
| ObjectConnectorTorsionalSpringDamper | Object (Connector) | 91 | 136, 2 | - | 0 of 15 | 3 | yes | 9 |
| ObjectConnectorCoordinateSpringDamper | Object (Connector) | 48 | 211, 2 | - | 0 of 10 | 3 | yes | 39 |
| ObjectConnectorCoordinateSpringDamperExt | Object (Connector) | 93 | 628, 2 | - | 0 of 26 | 3 | - | 5 |
| ObjectConnectorGravity | Object (Connector) | 48 | 175, 2 | - | 0 of 10 | 3 | yes | 1 |
| ObjectConnectorHydraulicActuatorSimple | Object (Connector) | 43 | 475, 3 | - | 0 of 30 | 5 | - | 3 |
| ObjectConnectorReevingSystemSprings | Object (Connector) | 91 | 674, 4 | yes | 0 of 17 | 3 | - | 2 |
| ObjectConnectorDistance | Object (Constraint) | 11 | 98, 2 | - | 0 of 7 | 4 | yes | 15 |
| ObjectConnectorCoordinate | Object (Constraint) | 45 | 188, 2 | - | 0 of 11 | 4 | yes | 80 |
| ObjectConnectorCoordinateVector | Object (Constraint) | 33 | 429, 3 | - | 0 of 13 | 4 | - | 3 |
| ObjectConnectorRollingDiscPenalty | Object (Connector) | 90 | 544, 4 | yes | 0 of 18 | 5 | - | 12 |
| ObjectContactConvexRoll | Object (Connector) | 71 | 550, 2 | yes | 0 of 19 | 4 | - | 1 |
| ObjectContactCoordinate | Object (Connector) | 48 | - | - | 0 of 10 | 0 | - | 4 |
| ObjectContactCircleCable2D | Object (Connector) | 88 | 24, 1 | - | 0 of 13 | 0 | - | 7 |
| ObjectContactFrictionCircleCable2D | Object (Connector) | 89 | 2057, 7 | yes | 0 of 16 | 3 | - | 8 |
| ObjectContactSphereSphere | Object (Connector) | 55 | 1040, 2 | yes | 0 of 20 | 7 | - | 7 |
| ObjectContactSphereTorus | Object (Connector) | 29 | 107, 2 | - | 0 of 18 | 8 | - | 0 |
| ObjectContactSphereTriangle | Object (Connector) | 28 | 107, 2 | - | 0 of 17 | 6 | - | 2 |
| ObjectContactCurveCircles | Object (Connector) | 38 | 58, 2 | - | 0 of 19 | 3 | - | 5 |
| ObjectJointGeneric | Object (Joint) | 43 | 370, 2 | yes | 0 of 14 | 8 | - | 61 |
| ObjectJointRevoluteZ | Object (Joint) | 85 | 246, 2 | yes | 0 of 9 | 8 | yes | 25 |
| ObjectJointPrismaticX | Object (Joint) | 80 | 240, 2 | yes | 0 of 9 | 8 | - | 1 |
| ObjectJointSpherical | Object (Joint) | 18 | 223, 3 | yes | 0 of 7 | 4 | - | 25 |
| ObjectJointRollingDisc | Object (Joint) | 131 | 406, 3 | - | 0 of 10 | 4 | - | 7 |
| ObjectJointRevolute2D | Object (Joint) | 15 | - | - | 0 of 6 | 0 | - | 42 |
| ObjectJointPrismatic2D | Object (Joint) | 13 | 89, 1 | - | 0 of 9 | 0 | - | 5 |
| ObjectJointSliding | Object (Joint) | 38 | 585, 4 | - | 0 of 12 | 4 | - | 2 |
| ObjectJointSliding2D | Object (Joint) | 37 | 655, 5 | - | 0 of 12 | 4 | - | 5 |
| ObjectJointALEMoving2D | Object (Joint) | 43 | 540, 4 | yes | 0 of 12 | 6 | - | 2 |
| MarkerBodyMass | Marker | 15 | - | - | 0 of 3 | 0 | - | 23 |
| MarkerBodyPosition | Marker | 52 | 124, 0 | - | 0 of 4 | 0 | - | 82 |
| MarkerBodyRigid | Marker | 49 | - | - | 0 of 4 | 0 | - | 127 |
| MarkerNodePosition | Marker | 28 | 96, 0 | - | 0 of 3 | 0 | - | 60 |
| MarkerNodeRigid | Marker | 43 | 123, 0 | - | 0 of 3 | 0 | - | 57 |
| MarkerNodeCoordinate | Marker | 31 | - | - | 0 of 4 | 0 | - | 111 |
| MarkerNodeCoordinates | Marker | 28 | - | - | 0 of 3 | 0 | - | 3 |
| MarkerNodeODE1Coordinate | Marker | 9 | - | - | 0 of 4 | 0 | - | 0 |
| MarkerNodeRotationCoordinate | Marker | 22 | - | - | 1 of 4: `rotationCoordinate` | 0 | - | 5 |
| MarkerBodiesRelativeTranslationCoordinate | Marker | 80 | 107, 0 | - | 0 of 7 | 0 | - | 2 |
| MarkerBodiesRelativeRotationCoordinate | Marker | 80 | 156, 0 | - | 0 of 8 | 0 | - | 2 |
| MarkerSuperElementPosition | Marker | 41 | 140, 1 | - | 0 of 6 | 0 | yes | 10 |
| MarkerSuperElementRigid | Marker | 78 | 1100, 4 | yes | 0 of 9 | 0 | - | 16 |
| MarkerKinematicTreeRigid | Marker | 103 | 148, 1 | - | 0 of 5 | 0 | - | 6 |
| MarkerObjectODE2Coordinates | Marker | 29 | - | - | 0 of 3 | 0 | - | 1 |
| MarkerBodyCable2DShape | Marker | 13 | - | - | 0 of 5 | 0 | - | 13 |
| MarkerBodyCable2DCoordinates | Marker | 14 | - | - | 0 of 3 | 0 | - | 6 |
| MarkerBodyBeamShape | Marker | 18 | - | - | 0 of 3 | 0 | - | 2 |
| LoadForceVector | Load | 9 | 43, 1 | - | 0 of 6 | 0 | - | 109 |
| LoadTorqueVector | Load | 9 | 42, 1 | - | 0 of 6 | 0 | - | 51 |
| LoadMassProportional | Load | 21 | 30, 1 | - | 0 of 5 | 0 | yes | 23 |
| LoadCoordinate | Load | 38 | 43, 1 | - | 0 of 5 | 0 | - | 36 |
| SensorNode | Sensor | 38 | - | - | 0 of 7 | 0 | - | 68 |
| SensorObject | Sensor | 54 | - | - | 0 of 7 | 0 | - | 48 |
| SensorBody | Sensor | 54 | - | - | 0 of 8 | 0 | - | 75 |
| SensorSuperElement | Sensor | 57 | - | - | 0 of 8 | 0 | - | 15 |
| SensorKinematicTree | Sensor | 75 | - | - | 0 of 9 | 0 | - | 8 |
| SensorMarker | Sensor | 65 | - | - | 0 of 7 | 0 | - | 13 |
| SensorLoad | Sensor | 35 | - | - | 0 of 6 | 0 | - | 11 |
| SensorUserFunction | Sensor | 56 | 55, 0 | - | 0 of 8 | 0 | - | 8 |
