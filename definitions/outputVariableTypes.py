#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Exudyn: the OutputVariableType registrator
#
# One entry per output variable. This is the ONLY place where an output variable is declared:
# the C++ enum, GetOutputVariableTypeString(), IsOutputVariableTypeForReferenceConfiguration()
# and the pybind11 / documentation table are all generated from it. Before this file existed
# those four were kept in step by hand, and they had drifted - CoordinatesTotal had no string
# (issue #2408) and the two energies never reached Python.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# bit:  the position in the 64 bit mask. ALLOCATED ONCE AND NEVER REUSED, so a value written
#       to a file by an older version keeps its meaning. Add a new variable with the next free
#       bit; the emitter refuses duplicates and anything >= 64.
# Author:   Johannes Gerstmayr
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


class OutputVariable:
    """One output variable: its name, its permanently allocated bit, the description shown in
    Python and in the documentation, and whether it can be evaluated in the reference
    configuration."""

    def __init__(self, name, bit, description, referenceConfiguration=False, reserved=False):
        self.name = name
        self.bit = bit
        self.description = description
        self.referenceConfiguration = referenceConfiguration
        self.reserved = reserved


#the listing order is the order in which the variables appear in Python and in the generated
#documentation table; the bit is what defines the value, so the two are independent.
outputVariableTypes = [
    OutputVariable('_None', None,
                   'no value; used, e.g., to select no output variable in contour plot'),
    OutputVariable('Distance', 0,
                   'e.g., measure distance in spring damper connector',
                   referenceConfiguration=True),
    OutputVariable('Position', 1,
                   'measure 3D position, e.g., of node or body',
                   referenceConfiguration=True),
    OutputVariable('Displacement', 2,
                   'measure displacement; usually difference between current position and reference position',
                   referenceConfiguration=True),
    OutputVariable('DisplacementLocal', 3,
                   'measure local displacement, e.g., in local joint coordinates',
                   referenceConfiguration=True),
    OutputVariable('Velocity', 4,
                   'measure (translational) velocity of node or object'),
    OutputVariable('VelocityLocal', 5,
                   'measure local (translational) velocity, e.g., in local body or joint coordinates'),
    OutputVariable('Acceleration', 6,
                   'measure (translational) acceleration of node or object'),
    OutputVariable('AccelerationLocal', 7,
                   'measure (translational) acceleration of node or object in local coordinates'),
    OutputVariable('RotationMatrix', 8,
                   'measure rotation matrix of rigid body node or object',
                   referenceConfiguration=True),
    OutputVariable('Rotation', 13,
                   'measure, e.g., scalar rotation of 2D body, Euler angles of a 3D object or rotation within a joint',
                   referenceConfiguration=True),
    OutputVariable('AngularVelocity', 9,
                   'measure angular velocity of node or object'),
    OutputVariable('AngularVelocityLocal', 10,
                   'measure local (body-fixed) angular velocity of node or object'),
    OutputVariable('AngularAcceleration', 11,
                   'measure angular acceleration of node or object'),
    OutputVariable('AngularAccelerationLocal', 12,
                   'measure angular acceleration of node or object in local coordinates'),
    OutputVariable('CoordinatesTotal', 14,
                   'measure the total coordinates (including reference configuration) of a node or object; otherwise the same as Coordinates'),
    OutputVariable('Coordinates', 15,
                   'measure the coordinates of a node or object; coordinates just contain displacements, but not the reference (position or rotation) values - see also definition of respective nodes or objects',
                   referenceConfiguration=True),
    OutputVariable('Coordinates_t', 16,
                   'measure the time derivative of coordinates (= velocity coordinates) of a node or object'),
    OutputVariable('Coordinates_tt', 17,
                   'measure the second time derivative of coordinates (= acceleration coordinates) of a node or object'),
    OutputVariable('SlidingCoordinate', 18,
                   'measure sliding coordinate in sliding joint',
                   referenceConfiguration=True),
    OutputVariable('Director1', 19,
                   'measure a director (e.g., of a rigid body frame), or a slope vector in local 1 or x-direction',
                   referenceConfiguration=True),
    OutputVariable('Director2', 20,
                   'measure a director (e.g., of a rigid body frame), or a slope vector in local 2 or y-direction',
                   referenceConfiguration=True),
    OutputVariable('Director3', 21,
                   'measure a director (e.g., of a rigid body frame), or a slope vector in local 3 or z-direction',
                   referenceConfiguration=True),
    OutputVariable('Force', 22,
                   'measure global force, e.g., in joint or beam (resultant force), or generalized forces; see description of according object'),
    OutputVariable('ForceLocal', 23,
                   'measure local force, e.g., in joint or beam (resultant force)'),
    OutputVariable('Torque', 24,
                   'measure torque, e.g., in joint or beam (resultant couple/moment)'),
    OutputVariable('TorqueLocal', 25,
                   'measure local torque, e.g., in joint or beam (resultant couple/moment)'),
    OutputVariable('StrainLocal', 28,
                   'measure local strain, e.g., axial strain in cross section frame of beam or Green-Lagrange strain'),
    OutputVariable('StressLocal', 29,
                   'measure local stress, e.g., axial stress in cross section frame of beam or Second Piola-Kirchoff stress; choosing component==-1 will result in the computation of the Mises stress'),
    OutputVariable('CurvatureLocal', 30,
                   'measure local curvature; may be scalar or vectorial: twist and curvature of beam in cross section frame'),
    OutputVariable('ConstraintEquation', 31,
                   'evaluates constraint equation (=current deviation or drift of constraint equation)',
                   referenceConfiguration=True),
    OutputVariable('KineticEnergy', 32,
                   'measure kinetic energy of a body, position independent'),
    OutputVariable('PotentialEnergy', 33,
                   'measure potential (=elastic) energy of a body or connector, position independent'),
    OutputVariable('HomogeneousTransformation', 34,
                   'measure the homogeneous transformation of a node, body point or marker: its rotation matrix A and position p as the 4x4 matrix [A p; 0 1]; the Get...Output functions return it as exu.HT, a sensor stores its 16 components row by row; every item with Position and RotationMatrix provides it',
                   referenceConfiguration=True),
    OutputVariable('HomogeneousTransformationLocal', 35,
                   'measure the homogeneous transformation of a frame relative to another, e.g., of joint frame J1 in joint frame J0: the 4x4 matrix [A p; 0 1] with the relative rotation matrix A and the relative position p in the first frame; returned and stored as HomogeneousTransformation',
                   referenceConfiguration=True),
    ]

#bits allocated to variables that were considered and are not implemented. They are listed so
#that the numbers are never handed out again; the emitter writes them as a comment.
reservedBits = [
    OutputVariable('Strain', 26, 'considered for finite elements or fluids; never implemented', reserved=True),
    OutputVariable('Stress', 27, 'considered for finite elements or fluids; never implemented', reserved=True),
    ]

#one module-level constant per variable - OVPosition, OVForceLocal, ... - so an item definition
#names an output variable instead of spelling it as a string. A typo is then a NameError at load
#time rather than a key that silently never matches. Written as a loop and not as 34 hand-kept
#lines, because a second list is exactly what this file exists to remove.
for _variable in outputVariableTypes:
    globals()['OV' + _variable.name] = _variable
del _variable
