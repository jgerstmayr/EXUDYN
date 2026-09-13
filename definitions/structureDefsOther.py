#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Other definitions
#
# Details:  2 definitions, emitted from systemStructuresDefinition.py (revision plan step 31a).
#           This IS Python: import it and read "definitions", a list of dicts.
#
#           ORDER MATTERS. The generators emit in the order the definitions appear,
#           and the generated C++/pybind/RST is compared byte-for-byte, so
#           reordering this list changes generated files. Append at the end unless
#           you mean to reorder.
#
#           Only descriptions, LaTeX and C++ code are raw strings; every other field
#           is a name, a flag constant or a short literal and needs no escaping.
#
#           The constants come from definitionTypes.py, which is hand-written: a
#           value used here with no constant there stops the emit and says what to
#           add, so the two can never drift apart silently.
#
# Contents: PyBeamSection, BeamSectionGeometry
#
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from definitionTypes import *

definitions = []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   PyBeamSection   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(StructureDefinition(
    className='PyBeamSection',
    classDescription=r"""Data structure for definition of 2D and 3D beam (cross) section mechanical properties. The beam has local coordinates, in which $X$ represents the beam centerline (beam axis) coordinate, being the neutral fiber w.r.t.\ bending; $Y$ and $Z$ are the local cross section coordinates. Note that most elements do not accept all parameters, which results in an error if those parameters (e.g., stiffness parameters) are non-zero.""",
    cppText=r"""#include "Main/StructuralElementsDataStructures.h"
#include "Pymodules/PybindUtilities.h"
""",
    latexText=r"""
%++++++++++++++++++++++++++++++++++++++
\mysubsection{Structures for structural elements}
This section includes data structures for structural elements, such as beams (and plates in future). These classes are used as interface between Python libraries for structural elements and Exudyn internal classes.
""",
    parentClass='BeamSection',
    pythonClass='BeamSection',
    writePybindIncludes=True,
    members=[
        StructureParameter(type=TMatrixND(6, 6), cFlags=SFNoDictType+SFPybind, isLinked=True,
            pythonName='stiffnessMatrix',
            defaultValue='Matrix6D(6,6,0.)',
            description=r"""$\LU{c}{\Cm} \in \Rcal^{6 \times 6}\,$ [SI:Nm$^2$, Nm and N (mixed)] sectional stiffness matrix related to $\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Cm} \vp{\LU{c}{\teps}}{\LU{c}{\tkappa}}$ with sectional normal force $\LU{c}{\nv}$, torque $\LU{c}{\mv}$, strain $\LU{c}{\teps}$ and curvature $\LU{c}{\tkappa}$, all quantities expressed in the cross section frame $c$. Set with list of lists or numpy array."""),
        StructureParameter(type=TMatrixND(6, 6), cFlags=SFNoDictType+SFPybind, isLinked=True,
            pythonName='dampingMatrix',
            defaultValue='Matrix6D(6,6,0.)',
            description=r"""$\LU{c}{\Dm} \in \Rcal^{6 \times 6}\,$ [SI:Nsm$^2$, Nsm and Ns (mixed)] sectional linear damping matrix related to $\vp{\LU{c}{\nv}}{\LU{c}{\mv}} = \LU{c}{\Dm} \vp{\LU{c}{\tepsDot}}{\LU{c}{\tkappaDot}}$; note that this damping models is highly simplified and usually, it cannot be derived from material parameters; however, it can be used to adjust model damping to observed damping behavior. Set with list of lists or numpy array."""),
        StructureParameter(type=TReal(minimum=0), cFlags=SFNoDictType+SFPybind, isLinked=True,
            pythonName='massPerLength',
            defaultValue=0.,
            description=r'$\rho A\,$ [SI:kg/m] mass per unit length of the beam'),
        StructureParameter(type=TMatrixND(3, 3), cFlags=SFNoDictType+SFPybind, isLinked=True,
            pythonName='inertia',
            defaultValue='EXUmath::zeroMatrix3D',
            description=r"""$\LU{c}{\Jm} \in \Rcal^{3 \times 3}\,$ [SI:kg$\,$m$^2$] sectional inertia for shear-deformable beams. Set with list of lists or numpy array."""),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   BeamSectionGeometry   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(StructureDefinition(
    className='BeamSectionGeometry',
    addDictionaryAccess=False,
    appendToFile=False,
    classDescription=r'Data structure for definition of 2D and 3D beam (cross) section geometrical properties. Used for visualization and contact.',
    writePybindIncludes=True,
    members=[
        StructureParameter(type=TCrossSectionType, cFlags=SFNoDictType+SFPybind,
            pythonName='crossSectionType',
            defaultValue='CrossSectionType::Polygon',
            description=r'Type of cross section: Polygon, Circular, etc.'),
        StructureParameter(type=TReal(minimum=0), cFlags=SFNoDictType+SFPybind,
            pythonName='crossSectionRadiusY',
            defaultValue=0.,
            description=r'$c_Y\,$ [SI:m] $Y$ radius for circular cross section'),
        StructureParameter(type=TReal(minimum=0), cFlags=SFNoDictType+SFPybind,
            pythonName='crossSectionRadiusZ',
            defaultValue=0.,
            description=r'$c_Z\,$ [SI:m] $Z$ radius for circular cross section'),
        StructureParameter(type=TVector2DList, cFlags=SFNoDictType+SFPybind,
            pythonName='polygonalPoints',
            defaultValue=NoDefaultValue,
            description=r"""$\pv_{pg}\,$ [SI: (m,m) ] list of polygonal ($Y,Z$) points in local beam cross section coordinates, defined in positive rotation direction"""),
        ],
    ))
