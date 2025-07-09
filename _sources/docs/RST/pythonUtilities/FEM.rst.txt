
.. _sec-module-fem:

Module: FEM
===========

Support functions and helper classes for import of meshes, finite element models (ABAQUS, ANSYS, NETGEN) and for generation of FFRF (floating frame of reference) objects.
Note that since exudyn version 1.8.69 the mass and stiffness matrices in FEMinterface are either None or given in SciPy-sparse csr format (leading also to a new load/save fileVersion of FEMinterface)

- | Author:
  | Johannes Gerstmayr; Stefan Holzinger (Abaqus and Ansys import utilities); Joachim Schöberl (support for Netgen and NGsolve  import and eigen computations)
- Date:      2020-03-10 (created) 
- Notes:     OLD internal CSR matrix storage format contains 3 float numbers per row: [row, column, value], can be converted to scipy csr sparse matrices with function CSRtoScipySparseCSR(...); the NEW format uses scipy's internal sparse csr format! To switch to the old format, set exudyn.FEM.useOldCSRformat=True 


.. _sec-fem-compressedrowsparsetodensematrix:

Function: CompressedRowSparseToDenseMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompressedRowSparseToDenseMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L72>`__\ (\ ``sparseData``\ )

- | \ *function description*\ :
  | convert zero-based sparse matrix data to dense numpy matrix
- | \ *input*\ :
  | sparseData: format (per row): [row, column, value] ==> converted into dense format
- | \ *output*\ :
  | a dense matrix as np.array



----


.. _sec-fem-mapsparsematrixindices:

Function: MapSparseMatrixIndices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`MapSparseMatrixIndices <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L85>`__\ (\ ``matrix``\ , \ ``sorting``\ )

- | \ *function description*\ :
  | resort a sparse matrix (internal CSR format) with given sorting for rows and columns; changes matrix directly! used for ANSYS matrix import



----


.. _sec-fem-vectordiadicunitmatrix3d:

Function: VectorDiadicUnitMatrix3D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`VectorDiadicUnitMatrix3D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L93>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | compute diadic product of vector v and a 3D unit matrix = diadic(v,I\ :math:`_{3x3}`\ ); used for ObjectFFRF and CMS implementation



----


.. _sec-fem-cycliccomparereversed:

Function: CyclicCompareReversed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CyclicCompareReversed <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L99>`__\ (\ ``list1``\ , \ ``list2``\ )

- | \ *function description*\ :
  | compare cyclic two lists, reverse second list; return True, if any cyclic shifted lists are same, False otherwise



----


.. _sec-fem-addentrytocompressedrowsparsearray:

Function: AddEntryToCompressedRowSparseArray
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddEntryToCompressedRowSparseArray <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L108>`__\ (\ ``sparseData``\ , \ ``row``\ , \ ``column``\ , \ ``value``\ )

- | \ *function description*\ :
  | add entry to compressedRowSparse matrix, avoiding duplicates
  | value is either added to existing entry (avoid duplicates) or a new entry is appended



----


.. _sec-fem-csrtorowsandcolumns:

Function: CSRtoRowsAndColumns
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CSRtoRowsAndColumns <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L129>`__\ (\ ``sparseMatrixCSR``\ )

- | \ *function description*\ :
  | compute rows and columns of a compressed sparse matrix and return as tuple: (rows,columns)



----


.. _sec-fem-csrtoscipysparsecsr:

Function: CSRtoScipySparseCSR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CSRtoScipySparseCSR <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L143>`__\ (\ ``sparseMatrixCSR``\ )

- | \ *function description*\ :
  | DEPRECATED: convert internal compressed CSR to scipy.sparse csr matrix; should not be used and raises warning; use SparseTripletsToScipySparseCSR instead!

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `shapeOptimization.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/shapeOptimization.py>`_\  (Ex)



----


.. _sec-fem-sparsetripletstoscipysparsecsr:

Function: SparseTripletsToScipySparseCSR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SparseTripletsToScipySparseCSR <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L160>`__\ (\ ``sparseTriplets``\ )

- | \ *function description*\ :
  | convert list of sparse triplets (or numpy array with one sparse triplet per row) into scipy.sparse csr_matrix



----


.. _sec-fem-scipysparsecsrtocsr:

Function: ScipySparseCSRtoCSR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ScipySparseCSRtoCSR <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L172>`__\ (\ ``scipyCSR``\ )

- | \ *function description*\ :
  | convert scipy.sparse csr matrix to internal compressed CSR



----


.. _sec-fem-resortindicesofcsrmatrix:

Function: ResortIndicesOfCSRmatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ResortIndicesOfCSRmatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L180>`__\ (\ ``mXXYYZZ``\ , \ ``numberOfRows``\ )

- | \ *function description*\ :
  | resort indices of given NGsolve CSR matrix in XXXYYYZZZ format to XYZXYZXYZ format; numberOfRows must be equal to columns
  | needed for import from NGsolve



----


.. _sec-fem-resortindicesofngvector:

Function: ResortIndicesOfNGvector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ResortIndicesOfNGvector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L194>`__\ (\ ``vXXYYZZ``\ )

- | \ *function description*\ :
  | resort indices of given NGsolve vector in XXXYYYZZZ format to XYZXYZXYZ format



----


.. _sec-fem-resortindicesexudyn2ngvector:

Function: ResortIndicesExudyn2NGvector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ResortIndicesExudyn2NGvector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L214>`__\ (\ ``vXYZXYZ``\ )

- | \ *function description*\ :
  | resort indices of given Exudyun vector XYZXYZXYZ to NGsolve vector in XXXYYYZZZ format



----


.. _sec-fem-converthextotrigs:

Function: ConvertHexToTrigs
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ConvertHexToTrigs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L383>`__\ (\ ``nodeNumbers``\ )

- | \ *function description*\ :
  | convert list of Hex8/C3D8  element with 8 nodes in nodeNumbers into triangle-List
- | \ *notes*\ :
  | works for Hex20 elements, but does only take the corner nodes for drawing!

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TM)



----


.. _sec-fem-converttettotrigs:

Function: ConvertTetToTrigs
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ConvertTetToTrigs <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L394>`__\ (\ ``nodeNumbers``\ )

- | \ *function description*\ :
  | convert list of Tet4/Tet10 element with 4 or 10 nodes in nodeNumbers into triangle-List
- | \ *notes*\ :
  | works for Tet10 elements, but does only take the corner nodes for drawing!



----


.. _sec-fem-convertdensetocompressedrowmatrix:

Function: ConvertDenseToCompressedRowMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ConvertDenseToCompressedRowMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L406>`__\ (\ ``denseMatrix``\ )

- | \ *function description*\ :
  | convert numpy.array dense matrix to OLD FEM internal compressed row sparse format; do not use!



----


.. _sec-fem-readmatrixfromansysmmf:

Function: ReadMatrixFromAnsysMMF
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadMatrixFromAnsysMMF <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L459>`__\ (\ ``fileName``\ , \ ``verbose = False``\ )

- | \ *function description*\ :
  | This function reads either the mass or stiffness matrix from an Ansys
  | Matrix Market Format (MMF). The corresponding matrix can either be exported
  | as dense matrix or sparse matrix.
- | \ *input*\ :
  | fileName of MMF file
- | \ *output*\ :
  | internal compressed row sparse matrix (as (nrows x 3) numpy array)
- | \ *author*\ :
  | Stefan Holzinger
- | \ *notes*\ :
  | A MMF file can be created in Ansys by placing the following APDL code inside
  | the solution tree in Ansys Workbench:
  | !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  | ! APDL code that exports sparse stiffnes and mass matrix in MMF format. If
  | ! the dense matrix is needed, replace \*SMAT with \*DMAT in the following
  | ! APDL code.
  | ! Export the stiffness matrix in MMF format
  | \*SMAT,MatKD,D,IMPORT,FULL,file.full,STIFF
  | \*EXPORT,MatKD,MMF,fileNameStiffnessMatrix,,,
  | ! Export the mass matrix in MMF format
  | \*SMAT,MatMD,D,IMPORT,FULL,file.full,MASS
  | \*EXPORT,MatMD,MMF,fileNameMassMatrix,,,
  | !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  | In case a lumped mass matrix is needed, place the following APDL Code inside
  | the Modal Analysis Tree:
  | !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  | ! APDL code to force Ansys to use a lumped mass formulation (if available for
  | ! used elements)
  | LUMPM, ON, , 0
  | !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



----


.. _sec-fem-readmatrixdofmappingvectorfromansystxt:

Function: ReadMatrixDOFmappingVectorFromAnsysTxt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadMatrixDOFmappingVectorFromAnsysTxt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L505>`__\ (\ ``fileName``\ )

- | \ *function description*\ :
  | read sorting vector for ANSYS mass and stiffness matrices and return sorting vector as np.array
  | the file contains sorting for nodes and applies this sorting to the DOF (assuming 3 DOF per node!)
  | the resulting sorted vector is already converted to 0-based indices



----


.. _sec-fem-readnodalcoordinatesfromansystxt:

Function: ReadNodalCoordinatesFromAnsysTxt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadNodalCoordinatesFromAnsysTxt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L561>`__\ (\ ``fileName``\ , \ ``verbose = False``\ )

- | \ *function description*\ :
  | This function reads the nodal coordinates exported from Ansys.
- | \ *input*\ :
  | fileName (file name ending must be .txt!)
- | \ *output*\ :
  | nodal coordinates as numpy array
- | \ *author*\ :
  | Stefan Holzinger
- | \ *notes*\ :
  | The nodal coordinates can be exported from Ansys by creating a named selection
  | of the body whos mesh should to exported by choosing its geometry. Next,
  | create a second named selcetion by using a worksheet. Add the named selection
  | that was created first into the worksheet of the second named selection.
  | Inside the working sheet, choose 'convert' and convert the first created
  | named selection to 'mesh node' (Netzknoten in german) and click on generate
  | to create the second named selection. Next, right click on the second
  | named selection tha was created and choose 'export' and save the nodal
  | coordinates as .txt file.



----


.. _sec-fem-readelementsfromansystxt:

Function: ReadElementsFromAnsysTxt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadElementsFromAnsysTxt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L640>`__\ (\ ``fileName``\ , \ ``verbose = False``\ )

- | \ *function description*\ :
  | This function reads the nodal coordinates exported from Ansys.
- | \ *input*\ :
  | fileName (file name ending must be .txt!)
- | \ *output*\ :
  | element connectivity as numpy array
- | \ *author*\ :
  | Stefan Holzinger
- | \ *notes*\ :
  | The elements can be exported from Ansys by creating a named selection
  | of the body whos mesh should to exported by choosing its geometry. Next,
  | create a second named selcetion by using a worksheet. Add the named selection
  | that was created first into the worksheet of the second named selection.
  | Inside the worksheet, choose 'convert' and convert the first created
  | named selection to 'mesh element' (Netzelement in german) and click on generate
  | to create the second named selection. Next, right click on the second
  | named selection tha was created and choose 'export' and save the elements
  | as .txt file.



----


.. _sec-fem-cmsobjectcomputenorm:

Function: CMSObjectComputeNorm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CMSObjectComputeNorm <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1128>`__\ (\ ``mbs``\ , \ ``objectNumber``\ , \ ``outputVariableType``\ , \ ``norm = 'max'``\ , \ ``nodeNumberList = []``\ )

- | \ *function description*\ :
  | compute current (max, min, ...) value for chosen ObjectFFRFreducedOrder object (CMSobject) with exu.OutputVariableType. The function operates on nodal values. This is a helper function, which can be used to conveniently compute output quantities of the CMSobject efficiently and to use it in sensors
- | \ *input*\ :
  | \ ``mbs``\ : MainSystem of objectNumber
  | \ ``objectNumber``\ : number of ObjectFFRFreducedOrder in mbs
  | \ ``outputVariableType``\ : a exu.OutputVariableType out of [StressLocal, DisplacementLocal, VelocityLocal]
  | \ ``norm``\ : string containing chosen norm to be computed, out of 'Mises', 'maxNorm', 'min', 'max'; 'max' will return maximum of all components (component wise), 'min' does same but for minimum; 'maxNorm' computes np.linalg.norm for every node and then takes maximum of all norms; Mises computes von-Mises stress for every node and then takes maximum of all nodes
  | \ ``nodeNumberList``\ : list of mesh node numbers (from FEMinterface); if empty [], all nodes are used; otherwise, only given nodes are evaluated
- | \ *output*\ :
  | return value or list of values according to chosen norm as np.array

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `netgenSTLtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/netgenSTLtest.py>`_\  (Ex), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex)


.. _sec-module-fem-class-materialbaseclass:

CLASS MaterialBaseClass (in module FEM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    INTERNAL material base class, e.g., for FiniteElement



.. _sec-module-fem-class-kirchhoffmaterial(materialbaseclass):

CLASS KirchhoffMaterial(MaterialBaseClass) (in module FEM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    class for representation of Kirchhoff (linear elastic, 3D and 2D) material

- | \ *notes*\ :
  | use planeStress=False for plane strain


.. _sec-fem-kirchhoffmaterial(materialbaseclass)---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L732>`__\ (\ ``self``\ , \ ``youngsModulus = None``\ , \ ``poissonsRatio = None``\ , \ ``density = 0``\ , \ ``materials = None``\ , \ ``fes = None``\ , \ ``planeStress = True``\ )

- | \ *classFunction*\ :
  | add according nodes, objects and constraints for FFRF object to MainSystem mbs; only implemented for Euler parameters
- | \ *input*\ :
  | \ ``youngsModulus``\ : Young's modulus for single domain and material; in case of multi-domain, it must be None
  | \ ``poissonsRatio``\ : Poisson's ratio for single domain and material; in case of multi-domain, it must be None
  | \ ``density``\ : density for for single domain and material; in case of multi-domain, it must be 0 or None
  | \ ``materials``\ : dictionary of material dictionaries according to names in NGsolve mesh, containing youngsModulus, poissonsRatio and density per material, see ImportMeshFromNGsolve
  | \ ``fes``\ : in case of materials dictionary, fes (as returned by ImportMeshFromNGsolve) has to be provided
  | \ ``planeStress``\ : set True for 2D materials (currently not used)

----

.. _sec-fem-kirchhoffmaterial(materialbaseclass)-strain2stress:

Class function: Strain2Stress
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Strain2Stress <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L797>`__\ (\ ``self``\ , \ ``strain``\ )

- | \ *classFunction*\ :
  | convert strain tensor into stress tensor using elasticity tensor

----

.. _sec-fem-kirchhoffmaterial(materialbaseclass)-strainvector2stressvector:

Class function: StrainVector2StressVector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`StrainVector2StressVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L810>`__\ (\ ``self``\ , \ ``strainVector``\ )

- | \ *classFunction*\ :
  | convert strain vector into stress vector

----

.. _sec-fem-kirchhoffmaterial(materialbaseclass)-strainvector2stressvector2d:

Class function: StrainVector2StressVector2D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`StrainVector2StressVector2D <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L816>`__\ (\ ``self``\ , \ ``strainVector2D``\ )

- | \ *classFunction*\ :
  | compute 2D stress vector from strain vector

----

.. _sec-fem-kirchhoffmaterial(materialbaseclass)-lameparameters:

Class function: LameParameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`LameParameters <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L827>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | compute Lame parameters from internal Young's modulus and Poisson ratio
- | \ *output*\ :
  | return vector [mu, lam] of Lame parameters

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Ex), \ `netgenSTLtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/netgenSTLtest.py>`_\  (Ex), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolveFFRF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRF.py>`_\  (Ex), \ `NGsolveCMStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCMStest.py>`_\  (TM)


.. _sec-module-fem-class-finiteelement:

CLASS FiniteElement (in module FEM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    finite element base class for lateron implementations of other finite elements



.. _sec-module-fem-class-tet4(finiteelement):

CLASS Tet4(FiniteElement) (in module FEM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    simplistic 4-noded tetrahedral interface to compute strain/stress at nodal points



.. _sec-module-fem-class-objectffrfinterface:

CLASS ObjectFFRFinterface (in module FEM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    compute terms necessary for ObjectFFRF
    class used internally in FEMinterface to compute ObjectFFRF object
    this class holds all data for ObjectFFRF user functions


.. _sec-fem-objectffrfinterface---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L916>`__\ (\ ``self``\ , \ ``femInterface``\ )

- | \ *classFunction*\ :
  | initialize ObjectFFRFinterface with FEMinterface class
  | initializes the ObjectFFRFinterface with nodes, modes, surface description and systemmatrices from FEMinterface
  | data is then transfered to mbs object with classFunction AddObjectFFRF(...)

----

.. _sec-fem-objectffrfinterface-addobjectffrf:

Class function: AddObjectFFRF
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddObjectFFRF <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L961>`__\ (\ ``self``\ , \ ``exu``\ , \ ``mbs``\ , \ ``positionRef = [0,0,0]``\ , \ ``eulerParametersRef = [1,0,0,0]``\ , \ ``initialVelocity = [0,0,0]``\ , \ ``initialAngularVelocity = [0,0,0]``\ , \ ``gravity = [0,0,0]``\ , \ ``constrainRigidBodyMotion = True``\ , \ ``massProportionalDamping = 0``\ , \ ``stiffnessProportionalDamping = 0``\ , \ ``color = [0.1,0.9,0.1,1.]``\ )

- | \ *classFunction*\ :
  | add according nodes, objects and constraints for FFRF object to MainSystem mbs; only implemented for Euler parameters
- | \ *input*\ :
  | \ ``exu``\ : the exudyn module
  | \ ``mbs``\ : a MainSystem object
  | \ ``positionRef``\ : reference position of created ObjectFFRF (set in rigid body node underlying to ObjectFFRF)
  | \ ``eulerParametersRef``\ : reference euler parameters of created ObjectFFRF (set in rigid body node underlying to ObjectFFRF)
  | \ ``initialVelocity``\ : initial velocity of created ObjectFFRF (set in rigid body node underlying to ObjectFFRF)
  | \ ``initialAngularVelocity``\ : initial angular velocity of created ObjectFFRF (set in rigid body node underlying to ObjectFFRF)
  | \ ``gravity``\ : set [0,0,0] if no gravity shall be applied, or to the gravity vector otherwise
  | \ ``constrainRigidBodyMotion``\ : set True in order to add constraint (Tisserand frame) in order to suppress rigid motion of mesh nodes
  | \ ``color``\ : provided as list of 4 RGBA values
  | add object to mbs as well as according nodes

----

.. _sec-fem-objectffrfinterface-ufforce:

Class function: UFforce
^^^^^^^^^^^^^^^^^^^^^^^
`UFforce <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1043>`__\ (\ ``self``\ , \ ``exu``\ , \ ``mbs``\ , \ ``t``\ , \ ``q``\ , \ ``q_t``\ )

- | \ *classFunction*\ :
  | optional forceUserFunction for ObjectFFRF (per default, this user function is ignored)

----

.. _sec-fem-objectffrfinterface-ufmassgenericode2:

Class function: UFmassGenericODE2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`UFmassGenericODE2 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1089>`__\ (\ ``self``\ , \ ``exu``\ , \ ``mbs``\ , \ ``t``\ , \ ``q``\ , \ ``q_t``\ )

- | \ *classFunction*\ :
  | optional massMatrixUserFunction for ObjectFFRF (per default, this user function is ignored)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `objectFFRFTest2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest2.py>`_\  (TM)


.. _sec-module-fem-class-objectffrfreducedorderinterface:

CLASS ObjectFFRFreducedOrderInterface (in module FEM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    compute terms necessary for ObjectFFRFreducedOrder
    class used internally in FEMinterface to compute ObjectFFRFreducedOrder dictionary
    this class holds all data for ObjectFFRFreducedOrder user functions


.. _sec-fem-objectffrfreducedorderinterface---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1199>`__\ (\ ``self``\ , \ ``femInterface = None``\ , \ ``rigidBodyNodeType = 'NodeType.RotationEulerParameters'``\ , \ ``roundMassMatrix = 1e-13``\ , \ ``roundStiffnessMatrix = 1e-13``\ )

- | \ *classFunction*\ :
  | initialize ObjectFFRFreducedOrderInterface with FEMinterface class
  | initializes the ObjectFFRFreducedOrderInterface with nodes, modes, surface description and reduced system matrices from FEMinterface
  | data is then transfered to mbs object with classFunction AddObjectFFRFreducedOrderWithUserFunctions(...)
- | \ *input*\ :
  | \ ``femInterface``\ : must provide nodes, surfaceTriangles, modeBasis, massMatrix, stiffness; if femInterface=None, an empty ObjectFFRFreducedOrderInterface instance is created which may be used to load data with LoadFromFile()
  | \ ``roundMassMatrix``\ : use this value to set entries of reduced mass matrix to zero which are below the treshold
  | \ ``roundStiffnessMatrix``\ : use this value to set entries of reduced stiffness matrix to zero which are below the treshold

----

.. _sec-fem-objectffrfreducedorderinterface-savetofile:

Class function: SaveToFile
^^^^^^^^^^^^^^^^^^^^^^^^^^
`SaveToFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1350>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``fileVersion = 1``\ )

- | \ *classFunction*\ :
  | save all data to a data filename; can be used to avoid loading femInterface and FE data
- | \ *input*\ :
  | \ ``fileName``\ : string for path and file name without ending ==> ".npy" will be added
  | \ ``fileVersion``\ : FOR EXPERTS: this allows to store in older format, will be recovered when loading; must be integer; version must by > 0; the default value will change in future!
- | \ *output*\ :
  | stores file

----

.. _sec-fem-objectffrfreducedorderinterface-loadfromfile:

Class function: LoadFromFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`LoadFromFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1411>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``mode = None``\ )

- | \ *classFunction*\ :
  | load all data (nodes, elements, ...) from a data filename previously stored with SaveToFile(...).
  | this function is much faster than the text-based import functions
- | \ *input*\ :
  | \ ``fileName``\ : string for path and file name without ending ==> ".npy" will be added
  | \ ``mode``\ : choose between different file formats (NPY and NPZ); Note: NPY only works for Numpy 1.x, not for Numpy >= 2.0
- | \ *output*\ :
  | loads data into fem (note that existing values are not overwritten!)

----

.. _sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions:

Class function: AddObjectFFRFreducedOrderWithUserFunctions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddObjectFFRFreducedOrderWithUserFunctions <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1495>`__\ (\ ``self``\ , \ ``exu``\ , \ ``mbs``\ , \ ``positionRef = [0,0,0]``\ , \ ``initialVelocity = [0,0,0]``\ , \ ``rotationMatrixRef = []``\ , \ ``initialAngularVelocity = [0,0,0]``\ , \ ``gravity = [0,0,0]``\ , \ ``UFforce = 0``\ , \ ``UFmassMatrix = 0``\ , \ ``massProportionalDamping = 0``\ , \ ``stiffnessProportionalDamping = 0``\ , \ ``color = [0.1,0.9,0.1,1.]``\ , \ ``eulerParametersRef = []``\ )

- | \ *classFunction*\ :
  | add according nodes, objects and constraints for ObjectFFRFreducedOrder object to MainSystem mbs; use this function with userfunctions=0 in order to use internal C++ functionality, which is approx. 10x faster; implementation of userfunctions also available for rotation vector (Lie group formulation), which needs further testing
- | \ *input*\ :
  | \ ``exu``\ : the exudyn module
  | \ ``mbs``\ : a MainSystem object
  | \ ``positionRef``\ : reference position of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
  | \ ``initialVelocity``\ : initial velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
  | \ ``rotationMatrixRef``\ : reference rotation of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder); if [], it becomes the unit matrix
  | \ ``initialAngularVelocity``\ : initial angular velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
  | \ ``eulerParametersRef``\ : DEPRECATED, use rotationParametersRef or rotationMatrixRef in future: reference euler parameters of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
  | \ ``gravity``\ : set [0,0,0] if no gravity shall be applied, or to the gravity vector otherwise
  | \ ``UFforce``\ : (OPTIONAL, computation is slower) provide a user function, which computes the quadratic velocity vector and applied forces; see example
  | \ ``UFmassMatrix``\ : (OPTIONAL, computation is slower) provide a user function, which computes the quadratic velocity vector and applied forces; see example
  | \ ``massProportionalDamping``\ : Rayleigh damping factor for mass proportional damping (multiplied with reduced mass matrix), added to floating frame/modal coordinates only
  | \ ``stiffnessProportionalDamping``\ : Rayleigh damping factor for stiffness proportional damping, added to floating frame/modal coordinates only (multiplied with reduced stiffness matrix)
  | \ ``color``\ : provided as list of 4 RGBA values
- | \ *example*\ :

.. code-block:: python

  #example of a user function for forces:
  def UFforceFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
      return cms.UFforceFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)
  #example of a user function for mass matrix:
  def UFmassFFRFreducedOrder(mbs, t, itemIndex, qReduced, qReduced_t):
      return cms.UFmassFFRFreducedOrder(exu, mbs, t, qReduced, qReduced_t)


----

.. _sec-fem-objectffrfreducedorderinterface-ufmassffrfreducedorder:

Class function: UFmassFFRFreducedOrder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`UFmassFFRFreducedOrder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1648>`__\ (\ ``self``\ , \ ``exu``\ , \ ``mbs``\ , \ ``t``\ , \ ``qReduced``\ , \ ``qReduced_t``\ )

- | \ *classFunction*\ :
  | CMS mass matrix user function; qReduced and qReduced_t contain the coordiantes of the rigid body node and the modal coordinates in one vector!

----

.. _sec-fem-objectffrfreducedorderinterface-ufforceffrfreducedorder:

Class function: UFforceFFRFreducedOrder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`UFforceFFRFreducedOrder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1699>`__\ (\ ``self``\ , \ ``exu``\ , \ ``mbs``\ , \ ``t``\ , \ ``qReduced``\ , \ ``qReduced_t``\ )

- | \ *classFunction*\ :
  | CMS force matrix user function; qReduced and qReduced_t contain the coordiantes of the rigid body node and the modal coordinates in one vector!

----

.. _sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorder:

Class function: AddObjectFFRFreducedOrder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddObjectFFRFreducedOrder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1782>`__\ (\ ``self``\ , \ ``mbs``\ , \ ``positionRef = [0,0,0]``\ , \ ``initialVelocity = [0,0,0]``\ , \ ``rotationMatrixRef = []``\ , \ ``initialAngularVelocity = [0,0,0]``\ , \ ``massProportionalDamping = 0``\ , \ ``stiffnessProportionalDamping = 0``\ , \ ``gravity = [0,0,0]``\ , \ ``color = [0.1,0.9,0.1,1.]``\ )

- | \ *classFunction*\ :
  | add according nodes, objects and constraints for ObjectFFRFreducedOrder object to MainSystem mbs; use this function in order to use internal C++ functionality, which is approx. 10x faster than AddObjectFFRFreducedOrderWithUserFunctions(...)
- | \ *input*\ :
  | \ ``exu``\ : the exudyn module
  | \ ``mbs``\ : a MainSystem object
  | \ ``positionRef``\ : reference position of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
  | \ ``initialVelocity``\ : initial velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
  | \ ``rotationMatrixRef``\ : reference rotation of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder); if [], it becomes the unit matrix
  | \ ``initialAngularVelocity``\ : initial angular velocity of created ObjectFFRFreducedOrder (set in rigid body node underlying to ObjectFFRFreducedOrder)
  | \ ``massProportionalDamping``\ : Rayleigh damping factor for mass proportional damping, added to floating frame/modal coordinates only
  | \ ``stiffnessProportionalDamping``\ : Rayleigh damping factor for stiffness proportional damping, added to floating frame/modal coordinates only
  | \ ``gravity``\ : set [0,0,0] if no gravity shall be applied, or to the gravity vector otherwise
  | \ ``color``\ : provided as list of 4 RGBA values

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Ex), \ `netgenSTLtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/netgenSTLtest.py>`_\  (Ex), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolveFFRF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRF.py>`_\  (Ex), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TM), \ `NGsolveCMStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCMStest.py>`_\  (TM), \ `NGsolveCrankShaftTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCrankShaftTest.py>`_\  (TM)


.. _sec-module-fem-class-hcbstaticmodeselection(enum):

CLASS HCBstaticModeSelection(Enum) (in module FEM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    helper calss for function ComputeHurtyCraigBamptonModes, declaring some computation options. It offers the following options:
    
    - allBoundaryNodes:     compute a single static mode for every boundary coordinate
    
    - RBE2:                 static modes only for rigid body motion at boundary nodes; using rigid boundary surfaces (additional stiffening)
    
    - RBE3:                 static modes only for rigid body motion at boundary nodes; averaged rigid body motion at boundary surfaces (leads to deformation at boundaries)
    
    - noStaticModes:        do not compute static modes, only eigen modes (not recommended; usually only for tests)



.. _sec-module-fem-class-feminterface:

CLASS FEMinterface (in module FEM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    general interface to different FEM / mesh imports and export to EXUDYN functions
    use this class to import meshes from different meshing or FEM programs (NETGEN/NGsolve , ABAQUS, ANSYS, ..) and store it in a unique format
    do mesh operations, compute eigenmodes and reduced basis, etc.
    load/store the data efficiently with LoadFromFile(...), SaveToFile(...)  if import functions are slow
    export to EXUDYN objects


.. _sec-fem-feminterface---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1840>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | initalize all data of the FEMinterface by, e.g., \ ``fem = FEMinterface()``\
- | \ *example*\ :

.. code-block:: python

  #**** this is not an example, just a description for internal variables ****
  #default values for member variables stored internally in FEMinterface fem and typical structure:
  dictionary of different node lists:
  fem.nodes = {}                 # {'Position':np.array([[x0,y0,z0],...]), 'RigidBodyRxyz':np.array([[x0,y0,z0,alpha0,beta0,gamma0],...]),  },...]
  list of elements (element connectivity):
  fem.elements = []              # [{'Name':'identifier', 'Tet4':np.array([[n0,n1,n2,n3],...]), 'Hex8':np.array([[n0,...,n7],...]),  },...]
  fem.massMatrix = None          # scipy csr_matrix
  fem.stiffnessMatrix= None      # scipy csr_matrix
  surface sets with faces, usually for drawing:
  fem.surface = []               # [{'Name':'identifier', 'Trigs':np.array([[n0,n1,n2],...]), 'Quads':np.array([[n0,...,n3],...]),  },...]
  node sets for boundary conditions, etc.
  fem.nodeSets = []              # [{'Name':'identifier', 'NodeNumbers':np.array([n_0,...,n_ns]), 'NodeWeights':np.array([w_0,...,w_ns])},...]
  element sets, e.g., for different domains, etc.
  fem.elementSets = []           # [{'Name':'identifier', 'ElementNumbers':np.array([n_0,...,n_ns])},...]
  mode basis: 'NormalModes' are eigenmodes, 'HCBmodes' are Craig-Bampton modes including static modes:
  fem.modeBasis = {}             # {'matrix':np.array([[Psi_00,Psi_01, ..., Psi_0m],...,[Psi_n0,Psi_n1, ..., Psi_nm]]),'type':'NormalModes'}
  eigenvalues related to eigenvectors in mode basis:
  fem.eigenValues = []           # np.array([ev0, ev1, ...])
  fem.postProcessingModes = {}   # {'matrix':<matrix (np.array) containing stress components (xx,yy,zz,yz,xz,xy) in each column, rows are for every mesh node>,'outputVariableType':exudyn.OutputVariableType.StressLocal}


----

.. _sec-fem-feminterface-getdictionary:

Class function: GetDictionary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetDictionary <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1882>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | get dictionary containing current data of FEMinterface

----

.. _sec-fem-feminterface-setwithdictionary:

Class function: SetWithDictionary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SetWithDictionary <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1906>`__\ (\ ``self``\ , \ ``dictData``\ , \ ``warn = True``\ , \ ``fromNPZ = False``\ )

- | \ *classFunction*\ :
  | set dictionary containing current data for FEMinterface; used internally

----

.. _sec-fem-feminterface-savetofile:

Class function: SaveToFile
^^^^^^^^^^^^^^^^^^^^^^^^^^
`SaveToFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L1947>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``fileVersion = 3``\ , \ ``mode = None``\ )

- | \ *classFunction*\ :
  | save all data (nodes, elements, ...) to a data filename; this function is much faster than the text-based import functions; note that HDF5 and PKL formats lead to smaller files
- | \ *input*\ :
  | \ ``fileName``\ : string for path and file name; if no ending is provided ==> ".npy" will be added and NumPy format will be used; alternatives: '.pkl' ending uses Python's pickle method (smaller files) and '.hdf5' uses the HDF5 file format, but requires the python package h5py to be installed!
  | \ ``fileVersion``\ : FOR EXPERTS: this allows to store in older format, will be recovered when loading; must be integer; version must by > 0
  | \ ``mode``\ : default: numpy format ('NPZ'); alternatives: 'HDF5' (requires h5py package) and 'PKL' (pickle); NPY (deprecated, under Numpy 1.x)
- | \ *output*\ :
  | stores file
  | \ ``**nodes``\ : test with 10-node tets and 86154 nodes, 50752 elements and 20 modes (incl. stress modes) gives the timings for save+load: [NPY: 2.10s, PKL: 0.76s, HDF5: 0.69s] and file sizes [NPY: 1032MB, PKL: 580MB, HDF5: 581MB]

----

.. _sec-fem-feminterface-loadfromfile:

Class function: LoadFromFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`LoadFromFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2001>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``forceVersion = None``\ , \ ``mode = None``\ )

- | \ *classFunction*\ :
  | load all data (nodes, elements, ...) from a data filename previously stored with SaveToFile(...).
  | this function is much faster than the text-based import functions
- | \ *input*\ :
  | \ ``fileName``\ : string for path and file name; if no ending is provided ==> ".npz" will be added and NumPy format will be assumed; alternatives: '.pkl' ending uses Python's pickle method and '.hdf5' uses the HDF5 file format, but requires the python package h5py to be installed!
  | \ ``forceVersion``\ : FOR EXPERTS: this allows to store in older format, will be recovered when loading; must be integer; for old files, use forceVersion=0
- | \ *output*\ :
  | loads data into fem (note that existing values are not overwritten!); returns file version or None if version is not available

----

.. _sec-fem-feminterface-importfromabaqusinputfile:

Class function: ImportFromAbaqusInputFile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ImportFromAbaqusInputFile <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2069>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``typeName = 'Part'``\ , \ ``name = 'Part-1'``\ , \ ``verbose = False``\ , \ ``createSurfaceTrigs = True``\ , \ ``surfaceTrigsAll = False``\ )

- | \ *classFunction*\ :
  | import nodes and elements from Abaqus input file and create surface elements;
  | node numbers in elements are converted from 1-based indices to python's 0-based indices;
  | This function can only import one part or instance; this means that you have to merge all
  | instances or parts in order to use this function for import of flexible bodies for order reduction methods
- | \ *input*\ :
  | \ ``fileName``\ : file name incl. path
  | \ ``typeName``\ : this is what is searched for regarding nodes and elements, see your .inp file
  | \ ``name``\ : if there are several parts, this name should address the according part name
  | \ ``verbose``\ : use True for some debug information
  | \ ``createSurfaceTrigs``\ : if True, triangles are created for visualization (triangles both for Tet and Hex elements)
  | \ ``surfaceTrigsAll``\ : if False, visualization triangles are created at the surface; if True, surface triangles are created also for interior elements
- | \ *output*\ :
  | return node numbers as numpy array
- | \ *notes*\ :
  | only works for Hex8, Hex20, Tet4 and Tet10 (C3D4, C3D8, C3D8R, C3D10, C3D20, C3D20R) elements; some functionality is untested and works in limited cases; only works for one single part or instance

----

.. _sec-fem-feminterface-readmassmatrixfromabaqus:

Class function: ReadMassMatrixFromAbaqus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadMassMatrixFromAbaqus <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2239>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``type = 'SparseRowColumnValue'``\ )

- | \ *classFunction*\ :
  | read mass matrix from compressed row text format (exported from Abaqus); in order to export system matrices, write the following lines in your Abaqus input file:
  | \*STEP
  | \*MATRIX GENERATE, STIFFNESS, MASS
  | \*MATRIX OUTPUT, STIFFNESS, MASS, FORMAT=COORDINATE
  | \*End Step

----

.. _sec-fem-feminterface-readstiffnessmatrixfromabaqus:

Class function: ReadStiffnessMatrixFromAbaqus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadStiffnessMatrixFromAbaqus <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2248>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``type = 'SparseRowColumnValue'``\ )

- | \ *classFunction*\ :
  | read stiffness matrix from compressed row text format (exported from Abaqus)

----

.. _sec-fem-feminterface-getnodesofngsolveboundary:

Class function: GetNodesOfNGsolveBoundary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodesOfNGsolveBoundary <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2260>`__\ (\ ``self``\ , \ ``mesh``\ , \ ``boundaryName``\ )

- | \ *classFunction*\ :
  | internal function to get ngsolve mesh nodes of boundary with name boundaryName

----

.. _sec-fem-feminterface-createngsolveboundarynodesets:

Class function: CreateNGsolveBoundaryNodeSets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateNGsolveBoundaryNodeSets <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2277>`__\ (\ ``self``\ , \ ``mesh``\ , \ ``boundaryNamesList = None``\ , \ ``warnNodeSets = True``\ )

- | \ *classFunction*\ :
  | create node sets for given (or all) boundaries in NGsolve; node sets are added to existing node sets
- | \ *input*\ :
  | \ ``mesh``\ : a previously created \ ``ngs.mesh``\  (NGsolve mesh, see examples)
  | \ ``boundaryNamesList``\ : a List of boundary names used to define mesh boundaries or None; if given, node sets are only created for the given boundary names
- | \ *output*\ :
  | list of nodeSets according to FEMinterface nodeSets structure, a dictionary with 'Name', 'NodeNumbers' and 'NodeWeights'

----

.. _sec-fem-feminterface-importmeshfromngsolve:

Class function: ImportMeshFromNGsolve
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ImportMeshFromNGsolve <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2345>`__\ (\ ``self``\ , \ ``mesh``\ , \ ``density = None``\ , \ ``youngsModulus = None``\ , \ ``poissonsRatio = None``\ , \ ``materials = None``\ , \ ``createBoundaryNodeSets = True``\ , \ ``boundaryNamesList = None``\ , \ ``verbose = False``\ , \ ``meshOrder = 1``\ , \ ``**kwargs``\ )

- | \ *classFunction*\ :
  | import mesh from NETGEN/NGsolve and setup mechanical problem
- | \ *input*\ :
  | \ ``mesh``\ : a previously created \ ``ngs.mesh``\  (NGsolve mesh, see examples)
  | \ ``youngsModulus``\ : In case of single material: Young's modulus used for mechanical model
  | \ ``poissonsRatio``\ : In case of single material: Poisson's ratio used for mechanical model
  | \ ``density``\ : In case of single material: density used for mechanical model
  | \ ``materials``\ : dictionary of material dictionaries according to names in NGsolve mesh, containing youngsModulus, poissonsRatio and density per material, see example
  | \ ``createBoundaryNodeSets``\ : if True, during import named boundaries conditions of the mesh are transformed into node sets for further use during mode creation, etc.
  | \ ``boundaryNamesList``\ : given as list of boundary names to be used for boundary node sets or None (creating node sets for all boundaries)
  | \ ``meshOrder``\ : use 1 for linear elements and 2 for second order elements (recommended to use 2 for much higher accuracy!)
  | \ ``verbose``\ : set True to print out some status information
- | \ *output*\ :
  | creates according nodes, elements, in FEM and returns [bfM, bfK, fes] which are the (mass matrix M, stiffness matrix K) bilinear forms and the finite element space fes
- | \ *author*\ :
  | Johannes Gerstmayr, Joachim Schöberl
- | \ *notes*\ :
  | setting ngsolve.SetNumThreads(nt) you can select the number of treads that are used for assemble or other functionality with NGsolve functionality
- | \ *example*\ :

.. code-block:: python

  ... #assume you have a FEMinterface fem and a NGsolve mesh
  #we have to define specific materials and (if still in the mesh), the default material
  materials = {'default':{'youngsModulus':2.1e11, 'poissonsRatio':0.3, 'density':7800},
               'steel':{'youngsModulus':2.1e11, 'poissonsRatio':0.3, 'density':7800},
               'aluminum':{'youngsModulus':7e10, 'poissonsRatio':0.35, 'density':2700},
               }
  fem.ImportMeshFromNGsolve(self, mesh, materials)
  #==> fem has now nodes, elements, node sets, etc. set according to mesh


----

.. _sec-fem-feminterface-computeeigenmodesngsolve:

Class function: ComputeEigenmodesNGsolve
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeEigenmodesNGsolve <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2517>`__\ (\ ``self``\ , \ ``bfM``\ , \ ``bfK``\ , \ ``nModes``\ , \ ``maxEigensolveIterations = 40``\ , \ ``excludeRigidBodyModes = 0``\ , \ ``verbose = False``\ )

- | \ *classFunction*\ :
  | compute nModes smallest eigenvalues and eigenmodes from mass and stiffnessMatrix; store mode vectors in modeBasis, but exclude a number of 'excludeRigidBodyModes' rigid body modes from modeBasis; uses scipy for solution of generalized eigenvalue problem
- | \ *input*\ :
  | \ ``nModes``\ : prescribe the number of modes to be computed; total computed modes are  (nModes+excludeRigidBodyModes), but only nModes with smallest absolute eigenvalues are considered and stored
  | \ ``excludeRigidBodyModes``\ : if rigid body modes are expected (in case of free-free modes), then this number specifies the number of eigenmodes to be excluded in the stored basis (usually 6 modes in 3D)
  | \ ``maxEigensolveIterations``\ : maximum number of iterations for iterative eigensolver; default=40
  | \ ``verbose``\ : if True, output some relevant information during solving
- | \ *output*\ :
  | eigenmodes are stored internally in FEMinterface as 'modeBasis' and eigenvalues as 'eigenValues'
- | \ *author*\ :
  | Johannes Gerstmayr, Joachim Schöberl

----

.. _sec-fem-feminterface-computehurtycraigbamptonmodesngsolve:

Class function: ComputeHurtyCraigBamptonModesNGsolve
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeHurtyCraigBamptonModesNGsolve <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2559>`__\ (\ ``self``\ , \ ``bfM``\ , \ ``bfK``\ , \ ``boundaryNodesList``\ , \ ``nEigenModes``\ , \ ``maxEigensolveIterations = 40``\ , \ ``verbose = False``\ )

- | \ *classFunction*\ :
  | compute static  and eigen modes based on Hurty-Craig-Bampton, for details see theory part Section :ref:`sec-theory-cms`\ . This function uses internal computational functionality of NGsolve and is often much faster than the scipy variant
- | \ *input*\ :
  | \ ``bfM``\ : bilinearform for mass matrix as retured in ImportMeshFromNGsolve(...)
  | \ ``bfK``\ : bilinearform for stiffness matrix as retured in ImportMeshFromNGsolve(...)
  | \ ``boundaryNodesList``\ : [nodeList0, nodeList1, ...] a list of node lists, each of them representing a set of 'Position' nodes for which a rigid body interface (displacement/rotation and force/torque) is created; NOTE THAT boundary nodes may not overlap between the different node lists (no duplicated node indices!)
  | \ ``nEigenModes``\ : number of eigen modes in addition to static modes (may be zero for RBE2 computationMode); eigen modes are computed for the case where all rigid body motions at boundaries are fixed; only smallest nEigenModes absolute eigenvalues are considered
  | \ ``maxEigensolveIterations``\ : maximum number of iterations for iterative eigensolver; default=40
  | \ ``verbose``\ : if True, output some relevant information during solving
- | \ *output*\ :
  | stores computed modes in self.modeBasis and abs(eigenvalues) in self.eigenValues
- | \ *author*\ :
  | Johannes Gerstmayr, Joachim Schöberl

----

.. _sec-fem-feminterface-computepostprocessingmodesngsolve:

Class function: ComputePostProcessingModesNGsolve
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputePostProcessingModesNGsolve <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2700>`__\ (\ ``self``\ , \ ``fes``\ , \ ``material = 0``\ , \ ``outputVariableType = 'OutputVariableType.StressLocal'``\ , \ ``verbose = False``\ )

- | \ *classFunction*\ :
  | compute special stress or strain modes in order to enable visualization of stresses and strains in ObjectFFRFreducedOrder; takes a NGsolve fes as input and uses internal NGsolve methods to efficiently compute stresses or strains
- | \ *input*\ :
  | \ ``fes``\ : finite element space as retured in ImportMeshFromNGsolve(...)
  | \ ``material``\ : specify material properties for computation of stresses, using a material class, e.g. material = KirchhoffMaterial(Emodulus, nu, rho); not needed for strains (material = 0)
  | \ ``outputVariableType``\ : specify either exudyn.OutputVariableType.StressLocal or exudyn.OutputVariableType.StrainLocal as the desired output variables
- | \ *output*\ :
  | post processing modes are stored in FEMinterface in local variable postProcessingModes as a dictionary, where 'matrix' represents the modes and 'outputVariableType' stores the type of mode as a OutputVariableType
- | \ *author*\ :
  | Johannes Gerstmayr, Joachim Schöberl
- | \ *notes*\ :
  | This function is implemented in Python and rather slow for larger meshes; for NGsolve / Netgen meshes, see the according ComputePostProcessingModesNGsolve function, which is usually much faster

----

.. _sec-fem-feminterface-getmassmatrix:

Class function: GetMassMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetMassMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2775>`__\ (\ ``self``\ , \ ``sparse = True``\ )

- | \ *classFunction*\ :
  | get sparse mass matrix in according format

----

.. _sec-fem-feminterface-getstiffnessmatrix:

Class function: GetStiffnessMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetStiffnessMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2782>`__\ (\ ``self``\ , \ ``sparse = True``\ )

- | \ *classFunction*\ :
  | get sparse stiffness matrix in according format

----

.. _sec-fem-feminterface-numberofnodes:

Class function: NumberOfNodes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`NumberOfNodes <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2789>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | get total number of nodes

----

.. _sec-fem-feminterface-getnodepositionsasarray:

Class function: GetNodePositionsAsArray
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodePositionsAsArray <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2801>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | get node points as array; only possible, if there exists only one type of Position nodes
- | \ *notes*\ :
  | in order to obtain a list of certain node positions, see example
- | \ *example*\ :

.. code-block:: python

  p=GetNodePositionsAsArray(self)[42] #get node 42 position
  nodeList=[1,13,42]
  pArray=GetNodePositionsAsArray(self)[nodeList] #get np.array with positions of node indices


----

.. _sec-fem-feminterface-getnodepositionsmean:

Class function: GetNodePositionsMean
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodePositionsMean <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2809>`__\ (\ ``self``\ , \ ``nodeNumberList``\ )

- | \ *classFunction*\ :
  | get mean (average) position of nodes defined by list of node numbers

----

.. _sec-fem-feminterface-numberofcoordinates:

Class function: NumberOfCoordinates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`NumberOfCoordinates <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2816>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | get number of total nodal coordinates

----

.. _sec-fem-feminterface-getnodeatpoint:

Class function: GetNodeAtPoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodeAtPoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2824>`__\ (\ ``self``\ , \ ``point``\ , \ ``tolerance = 1e-5``\ , \ ``raiseException = True``\ )

- | \ *classFunction*\ :
  | get node number for node at given point, e.g. p=[0.1,0.5,-0.2], using a tolerance (+/-) if coordinates are available only with reduced accuracy
  | if not found, it returns an invalid index

----

.. _sec-fem-feminterface-getnodesinplane:

Class function: GetNodesInPlane
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodesInPlane <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2840>`__\ (\ ``self``\ , \ ``point``\ , \ ``normal``\ , \ ``tolerance = 1e-5``\ )

- | \ *classFunction*\ :
  | get node numbers in plane defined by point p and (normalized) normal vector n using a tolerance for the distance to the plane
  | if not found, it returns an empty list

----

.. _sec-fem-feminterface-getnodesincube:

Class function: GetNodesInCube
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodesInCube <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2856>`__\ (\ ``self``\ , \ ``pMin``\ , \ ``pMax``\ )

- | \ *classFunction*\ :
  | get node numbers in cube, given by pMin and pMax, containing the minimum and maximum x, y, and z coordinates
- | \ *output*\ :
  | returns list of nodes; if no nodes found, return an empty list
- | \ *example*\ :

.. code-block:: python

  nList = GetNodesInCube([-1,-0.2,0],[1,0.5,0.5])


----

.. _sec-fem-feminterface-getnodesonline:

Class function: GetNodesOnLine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodesOnLine <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2871>`__\ (\ ``self``\ , \ ``p1``\ , \ ``p2``\ , \ ``tolerance = 1e-5``\ )

- | \ *classFunction*\ :
  | get node numbers lying on line defined by points p1 and p2 and tolerance, which is accepted for points slightly outside the surface

----

.. _sec-fem-feminterface-getnodesoncylinder:

Class function: GetNodesOnCylinder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodesOnCylinder <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2877>`__\ (\ ``self``\ , \ ``p1``\ , \ ``p2``\ , \ ``radius``\ , \ ``tolerance = 1e-5``\ )

- | \ *classFunction*\ :
  | get node numbers lying on cylinder surface; cylinder defined by cylinder axes (points p1 and p2),
  | cylinder radius and tolerance, which is accepted for points slightly outside the surface
  | if not found, it returns an empty list

----

.. _sec-fem-feminterface-getnodesoncircle:

Class function: GetNodesOnCircle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodesOnCircle <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2905>`__\ (\ ``self``\ , \ ``point``\ , \ ``normal``\ , \ ``r``\ , \ ``tolerance = 1e-5``\ )

- | \ *classFunction*\ :
  | get node numbers lying on a circle, by point p, (normalized) normal vector n (which is the axis of the circle) and radius r
  | using a tolerance for the distance to the plane
  | if not found, it returns an empty list

----

.. _sec-fem-feminterface-getnodeweightsfromsurfaceareas:

Class function: GetNodeWeightsFromSurfaceAreas
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetNodeWeightsFromSurfaceAreas <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2925>`__\ (\ ``self``\ , \ ``nodeList``\ , \ ``normalizeWeights = True``\ )

- | \ *classFunction*\ :
  | return list of node weights based on surface triangle areas; surface triangles are identified as such for which all nodes of a triangle are on the surface
  | \ ``**nodes``\ : requires that surface triangles have been already built during import of finite element mesh, or by calling VolumeToSurfaceElements!
- | \ *input*\ :
  | \ ``nodeList``\ : list of local (Position) node numbers
  | \ ``normalizeWeights``\ : if True, weights are normalized to sum(weights)==1; otherwise, returned list contains areas according to nodes per
- | \ *output*\ :
  | numpy array with weights according to indices in node list

----

.. _sec-fem-feminterface-getsurfacetriangles:

Class function: GetSurfaceTriangles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetSurfaceTriangles <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2977>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return surface trigs as node number list (for drawing in EXUDYN and for node weights)

----

.. _sec-fem-feminterface-volumetosurfaceelements:

Class function: VolumeToSurfaceElements
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`VolumeToSurfaceElements <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L2987>`__\ (\ ``self``\ , \ ``verbose = False``\ )

- | \ *classFunction*\ :
  | generate surface elements from volume elements
  | stores the surface in self.surface
  | only works for one element list and only for element types 'Hex8', 'Hex20', 'Tet4' and 'Tet10'

----

.. _sec-fem-feminterface-getrigidbodyinertia:

Class function: GetRigidBodyInertia
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetRigidBodyInertia <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3109>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | get rigid body inertia parameters according to Exudyn's internal RigidBodyInertia class, to be used e.g. for CreateRigidBody

----

.. _sec-fem-feminterface-getgyroscopicmatrix:

Class function: GetGyroscopicMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetGyroscopicMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3135>`__\ (\ ``self``\ , \ ``rotationAxis = 2``\ , \ ``sparse = True``\ )

- | \ *classFunction*\ :
  | get gyroscopic matrix in according format; rotationAxis=[0,1,2] = [x,y,z]

----

.. _sec-fem-feminterface-scalemassmatrix:

Class function: ScaleMassMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ScaleMassMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3164>`__\ (\ ``self``\ , \ ``factor``\ )

- | \ *classFunction*\ :
  | scale (=multiply) mass matrix with factor

----

.. _sec-fem-feminterface-scalestiffnessmatrix:

Class function: ScaleStiffnessMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ScaleStiffnessMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3173>`__\ (\ ``self``\ , \ ``factor``\ )

- | \ *classFunction*\ :
  | scale (=multiply) stiffness matrix with factor

----

.. _sec-fem-feminterface-addelasticsupportatnode:

Class function: AddElasticSupportAtNode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddElasticSupportAtNode <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3183>`__\ (\ ``self``\ , \ ``nodeNumber``\ , \ ``springStiffness = [1e8,1e8,1e8]``\ )

- | \ *classFunction*\ :
  | modify stiffness matrix to add elastic support (joint, etc.) to a node; nodeNumber zero based (as everywhere in the code...)
  | springStiffness must have length according to the node size

----

.. _sec-fem-feminterface-addnodemass:

Class function: AddNodeMass
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`AddNodeMass <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3199>`__\ (\ ``self``\ , \ ``nodeNumber``\ , \ ``addedMass``\ )

- | \ *classFunction*\ :
  | modify mass matrix by adding a mass to a certain node, modifying directly the mass matrix

----

.. _sec-fem-feminterface-createlinearfemobjectgenericode2:

Class function: CreateLinearFEMObjectGenericODE2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateLinearFEMObjectGenericODE2 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3219>`__\ (\ ``self``\ , \ ``mbs``\ , \ ``color = [0.9,0.4,0.4,1.]``\ )

- | \ *classFunction*\ :
  | create GenericODE2 object out of (linear) FEM model; uses always the sparse matrix mode, independent of the solver settings; this model can be directly used inside the multibody system as a static or dynamic FEM subsystem undergoing small deformations; computation is several magnitudes slower than ObjectFFRFreducedOrder
- | \ *input*\ :
  | mbs: multibody system to which the GenericODE2 is added
- | \ *output*\ :
  | return list [oGenericODE2, nodeList] containing object number of GenericODE2 as well as the list of mbs node numbers of all NodePoint nodes

----

.. _sec-fem-feminterface-createnonlinearfemobjectgenericode2ngsolve:

Class function: CreateNonlinearFEMObjectGenericODE2NGsolve
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateNonlinearFEMObjectGenericODE2NGsolve <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3261>`__\ (\ ``self``\ , \ ``mbs``\ , \ ``mesh``\ , \ ``density``\ , \ ``youngsModulus``\ , \ ``poissonsRatio``\ , \ ``meshOrder = 1``\ , \ ``color = [0.9,0.4,0.4,1.]``\ )

- | \ *classFunction*\ :
  | create GenericODE2 object fully nonlinear FEM model using NGsolve; uses always the sparse matrix mode, independent of the solver settings; this model can be directly used inside the multibody system as a static or dynamic nonlinear FEM subsystem undergoing large deformations; computation is several magnitudes slower than ObjectFFRFreducedOrder
- | \ *input*\ :
  | \ ``mbs``\ : multibody system to which the GenericODE2 is added
  | \ ``mesh``\ : a previously created \ ``ngs.mesh``\  (NGsolve mesh, see examples)
  | \ ``youngsModulus``\ : Young's modulus used for mechanical model
  | \ ``poissonsRatio``\ : Poisson's ratio used for mechanical model
  | \ ``density``\ : density used for mechanical model
  | \ ``meshOrder``\ : use 1 for linear elements and 2 for second order elements (recommended to use 2 for much higher accuracy!)
- | \ *output*\ :
  | return list [oGenericODE2, nodeList] containing object number of GenericODE2 as well as the list of mbs node numbers of all NodePoint nodes
- | \ *author*\ :
  | Johannes Gerstmayr, Joachim Schöberl
- | \ *notes*\ :
  | The interface to NETGEN/NGsolve has been created together with Joachim Schöberl, main developer
  | of NETGEN/NGsolve ; Thank's a lot!
  | download NGsolve at: https://ngsolve.org/
  | NGsolve needs Python 3.7 (64bit) ==> use according EXUDYN version!
  | note that node/element indices in the NGsolve mesh are 1-based and need to be converted to 0-base!

----

.. _sec-fem-feminterface-computeeigenmodes:

Class function: ComputeEigenmodes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeEigenmodes <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3366>`__\ (\ ``self``\ , \ ``nModes``\ , \ ``excludeRigidBodyModes = 0``\ , \ ``useSparseSolver = True``\ )

- | \ *classFunction*\ :
  | compute nModes smallest eigenvalues and eigenmodes from mass and stiffnessMatrix; store mode vectors in modeBasis, but exclude a number of 'excludeRigidBodyModes' rigid body modes from modeBasis; uses scipy for solution of generalized eigenvalue problem
- | \ *input*\ :
  | \ ``nModes``\ : prescribe the number of modes to be computed; total computed modes are  (nModes+excludeRigidBodyModes), but only nModes with smallest absolute eigenvalues are considered and stored
  | \ ``excludeRigidBodyModes``\ : if rigid body modes are expected (in case of free-free modes), then this number specifies the number of eigenmodes to be excluded in the stored basis (usually 6 modes in 3D)
  | \ ``useSparseSolver``\ : for larger systems, the sparse solver needs to be used, which iteratively solves the problem and uses a random number generator (internally in ARPACK): therefore, results are not fully repeatable!!!
- | \ *output*\ :
  | eigenmodes are stored internally in FEMinterface as 'modeBasis' and eigenvalues as 'eigenValues'
- | \ *notes*\ :
  | for NGsolve / Netgen meshes, see the according ComputeEigenmodesNGsolve function, which is usually much faster

----

.. _sec-fem-feminterface-computeeigenmodeswithboundarynodes:

Class function: ComputeEigenModesWithBoundaryNodes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeEigenModesWithBoundaryNodes <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3410>`__\ (\ ``self``\ , \ ``boundaryNodes``\ , \ ``nEigenModes``\ , \ ``useSparseSolver = True``\ )

- | \ *classFunction*\ :
  | compute eigenmodes, using a set of boundary nodes that are all fixed; very similar to ComputeEigenmodes, but with additional definition of (fixed) boundary nodes.
- | \ *input*\ :
  | \ ``boundaryNodes``\ : a list of boundary node indices, refering to 'Position' type nodes in FEMinterface; all coordinates of these nodes are fixed for the computation of the modes
  | \ ``nEigenModes``\ : prescribe the number of modes to be computed; only nEigenModes with smallest abs(eigenvalues) are considered and stored
  | \ ``useSparseSolver``\ : [yet NOT IMPLEMENTED] for larger systems, the sparse solver needs to be used, which iteratively solves the problem and uses a random number generator (internally in ARPACK): therefore, results are not fully repeatable!!!
- | \ *output*\ :
  | eigenmodes are stored internally in FEMinterface as 'modeBasis' and eigenvalues as 'eigenValues'

----

.. _sec-fem-feminterface-computehurtycraigbamptonmodes:

Class function: ComputeHurtyCraigBamptonModes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeHurtyCraigBamptonModes <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3495>`__\ (\ ``self``\ , \ ``boundaryNodesList``\ , \ ``nEigenModes``\ , \ ``useSparseSolver = True``\ , \ ``computationMode = HCBstaticModeSelection.RBE2``\ , \ ``boundaryNodesWeights = []``\ , \ ``excludeRigidBodyMotion = True``\ , \ ``RBE3secondMomentOfAreaWeighting = True``\ , \ ``verboseMode = False``\ , \ ``timerTreshold = 20000``\ )

- | \ *classFunction*\ :
  | compute static  and eigen modes based on Hurty-Craig-Bampton, for details see theory part Section :ref:`sec-theory-cms`\ . Note that this function may need significant time, depending on your hardware, but 50.000 nodes will require approx. 1-2 minutes and more nodes typically raise time more than linearly.
- | \ *input*\ :
  | \ ``boundaryNodesList``\ : [nodeList0, nodeList1, ...] a list of node lists, each of them representing a set of 'Position' nodes for which a rigid body interface (displacement/rotation and force/torque) is created; NOTE THAT boundary nodes may not overlap between the different node lists (no duplicated node indices!)
  | \ ``nEigenModes``\ : number of eigen modes in addition to static modes (may be zero for RBE2/RBE3 computationMode); eigen modes are computed for the case where all rigid body motions at boundaries are fixed; only smallest nEigenModes absolute eigenvalues are considered
  | \ ``useSparseSolver``\ : for more than approx.~500 nodes, it is recommended to use the sparse solver; dense mode not available for RBE3
  | \ ``computationMode``\ : see class HCBstaticModeSelection for available modes; select RBE2 / RBE3 as standard, which is both efficient and accurate and which uses rigid-body-interfaces (6 independent modes) per boundary; RBE3 mode uses singular value decomposition, which requires full matrices for boundary nodes; this becomes slow in particular if the number of a single boundary node set gets larger than 500 nodes
  | \ ``boundaryNodesWeights``\ : according list of weights with same order as boundaryNodesList, as returned e.g. by FEMinterface.GetNodeWeightsFromSurfaceAreas(...)
  | \ ``excludeRigidBodyMotion``\ : if True (recommended), the first set of boundary modes is eliminated, which defines the reference conditions for the FFRF object
  | \ ``RBE3secondMomentOfAreaWeighting``\ : if True, the weighting of RBE3 boundaries is done according to second moment of area; if False, the more conventional (but less appropriate) quadratic distance to reference point weighting is used
  | \ ``verboseMode``\ : if True, some additional output is printed
  | \ ``timerTreshold``\ : for more DOF than this number, CPU times are printed even with verboseMode=False
- | \ *output*\ :
  | stores computed modes in self.modeBasis and abs(eigenvalues) in self.eigenValues
- | \ *notes*\ :
  | for NGsolve / Netgen meshes, see the according ComputeHurtyCraigBamptonModesNGsolve function, which is usually much faster - currently only implemented for RBE2 case

----

.. _sec-fem-feminterface-geteigenfrequencieshz:

Class function: GetEigenFrequenciesHz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetEigenFrequenciesHz <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3903>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | return list of eigenvalues in Hz of previously computed eigenmodes

----

.. _sec-fem-feminterface-computepostprocessingmodes:

Class function: ComputePostProcessingModes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputePostProcessingModes <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L3963>`__\ (\ ``self``\ , \ ``material = 0``\ , \ ``outputVariableType = 'OutputVariableType.StressLocal'``\ , \ ``numberOfThreads = 1``\ )

- | \ *classFunction*\ :
  | compute special stress or strain modes in order to enable visualization of stresses and strains in ObjectFFRFreducedOrder;
- | \ *input*\ :
  | \ ``material``\ : specify material properties for computation of stresses, using a material class, e.g. material = KirchhoffMaterial(Emodulus, nu, rho); not needed for strains
  | \ ``outputVariableType``\ : specify either exudyn.OutputVariableType.StressLocal or exudyn.OutputVariableType.StrainLocal as the desired output variables
  | \ ``numberOfThreads``\ : if numberOfThreads=1, it uses single threaded computation; if numberOfThreads>1, it uses the multiprocessing pools functionality, which requires that all code in your main file must be encapsulated within an if clause "if __name__ == '__main__':", see examples; if numberOfThreads==-1, it uses all threads/CPUs available
- | \ *output*\ :
  | post processing modes are stored in FEMinterface in local variable postProcessingModes as a dictionary, where 'matrix' represents the modes and 'outputVariableType' stores the type of mode as a OutputVariableType
- | \ *notes*\ :
  | This function is implemented in Python and rather slow for larger meshes; for NGsolve / Netgen meshes, see the according ComputePostProcessingModesNGsolve function, which is usually much faster

----

.. _sec-fem-feminterface-computecampbelldiagram:

Class function: ComputeCampbellDiagram
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeCampbellDiagram <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L4080>`__\ (\ ``self``\ , \ ``terminalFrequency``\ , \ ``nEigenfrequencies = 10``\ , \ ``frequencySteps = 25``\ , \ ``rotationAxis = 2``\ , \ ``plotDiagram = False``\ , \ ``verbose = False``\ , \ ``useCorotationalFrame = False``\ , \ ``useSparseSolver = False``\ )

- | \ *classFunction*\ :
  | compute Campbell diagram for given mechanical system
  | create a first order system Axd + Bx = 0 with x= [q,qd]' and compute eigenvalues
  | takes mass M, stiffness K and gyroscopic matrix G from FEMinterface
  | currently only uses dense matrices, so it is limited to approx. 5000 unknowns!
- | \ *input*\ :
  | \ ``terminalFrequency``\ : frequency in Hz, up to which the campbell diagram is computed
  | \ ``nEigenfrequencies``\ : gives the number of computed eigenfrequencies(modes), in addition to the rigid body mode 0
  | \ ``frequencySteps``\ : gives the number of increments (gives frequencySteps+1 total points in campbell diagram)
  | \ ``rotationAxis``\ :[0,1,2] = [x,y,z] provides rotation axis
  | \ ``plotDiagram``\ : if True, plots diagram for nEigenfrequencies befor terminating
  | \ ``verbose``\ : if True, shows progress of computation; if verbose=2, prints also eigenfrequencies
  | \ ``useCorotationalFrame``\ : if False, the classic rotor dynamics formulation for rotationally-symmetric rotors is used, where the rotor can be understood in a Lagrangian-Eulerian manner: the rotation is represented by an additional (Eulerian) velocity in rotation direction; if True, the corotational frame is used, which gives a factor 2 in the gyroscopic matrix and can be used for non-symmetric rotors as well
  | \ ``useSparseSolver``\ : for larger systems, the sparse solver needs to be used for creation of system matrices and for the eigenvalue solver (uses a random number generator internally in ARPACK, therefore, results are not fully repeatable!!!)
- | \ *output*\ :
  | [listFrequencies, campbellFrequencies]
  | \ ``listFrequencies``\ : list of computed frequencies
  | \ ``campbellFrequencies``\ : array of campbell frequencies per eigenfrequency of system

----

.. _sec-fem-feminterface-checkconsistency:

Class function: CheckConsistency
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CheckConsistency <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L4265>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | perform some consistency checks

----

.. _sec-fem-feminterface-readmassmatrixfromansys:

Class function: ReadMassMatrixFromAnsys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadMassMatrixFromAnsys <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L4286>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``dofMappingVectorFile``\ , \ ``sparse = True``\ , \ ``verbose = False``\ )

- | \ *classFunction*\ :
  | read mass matrix from CSV format (exported from Ansys)

----

.. _sec-fem-feminterface-readstiffnessmatrixfromansys:

Class function: ReadStiffnessMatrixFromAnsys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadStiffnessMatrixFromAnsys <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L4302>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``dofMappingVectorFile``\ , \ ``sparse = True``\ , \ ``verbose = False``\ )

- | \ *classFunction*\ :
  | read stiffness matrix from CSV format (exported from Ansys)

----

.. _sec-fem-feminterface-readnodalcoordinatesfromansys:

Class function: ReadNodalCoordinatesFromAnsys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadNodalCoordinatesFromAnsys <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L4318>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``verbose = False``\ )

- | \ *classFunction*\ :
  | read nodal coordinates (exported from Ansys as .txt-File)

----

.. _sec-fem-feminterface-readelementsfromansys:

Class function: ReadElementsFromAnsys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ReadElementsFromAnsys <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/FEM.py\#L4323>`__\ (\ ``self``\ , \ ``fileName``\ , \ ``verbose = False``\ )

- | \ *classFunction*\ :
  | read elements (exported from Ansys as .txt-File)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Ex), \ `netgenSTLtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/netgenSTLtest.py>`_\  (Ex), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolveFFRF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRF.py>`_\  (Ex), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TM), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `compareAbaqusAnsysRotorEigenfrequencies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/compareAbaqusAnsysRotorEigenfrequencies.py>`_\  (TM)

