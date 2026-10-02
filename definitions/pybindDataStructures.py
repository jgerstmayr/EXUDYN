#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the data structures MatrixContainer, GraphicsMaterialList and the vector/matrix lists;
#           their stubs go into stubEnums.pyi, as they are needed before the classes.
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py into pybind_manual_classes.h, the stub fragments and
#           the Python-C++ interface documentation.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created in autoGeneratePyBindings.py), 2026-09-14 (moved to definitions/)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()

pb.CreateNewRSTfile('DataStructures')

pb.AddDocu(text="""
This section describes a set of special data structures which are used in the Python-C++ interface, 
such as a MatrixContainer for dense/sparse matrices or a list of 3D vectors. 
Note that there are many native data types, such as lists, dicts and numpy arrays (e.g., 3D vectors), 
which are not described here as they are native to Pybind11, but can be passed as arguments when appropriate.
""", section='Data structures', sectionLevel=1,sectionLabel='sec:cinterface:dataStructures')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for MatrixContainer
classStr = 'PyMatrixContainer'
pyClassStr = 'MatrixContainer'

pb.DefPyStartClass(classStr, pyClassStr, 'The MatrixContainer is a versatile representation for dense and sparse matrices. NOTE: if the MatrixContainer is constructed from a numpy array or a list of lists, both representing a dense matrix, it will go into dense mode; if it is initialized with a scipy sparse csr matrix, it will go into sparse mode. Examples:',
                    subSection=True, 
                    labelName='sec:MatrixContainer') #section with this label was earlier in theory section

pb.AddDocuCodeBlock(code="""
#Create empty MatrixContainer:
from scipy.sparse import csr_matrix
from exudyn import MatrixContainer
mc = MatrixContainer() #empty matrix, dense mode

#Create MatrixContainer with dense matrix:
#container can be initialized with a dense matrix, using list of lists or a numpy array, e.g.:
matrix = np.eye(3)
#stores matrices internally in dense mode:
mcDense1 = MatrixContainer(matrix)
mcDense2 = MatrixContainer([[1,2],[3,4]])

#container can be initialized with a scipy csr sparse matrix, then being stored as sparse matrix
mcSparse = MatrixContainer(csr_matrix(matrix))

#Set with dense pyArray (a numpy array): 
pyArray = np.array(matrix)
mc.SetWithDenseMatrix(pyArray, useDenseMatrix = True)

#Set empty matrix:
mc.SetWithDenseMatrix([[]], useDenseMatrix = True)

#Set with list of lists, stored as sparse matrix:
mc.SetWithDenseMatrix([[1,2],[3,4]], useDenseMatrix = False)

#Set with sparse triplets (list of lists or numpy array):
mc.SetWithSparseMatrix([[0,0,13.3],[1,1,4.2],[1,2,42.]], 
                       numberOfRows=2, numberOfColumns=3, 
                       useDenseMatrix=True)

print(mc)
#gives dense matrix:
#[[13.3  0.   0. ]
# [ 0.   4.2 42. ]]

#Set with scipy matrix:
#WARNING: only use csr_matrix
#         csc_matrix would basically run, but gives the transposed!!!
spmat = csr_matrix(matrix) 
mc.SetWithSparseMatrix(spmat) #takes rows and column format automatically

#initialize and add triplets later on
mc.Initialize(3,3,useDenseMatrix=False)
mc.AddSparseMatrix(spmat, factor=1)
#can also add smaller matrix
mc.AddSparseMatrix(csr_matrix(np.eye(2)), factor=0.5)
print('mc8=',mc)

""")

pb.DefStartTable(pyClassStr)

pb.CppCode('        .def(py::init<const py::object&>(), py::arg("matrix"))\n') #constructor with numpy array or list of lists

pb.DefPyFunctionAccess(cClass=classStr, pyName='Initialize', cName='Initialize', 
                        argList=['numberOfRows', 'numberOfColumns', 'useDenseMatrix'],
                        defaultArgs=['','','True'],
                        description="initialize MatrixContainer with number of rows and columns and set dense/sparse mode",
                        argTypes=['int','int','bool'],
                        returnType='None',
                        )
                                              
pb.DefPyFunctionAccess(cClass=classStr, pyName='SetWithDenseMatrix', cName='SetWithDenseMatrix', 
                        argList=['pyArray','useDenseMatrix','factor'],
                        defaultArgs=['','False','1.'],
                        description="set MatrixContainer with dense numpy array of size (n x m); array (=matrix) contains values and matrix size information; if useDenseMatrix=True, matrix will be stored internally as dense matrix, otherwise it will be converted and stored as sparse matrix (which may speed up computations for larger problems); pyArray is multiplied with given factor",
                        argTypes=['ArrayLike','bool','float'],
                        returnType='None',
                        )
                                              
pb.DefPyFunctionAccess(cClass=classStr, pyName='SetWithSparseMatrix', cName='SetWithSparseMatrix',
                        argList=['sparseMatrix','numberOfRows', 'numberOfColumns', 'useDenseMatrix','factor'],
                        defaultArgs=['','EXUstd::InvalidIndex','EXUstd::InvalidIndex','False','1.'],
                        description="set with scipy sparse csr_matrix (NOT: csc_matrix!) or with internal sparse triplet format (denoted as CSR): 'sparseMatrix' either contains a scipy matrix create with csr_matrix or a list of lists of sparse triplets (row, col, value) or the list of lists converted into numpy array; numberOfRowsInit and numberOfColumnsInit denote the size of the matrices, which are ignored in case of a scipy sparse matrix; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix triplets; the values of sparseMatrix are multiplied with the given factor before storing",
                        argTypes=[sparseMatrixType,'int','int','bool','float'],
                        returnType='None',
                        )
                                              
pb.DefPyFunctionAccess(cClass=classStr, pyName='AddSparseMatrix', cName='AddSparseMatrix', 
                        argList=['sparseMatrix','factor'],
                        defaultArgs=['','1.'],
                        description="add scipy sparse csr_matrix with factor to already initilized MatrixContainer; sparseMatrix must contain according scipy csr format, otherwise the behavior is undefined! This function allows to efficiently add submatrices to the MatrixContainer",
                        argTypes=[sparseMatrixType,'float'],
                        returnType='None',
                        )
                                                                                            
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                        description="convert MatrixContainer to numpy array (dense) or dictionary (sparse): containing nr. of rows, nr. of columns, numpy matrix with sparse triplets",
                        returnType='Union[dict,ArrayLike]',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Convert2DenseMatrix', cName='Convert2DenseMatrix', 
                        description="convert MatrixContainer to dense numpy array (SLOW and may fail for too large sparse matrices)",
                        returnType='ArrayLike',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='UseDenseMatrix', cName='UseDenseMatrix', 
                        description="returns True if dense matrix is used, otherwise False",
                        returnType='bool',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetAllZero', cName='SetAllZero', 
                        description="Set all values to zero; dense mode: set all matrix entries to zero (slow); sparse mode: set number of triplets to zero (fast)",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetWithSparseMatrixCSR', cName='SetWithSparseMatrixCSR', 
                        argList=['numberOfRowsInit', 'numberOfColumnsInit', 'pyArrayCSR', 'useDenseMatrix','factor'],
                        defaultArgs=['','','','False','1.'],
                        description="DEPRECATED: set with sparse CSR matrix format: numpy array 'pyArrayCSR' contains sparse triplet (row, col, value) per row; numberOfRows and numberOfColumns given extra; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix; the values of pyArrayCSR are multiplied by the given factor",
                        argTypes=['int','int',sparseMatrixType,'bool','float'],
                        returnType='None',
                        )
                                              

pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const PyMatrixContainer &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                        description="return the string representation of the MatrixContainer",
                        isLambdaFunction = True,
                        )

#++++++++++++++++
pb.DefPyFinishClass('MatrixContainer')



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for HT, the homogeneous transformation (#2780)
classStr = 'PyHT'
pyClassStr = 'HT'

pb.DefPyStartClass(classStr, pyClassStr, 'The HT is a homogeneous transformation - a rotation matrix A and a translation p, the 4x4 matrix [A p; 0 1] -, the frame of a rigid body, marker or joint. It is the C++ class of Exudyn, faster than the 4x4 numpy arrays of exudyn.rigidBodyUtilities: it stores the 12 numbers it needs, and a transformation set without rotation (identity, SetTranslation) skips the rotation in its products. Examples:',
                    subSection=True, labelName='sec:HT')

pb.AddDocuCodeBlock(code="""
import exudyn as exu
from exudyn.rigidBodyUtilities import RotationMatrixZ
H0 = exu.HT()                                              #identity
H1 = exu.HT(rotation=RotationMatrixZ(0.5), translation=[1,0,0])
H2 = exu.HT(translation=[0,2,0])                           #translation only
H = H1 * H2                                                #composition, an HT
p = H1 * [0.1,0,0]                                         #a point transformed, a numpy array
A, t = H.Get()                                             #rotation and translation
H44 = H.HT44()                                             #4x4 numpy array
Hinv = H.Inverse()
H.translation = [0,0,1]                                    #write access, the rotation is kept
""")

pb.DefStartTable(pyClassStr)

pb.CppCode('        .def(py::init<const py::object&, const py::object&>(), py::arg("rotation") = py::none(), py::arg("translation") = py::none())\n')

pb.CppCode('        .def_property("rotation", &PyHT::GetRotationPy, &PyHT::SetRotationPy)\n')
pb.DefDataAccess('rotation', 'the 3x3 rotation matrix as numpy array; setting it keeps the translation', dataType='ArrayLike')
pb.CppCode('        .def_property("translation", &PyHT::GetTranslationPy, &PyHT::SetTranslationPy)\n')
pb.DefDataAccess('translation', 'the translation as numpy array; setting it keeps the rotation', dataType='ArrayLike')

pb.DefPyFunctionAccess(cClass=classStr, pyName='Get', cName='GetPy',
                       description="[rotation, translation] as numpy arrays",
                       returnType='List[ArrayLike]',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Set', cName='SetPy',
                       argList=['rotation', 'translation'],
                       description="set the 3x3 rotation matrix and the translation",
                       argTypes=['ArrayLike', 'ArrayLike'],
                       returnType='None',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetIdentity', cName='SetIdentity',
                       description="set the identity: unit rotation, zero translation",
                       returnType='None',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetTranslation', cName='SetTranslationOnlyPy',
                       argList=['translation'],
                       description="set a translation and the unit rotation",
                       argTypes=['ArrayLike'],
                       returnType='None',
                       )

for axis in ['X', 'Y', 'Z']:
    pb.DefPyFunctionAccess(cClass=classStr, pyName='SetRotation' + axis, cName='SetRotation' + axis,
                           argList=['angle'],
                           description="set a rotation about the " + axis.lower() + "-axis by angle (in radians) and zero translation",
                           argTypes=['float'],
                           returnType='None',
                           )

pb.DefPyFunctionAccess(cClass=classStr, pyName='HT44', cName='GetHT44Py',
                       description="the 4x4 matrix [A p; 0 1] as numpy array",
                       returnType='ArrayLike',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Inverse', cName='GetInversePy',
                       description="the inverse transformation [A^T, -A^T p], an HT",
                       returnType='HT',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Invert', cName='Invert',
                       description="invert the transformation in place",
                       returnType='None',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='RotateVector', cName='RotateVectorPy',
                       argList=['vector'],
                       description="the rotated vector A*v, without the translation",
                       argTypes=['ArrayLike'],
                       returnType='ArrayLike',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='RotateVectorTransposed', cName='RotateVectorTransposedPy',
                       argList=['vector'],
                       description="the vector rotated back, A^T*v",
                       argTypes=['ArrayLike'],
                       returnType='ArrayLike',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='HasNoRotation', cName='HasNoRotation',
                       description="True if the transformation was set without rotation (identity, SetTranslation, or a product of such), which its products then skip; a given unit matrix does not set this",
                       returnType='bool',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__mul__',
                       cName='[](const PyHT &item, const py::object &other) {\n            return item.Multiply(other); }',
                       description="H1*H2, the composition of two transformations, an HT; H*v, the transformed point A*v+p of a 3D vector, a numpy array",
                       argList=['other'], argTypes=['Union[HT, ArrayLike]'],
                       returnType='Union[HT, ArrayLike]',
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__eq__',
                       cName='[](const PyHT &item, const PyHT &other) {\n            return (const HomogeneousTransformation&)item == (const HomogeneousTransformation&)other; }',
                       description="True if rotation and translation are equal, component by component",
                       argList=['other'], argTypes=['HT'],
                       returnType='bool',
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__',
                       cName='[](const PyHT &item) {\n            return item.ToString(); }',
                       description="the string representation of the HT",
                       returnType='str',
                       isLambdaFunction = True,
                       )

#++++++++++++++++
pb.DefPyFinishClass('HT')



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for GraphicsMaterialList


classStr = 'MainGraphicsMaterialList'
pyClassStr = 'GraphicsMaterialList'
pb.DefPyStartClass(classStr, pyClassStr, "The GraphicsMaterialList contains the list of materials (material properties) for visualization; currently, only the raytracer uses materials. Materials can be accessed via the variable materials in renderer of SystemContainer.", 
                    subSection=True, labelName='sec:GraphicsMaterialList')

pb.AddDocuCodeBlock(code="""
#access material 0:
mat0 = SC.renderer.materials[0]
#convert into dictionary for easier processing:
matDict = mat0.GetDictionary()
matDict['alpha'] = 0.5
#change material (e.g., using a data base):
mat0.SetDictionary(matDict)
#or directly update material
mat0.name = 'new name'
mat0.emission = [0.8,0.6,0.]

#update material in renderer:
SC.renderer.materials.Set(0,mat0)
#update material directly with dictionary:
SC.renderer.materials.Set(0,matDict)

#create new material:
mat10 = SC.renderer.materials.New()
mat10.reflectivity = 0.8
SC.renderer.materials.Append(mat10) #returns index of mat10

#10 default graphics materials in Exudyn
#listed here with default color and some properties:
#note the increased computational costs for reflection & transparency
#the material names are as follows:
SC.renderer.materials[0].name == "default"    #steel blue
SC.renderer.materials[1].name == "matt"       #green
SC.renderer.materials[2].name == "steel"      #grey (reflection)
SC.renderer.materials[3].name == "plastic"    #red (reflection)
SC.renderer.materials[4].name == "chrome"     #light grey (reflection)
SC.renderer.materials[5].name == "shiny"      #orange (reflection)
SC.renderer.materials[6].name == "transparent"#(transparency,slight refraction)
SC.renderer.materials[7].name == "glass"      #light grey (reflection,transparency,refraction)
SC.renderer.materials[8].name == "mirror"     #light grey (reflection)
SC.renderer.materials[9].name == "emission"   #light yellow

""") #keep empty line for RST


pb.DefStartTable(pyClassStr)

pb.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                        description="reset materials to 10 default materials",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                        argList=['material'],
                        description="add single material as dict or VSettingsMaterial to list; returns index of newly added material",
                        argTypes=['Any'],
                        returnType='int',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='New', cName='NewMaterial', 
                        description="Get new default material, which can be modified or appended to materials list",
                        returnType='VSettingsMaterial',
                        )
                                                                                                            
pb.DefPyFunctionAccess(cClass=classStr, pyName='Set', cName='PySetMaterial', 
                        argList=['indexOrName','material'],
                        description="set material with index 'materialIndex' as dict or VSettingsMaterial",
                        argTypes=['int','Any'],
                        returnType='None',
                        )
                                                                                                            
pb.DefPyFunctionAccess(cClass=classStr, pyName='Get', cName='GetMaterial', 
                        argList=['indexOrName'],
                        description="get material with index 'materialIndex' as VSettingsMaterial",
                        argTypes=['int'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetDict', cName='PyGetMaterialDict', 
                        argList=['indexOrName'],
                        description="get material with index 'materialIndex' as dict",
                        argTypes=['int'],
                        returnType='None',
                        )
                                                                                                            
pb.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const MainGraphicsMaterialList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Vector3DList, using len(data) where data is the Vector3DList",
                       isLambdaFunction = True,
                       )

#not possible, because sync with visSettings would be lost:
# pb.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
#                        cName='[](MainGraphicsMaterialList &item, Index index, const py::object& material) {\n            item.PySetMaterial(index, material); }', 
#                        description="set list item 'index' with material; usage: materials[index] = material",
#                        isLambdaFunction = True,
#                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                        cName='[](const MainGraphicsMaterialList &item, py::object index) {\n            return py::cast<const VSettingsMaterial&>(item.GetMaterial(index)); }', 
                        description="get reference access of material with 'index' as VSettingsMaterial",
                        isLambdaFunction = True,
                        )


# pb.DefPyFunctionAccess(cClass=classStr, pyName='__copy__', 
#                        cName='[](const PyVector3DList &item) {\n            return PyVector3DList(item); }', 
#                        description="copy method to be used for copy.copy(...); in fact does already deep copy",
#                        isLambdaFunction = True,
#                        )

# pb.DefPyFunctionAccess(cClass=classStr, pyName='__deepcopy__', 
#                        cName='[](const PyVector3DList &item, py::dict) {\n            return PyVector3DList(item); }, "memo"_a', 
#                        description="deepcopy method to be used for copy.copy(...)",
#                        isLambdaFunction = True,
#                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const MainGraphicsMaterialList &item) {\n            return EXUstd::ToString(item); }', 
                       description="return the string representation of the GraphicsMaterialList",
                       isLambdaFunction = True,
                       )

#++++++++++++++++
pb.DefPyFinishClass('GraphicsMaterialList')





#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyVector3DList


classStr = 'PyVector3DList'
pyClassStr = 'Vector3DList'
pb.DefPyStartClass(classStr, pyClassStr, r"""The Vector3DList is used to represent lists of 3D vectors. This is used to transfer such lists from Python to C++.



Usage:

- Create empty `Vector3DList` with `x = Vector3DList()`
- Create `Vector3DList` with list of numpy arrays:`x = Vector3DList([ numpy.array([1.,2.,3.]), numpy.array([4.,5.,6.]) ])`
- Create `Vector3DList` with list of lists `x = Vector3DList([[1.,2.,3.], [4.,5.,6.]])`
- Append item: `x.Append([0.,2.,4.])`
- Convert into list of numpy arrays: `x.GetPythonObject()`

""", subSection=True)

pb.DefStartTable(pyClassStr)

pb.CppCode('        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n') #constructor with numpy array or list of lists

pb.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                        argList=['pyArray'],
                        description="add single array or list to Vector3DList; array or list must have appropriate dimension!",
                        argTypes=[vector3D],
                        returnType='None',
                        )
                                                                                                            
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Vector3DList into (copied) list of numpy arrays",
                       returnType='List['+vector3D+']',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyVector3DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Vector3DList, using len(data) where data is the Vector3DList",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyVector3DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                       description="set list item 'index' with data, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyVector3DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                       description="get copy of list item with 'index' as vector",
                       isLambdaFunction = True,
                       )
#copy and deepcopy according to Pybind11, see https://pybind11.readthedocs.io/en/latest/advanced/classes.html#pickling-support
# .def("__copy__",  [](const Copyable &self) {
#     return Copyable(self);
# })
# .def("__deepcopy__", [](const Copyable &self, py::dict) {
#     return Copyable(self);
# }, "memo"_a);
pb.DefPyFunctionAccess(cClass=classStr, pyName='__copy__', 
                       cName='[](const PyVector3DList &item) {\n            return PyVector3DList(item); }', 
                       description="copy method to be used for copy.copy(...); in fact does already deep copy",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__deepcopy__', 
                       cName='[](const PyVector3DList &item, py::dict) {\n            return PyVector3DList(item); }, "memo"_a', 
                       description="deepcopy method to be used for copy.copy(...)",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyVector3DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Vector3DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

#++++++++++++++++
pb.DefPyFinishClass('PyVector3DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyVector2DList
classStr = 'PyVector2DList'
pyClassStr = 'Vector2DList'
pb.DefPyStartClass(classStr, pyClassStr, r"""The Vector2DList is used to represent lists of 2D vectors. This is used to transfer such lists from Python to C++.



Usage:
- Create empty `Vector2DList` with `x = Vector2DList()`
- Create `Vector2DList` with list of numpy arrays:

  `x = Vector2DList([ numpy.array([1.,2.]), numpy.array([4.,5.]) ])`
- Create `Vector2DList` with list of lists `x = Vector2DList([[1.,2.], [4.,5.]])`
- Append item: `x.Append([0.,2.])`
- Convert into list of numpy arrays: `x.GetPythonObject()`
- similar to Vector3DList !

""", subSection=True)

pb.DefStartTable(pyClassStr)

pb.CppCode('        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n') #constructor with numpy array or list of lists

pb.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                       argList=['pyArray'],
                       description="add single array or list to Vector2DList; array or list must have appropriate dimension!",
                       argTypes=[vector2D],
                       returnType='None',
                       )
                                                                                                    
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Vector2DList into (copied) list of numpy arrays",
                       returnType='List['+vector2D+']',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyVector2DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Vector2DList, using len(data) where data is the Vector2DList",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyVector2DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                       description="set list item 'index' with data, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyVector2DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                       description="get copy of list item with 'index' as vector",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__copy__', 
                       cName='[](const PyVector2DList &item) {\n            return PyVector2DList(item); }', 
                       description="copy method to be used for copy.copy(...); in fact does already deep copy",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__deepcopy__', 
                       cName='[](const PyVector2DList &item, py::dict) {\n            return PyVector2DList(item); }, "memo"_a', 
                       description="deepcopy method to be used for copy.copy(...)",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyVector2DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Vector2DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

#++++++++++++++++
pb.DefPyFinishClass('PyVector2DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyVector6DList
classStr = 'PyVector6DList'
pyClassStr = 'Vector6DList'
pb.DefPyStartClass(classStr, pyClassStr, r"""The Vector6DList is used to represent lists of 6D vectors. This is used to transfer such lists from Python to C++.



Usage:
- Create empty `Vector6DList` with `x = Vector6DList()`
- Convert into list of numpy arrays: `x.GetPythonObject()`
- similar to Vector3DList !

""", subSection=True)

pb.DefStartTable(pyClassStr)

pb.CppCode('        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n') #constructor with numpy array or list of lists

pb.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                       argList=['pyArray'],
                       description="add single array or list to Vector6DList; array or list must have appropriate dimension!",
                       argTypes=[vector6D],
                       returnType='None',
                       )
                                                                                                    
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Vector6DList into (copied) list of numpy arrays",
                       returnType='List['+vector6D+']',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyVector6DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Vector6DList, using len(data) where data is the Vector6DList",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyVector6DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                       description="set list item 'index' with data, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyVector6DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                       description="get copy of list item with 'index' as vector",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__copy__', 
                       cName='[](const PyVector6DList &item) {\n            return PyVector6DList(item); }', 
                       description="copy method to be used for copy.copy(...); in fact does already deep copy",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__deepcopy__', 
                       cName='[](const PyVector6DList &item, py::dict) {\n            return PyVector6DList(item); }, "memo"_a', 
                       description="deepcopy method to be used for copy.copy(...)",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyVector6DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Vector6DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

#++++++++++++++++
pb.DefPyFinishClass('PyVector6DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyMatrix3DList
classStr = 'PyMatrix3DList'
pyClassStr = 'Matrix3DList'
pb.DefPyStartClass(classStr, pyClassStr, r"""The Matrix3DList is used to represent lists of 3D Matrices. . This is used to transfer such lists from Python to C++.



Usage:
- Create empty `Matrix3DList` with `x = Matrix3DList()`
- Create `Matrix3DList` with list of numpy arrays:

  `x = Matrix3DList([ numpy.eye(3), numpy.array([[1.,2.,3.],[4.,5.,6.],[7.,8.,9.]]) ])`
- Create `Matrix3DList` with one matrix `x = Matrix3DList(13.*numpy.eye(3))`
- Append item: `x.Append(numpy.eye(3))`
- Convert into list of numpy arrays: `x.GetPythonObject()`
- similar to Vector3DList !

""", subSection=True)

pb.DefStartTable(pyClassStr)

pb.CppCode('        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n') #constructor with numpy array or list of lists

pb.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                       argList=['pyArray'],
                       description="add single 3D array or list of lists to Matrix3DList; array or lists must have appropriate dimension!",
                       argTypes=[matrix3D],
                       returnType='None',
                       )
                                                                                                    
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Matrix3DList into (copied) list of 3x3 numpy arrays",
                       returnType='List['+matrix3D+']',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyMatrix3DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Matrix3DList, using len(data) where data is the Matrix3DList",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyMatrix3DList &item, Index index, const py::object& matrix) {\n            item.PySetItem(index, matrix); }', 
                       description="set list item 'index' with matrix, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyMatrix3DList &item, Index index) {\n            return item.PyGetItem(index); }', 
                       description="get copy of list item with 'index' as matrix",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyMatrix3DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Matrix3DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

#++++++++++++++++
pb.DefPyFinishClass('PyMatrix3DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyMatrix6DList
classStr = 'PyMatrix6DList'
pyClassStr = 'Matrix6DList'
pb.DefPyStartClass(classStr, pyClassStr, r"""The Matrix6DList is used to represent lists of 6D Matrices. . This is used to transfer such lists from Python to C++.



Usage:
- Create empty `Matrix6DList` with `x = Matrix6DList()`
- Create `Matrix6DList` with list of numpy arrays:

  `x = Matrix6DList([ numpy.eye(6), 2*numpy.eye(6) ])`
- Append item: `x.Append(numpy.eye(6))`
- Convert into list of numpy arrays: `x.GetPythonObject()`
- similar to Matrix3DList !

""", subSection=True)

pb.DefStartTable(pyClassStr)

pb.CppCode('        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n') #constructor with numpy array or list of lists

pb.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                       argList=['pyArray'],
                       description="add single 6D array or list of lists to Matrix6DList; array or lists must have appropriate dimension!",
                       argTypes=[matrix6D],
                       returnType='None',
                       )
                                                                                                    
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Matrix6DList into (copied) list of 6x6 numpy arrays",
                       returnType='List['+matrix6D+']',
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyMatrix6DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Matrix6DList, using len(data) where data is the Matrix6DList",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyMatrix6DList &item, Index index, const py::object& matrix) {\n            item.PySetItem(index, matrix); }', 
                       description="set list item 'index' with matrix, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyMatrix6DList &item, Index index) {\n            return item.PyGetItem(index); }', 
                       description="get copy of list item with 'index' as matrix",
                       isLambdaFunction = True,
                       )

pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyMatrix6DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Matrix6DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

#++++++++++++++++
pb.DefPyFinishClass('PyMatrix6DList')
