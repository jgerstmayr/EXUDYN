#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  The declaration types of the hand-written Python interface (definitions/pybind*.py):
#           PybindInterface records the declaration calls in order, and
#           tools/generators/pybindEmitter.py replays them into pybind_manual_classes.h, the stub
#           fragments and the documentation (revision2026 step R4.3, part 2d). Also the stub type
#           names and C++ templates the declarations share.
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created in autoGeneratePyBindings.py), 2026-09-14 (moved to definitions/)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#the declaration calls; each writes C++ binding code, stub text and documentation (see
#PyLatexRST in src/pythonGenerator/autoGenerateHelper.py)
declarationCalls = ['AddDocu', 'AddDocuCodeBlock', 'AddDocuList', 'AddEnumValue', 'CreateNewRSTfile',
                    'DefLatexDataAccess', 'DefLatexFinishTable', 'DefLatexOperator', 'DefLatexStartClass',
                    'DefLatexStartTable', 'DefPyFinishClass', 'DefPyFunctionAccess', 'DefPyStartClass',
                    'DefStartEnumClass']

#the calls which only steer where the output goes
steeringCalls = {
    'CppCode': 'append C++ binding code written literally',
    'LatexCode': 'append LaTeX text written literally',
    'StubCode': 'append stub text written literally',
    'ResetRST': 'drop the RST text written so far',
    'BeginCppWrittenByHand': 'until EndCppWrittenByHand, the C++ binding exists by hand in C++: document only',
    'EndCppWrittenByHand': '',
    'BeginNoStub': 'until EndNoStub, no stub text is kept (pybind11 provides enough type information)',
    'EndNoStub': '',
    'EndStubSection': 'close the stub text of one class; the sections are written in reverse order',
    'CppFinishClass': 'finish the pybind class definition only, without the documentation side',
    'ExtensionRST': 'append a generated RST file (MainSystem extensions, written by mainSystemExtensionDocsEmitter.py)',
    }


class PybindInterface:
    """records the declaration calls of one interface file, in order"""
    def __init__(self):
        self.calls = []  #list of (name, args, kwargs)

    def __getattr__(self, name):
        if name not in declarationCalls and name not in steeringCalls:
            raise AttributeError('PybindInterface: unknown declaration call ' + name)
        def Record(*args, **kwargs):
            self.calls.append((name, args, kwargs))
        return Record


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#itemDict = 'dict'               #stub type for item classes (in fact convertable to dict)
itemDict = 'Any'                #stub type for item classes (in fact convertable to dict)
returnedArray = 'List[float]'   #stub type for for returned numpy array
listOrArray = 'List[float]'     #stub type for input as list or numpy array
vector2D = '[float,float]'#stub type for Vector3D
vector3D = '[float,float,float]'#stub type for Vector3D
vector6D = '[float,float,float,float,float,float]'#stub type for Vector6D

matrix3D = 'NDArray[Shape2D[3,3], float]'#stub type for Matrix3D
matrix6D = 'NDArray[Shape2D[6,6], float]'#stub type for Matrix6D

sparseMatrixType = 'Any'#currently Any, but will be adapted
matrixContainerType = 'Any'#currently Any, but will be adapted

#for objects with trivial or implemented copy constructor:
pickleDictTemplate = """        .def(py::pickle(
            [](const {ClassName}& self) {
                return py::make_tuple(self.GetDictionary());
            },
            [](const py::tuple& t) {
                CHECKandTHROW(t.size() == 1, "{ClassName}: loading data with pickle received invalid data structure!");
                {ClassName} self;
                self.SetDictionary(py::cast<py::dict>(t[0]));
                return self;
            }))
"""
#for objects which cannot be copied:
pickleDictTemplateNew = """        .def(py::pickle(
            [](const {ClassName}& self) {
                return py::make_tuple(self.GetDictionary());
            },
            [](const py::tuple& t) {
                CHECKandTHROW(t.size() == 1, "{ClassName}: loading data with pickle received invalid data structure!");
                {ClassName}* self = new {ClassName}();
                self->SetDictionary(py::cast<py::dict>(t[0]));
                return self;
            }))
"""


#enumExportValues = '.export_values()' #don't do that for enum class => would be visible at global scope
enumExportValues = '' #since 1.6.98
mainViewID = '0' #just used to avoid using value 0 for main view
