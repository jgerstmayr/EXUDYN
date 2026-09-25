#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the introduction of the Python-C++ command interface chapter (documentation only).
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

pb.AddDocu('',section='Python-C++ command interface', sectionLevel=0, sectionLabel='sec:PCpp:command:interface')
pb.ResetMarkdown() #the chapter heading lives in the index page

pb.AddDocu(r"""This chapter lists the basic interface functions which can be used to set up a Exudyn model in Python.""")

pb.AddDocu(r"""This chapter lists the basic interface functions which can be used to set up 
a Exudyn model in Python. Note that some functions or classes will be used in examples, which are explained in detail later on.
In the following, some basic steps and concepts for usage are shown, references to all functions are placed hereafter:
""", section='General information on Python-C++ interface', sectionLevel=1, sectionLabel='sec:generalPythonInterface')

pb.AddDocu(r"""To import the module, just include the Exudyn module in Python:""")
pb.AddDocuCodeBlock(code="""
import exudyn as exu
""")

#pb.AddDocuList(itemList=['\\texttt{import exudyn as exu}'], itemText='[]')
pb.AddDocu(r"""For compatibility with examples and other users, we recommend to use the `exu` abbreviation throughout. In addition, you may work with a convenient interface for your items, therefore also always include:""")
pb.AddDocuCodeBlock(code="""
from exudyn.itemInterface import *
""")
#pb.AddDocuList(itemList=['\\texttt{from exudyn.itemInterface import *}'], itemText='[]')
pb.AddDocu(r"""Note that including `exudyn.utilities` will cover `itemInterface`. Also note that `from ... import *` is not recommended in general and it will not work in certain cases, e.g., if you like to compute on a cluster. However, it greatly simplifies life for smaller models and you may replace imports in your files afterwards by removing the star import.""")

pb.AddDocu(r"""The general hub to multibody dynamics models is provided by the classes `SystemContainer` and `MainSystem`, except for some very basic system functionality (which is inside the Exudyn module). 

You can create a new `SystemContainer`, which is a class that is initialized by assigning a system container to a variable, usually denoted as `SC`:""")
pb.AddDocuCodeBlock(code="""
SC = exu.SystemContainer()
""")
#pb.AddDocuList(itemList=['\\texttt{SC = exu.SystemContainer()}'], itemText='[]')
pb.AddDocu(r"""Note that creating a second `exu.SystemContainer()` will be independent of `SC` and therefore makes no sense if you do not intend to work with two different containers.
""")

pb.AddDocu(r"""To add a MainSystem to system container `SC` and store as variable `mbs`, write:""")

pb.AddDocuCodeBlock(code="""
mbs = SC.AddSystem()
""")
#pb.AddDocuList(itemList=['\\texttt{mbs = SC.AddSystem()}'], itemText='[]')

pb.AddDocu(r"""Furthermore, there are a couple of commands available directly in the `exudyn` module, given in the following subsections. Regarding the **(basic) module access**, functions are related to the `exudyn = exu` module, see these examples:""")

pb.AddDocuCodeBlock(code="""
#  import exudyn module:
import exudyn as exu
#  print detailed exudyn version, Python version (at which it is compiled):
exu.config.Version(addDetails = True)
#  set precision of C++ output to console
exu.config.precision = numberOfDigits
#  turn on/off output to console
exu.config.printToConsole = False
#  invalid index, may depend on compilation settings:
nInvalid = exu.InvalidIndex() #the invalid index, depends on architecture and version
#  run basic demos (without/with graphics):
exu.demos.Demo1()
exu.demos.Demo2()
""")

pb.AddDocu(r"""Understanding the usage of functions for python object `SystemContainer` of the module `exudyn`, the following examples might help:""")
pb.AddDocuCodeBlock(code="""
#import exudyn module:
import exudyn as exu
#  import utilities (includes itemInterface, basicUtilities, 
#                  advancedUtilities, rigidBodyUtilities, graphics):
from exudyn.utilities import *
#  create system container and store in SC:
SC = exu.SystemContainer()
#  add a MainSystem (multibody system) to system container SC and store as mbs:
mbs = SC.AddSystem()
#  add a second MainSystem to system container SC and store as mbs2:
mbs2 = SC.AddSystem()
#  print number of systems available:
nSys = SC.NumberOfSystems()
exu.Print(nSys) #or just print(nSys)
#  delete reference to mbs and mbs2 (usually not necessary):
del mbs, mbs2
#  reset system container (mbs becomes invalid):
SC.Reset()
""")
pb.AddDocu(r"""If you run a parameter variation (check `Examples/parameterVariationExample.py`), you may reset or delete the created `MainSystem` `mbs` and the `SystemContainer` `SC` before creating new instances in order to avoid memory growth.""")

#+++++++++++++++++++++++++++++++++++
#ITEMINDEX
pb.AddDocu(r"""Many functions will work with node numbers (`NodeIndex`), object numbers (`ObjectIndex`),marker numbers (`MarkerIndex`) and others. These numbers are special Python objects, which have been introduced in order to avoid mixing up, e.g., node and object numbers. 

For example, the command `mbs.AddNode(...)` returns a `NodeIndex`. For these indices, the following rules apply:""",
                section='Item index', sectionLevel=2,sectionLabel='sec:itemIndex')
pb.AddDocuList(itemList=[
r"""`mbs.Add[Node|Object|...](...)` returns a specific `NodeIndex`, `ObjectIndex`, ...""",
r"""You can create any item index, e.g., using `ni = NodeIndex(42)` or `oi = ObjectIndex(42)`""",
'The benefit of these indices comes as they may not be mixed up, e.g., using an object index instead of a node index.',
r"""You can convert any item index, e.g., NodeIndex `ni` into an integer number using `int(ni)` of `ni.GetIndex()`""",
r"""Still, you can use integers as initialization for item numbers, e.g.:

`mbs.AddObject(MassPoint(nodeNumber=13, ...))`

However, it must be a pure integer type.""",
r"""You can make integer calculations with such indices, e.g., `oi = 2*ObjectIndex(42)+1` restricing to addition, subtraction and multiplication. Currently, the result of such calculations is a `int` type andoperating on mixed indices is not checked (but may raise exceptions in future).""",
r"""You can also print item indices, e.g., `print(ni)` as it converts to string by default.""",
r"""If you are unsure about the type of an index, use `ni.GetTypeString()` to show the index type."""
    ], itemText='[]')

pb.AddDocu(r"""As a key concept to working with Exudyn , most data which is retrieved by C++ interface functions is copied.
Experienced Python users may know that it is a key concept to Python to often use references instead of copying, which is
sometimes error-prone but offers a computationally efficient behavior.
There are only a few very important cases where data is referenced in Exudyn , the main ones are 
`SystemContainer`, 
`MainSystem`, 
`VisualizationSettings`, and
`SimulationSettings` which are always references to internal C++ classes.
The following code snippets and comments should explain this behavior:
""", section='Copying and referencing C++ objects', sectionLevel=2, sectionLabel='sec:generalPythonInterface:copyref')

pb.AddDocuCodeBlock(code="""
import copy                        #for real copying
import exudyn as exu
from exudyn.utilities import *
#create system container, referenced from SC:
SC = exu.SystemContainer()
SC2 = SC                           #this will only put a reference to SC
                                   #SC2 and SC represent the SAME C++ object
#add a MainSystem (multibody system):
mbs = SC.AddSystem()               #get reference mbs to C++ system
mbs2=mbs                           #again, mbs2 and mbs refer to the same C++ object
og = mbs.AddObject(ObjectGround()) #copy data of ObjectGround() into C++
o0 = mbs.GetObject(0)              #get copy of internal data as dictionary

mbsCopy=copy.copy(mbs)             #mbsCopy is now a real copy of mbs; uses pickle; experimental!
SC.Append(mbsCopy)                 #this is needed to work with mbsCopy

del o0                             #delete the local dictionary; C++ data not affected
del mbs, mbs2                      #references to mbs deleted (C++ data still available)
del mbsCopy                        #now also copy of mbs destroyed
del SC                             #references to SystemContainer deleted
#at this point, mbs and SC are not available any more (data will be cleaned up by Python)
""")

#+++++++++++++++++++++++++++++++++++
#EXCEPTIONS
pb.AddDocu(r"""There are several levels of type and argument checks, leading to different types of errors and exceptions. The according error messages are non-unique, because they may be raised in Python modules or in C++, and they may be raised on different levels of the code. Error messages depend on Python version and on your iPython console. Very often the exception may be called `ValueError`, but it mustnot mean that it is a wrong error, but it could also be, e.g., a wrong order of function calls.""",
                section='Exceptions and Error Messages', sectionLevel=2,sectionLabel='sec:cinterface:exceptions')

pb.AddDocu(r"""As an example, a type conversion error is raised when providing wrong argument types, e.g., try `exu.config.Version('abc')`:""")

pb.AddDocuCodeBlock(code=r"""
Traceback (most recent call last):

File "C:\Users\username\AppData\Local\Temp\ipykernel_24988\2212168679.py", line 1, in <module>
    exu.config.Version('abc')

TypeError: Version(): incompatible function arguments. The following argument types are supported:
    1. (addDetails: bool = False) -> str

Invoked with: 'abc'
""",
pythonStyle=False)

pb.AddDocu('Note that your particular error message may be different.')
pb.AddDocu('Another error results from internal type and range checking, saying User ERROR, '+
                'as it is due to a wrong input of the user. For this, we try')

pb.AddDocuCodeBlock(code="mbs.AddObject('abc')")

pb.AddDocu('Which results in an error message similar to:')
pb.AddDocuCodeBlock(code=r"""
=========================================
User ERROR [file 'C:\Users\username\AppData\Local\Temp\ipykernel_24988\2838049308.py', line 1]: 
Error in AddObject(...):
Check your python code (negative indices, invalid or undefined parameters, ...)

=========================================

Traceback (most recent call last):

  File "C:\Users\username\AppData\Local\Temp\ipykernel_24988\2838049308.py", line 1, in <module>
    mbs.AddObject('abc')

RuntimeError: Exudyn: parsing of Python file terminated due to Python (user) error

""", pythonStyle=False)

pb.AddDocu(r"""Finally, there may be system errors. They may be caused due to previous wrong input, but if there is no reason seen, it may be appropriate to report this error on [github.com/jgerstmayr/EXUDYN/](https://github.com/jgerstmayr/EXUDYN) .""")

pb.AddDocu(r"""Be careful in reading and interpreting such error messages. You should **read them from top to bottom**, as the cause may be in the beginning. Often files and line numbers of errors are provided (e.g., if you have a longer script). In the ultimate case, try to comment parts of your code or deactivate items to see where the error comes from. See also section on Trouble shooting and FAQ.""")
