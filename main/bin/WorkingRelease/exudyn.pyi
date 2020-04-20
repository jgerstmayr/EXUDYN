#++++++++++++++++++++++++++++++++++++
#date: 2020-01-05
#author: Johannes Gerstmayr
#
#this is the stub file for exudyn which provides information on function names, function argument
#and classes, in order to enable type completion

class SystemContainer():
    def __init__(self) -> None: ...
	def AddSystem(self, mbs: MainSystem) -> MainSystem: ...


class MainSystem():
    def __init__(self) -> None: ...
	def AddObject(self, itemDict:dict) -> int: ...
	def AddNode(self, itemDict:dict) -> int: ...

	
	