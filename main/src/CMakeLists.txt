cmake_minimum_required(VERSION 3.12.2)

project(cppsrc)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release"
    CACHE STRING "Configuration types" FORCE)
add_executable(cppsrc
  ..\src\Graphics\GlfwClient.cpp
  ..\src\Graphics\VisualizationSystem.cpp
  ..\src\Graphics\VisualizationSystemContainer.cpp
  ..\src\Graphics\VisualizationSystemData.cpp
  ..\src\Linalg\LinearSolver.cpp
  ..\src\Linalg\Matrix.cpp
  ..\src\Linalg\Vector.cpp
  ..\src\Main\CSystem.cpp
  ..\src\Main\MainObjectFactory.cpp
  ..\src\Main\MainSystem.cpp
  ..\src\Main\Stdoutput.cpp
  ..\src\Objects\CMarkerBodyCable2DCoordinates.cpp
  ..\src\Objects\CMarkerBodyCable2DShape.cpp
  ..\src\Objects\CMarkerBodyMass.cpp
  ..\src\Objects\CMarkerBodyPosition.cpp
  ..\src\Objects\CMarkerBodyRigid.cpp
  ..\src\Objects\CMarkerNodeCoordinate.cpp
  ..\src\Objects\CMarkerNodePosition.cpp
  ..\src\Objects\CMarkerNodeRigid.cpp
  ..\src\Objects\CNodeGenericData.cpp
  ..\src\Objects\CNodeGenericODE2.cpp
  ..\src\Objects\CNodePoint.cpp
  ..\src\Objects\CNodePoint2D.cpp
  ..\src\Objects\CNodePoint2DSlope1.cpp
  ..\src\Objects\CNodePointGround.cpp
  ..\src\Objects\CNodeRigidBody.cpp
  ..\src\Objects\CNodeRigidBody2D.cpp
  ..\src\Objects\CObjectALEANCFCable2D.cpp
  ..\src\Objects\CObjectANCFCable2D.cpp
  ..\src\Objects\CObjectConnectorCartesianSpringDamper.cpp
  ..\src\Objects\CObjectConnectorCoordinate.cpp
  ..\src\Objects\CObjectConnectorCoordinateSpringDamper.cpp
  ..\src\Objects\CObjectConnectorDistance.cpp
  ..\src\Objects\CObjectConnectorSpringDamper.cpp
  ..\src\Objects\CObjectContactCircleCable2D.cpp
  ..\src\Objects\CObjectContactCoordinate.cpp
  ..\src\Objects\CObjectContactFrictionCircleCable2D.cpp
  ..\src\Objects\CObjectGround.cpp
  ..\src\Objects\CObjectJointALEMoving2D.cpp
  ..\src\Objects\CObjectJointPrismatic2D.cpp
  ..\src\Objects\CObjectJointRevolute2D.cpp
  ..\src\Objects\CObjectJointSliding2D.cpp
  ..\src\Objects\CObjectMassPoint.cpp
  ..\src\Objects\CObjectMassPoint2D.cpp
  ..\src\Objects\CObjectRigidBody.cpp
  ..\src\Objects\CObjectRigidBody2D.cpp
  ..\src\Objects\VisuNodePoint.cpp
  ..\src\Pymodules\PybindModule.cpp
  ..\src\Solver\CSolver.cpp
  ..\src\Solver\CSolverBase.cpp
  ..\src\Solver\CSolverImplicitSecondOrder.cpp
  ..\src\Solver\CSolverStatic.cpp
  ..\src\Solver\MainSolver.cpp
  ..\src\Solver\MainSolverBase.cpp
  ..\src\Solver\StaticSolver.cpp
  ..\src\solver\TimeIntegrationSolver.cpp
  ..\src\System\CLoad.cpp
  ..\src\System\CNode.cpp
  ..\src\System\CObjectBody.cpp
  ..\src\System\MainNode.cpp
  ..\src\System\MainObject.cpp
  ..\src\tests\UnitTestBase.cpp
  ..\src\Utilities\BasicFunctions.cpp
)

# Additional include directories
set_property(TARGET cppsrc
  APPEND PROPERTY INCLUDE_DIRECTORIES
  $<$<CONFIG:Debug>:
    ${CMAKE_SOURCE_DIR}/src;
    ${CMAKE_SOURCE_DIR}/include/lest;
    ${CMAKE_SOURCE_DIR}/include/glfw;
    ${CMAKE_SOURCE_DIR}/include/glfw/deps;
    ${CMAKE_SOURCE_DIR}/include/pybind11;
    "C:\Program Files (x86)\python38\include">
  $<$<CONFIG:Release>:
    ${CMAKE_SOURCE_DIR}/src;
    ${CMAKE_SOURCE_DIR}/include/lest;
    ${CMAKE_SOURCE_DIR}/include/glfw;
    ${CMAKE_SOURCE_DIR}/include/glfw/deps;
    ${CMAKE_SOURCE_DIR}/include/pybind11;
    "C:\Program Files (x86)\python38\include">
)
