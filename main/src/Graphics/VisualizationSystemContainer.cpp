/** ***********************************************************************************************
* @brief		Implementation of class VisualizationSystemContainer
* @details		Details:
 				- a visualization system container; links to OpenGL renderer and contains several visualization systems
*
* @author		Gerstmayr Johannes
* @date			2019-06-11 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */

#include "Utilities/ExceptionsTemplates.h" //for exceptions in solver steps
#include "Graphics/VisualizationSystemContainer.h"
#include "Graphics/VisualizationPrimitives.h"

#include "Graphics/GlfwClient.h" //in order to link to graphics engine

//#ifdef USE_GLFW_GRAPHICS
//#endif

bool VisualizationSystemContainer::AttachToRenderEngine()
{
#ifdef USE_GLFW_GRAPHICS

	glfwRenderer.DetachVisualizationSystem(nullptr); //nullptr means, that every new systemcontainer links to the render engine and the old container is lost; necessary if an old systemcontainer is still linked

	if (!glfwRenderer.LinkVisualizationSystem(&graphicsDataList, &settings, this, &renderState)) 
		//(&graphicsData, &settings, this, &renderState))
	{
		SysError("VisualizationSystemContainer::AttachToRenderEngine: Visualization cannot be linked to several SystemContainers at the same time; detach other SystemContainer first!");
		return false;
	}
	return true;
#else
	PyWarning("AttachToRenderEngine(): has no effect as GLFW_GRAPHICS is deactivated in your exudyn module (needs recompile or another version)");
	return false;
#endif
}

bool VisualizationSystemContainer::DetachFromRenderEngine(VisualizationSystemContainer* detachingVisualizationSystemContainer)
{
#ifdef USE_GLFW_GRAPHICS
	return glfwRenderer.DetachVisualizationSystem(detachingVisualizationSystemContainer);
#else
	PyWarning("DetachFromRenderEngine(): has no effect as GLFW_GRAPHICS is deactivated in your exudyn module (needs recompile or another version)");
	return false;
#endif
	
}


//! OpenGL renderer sends message that graphics shall be updated ==> update graphics data
void VisualizationSystemContainer::UpdateGraphicsData()
{
	//this is now done in GLFWclient for zoom all, InitializeView done
	//if (zoomAllRequest)
	//{
	//	zoomAllRequest = false; //this is not fully thread safe, but should not happen very often ==> user needs to zoom manually then ...

	//	InitializeView();
	//}
	if (updateGraphicsDataNow) { updateGraphicsDataNowInternal = true; updateGraphicsDataNow = false; } //enables immediate new set of updateGraphicsDataNow
	if (saveImage) { saveImageOpenGL = true; } //as graphics are updated now, the saveImageOpenGL flag can be set

	Index cnt = 0;
	for (auto item : visualizationSystems)
	{
		//pout << "UpdateGraphicsData1\n";
		item->UpdateGraphicsData(*this);
		//pout << "UpdateGraphicsData2\n";
		if (cnt == 0 && settings.general.drawWorldBasis)
		{
			EXUvis::DrawOrthonormalBasis(Vector3D({ 0,0,0 }), EXUmath::unitMatrix3D, settings.general.worldBasisSize,
				0.005*settings.general.worldBasisSize, item->GetGraphicsData(), Index2ItemID(-1, ItemType::_None, 0)); //world basis has no special index
		}
		cnt++;
	}

	updateGraphicsDataNowInternal = false; //only valid for one run; may not be earlier, as item->UpdateGraphicsData(...) needs this flag!
}

// put this to SystemContainer ...
//! perform render update and save the current openGL window to file using the visualization settings
void VisualizationSystemContainer::RedrawAndSaveImage()
{
	//now a new saveImage message can be sent
	saveImage = true;			//flag initiates saveImageOpenGL at next UpdateGraphicsData() called from Renderer
	saveImageOpenGL = false;	//after graphics update, the scene is saved and flags (saveImage, saveImageOpenGL) are set to false
	UpdateGraphicsDataNow();	//if a current redraw is performed, it will also initiate a second redraw operation ...

	Index timeOut = 500; //max iterations to wait, before frame is redrawn and saved
	Index timerMilliseconds = settings.exportImages.saveImageTimeOut / timeOut;
	if (timerMilliseconds == 0) { timerMilliseconds = 1; } //min wait time per iteration

	//now wait until the saveImage flag has been deleted by the current redraw operation
	Index i = 0;
	while (i++ < timeOut && (saveImageOpenGL || saveImage)) //wait timeOut*timerMilliseconds seconds for last operation to finish
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(timerMilliseconds));
	}
	if (saveImageOpenGL || saveImage)
	{
		PyWarning("PostProcessData::RedrawAndSaveImage: save frame to image file did not finish; increase timeout parameter");
	}
}


//! Renderer reports to simulation that simulation shall be interrupted
void VisualizationSystemContainer::StopSimulation()
{
	//as we do not know, which simulation is executed, all system computations are interrupted
	stopSimulationFlag = true; //tell also VisualizationSystemContainer
	for (auto item : visualizationSystems)
	{
		item->postProcessData->stopSimulation = true;
	}

}

//! Renderer reports to simulation that simulation shall be interrupted
void VisualizationSystemContainer::ForceQuitSimulation(bool flag)
{
	//as we do not know, which simulation is executed, all system computations are interrupted
	for (auto item : visualizationSystems)
	{
		item->postProcessData->forceQuitSimulation = flag;
	}

}


//! Renderer reports to simulation that simulation can be continued
void VisualizationSystemContainer::ContinueSimulation()
{
	//as we do not know, which simulation is executed, all system computations are interrupted
	for (auto item : visualizationSystems)
	{
		item->postProcessData->simulationPaused = false;
	}

}

//not needed any more, as visualizationSystem has backlink possibility to VisualizationSystemContainer; 
//==> VisualizationSystemContainer has RendererIsRunning()
////! renderer signals that visualizationIsRunning flag should be set to "flag"
//void VisualizationSystemContainer::SetVisualizationIsRunning(bool flag)
//{
//	//as we do not know, which simulation is executed, all system computations are interrupted
//	for (auto item : visualizationSystems)
//	{
//		item->postProcessData->visualizationIsRunning = flag;
//	}
//}

//! check GLFW if renderer is running
bool VisualizationSystemContainer::RendererIsRunning() const
{
#ifdef USE_GLFW_GRAPHICS
	return glfwRenderer.WindowIsInitialized();
#else
	return false;
#endif
}


//! this function does any idle operations (execute some python commands) and returns false if stop flag in the render engine, otherwise true;
bool VisualizationSystemContainer::DoIdleOperations()
{
#ifdef USE_GLFW_GRAPHICS
	if (!stopSimulationFlag && RendererIsRunning())
	{
		//std::this_thread::sleep_for(std::chrono::milliseconds(50));
		PyProcessExecuteQueue(); //use time to execute incoming python tasks
		for (auto item : visualizationSystems)
		{
			item->postProcessData->ProcessUserFunctionDrawing(); //check if user functions to be drawn and do user function evaluations
		}
		RendererDoSingleThreadedIdleTasks();
		return true;
	}
	else
	{
		stopSimulationFlag = false; //initialize the flag, if used several times; this is thread safe
	}
#endif
	return false;
}


//! this function waits for the stop flag in the render engine;
bool VisualizationSystemContainer::WaitForRenderEngineStopFlag()
{
#ifdef USE_GLFW_GRAPHICS
	while (DoIdleOperations())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
#endif
	return true;
}

//! if the system has changed or loaded, compute maximum box of all items and reset scene to the maximum box
void VisualizationSystemContainer::InitializeView()
{
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	renderState.zoom = settings.openGL.initialZoom;
	renderState.maxSceneSize = settings.openGL.initialMaxSceneSize;
	renderState.centerPoint = settings.openGL.initialCenterPoint; //this is the initial centerPoint; hereafter it can be changed!
	//renderState.rotationCenterPoint.SetAll(0);

	renderState.currentWindowSize = settings.window.renderWindowSize;
	if (renderState.currentWindowSize[0] < 1) { renderState.currentWindowSize[0] = 1; } //avoid division by zero
	if (renderState.currentWindowSize[1] < 1) { renderState.currentWindowSize[1] = 1; } //avoid division by zero

	//set modelRotation to identity matrix (4x4); Use rotation part only from Float9 initialModelRotation
	renderState.modelRotation.SetAll(0.f);
	renderState.modelRotation[0] = settings.openGL.initialModelRotation[0][0];
	renderState.modelRotation[1] = settings.openGL.initialModelRotation[0][1];
	renderState.modelRotation[2] = settings.openGL.initialModelRotation[0][2];
	renderState.modelRotation[4] = settings.openGL.initialModelRotation[1][0];
	renderState.modelRotation[5] = settings.openGL.initialModelRotation[1][1];
	renderState.modelRotation[6] = settings.openGL.initialModelRotation[1][2];
	renderState.modelRotation[8] = settings.openGL.initialModelRotation[2][0];
	renderState.modelRotation[9] = settings.openGL.initialModelRotation[2][1];
	renderState.modelRotation[10] = settings.openGL.initialModelRotation[2][2];
	renderState.modelRotation[15] = 1.;

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	////for now, use nodal reference coordinates to estimate the maximum zoom level
	//Vector3D pmax0({ -1e30,-1e30,-1e30 });
	//Vector3D pmin0({ 1e30,1e30,1e30 });
	//Vector3D pmax = pmax0;
	//Vector3D pmin = pmin0;
	//for (auto visSystem : visualizationSystems)
	//{
	//	//! @todo extend VisualizationSystemContainer::InitializeView for objects, markers and loads; maybe better to first draw all and zoom to full region?
	//	for (auto item : visSystem->systemData->GetCNodes())
	//	{
	//		if ((Index)item->GetNodeGroup() & (Index)CNodeGroup::ODE2variables)
	//		{
	//			//LinkedDataVector pref = item->GetReferenceCoordinateVector();
	//			Vector3D pref = ((CNodeODE2*)item)->GetPosition(ConfigurationType::Reference);

	//			pmax[0] = EXUstd::Maximum(pref[0], pmax[0]);
	//			pmax[1] = EXUstd::Maximum(pref[1], pmax[1]);
	//			pmax[2] = EXUstd::Maximum(pref[2], pmax[2]);
	//			pmin[0] = EXUstd::Minimum(pref[0], pmin[0]);
	//			pmin[1] = EXUstd::Minimum(pref[1], pmin[1]);
	//			pmin[2] = EXUstd::Minimum(pref[2], pmin[2]);
	//		}

	//	}
	//}
	////check if some bad coordinates result
	//if (pmax == pmax0 || pmin == pmin0) 
	//{
	//	pmin = Vector3D({ 0.,0.,0. });
	//	pmax = Vector3D({ 1.,1.,1. });
	//}
	//else if (pmax == pmin)
	//{
	//	Real d = 0.5;
	//	pmax += Vector3D({ d,d,d });
	//	pmin -= Vector3D({ d,d,d});
	//}
	//Vector3D center = 0.5*(pmin + pmax);

	//if (renderState.centerPoint[0] == 0 && renderState.centerPoint[1] == 0 && renderState.centerPoint[2] == 0) {
	//	renderState.centerPoint.CopyFrom(center);
	//}

	//renderState.maxSceneSize = (float)((pmax - pmin).GetL2Norm());
	//if (renderState.maxSceneSize < settings.general.minSceneSize) 
	//{ 
	//	renderState.maxSceneSize = settings.general.minSceneSize; 
	//}

	//if (settings.general.autoFitScene)
	//{
	//	renderState.zoom = 0.4f*renderState.maxSceneSize;
	//}
}

//! any multi-line text message from computation to be shown in renderer (e.g. time, solver, ...)
std::string VisualizationSystemContainer::GetComputationMessage(bool solverInformation, bool solutionInformation, bool solverTime)
{
	//workaround: take message of first system to be shown
	if (visualizationSystems.NumberOfItems())
	{
		return visualizationSystems[0]->GetComputationMessage(solverInformation, solutionInformation, solverTime);
	}
	return std::string();
}

//! REMOVE: get backlink of ith main system (0 if not existing), temporary for selection
MainSystem* VisualizationSystemContainer::GetMainSystemBacklink(Index iSystem)
{
	if (iSystem < visualizationSystems.NumberOfItems())
	{
		return visualizationSystems[iSystem]->GetMainSystemBacklink();
	}
	else
	{
		return nullptr;
	}
}

Index VisualizationSystemContainer::NumberOFMainSystemsBacklink() const
{
	return visualizationSystems.NumberOfItems();
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! object graphics data for ground objects, rigid bodies and mass points
bool PyWriteBodyGraphicsDataList(const py::dict& d, const char* item, BodyGraphicsData& data)
{
	data.FlushData(); //this is the body data, which cannot be filled incrementally
	if (d.contains(item))
	{
		GenericExceptionHandling([&]
		{
			py::object other = d[item]; //this is necessary to make isinstance work
			return PyWriteBodyGraphicsDataList(other, data);
		}, "Exception raised when writing BodyGraphicsData: check format");

	}//if "GraphicsData" does not exist, no error is displayed
	return true;
}
//! python function to read BodyGraphicsData from py::object, which must be a list of graphicsData dictionaries
bool PyWriteBodyGraphicsDataList(const py::object object, BodyGraphicsData& data)
{
	if (py::isinstance<py::list>(object)) //must be a list of graphicsData dictionaries
	{
		py::list list = (py::list)(object);

		for (auto graphicsItem : list)
		{
			//now read out dictionaries, containing graphics structures:
			//{'type':'Line', 'color':[1,0,0,1], 'data':[x1,y1,z1, x2,y2,z2, ...]}
			//{'type':'Circle', 'color':[1,0,0,1], 'radius': r, 'position': [x,y,z], 'normal': [x,y,z]}
			//{'type':'Text', 'color':[1,0,0,1], 'text':"sample text", 'position':[x,y,z]}

			if (py::isinstance<py::dict>(graphicsItem)) //must be a dictionary of graphicsData
			{
				const py::dict& gDict = (py::dict&)graphicsItem;
				if (gDict.contains("type"))
				{
					py::object pyType = gDict["type"]; //this is necessary to make isinstance work
					if (py::isinstance<py::str>(pyType))
					{
						std::string pyTypeStr = py::cast<std::string>(pyType); //! read out dictionary and cast to C++ type

						//add lines, circles, text, and triangles ....
						if (pyTypeStr == "Line")
						{

							GLLine line; line.itemID = -1;
							line.color1 = line.color2 = EXUvis::defaultColorFloat4;
							if (gDict.contains("color"))
							{
								py::object gColor = gDict["color"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gColor)) //must be a list of graphicsData dictionaries
								{
									py::list colorList = (py::list)(gColor);
									std::vector<float> stdColorList = py::cast<std::vector<float>>(colorList); //! # read out dictionary and cast to C++ type

									if (stdColorList.size() == 4)
									{
										line.color1 = line.color2 = Float4(stdColorList);
									}
									else { PyError("GraphicsData Line: color must be a float vector with 4 components"); return false; }
								}
								else { PyError("GraphicsData Line: color must be a float vector with 4 components"); return false; }
							}
							if (gDict.contains("data"))
							{
								py::object gData = gDict["data"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gData)) //must be a list of graphicsData dictionaries
								{
									py::list dataList = (py::list)(gData);
									std::vector<float> gd = py::cast<std::vector<float>>(dataList); //! # read out dictionary and cast to C++ type

									Index n = (Index)gd.size() / 3;
									if (n * 3 != (Index)gd.size() || n < 2)
									{
										PyError("GraphicsData Line: data must be a float vector with exactly 3*n components and n > 1"); return false;
									}

									for (Index k = 1; k < n; k++)
									{
										line.point1 = Float3({ gd[3 * (k - 1)],gd[3 * (k - 1) + 1] ,gd[3 * (k - 1) + 2] });
										line.point2 = Float3({ gd[3 * k],gd[3 * k + 1] ,gd[3 * k + 2] });
										data.glLines.Append(line);
									}
								}
								else { PyError("GraphicsData Line: data must be a float vector with 3*n components"); return false; }

							}
							else { PyError("GraphicsData Line: must contain 'data' with (x1,y1,z1,...) line coordinates "); return false; }
						} //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
						else if (pyTypeStr == "Lines")
						{
							GLLine line; line.itemID = -1;
							line.color1 = line.color2 = EXUvis::defaultColorFloat4;
							std::vector<float> stdColorsList;
							std::vector<float> stdPointsList;
							Index nLines = 0;

							if (gDict.contains("points"))
							{
								py::object gData = gDict["points"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gData)) //must be a list of graphicsData dictionaries
								{
									py::list dataList = (py::list)(gData);
									stdPointsList = py::cast<std::vector<float>>(dataList); //! # read out dictionary and cast to C++ type

									nLines = (Index)stdPointsList.size() / 6;
									if (nLines * 6 != (Index)stdPointsList.size() || nLines < 1)
									{
										PyError("GraphicsData Lines: for n lines, points must be a float vector with exactly 6*n components and n > 1"); return false;
									}
								}
								else { PyError("GraphicsData Lines: for n lines, points must be a float vector with 6*n floats, 3 floats per point"); return false; }
							}
							else { PyError("GraphicsData Lines: must contain 'points' with (x1,y1,z1,...) line coordinates "); return false; }

							if (gDict.contains("colors"))
							{
								py::object gColor = gDict["colors"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gColor)) //must be a list of graphicsData dictionaries
								{
									py::list colorList = (py::list)(gColor);
									stdColorsList = py::cast<std::vector<float>>(colorList); //! # read out dictionary and cast to C++ type

									if (stdColorsList.size() != nLines * 8)
									{ PyError("GraphicsData Line: for n lines, colors must contain 8*n floats, 4 floats per line point"); return false; }
								}
								else { PyError("GraphicsData Line: for n lines, colors must contain 8*n floats, 4 floats per line point"); return false; }
							}
							else { PyError("GraphicsData Lines: must contain 'colors', containing 4 floats per line point"); return false; }

							for (Index k = 0; k < nLines; k++)
							{
								line.point1 = Float3({ stdPointsList[6 * k + 0],stdPointsList[6 * k + 1],stdPointsList[6 * k + 2] });
								line.point2 = Float3({ stdPointsList[6 * k + 3],stdPointsList[6 * k + 4],stdPointsList[6 * k + 5] });
								line.color1 = Float4({ stdColorsList[8 * k + 0],stdColorsList[8 * k + 1],stdColorsList[8 * k + 2],stdColorsList[8 * k + 3] });
								line.color2 = Float4({ stdColorsList[8 * k + 4],stdColorsList[8 * k + 5],stdColorsList[8 * k + 6],stdColorsList[8 * k + 7] });
								data.glLines.Append(line);
							}

						} //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
						else if (pyTypeStr == "Circle")
						{
							GLCircleXY circle; circle.itemID = -1;
							circle.color = EXUvis::defaultColorFloat4;
							circle.numberOfSegments = 0; //use default

							if (gDict.contains("color"))
							{
								py::object gColor = gDict["color"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gColor)) //must be a list of graphicsData dictionaries
								{
									py::list colorList = (py::list)(gColor);
									std::vector<float> stdColorList = py::cast<std::vector<float>>(colorList); //! # read out dictionary and cast to C++ type

									if (stdColorList.size() == 4)
									{
										circle.color = Float4(stdColorList);
									}
									else { PyError("GraphicsData Circle: color must be a float vector with 4 components"); return false; }
								}
								else { PyError("GraphicsData Circle: color must be a float vector with 4 components"); return false; }
							}

							if (gDict.contains("radius"))
							{
								py::object gData = gDict["radius"]; //this is necessary to make isinstance work
								if (py::isinstance<py::float_>(gData) || py::isinstance<py::int_>(gData)) //must be a scalar value
								{
									circle.radius = (py::float_)(gData);
								}
								else { PyError("GraphicsData Circle: radius must be a scalar value"); return false; }

							}
							else { PyError("GraphicsData Circle: must contain 'radius'"); return false; }

							if (gDict.contains("position"))
							{
								py::object gData = gDict["position"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gData))  //must be a list of 3 coordinates
								{
									py::list dataList = (py::list)(gData);
									std::vector<float> gd = py::cast<std::vector<float>>(dataList); //! # read out dictionary and cast to C++ type

									if (gd.size() != 3)
									{
										PyError("GraphicsData Circle: position must be a float vector with 3 components"); return false;
									}

									circle.point = Float3(gd);
								}
								else { PyError("GraphicsData Circle: position must be a float vector with 3 components"); return false; }

							}
							else { PyError("GraphicsData Circle: must contain 'position'"); return false; }

							data.glCirclesXY.Append(circle);

						} //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
						else if (pyTypeStr == "Text")
						{
							GLText text;
							text.color = EXUvis::defaultColorFloat4;
							text.offsetX = 0.f;
							text.offsetY = 0.f;
							text.size = 0; //indicates to use default size


							if (gDict.contains("color"))
							{
								py::object gColor = gDict["color"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gColor)) //must be a list of graphicsData dictionaries
								{
									py::list colorList = (py::list)(gColor);
									std::vector<float> stdColorList = py::cast<std::vector<float>>(colorList); //! # read out dictionary and cast to C++ type

									if (stdColorList.size() == 4)
									{
										text.color = Float4(stdColorList);
									}
									else { PyError("GraphicsData Text: color must be a float vector with 4 components"); return false; }
								}
								else { PyError("GraphicsData Text: color must be a float vector with 4 components"); return false; }
							}

							if (gDict.contains("position"))
							{
								py::object gData = gDict["position"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gData))  //must be a list of 3 coordinates
								{
									py::list dataList = (py::list)(gData);
									std::vector<float> gd = py::cast<std::vector<float>>(dataList); //! # read out dictionary and cast to C++ type

									if (gd.size() != 3)
									{
										PyError("GraphicsData Text: position must be a float vector with 3 components"); return false;
									}

									text.point = Float3(gd);

								}
								else { PyError("GraphicsData Text: position must be a float vector with 3 components"); return false; }
							}
							else { PyError("GraphicsData Text: must contain 'position'"); return false; }

							if (gDict.contains("text"))
							{
								py::object gData = gDict["text"]; //this is necessary to make isinstance work
								if (py::isinstance<py::str>(gData)) //must be a scalar value
								{
									std::string gText = (py::str)(gData);
									int len = (int)gText.size();
									text.text = new char[len + 1]; //will be deleted in destructor of GraphicsData
									//strcpy_s(text.text, len + 1, gText.c_str()); //not working with gcc
									strcpy(text.text, gText.c_str());

									data.glTexts.Append(text);
								}
								else { PyError("GraphicsData Text: 'text' must be of type string"); return false; }

							}
							else { PyError("GraphicsData Text: must contain 'text' providing a string"); return false; }

						} //end Text ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
						else if (pyTypeStr == "TriangleList")
						{
							//dataTriangleList = { 'type':'TriangleList', 
							//'points' : [x0,y0,z0, x1,y1,z1, ...],
							//'colors' : [R0,G0,B0,A0, R1,G2,B1,A1, ...],
							//'normals' : [n0x,n0y,n0z, ...],
							//'triangles' : [T0point0, T0point1, T0point2, ...] }

							ResizableArray<Float3> points; //temporary data, used during evaluation of data
							ResizableArray<Float3> normals; //temporary data, used during evaluation of data (must be size of points)
							ResizableArray<Float4> colors; //temporary data, used during evaluation of data (must be size of points)

							if (gDict.contains("points"))
							{
								py::object gDictList = gDict["points"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gDictList)) //must be a list of graphicsData dictionaries
								{
									py::list gList = (py::list)(gDictList);
									std::vector<float> stdGList = py::cast<std::vector<float>>(gList); //! # read out dictionary and cast to C++ type

									if ((stdGList.size() % 3) == 0)
									{
										Index n = (Index)stdGList.size() / 3;
										points.SetNumberOfItems(n);
										for (Index i = 0; i < n; i++)
										{
											points[i] = Float3({ stdGList[i * 3],stdGList[i * 3 + 1],stdGList[i * 3 + 2] });
											//pout << "p" << i << " = " << points[i] << "\n";
										}
									}
									else { PyError("GraphicsData::TriangleList::points must be a float list with 3*n components, n being the number of points"); return false; }
								}
								else { PyError("GraphicsData::TriangleList::points must be a float list"); return false; }
							}
							else
							{
								PyError("GraphicsData::TriangleList must contain 'points' being a float list with n*(x,y,z)-components, n being the number of points");
							}

							if (gDict.contains("colors"))
							{
								py::object gDictList = gDict["colors"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gDictList)) //must be a list of graphicsData dictionaries
								{
									py::list gList = (py::list)(gDictList);
									std::vector<float> stdGList = py::cast<std::vector<float>>(gList); //! # read out dictionary and cast to C++ type

									if ((stdGList.size() % 4) == 0 && (int)(stdGList.size() / 4) == points.NumberOfItems())
									{
										Index n = (Index)stdGList.size() / 4;
										colors.SetNumberOfItems(n);
										for (Index i = 0; i < n; i++)
										{
											colors[i] = Float4({ stdGList[i * 4], stdGList[i * 4 + 1], stdGList[i * 4 + 2], stdGList[i * 4 + 3] });
										}
									}
									else { PyError("GraphicsData::TriangleList::colors must be a float list with 4*n components (R,G,B,A), n being the identical to the number of points"); return false; }
								}
								else { PyError("GraphicsData::TriangleList::colors must be a float list"); return false; }
							}
							else
							{   //set default color
								colors.SetNumberOfItems(points.NumberOfItems());
								for (auto& color : colors) { color = EXUvis::defaultColorBlue4; }
							}

							bool normalsDefined = true;
							if (gDict.contains("normals"))
							{
								py::object gDictList = gDict["normals"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gDictList)) //must be a list of graphicsData dictionaries
								{
									py::list gList = (py::list)(gDictList);
									std::vector<float> stdGList = py::cast<std::vector<float>>(gList); //! # read out dictionary and cast to C++ type

									if ((stdGList.size() % 3) == 0 && (int)(stdGList.size() / 3) == points.NumberOfItems())
									{
										Index n = (Index)stdGList.size() / 3;
										normals.SetNumberOfItems(n);
										for (Index i = 0; i < n; i++)
										{
											normals[i] = Float3({ stdGList[i * 3],stdGList[i * 3 + 1],stdGList[i * 3 + 2] });
										}
									}
									else { PyError("GraphicsData::TriangleList::normals must be a float list with 3*n components (nx,ny,nz), n being the identical to the number of points"); return false; }
								}
								else { PyError("GraphicsData::TriangleList::normals must be a float list"); return false; }
							}
							else
							{   //set default normal
								normals.SetNumberOfItems(points.NumberOfItems());
								for (auto& normal : normals) { normal = Float3({ 0,0,0 }); }
								normalsDefined = false;
							}

							if (gDict.contains("triangles"))
							{
								py::object gDictList = gDict["triangles"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gDictList)) //must be a list of graphicsData dictionaries
								{
									py::list gList = (py::list)(gDictList);
									std::vector<Index> stdGList = py::cast<std::vector<Index>>(gList); //! # read out dictionary and cast to C++ type

									if ((stdGList.size() % 3) == 0)
									{
										Index n = (Index)stdGList.size() / 3;
										Index np = points.NumberOfItems();
										GLTriangle trig;
										trig.isFiniteElement = false;
										for (Index i = 0; i < n; i++)
										{
											Index3 pointInd = Index3({ stdGList[i * 3], stdGList[i * 3 + 1], stdGList[i * 3 + 2] });
											for (Index j = 0; j < 3; j++)
											{
												Index ind = pointInd[j];
												if (EXUstd::IndexIsInRange(ind, 0, np))
												{
													trig.points[j] = points[ind];
													trig.normals[j] = normals[ind];
													trig.colors[j] = colors[ind];
												}
												else
												{
													PyError(STDstring("GraphicsData::TriangleList::triangles: point indices need to be in range [0, points.size()-1], but got index: ") + EXUstd::ToString(ind)); return false;
												}
											}
											if (!normalsDefined)
											{
												EXUvis::ComputeTriangleNormals(trig.points, trig.normals);
												//Float3 v1 = trig.points[1] - trig.points[0];
												//Float3 v2 = trig.points[2] - trig.points[0];
												//Float3 n = v1.CrossProduct(v2); //@todo: need to check correct outward normal direction in openGL
												//float len = n.GetL2Norm();
												//if (len != 0.f) { n *= 1.f/len; }
												//trig.normals[0] = n;
												//trig.normals[1] = n;
												//trig.normals[2] = n;
											}
											data.glTriangles.Append(trig);
											//pout << "trig" << i << " = " << trig.points[0] << "," << trig.points[0] << "," << trig.points[0] << "\n";
										}
									}
									else { PyError("GraphicsData::TriangleList::triangles must be a float list with 3*n components, n being the number of triangles"); return false; }
								}
								else { PyError("GraphicsData::TriangleList::triangles must be a float list"); return false; }
							}
							else
							{
								PyError("GraphicsData::TriangleList must contain 'triangles' being a float list with n*(point0,point1,point2)-components, n being the number of triangles; point0, point1, point2 ... point indices of one triangle");
							}

							//++++++++++++++++++++++++++++++++++++++++
							//NEW: add edges:
							Float4 edgeColor({0,0,0,1});
							if (gDict.contains("edgeColor"))
							{
								py::object gColor = gDict["edgeColor"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gColor)) //must be a list of graphicsData dictionaries
								{
									py::list colorList = (py::list)(gColor);
									std::vector<float> stdColorList = py::cast<std::vector<float>>(colorList); //! # read out dictionary and cast to C++ type

									if (stdColorList.size() == 4)
									{
										edgeColor = Float4(stdColorList);
									}
									else { PyError("GraphicsData TriangleList: edgeColor must be a float vector with 4 components"); return false; }
								}
								else { PyError("GraphicsData TriangleList: edgeColor must be a float vector with 4 components"); return false; }
							}

							if (gDict.contains("edges"))
							{
								std::vector<int> stdEdgesList;
								Index nEdges = 0;
								py::object gData = gDict["edges"]; //this is necessary to make isinstance work
								if (py::isinstance<py::list>(gData)) //must be a list of graphicsData dictionaries
								{
									py::list dataList = (py::list)(gData);
									stdEdgesList = py::cast<std::vector<int>>(dataList); //! # read out dictionary and cast to C++ type

									nEdges = (Index)stdEdgesList.size() / 2;

									if (nEdges * 2 != (Index)stdEdgesList.size())
									{
										PyError("GraphicsData TriangleList: for n edges, points must be an int vector with exactly 2*n components"); return false;
									}
								}
								else { PyError("GraphicsData TriangleList: 'edges' must be a list with 2*n integer values"); return false; }

								GLLine line; line.itemID = -1;
								line.color1 = edgeColor;
								line.color2 = edgeColor;
								for (Index k = 0; k < nEdges; k++)
								{
									line.point1 = points[stdEdgesList[2 * k]];
									line.point2 = points[stdEdgesList[2 * k+1]];
									data.glLines.Append(line);
								}
							}








						} //end triangles +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
						else
						{
							PyError(STDstring("GraphicsData type '") + pyTypeStr + "' not supported");
						}
					} //if (py::isinstance<py::str>(pyType))
				} //gDict.contains("type")
				else
				{
					PyError("GraphicsData must contain a 'type'"); return false;
				}
			}
		}//for-loop graphics items
	}
	else 
	{ 
		PyError("GraphicsData must be of type list: [graphicsDict1, graphicsDict2, ...]"); return false; 
	}
	return true;
}

//! python function to write BodyGraphicsData to dictionary, e.g. for testing; 
py::list PyGetBodyGraphicsDataList(const BodyGraphicsData& data, bool addGraphicsData)
{
	auto list = py::list();
	//d["type"] = std::string("Line");
	//ResizableArray<GLLine> glLines;				//!< lines to be displayed
	//ResizableArray<GLCircleXY> glCirclesXY;		//!< circles to be displayed
	//ResizableArray<GLText> glTexts;				//!< texts to be displayed
	//ResizableArray<GLTriangle> glTriangles;	//!< triangles to be displayed

	if (!addGraphicsData)
	{
		auto d = py::dict();
		d["graphicsData"] = std::string("<not requested>");
		list.append(d);
	}
	else
	{
		//CHECKandTHROWstring("PyGetBodyGraphicsDataDictionary: to be implemented");
		for (auto item : data.glCirclesXY)
		{
			auto d = py::dict();
			d["type"] = std::string("Circle");
			d["radius"] = item.radius;
			d["color"] = (std::vector<float>)item.color;
			d["position"] = (std::vector<float>)item.point;
			list.append(d);
		}

		for (auto item : data.glTexts)
		{
			auto d = py::dict();
			d["type"] = std::string("Text");
			d["text"] = std::string(item.text);
			d["color"] = (std::vector<float>)item.color;
			d["position"] = (std::vector<float>)item.point;
			list.append(d);
		}

		Index nLines = data.glLines.NumberOfItems();
		if (nLines != 0)
		{
			auto d = py::dict();
			d["type"] = std::string("Lines");
			std::vector<float> colors; colors.reserve((size_t)(nLines * 8));
			std::vector<float> points; points.reserve((size_t)(nLines * 6));

			for (auto item : data.glLines)
			{
				for (Index i = 0; i < 4; i++) { colors.push_back(item.color1[i]); }
				for (Index i = 0; i < 4; i++) { colors.push_back(item.color2[i]); }
				for (Index i = 0; i < 3; i++) { points.push_back(item.point1[i]); }
				for (Index i = 0; i < 3; i++) { points.push_back(item.point2[i]); }
			}
			d["colors"] = colors;
			d["points"] = points;
			list.append(d);
		}

		Index nTrigs = data.glTriangles.NumberOfItems();
		if (nTrigs != 0)
		{
			auto d = py::dict();
			d["type"] = std::string("TriangleList");
			std::vector<float> points; points.reserve((size_t)(nTrigs * 3 * 3));
			std::vector<float> colors; colors.reserve((size_t)(nTrigs * 4 * 3));
			std::vector<float> normals; normals.reserve((size_t)(nTrigs * 3 * 3));
			std::vector<int> triangles; triangles.reserve((size_t)(nTrigs * 3));

			Index cnt = 0; //counts points
			for (auto item : data.glTriangles)
			{
				for (Index i = 0; i < 3; i++) { triangles.push_back(cnt++); } //local indices of points
				for (Index j = 0; j < 3; j++)
				{
					for (Index i = 0; i < 4; i++) { colors.push_back(item.colors[j][i]); }
					for (Index i = 0; i < 3; i++) { points.push_back(item.points[j][i]); }
					for (Index i = 0; i < 3; i++) { normals.push_back(item.normals[j][i]); }
				}
			}
			d["points"] = points;
			d["colors"] = colors;
			d["normals"] = normals;
			d["triangles"] = triangles;
			list.append(d);
		}
	}
	return list;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//for BodyGraphicsData lists (KinematicTree)
//! object graphics data for ground objects, rigid bodies and mass points
bool PyWriteBodyGraphicsDataListOfLists(const py::dict& d, const char* item, BodyGraphicsDataList& data)
{
	data.Flush(); //erases all objects
	if (d.contains(item))
	{
		py::object other = d[item]; //this is necessary to make isinstance work
		return PyWriteBodyGraphicsDataListOfLists(other, data);
	}//if "GraphicsDataList" does not exist, no error is displayed
	return true;
}

//! python function to read BodyGraphicsData from py::object, which must be a list of graphicsData dictionaries
bool PyWriteBodyGraphicsDataListOfLists(const py::object object, BodyGraphicsDataList& data)
{

	if (py::isinstance<py::list>(object)) //must be a list of graphicsData dictionaries
	{
		GenericExceptionHandling([&]
		{
			data.Flush(); //erases all objects
			py::list list = (py::list)(object);

			for (auto graphicsItem : list)
			{
				const py::object& pyObject = (const py::object&)graphicsItem;
				BodyGraphicsData oneData;
				Index i = data.Append(oneData);

				PyWriteBodyGraphicsDataList(pyObject, data[i]);
			}
		}, "Exception raised when writing BodyGraphicsDataList: check format");
	}
	else
	{
		PyError("GraphicsDataList must be of type list: [graphicsData, graphicsData, ...]"); return false;
	}
	return true;
}

//! python function to write BodyGraphicsData to dictionary, e.g. for testing; 
py::list PyGetBodyGraphicsDataListOfLists(const BodyGraphicsDataList& data, bool addGraphicsData)
{
	auto listOfLists = py::list();

	//CHECKandTHROWstring("GetBodyGraphicsDataList: not yet implemented");

	for (auto graphicsData : data)
	{
		auto list = PyGetBodyGraphicsDataList(*graphicsData, addGraphicsData);
	}
	return listOfLists;
}

