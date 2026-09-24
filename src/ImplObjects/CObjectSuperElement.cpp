/** ***********************************************************************************************
* @brief        Implementation of VisualizationObjectSuperElement::UpdateGraphics, moved here
*               from src/Objects/VisuNodePoint.cpp (#2555). The
*               object is a base class and had no .cpp of its own until now.
*
* @author       Gerstmayr Johannes
* @date         2026-09-20 (created)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include "Main/CSystemData.h"

//for the UpdateGraphics of this item, moved here from VisuNodePoint.cpp
//(#2555):
#include "Graphics/VisualizationItemHelpers.h" //revision2026 step R11.4.4 (#2555)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//VISUALIZATION
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! Update visualizationSystem -> graphicsData for item
void VisualizationObjectSuperElement::UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber)
{
	Index itemID = Index2ItemID(itemNumber, ItemType::Object, vSystem->GetSystemID());
	Float4 currentColor = visualizationSettings.bodies.defaultColor;
	if (GetColor()[0] != -1.f) { currentColor = GetColor(); }

	CObjectSuperElement* cObject = (CObjectSuperElement*)vSystem->systemData->GetCObjects()[itemNumber];

	Index localRigidBodyNodeNumber; //local number in body!
	bool hasReferenceFrame = cObject->HasReferenceFrame(localRigidBodyNodeNumber);

	Matrix3D refRot = EXUmath::unitMatrix3D;
	Vector3D refPos({ 0,0,0 });

	if (hasReferenceFrame)
	{
		refRot = ((const CNodeRigidBody*)cObject->GetCNode(localRigidBodyNodeNumber))->GetRotationMatrix(ConfigurationType::Visualization); //cObject->GetCNode(...) takes local number
		refPos = ((const CNodeRigidBody*)cObject->GetCNode(localRigidBodyNodeNumber))->GetPosition(ConfigurationType::Visualization);
	}

	Real scaleFactor = visualizationSettings.bodies.deformationScaleFactor;

	if (GetShowNodes() && visualizationSettings.nodes.show) //show nodes with correct reference frame
	{
		//node size only defined globally
		float radius = 0.5f*visualizationSettings.nodes.defaultSize;
		if (visualizationSettings.nodes.defaultSize == -1.f) { radius = 0.5f*visualizationSettings.openGL.advanced.initialMaxSceneSize * 0.002f; } //{ radius = 0.5f*vSystem->renderState.maxSceneSize * 0.002f; }

		Vector3D nodePos;

		for (Index i = 0; i < cObject->GetNumberOfMeshNodes(); i++)
		{
			if (scaleFactor == 1.)
			{
				nodePos = cObject->GetMeshNodePosition(i, ConfigurationType::Visualization);
			}
			else
			{
				nodePos = cObject->GetMeshNodeLocalPosition(i, ConfigurationType::Visualization);
				Vector3D nodeRefPos = cObject->GetMeshNodeLocalPosition(i, ConfigurationType::Reference);
				nodePos = scaleFactor * (nodePos - nodeRefPos) + nodeRefPos;
				nodePos = refPos + refRot * nodePos;
			}

			Index tiling = visualizationSettings.nodes.drawNodesAsPoint ? 0 : visualizationSettings.nodes.tiling;
			EXUvis::DrawNode(nodePos, radius, currentColor, vSystem->graphicsData, itemID, drawNodesMarkersLoadsWithFaces, tiling); //itemID of SuperElement object!!!

			if (visualizationSettings.nodes.showNumbers) 
			{ 
				EXUvis::DrawItemNumber(nodePos, vSystem, Index2ItemID(i, ItemType::_None, vSystem->GetSystemID()), "NF", EXUvis::grey1); //use ItemType::_None, because should not be identified as node ...
			}
		}
	}

	if (GetTriangleMesh().NumberOfRows() != 0)
	{

		//process triangles of mesh to draw
		std::array<Float4, 3> colors;
		colors[0] = currentColor;
		colors[1] = currentColor;
		colors[2] = currentColor;
		//Vector contourValue; //memory allocation only in case of contour plot, but only once for whole mesh ...!
		Vector& contourValue = vSystem->tempVector;

		std::array<Vector3D, 3> nodes;
		std::array<Vector3D, 3> normals;
		//V, V, useFirstNodeAsReferenceFrame, , , bool, "false", , IO, "set true, if first node ($n_0$) is used as floating reference frame; all other nodes are interpreted relative to the reference frame; used to implement FFRF (floating frame of reference formulation); NOTE that in this case, nodes $[n_1,\,\ldots,\,n_n]\tp$ are still drawn without the reference frame"
		//	V, V, showNodes, , , bool, "false", , IO, "set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False"

		for (Index i = 0; i < GetTriangleMesh().NumberOfRows(); i++)
		{
			for (Index j = 0; j < 3; j++)
			{
				colors[j] = currentColor; //set back to default if some values are invalid
				Index meshNodeIndex = (Index)GetTriangleMesh()(i, j);
				if (scaleFactor == 1.)
				{
					nodes[j] = cObject->GetMeshNodePosition(meshNodeIndex, ConfigurationType::Visualization);
				}
				else
				{
					nodes[j] = cObject->GetMeshNodeLocalPosition(meshNodeIndex, ConfigurationType::Visualization);
					Vector3D nodeRefPos = cObject->GetMeshNodeLocalPosition(meshNodeIndex, ConfigurationType::Reference);
					nodes[j] = scaleFactor * (nodes[j] - nodeRefPos) + nodeRefPos;
					nodes[j] = refPos + refRot * nodes[j];
				}

				//add contour plot values to color; may NOT be called if contour.outputVariable == None (GetOutputVariable(...) fails!)
				if (EXUstd::IsOfTypeAndNotNone(cObject->GetOutputVariableTypesSuperElement(meshNodeIndex), visualizationSettings.contour.outputVariable))
				{
					cObject->GetOutputVariableSuperElement(visualizationSettings.contour.outputVariable, meshNodeIndex, ConfigurationType::Visualization, contourValue); //memory allocation!
					EXUvis::ComputeContourColor< Vector>(contourValue, visualizationSettings.contour.outputVariable, 
						visualizationSettings.contour.outputVariableComponent, colors[j]);
				}

			}
			//compute normals:
			Vector3D v0 = nodes[1] - nodes[0];
			Vector3D v1 = nodes[2] - nodes[0];
			Vector3D n = v0.CrossProduct(v1);
			Real len = n.GetL2Norm();
			if (len != 0) { n *= 1. / len; }
			normals[0] = n;
			normals[1] = n;
			normals[2] = n;

			vSystem->graphicsData.AddTriangle(nodes, normals, colors, itemID, true);
		}
	}


	//draw body number at reference frame node position
	if (visualizationSettings.bodies.showNumbers)
	{
		Vector3D refPos3D = cObject->GetPosition(Vector3D({ 0,0,0 }), ConfigurationType::Visualization);
		EXUvis::DrawItemNumber(refPos3D, vSystem, itemID, "RF", currentColor);
	}

}
