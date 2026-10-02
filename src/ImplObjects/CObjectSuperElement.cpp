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
#include "Graphics/VisualizationItemHelpers.h" //#2555

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//VISUALIZATION
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Vector3D CObjectSuperElement::GetMeshNodePositionVisualization(Index meshNodeNumber, Real deformationScaleFactor) const
{
	if (deformationScaleFactor == 1.) { return GetMeshNodePosition(meshNodeNumber, ConfigurationType::Visualization); }

	Vector3D position = GetMeshNodeLocalPositionVisualization(meshNodeNumber, deformationScaleFactor);
	Index localRigidBodyNodeNumber; //local number in body!
	if (HasReferenceFrame(localRigidBodyNodeNumber))
	{
		const CNodeRigidBody* frameNode = (const CNodeRigidBody*)GetCNode(localRigidBodyNodeNumber);
		position = frameNode->GetPosition(ConfigurationType::Visualization) + frameNode->GetRotationMatrix(ConfigurationType::Visualization) * position;
	}
	return position;
}

//! Update visualizationSystem -> graphicsData for item
void VisualizationObjectSuperElement::UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber)
{
	Index itemID = Index2ItemID(itemNumber, ItemType::Object, vSystem->GetSystemID());
	Float4 currentColor = visualizationSettings.bodies.defaultColor;
	if (GetColor()[0] != -1.f) { currentColor = GetColor(); }

	CObjectSuperElement* cObject = (CObjectSuperElement*)vSystem->systemData->GetCObjects()[itemNumber];

	Real scaleFactor = visualizationSettings.bodies.deformationScaleFactor;

	if (GetShowNodes() && visualizationSettings.nodes.show) //show nodes with correct reference frame
	{
		//node size only defined globally
		float radius = 0.5f*visualizationSettings.nodes.defaultSize;
		if (visualizationSettings.nodes.defaultSize == -1.f) { radius = 0.5f*visualizationSettings.openGL.advanced.initialMaxSceneSize * 0.002f; } //{ radius = 0.5f*vSystem->renderState.maxSceneSize * 0.002f; }

		Vector3D nodePos;

		for (Index i = 0; i < cObject->GetNumberOfMeshNodes(); i++)
		{
			nodePos = cObject->GetMeshNodePositionVisualization(i, scaleFactor);

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
		//the triangles of the mesh: 3 columns flat, 6 columns 6-node triangles, which the renderers split when they draw
		//(#2709); points and contour colors at the deformed mesh nodes
		const Index nColumns = GetTriangleMesh().NumberOfColumns();
		Vector& contourValue = vSystem->tempVector; //memory allocation only in case of contour plot, but only once for whole mesh
		std::array<Vector3D, 6> nodes;
		std::array<Float4, 6> colors;

		for (Index i = 0; i < GetTriangleMesh().NumberOfRows(); i++)
		{
			for (Index j = 0; j < nColumns; j++)
			{
				colors[j] = currentColor; //set back to default if some values are invalid
				Index meshNodeIndex = (Index)GetTriangleMesh()(i, j);
				nodes[j] = cObject->GetMeshNodePositionVisualization(meshNodeIndex, scaleFactor);

				//add contour plot values to color; may NOT be called if contour.outputVariable == None (GetOutputVariable(...) fails!)
				if (EXUstd::IsOfTypeAndNotNone(cObject->GetOutputVariableTypesSuperElement(meshNodeIndex), visualizationSettings.contour.outputVariable))
				{
					cObject->GetOutputVariableSuperElement(visualizationSettings.contour.outputVariable, meshNodeIndex, ConfigurationType::Visualization, contourValue); //memory allocation!
					EXUvis::ComputeContourColor< Vector>(contourValue, visualizationSettings.contour.outputVariable,
						visualizationSettings.contour.outputVariableComponent, colors[j]);
				}
			}
			if (nColumns == 6)
			{
				GLTriangle6 trig6;
				trig6.itemID = itemID;
				trig6.hasNormals = false; //the normals of the deformed geometry, computed by the split
				trig6.isFiniteElement = true;
				for (Index j = 0; j < 6; j++)
				{
					trig6.points[j] = Float3({ (float)nodes[j][0], (float)nodes[j][1], (float)nodes[j][2] });
					trig6.normals[j] = Float3({ 0.f, 0.f, 0.f });
					trig6.colors[j] = colors[j];
				}
				vSystem->graphicsData.glTriangles6.Append(trig6);
			}
			else
			{
				std::array<Vector3D, 3> points = { nodes[0], nodes[1], nodes[2] };
				std::array<Float4, 3> colors3 = { colors[0], colors[1], colors[2] };
				vSystem->graphicsData.AddTriangle(points, colors3, itemID, true); //normal of the flat triangle
			}
		}
	}


	//draw body number at reference frame node position
	if (visualizationSettings.bodies.showNumbers)
	{
		Vector3D refPos3D = cObject->GetPosition(Vector3D({ 0,0,0 }), ConfigurationType::Visualization);
		EXUvis::DrawItemNumber(refPos3D, vSystem, itemID, "RF", currentColor);
	}

}
