/** ***********************************************************************************************
* @brief        Implementation of GlfwClient
*
* @author       Gerstmayr Johannes
* @date         2019-05-24 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

//++++++++++++++++++++++++++++++++++++++++

#include "Graphics/Raytracing.h"

#include "Utilities/Parallel.h" //include after 

extern bool PyIsRendererActive(bool deprecationWarning);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Raytracer raytracer;	//!< currently, use global raytracer to avoid sharing among BasicVisualizationSystemContainer

#define perspectiveFactorLimit (0.001f)

bool rendererDebug = false;
Index nHits = 0;
Index warned = 10000;

Raytracer::Raytracer()
{
	basicVisualizationSystemContainer = NULL;
	//graphicsDataList = NULL;
	visSettings = NULL;
	renderState = NULL;

	bitmapFont = BitmapFont(charBitmap64::fontSize, charBitmap64::nCharacters, charBitmap64::characterOffset,
		charBitmap64::characterWidth, charBitmap64::characterHeight,
		charBitmap64::characterByteWidth, charBitmap64::characterBytes,
		charBitmap64::OpenGLtextBitmap);
}

Raytracer::~Raytracer()
{
	if (fontBitmaps.NumberOfItems() != 0)
	{   //free memory
		for (gchar* item : fontBitmaps)
		{
			delete[] item;
		}
	}
}

// Perform ray-triangle intersection test
void Raytracer::IntersectRayWithTriangle(const RTVector3D& rayOrigin, const RTVector3D& rayDirection, const GLTriangle& triangle, 
	const RaytracingSettings& RTS, IntersectionResult& result)
{
	//if (rendererDebug) { nHits++; }
	result.Init();

	const RTVector3D& v0 = triangle.points[0];
	const RTVector3D& v1 = triangle.points[1];
	const RTVector3D& v2 = triangle.points[2];

	RTVector3D edge1 = v1 - v0;
	RTVector3D edge2 = v2 - v0;
	RTVector3D pVec = rayDirection.CrossProduct(edge2);
	RTfloat det = edge1 * pVec;

	if (det * det <= RTS.tolParallel2) { return; }  // Ray parallel to triangle

	RTVector3D tVec = rayOrigin - v0;
	RTfloat invDet = 1.0f / det; //division is done here, to save time
	RTfloat u = (tVec * pVec) * invDet;
	if ((u < -RTS.epsilon5) || (u > 1.f+RTS.epsilon5)) { return; }

	RTVector3D qVec = tVec.CrossProduct(edge1);
	RTfloat v = (rayDirection * qVec) * invDet;
	if ((v < -RTS.epsilon5) || (u + v > 1.f+RTS.epsilon5) ) { return; }

	RTfloat t = (edge2 * qVec) * invDet;
	if (t < RTS.minZ) { return; }

	// Fill result
	RTfloat w = 1.0f - u - v;
	result.hit = true;
	result.distance = t;
	result.u = u;
	result.v = v;
	//result.triangleIndex = -1; // to be filled by caller
	result.normal = triangle.normals[0] * w + triangle.normals[1] * u + triangle.normals[2] * v;
	result.normal.NormalizeSafe();
	result.color = triangle.colors[0] * w + triangle.colors[1] * u + triangle.colors[2] * v;
	result.color[3] = triangle.colors[0][3]; //do not interpolate material index!
	RTS.ColorRGBA2MaterialColor(result.color, result.materialIndex, result.alpha);

	result.hitFromBack = (rayDirection * result.normal) > 0.f;

	if (result.hitFromBack) { result.normal = -result.normal; } //adjust, such that nobody notices ...

	return;
}

//! check if point is in shadow with given light
bool Raytracer::IsInShadow(const RTVector3D& point, const RTVector3D& lightDirection, const ResizableArray<GLTriangle>& triangles, 
	const RaytracingSettings& RTS, RTfloat maxDistance) 
{
	IntersectionResult shadowHit;
	IntersectRayWithTriangles(point, lightDirection, maxDistance, triangles, RTS, shadowHit, true);

	return (shadowHit.hit && shadowHit.distance < maxDistance);
}


//refraction for transparency
RTVector3D Raytracer::Refract(const RTVector3D& I, const RTVector3D& N, RTfloat eta) 
{
	RTfloat cosi = std::clamp(I * N, -1.0f, 1.0f);
	RTfloat etai = 1.0f, etat = eta;
	RTVector3D n = N;
	if (cosi < 0) {
		cosi = -cosi;
	}
	else {
		std::swap(etai, etat);
		n = -N;
	}
	RTfloat etaRatio = etai / etat;
	RTfloat k = 1 - etaRatio * etaRatio * (1 - cosi * cosi);
	if (k < 0) {
		return RTVector3D{ 0, 0, 0 }; // Total internal reflection
	}
	return I * etaRatio + n * (etaRatio * cosi - std::sqrt(k));
}

void Raytracer::IntersectRayWithTriangles(const RTVector3D& rayOrigin, const RTVector3D& rayDirection, RTfloat rayLength,
	const ResizableArray<GLTriangle>& triangles, const RaytracingSettings& RTS, IntersectionResult& closestHit,
	bool returnOnSingleHit, RTfloat minDistanceZ)
{
	Index threadID = RTS.numberOfThreads == 1 ? 0 : ExuThreading::TaskManager::GetThreadId();

	constexpr bool useFullLineSearch = false; //false is faster but can have problems

	IntersectionResult hit;
	closestHit.Init();
	closestHit.distance = rayLength; //for lights, we do not look for anything beyond rayLength

	if (useFullLineSearch)
	{
		RTS.searchTree.GetBinsCrossedByLine(rayOrigin, rayDirection, RTS.tempSearchTreeBins[threadID]);
		for (Index binInd : RTS.tempSearchTreeBins[threadID])
		{
			const ArrayIndex& trigIndices = RTS.searchTree.GetItemsOfBin(binInd);
			for (Index iInd : trigIndices)
			{
				IntersectRayWithTriangle(rayOrigin, rayDirection, triangles[iInd], RTS, hit);
				if (hit.hit && hit.distance < closestHit.distance &&
					(hit.distance*(-rayDirection[2]) >= minDistanceZ))
				{
					closestHit = hit;
					closestHit.triangleIndex = iInd;
					if (returnOnSingleHit || IsStaticTriangle(closestHit.triangleIndex)) { return; }
				}
			}
		}
	}
	else
	{
		//RTfloat tmax = 0;
		Index stepX = 0, stepY = 0, stepZ = 0;
		RTfloat tDeltaX = 0, tDeltaY = 0, tDeltaZ = 0;
		Index ix = 0, iy = 0, iz = 0;
		RTfloat tMaxX = 0, tMaxY = 0, tMaxZ = 0;

		//bool search = RTS.searchTree.GetBinsCrossedByLineInit(rayOrigin, rayDirection * rayLength, RTS.tempSearchTreeBins[threadID],
		bool search = RTS.searchTree.GetBinsCrossedByLineInit(rayOrigin, rayDirection, RTS.tempSearchTreeBins[threadID],
			//tmax,
			stepX, stepY, stepZ,
			tDeltaX, tDeltaY, tDeltaZ,
			ix, iy, iz,
			tMaxX, tMaxY, tMaxZ);

		Index sx = RTS.searchTree.SizeX();
		Index sy = RTS.searchTree.SizeY();
		Index sz = RTS.searchTree.SizeZ();

		//Box3DBase<RTfloat> box = RTS.searchTree.GetBinBox(0,0,0); //must always exist
		RTVector3D boxSize;
		RTS.searchTree.GetCellLengths(boxSize[0], boxSize[1], boxSize[2]);
		RTfloat boxRadius = boxSize.GetL2Norm() * (RTfloat)0.5;

		// Traverse until the ray exits the box
		while (search && ix >= 0 && ix < sx && iy >= 0 && iy < sy && iz >= 0 && iz < sz)
		{
			const ArrayIndex& trigIndices = RTS.searchTree.GetItemsOfBin(RTS.searchTree.GlobalIndex(ix, iy, iz));
			if (closestHit.hit)
			{
				Box3DBase<RTfloat> box = RTS.searchTree.GetBinBox(ix, iy, iz);
				//a conservative calculation that stops as soon as boxes are further away than the closest hit
				if ((box.Center() - rayOrigin).GetL2NormSquared() > EXUstd::Square(closestHit.distance + boxRadius + 20.f*RTS.epsilon)) { return; }
			}

			for (Index iInd : trigIndices)
			{
				IntersectRayWithTriangle(rayOrigin, rayDirection, triangles[iInd], RTS, hit);
				if (hit.hit && hit.distance < closestHit.distance &&
					(hit.distance* (-rayDirection[2]) >= minDistanceZ))
				{
					closestHit = hit;
					closestHit.triangleIndex = iInd;
					if (returnOnSingleHit || IsStaticTriangle(closestHit.triangleIndex)) { return; }
				}
			}

			search = RTS.searchTree.GetBinsCrossedByLineUpdate(//tmax,
				stepX, stepY, stepZ,
				tDeltaX, tDeltaY, tDeltaZ,
				ix, iy, iz,
				tMaxX, tMaxY, tMaxZ);// && (tMaxX < closestHit.distance || tMaxY < closestHit.distance || tMaxZ < closestHit.distance);
		}

	}



}


//for screen pixel or for reflection/refraction: compute pixel color of given ray origin and direction
RTVector3D Raytracer::ComputePixelColor(const RTVector3D& rayOrigin, const RTVector3D& rayDirection,
	const ResizableArray<GLTriangle>& triangles, const RaytracingSettings& RTS, RTfloat& zDepth, ShadowValue& shadowValue,
	Index reflectionDepth, Index transparencyDepth, Index shadowMode, const RTVector3D& rayOriginNormalized)
{
	IntersectionResult closestHit;
	RTfloat minDistanceZ = std::numeric_limits<RTfloat>::lowest();
	RTfloat maxDistanceTriangle = std::numeric_limits<RTfloat>::max();
	if (RTS.nearFarPlaneOffset[2] == 1.f)
	{
		minDistanceZ = rayOrigin[2] + RTS.nearFarPlaneOffset[0];
		maxDistanceTriangle = rayOrigin[2] + RTS.nearFarPlaneOffset[1];
	}

	if (RTS.useClippingPlane)// && reflectionDepth == 0 && transparencyDepth == 0)
	{
		bool hitFront;

		RTfloat distance;

		hitFront = RTS.clippingPlaneNormal * rayDirection < 0.f;
		if (EGeometry::LinePlaneIntersection(RTS.clippingPlanePoint, -RTS.clippingPlaneNormal,
			rayOrigin, rayDirection, distance))
		{
			RTfloat minDistanceZlocal = hitFront ? EXUstd::Maximum(distance*(-rayDirection[2]), minDistanceZ) : minDistanceZ;
			IntersectRayWithTriangles(rayOrigin, rayDirection,
				EXUstd::Minimum(RTS.maxSceneSize * RTS.zMaxSceneFactor * 2, maxDistanceTriangle),
				triangles, RTS,
				closestHit, false, minDistanceZlocal);
			if (!(closestHit.hit && IsStaticShadedTriangle(closestHit.triangleIndex)) )
			{
				if (hitFront)
				{
					if (closestHit.hit && closestHit.hitFromBack) //fails for wrong normals!
					{
						if (RTS.useClippingPlaneColor == 1)
						{
							return  RTS.clippingPlaneColor;
						}
						else if (RTS.useClippingPlaneColor == 2)
						{
							//special, but this doesnt work well for objects in objects
							return 0.75f * ColorRGBA2RGB(closestHit.color);
						}
						else
						{
							//do regular rendering
						}
					}
					//else: do regular rendering
				}
				else
				{
					if (closestHit.distance > distance)
					{
						closestHit.hit = false; //to use background
					}
				}
			}
		}
		else //orthogonal => just check the side
		{
			IntersectRayWithTriangles(rayOrigin, rayDirection, 
				//RTS.searchTreeBoxMaxRadius * 4, //original, too small for geometries with large aspect
				EXUstd::Minimum(RTS.maxSceneSize * RTS.zMaxSceneFactor * 2, maxDistanceTriangle),
				triangles, RTS,
				closestHit, false, minDistanceZ);

			if (closestHit.hit && !IsStaticShadedTriangle(closestHit.triangleIndex) &&
				(RTS.clippingPlaneNormal * (rayOrigin - RTS.clippingPlanePoint) > 0) )
			{
				closestHit.hit = false;
			}
		}
	}
	else
	{
		IntersectRayWithTriangles(rayOrigin, rayDirection, 
			EXUstd::Minimum(RTS.maxSceneSize * RTS.zMaxSceneFactor * 2, maxDistanceTriangle),
			triangles, RTS,
			closestHit, false, minDistanceZ);
	}

	if (!closestHit.hit)
	{
		shadowValue = { 1.f,1.f,1.f,1.f }; //no shadow at free sight! ( <-> artefacts at boundaries!)
		if (reflectionDepth == 0)
		{
			zDepth = 0.5f * rayDirection[2] * std::numeric_limits<RTfloat>::max(); //add 0.5 to avoid overflow
			if (!RTS.useGradientBackground)
			{
				return RTS.backgroundColor;
			}
			else
			{
				return RTS.backgroundColor * 0.5f * (1.f + rayOriginNormalized[1]) +
					RTS.backgroundColorBottom * 0.5f * (1.f - rayOriginNormalized[1]);
			}
		}
		else { return RTS.backgroundColorReflections; }
	}
	else
	{
		zDepth = rayOrigin[2] + rayDirection[2] * closestHit.distance;
	}
	const GLMaterial& material = (*RTS.materials)[closestHit.materialIndex];

	RTVector3D hitPoint = rayOrigin + rayDirection * closestHit.distance;

	// Ambient base color (could also be black without lights ...)
	RTVector3D shadedColor;
	EXUmath::MultVectorComponents(RTS.ambientLightColor,
		RTVector3D({ closestHit.color[0], closestHit.color[1], closestHit.color[2] }),
		shadedColor);
	shadedColor += material.emission;
	RTfloat otherZ; //unused, dummy

	//shadowMode: 0=normal, 1=precompute shadow (returns shadow in RTVector3D[0]), 2=use computed shadow
	if (shadowMode == 1) { shadowValue = { 0.f, 0.f, 0.f, 0.f }; } //Initialize: 0=shadow, 1=full light

	//for overlay triangles (e.g. colorbar), we only use the basic color value
	if (IsStaticTriangle(closestHit.triangleIndex)) { return RTVector3D({ closestHit.color[0], closestHit.color[1], closestHit.color[2] }); }

	for (Index lightID = 0; lightID < (Index)RTS.lights.size(); lightID++)
	{
		const RaytracingLight& light = RTS.lights[lightID];
		if (!light.active) { continue; }

		//variations: special spiral to minimize artifacts
		Index nLightPoints = (light.shadow
			&& light.position[3] != 0.f
			&& light.lightRadius > 0.f
			&& RTS.lightRadiusVariations > 1
			&& shadowMode != 2
			&& reflectionDepth == 0
			&& transparencyDepth == 0) ? RTS.lightRadiusVariations : 1;

		RTfloat factLights = 1.f / (RTfloat)nLightPoints;

		for (Index np = 0; np < nLightPoints; np++)
		{
			RTfloat lightDistance = std::numeric_limits<RTfloat>::max();
			RTVector3D lightPos = { light.position[0], light.position[1], light.position[2] };
			RTVector3D lightDir = lightPos;
			if (!light.useCameraFrame) //in camera frame, everything is already correct, as model is already converted to sceen/camera coordinates
			{
				//openGL lights transform this way (from camera to modelview frame):
				//state->GetRotationTranslationFWithMarker(rotationMV, translationMV,
				//	basicVisualizationSystemContainer, settingsView, false);
				//lightPos = lightPos * rotationMV + useLightDir * (translationMV * rotationMV); //light is rotated/translated back to camera view

				Matrix3DF rotationMV = EXUmath::Matrix4DtoMatrix3D(RTS.modelViewRM); //row-major
				lightDir = rotationMV * lightPos - light.position[3] * (RTS.translationView);
				//same: TransformVertexRM(lightPos, RTS.modelViewRM, lightDir); //lights are not rotated; now in rotated config
			}


			//std::cout << "np=" << np << "\n";
			if (light.position[3] == 0.f) //only direction is used!
			{
				lightDir.NormalizeSafe();
				//lightDistance = 1.e3f; //large distance, but not too large to keep angle computations right
				lightDistance = (lightDir - hitPoint).GetL2Norm(); //for attenuation, we still compute distance!
			}
			else //light is used as position
			{
				//option to use several point lights (but systematic, to avoid noise)
				lightDir -= hitPoint; //now lightDir is the direction
				lightDistance = EXUstd::Maximum(1e-5f, lightDir.GetL2Norm());
				lightDir *= 1.f / lightDistance;
				RTVector3D vec(0.f);
				if (nLightPoints > 1)
				{
					RTfloat fract = ((RTfloat)np / (RTfloat)nLightPoints);
					RTfloat radFact = sqrt(fract);
					//RTfloat x = RTS.lightRadius0 / lightDistance * (RTfloat)sin(EXUstd::pi_f * 2.f * fract); //original
					//RTfloat y = RTS.lightRadius0 / lightDistance * (RTfloat)cos(EXUstd::pi_f * 2.f * fract);

					fract *= sqrt((RTfloat)nLightPoints);
					RTfloat x = radFact * light.lightRadius / lightDistance * (RTfloat)sin(EXUstd::pi_f * 2.f * fract);
					RTfloat y = radFact * light.lightRadius / lightDistance * (RTfloat)cos(EXUstd::pi_f * 2.f * fract);
					RTVector3D basisN1, basisN2;
					EXUmath::ComputeOrthogonalBasisVectors(lightDir, basisN1, basisN2);

					lightDir += x * basisN1 + y * basisN2; //lightDir not normalized any more, but should be ok
				}
			}
			
			if (shadowMode != 2)
			{
				if (light.shadow)
				{
					if (IsInShadow(hitPoint + lightDir * 0.01f, lightDir, triangles, RTS, lightDistance)) 
					{
						continue;
					}
				}
			}
			else
			{
				factLights = shadowValue[lightID]; //precomputed shadowValue (0=full shadow, 1=full light) influences light factor
			}

			if (shadowMode == 1)
			{
				shadowValue[lightID] += factLights; //for each light computation, we add the fraction of light contribution ( sum(factLights)==1 )
			}
			else
			{
				// Diffuse
				RTfloat diffuseIntensity = std::max(0.0f, closestHit.normal * lightDir);
				RTfloat attenuation = 1.0f / (
					light.constantAttenuation +
					light.linearAttenuation * lightDistance +
					light.quadraticAttenuation * lightDistance * lightDistance
					);

				RTVector3D lightContribution = {
					closestHit.color[0] * light.diffuse * diffuseIntensity * attenuation,
					closestHit.color[1] * light.diffuse * diffuseIntensity * attenuation,
					closestHit.color[2] * light.diffuse * diffuseIntensity * attenuation
				};

				shadedColor += factLights * lightContribution;

				// Specular (Phong)
				RTVector3D viewDir = -rayDirection;
				RTVector3D halfVec = (lightDir + viewDir);
				halfVec.NormalizeSafe();

				RTfloat adjustedShininess = std::max(1.0f, material.shininess);
				//RTfloat adjustedShininess = std::max(1.0f, material.shininess * (1.0f - material.roughness)); //for simplification, roughness is not used for now!
				RTfloat specIntensity = std::pow(std::max(0.0f, closestHit.normal * halfVec), adjustedShininess);
				shadedColor += factLights * material.specular * light.specular * specIntensity * attenuation;
			}
		}
	}
	if (shadowMode == 1) //return and retrieve computed shadowValue
	{ 
		return RTVector3D({ 0.f,0.f,0.f }); 
	} 

	if (IsStaticShadedTriangle(closestHit.triangleIndex)) 
	{ //we only like to have diffuse and specular effects, but no shadow
		//simple shading with specular component:
		RTVector3D lightDir({ 0.f,0.f,1.f });
		RTfloat intensity = 0.4f + 0.6f * std::max(0.0f, closestHit.normal * lightDir);
		RTVector3D shadedColor({
			closestHit.color[0] * intensity,
			closestHit.color[1] * intensity,
			closestHit.color[2] * intensity });

		shadedColor += RTVector3D(0.6f * std::pow(std::max(0.0f, closestHit.normal * lightDir), 32.f) );

		return shadedColor;
	}

	RTfloat reflRefrOffset = 0.0001f; //original: 0.002f; distance at which we search for next triangle intersection with reflection and refraction; should be proportional to maxSceneSize? * 1e-5
	// Reflection (recursive)
	RTVector3D reflectionColor = RTS.backgroundColorReflections; // OLD: { 0, 0, 0 };
	if (material.reflectivity > 0 && reflectionDepth < RTS.maxReflectionDepth )
	{
		RTVector3D reflectDir = rayDirection - 2.0f * (rayDirection * closestHit.normal) * closestHit.normal;
		reflectDir.NormalizeSafe();
		reflectionColor = ComputePixelColor(hitPoint + reflectDir * reflRefrOffset, reflectDir, triangles, RTS, otherZ, shadowValue,
			reflectionDepth + 1,
			EXUstd::Maximum(reflectionDepth, transparencyDepth)); //for now, transparency not used with reflection
	}

	//Transparency/Refraction
	RTVector3D refractionColor = { 0,0,0 };
	RTfloat alphaUsed = closestHit.alpha;
	//alphaUsed = 1.f;

	if (alphaUsed < 1.0f && transparencyDepth < RTS.maxTransparencyDepth) {
		RTVector3D refractDir = Refract(rayDirection, closestHit.normal, material.ior);
		refractDir.NormalizeSafe();
		if (refractDir.GetL2NormSquared() > 0.0f)
		{  // No total internal reflection
			refractionColor = ComputePixelColor(hitPoint + refractDir * reflRefrOffset, refractDir,
				triangles, RTS, otherZ, shadowValue, EXUstd::Maximum(reflectionDepth, transparencyDepth), transparencyDepth + 1, 0,
				rayOriginNormalized); //use original rayOriginNormalized for gradient background in case of pure transparency, if we look through objects (not considering then refraction!)
		}
		else
		{
			refractionColor = RTS.backgroundColor;
		}
	}
	else { alphaUsed = 1.f; } //to keep original color in case that there is no transparency!

	// Combine: reflection + refraction + local shading
	RTVector3D localWithReflection = shadedColor * (1.0f - material.reflectivity) + reflectionColor * material.reflectivity;
	RTVector3D finalColor = localWithReflection * alphaUsed + refractionColor * (1.0f - alphaUsed);

	if (RTS.globalFogDensity > 0.f)
	{
		// distance from camera to hit point
		RTfloat fogDistance = closestHit.distance; (hitPoint - rayOrigin).GetL2Norm();
		fogDistance /= 2.f * RTS.searchTreeBoxMaxRadius; //normalize distance

		// exponential fog factor (more realistic than linear)
		RTfloat fogFactor = std::exp(-RTS.globalFogDensity * fogDistance);
		fogFactor = std::clamp(fogFactor, 0.0f, 1.0f); // ensure in [0, 1]

		// blend final color with fog
		finalColor = finalColor * fogFactor + RTS.globalFogColor * (1.0f - fogFactor);
	}

	return finalColor;
}

void Raytracer::RasterizeLine(const RTVector3D& p0, const RTVector3D& p1, const RTVector4D& color0, const RTVector4D& color1,
						      ResizableArray<RTfloat>& zdepths, ResizableArray<unsigned char>& pixelsRGBA,
							  const RaytracingSettings& RTS, bool isStaticObject, RTfloat nearPlaneZ)
{
	const RTfloat p = isStaticObject ? 0 : RTS.perspectiveFactor;
	bool usePerspective = (p > perspectiveFactorLimit);
	const RTfloat dist = usePerspective ? (RTS.zoom / p) : 0.0f;
	//DELETE:
	//RTfloat projectionNearMinPlane = RTS.projectionNearMin; //needed for projection, not for the clipping of screen
	//if (RTS.nearFarPlaneOffset[2] == 1.f)
	//{
	//	projectionNearMinPlane = EXUstd::Maximum(RTS.nearFarPlaneOffset[0], RTS.projectionNearMin);
	//}


	auto ModelViewToScreen = [&](const RTVector3D& p_world) -> RTVector3D {
		//must be synchronized with ToPerspectiveRay !
		if (!usePerspective) 
		{
			//orthographic projection
			RTfloat sx = (p_world[0] / (RTS.zoom * RTS.ratio) + 1.f) * 0.5f * RTS.imageWidth;
			RTfloat sy = (p_world[1] / RTS.zoom + 1.f) * 0.5f * RTS.imageHeight;
			return RTVector3D({ sx, sy, p_world[2] });
		}
		else 
		{
			//perspective projection: scale based on distance from eye (dist - z)
			//objects at z=0 have scale 1.0.
			RTfloat scale = dist / EXUstd::Maximum(dist - p_world[2], 1e-6f);
			if (!RTS.modelCentricView)
			{
				//dist = 0 => only projection counts (zoom not used here, as we use p to represent FOV)
				scale = 1.f  / (p * EXUstd::Maximum(-p_world[2], 1e-6f));
			}
			RTfloat sx = (p_world[0] * scale / (RTS.zoom * RTS.ratio) + 1.f) * 0.5f * RTS.imageWidth;
			RTfloat sy = (p_world[1] * scale / RTS.zoom + 1.f) * 0.5f * RTS.imageHeight;
			return RTVector3D({ sx, sy, p_world[2] });
		}
	};

	//reverse function to find world coordinates at z_world
	auto ScreenToModelView = [&](RTfloat x_scr, RTfloat y_scr, RTfloat z_world) -> RTVector3D {
		if (!usePerspective) {
			RTfloat mx = RTS.zoom * RTS.ratio * ((x_scr + 0.5f) / (0.5f * RTS.imageWidth) - 1.f);
			RTfloat my = RTS.zoom * ((y_scr + 0.5f) / (0.5f * RTS.imageHeight) - 1.f);
			return RTVector3D({ mx, my, z_world });
		}
		else {
			RTfloat invScale = (dist - z_world) / dist;
			RTfloat mx = RTS.zoom * RTS.ratio * ((x_scr + 0.5f) / (0.5f * RTS.imageWidth) - 1.f) * invScale;
			RTfloat my = RTS.zoom * ((y_scr + 0.5f) / (0.5f * RTS.imageHeight) - 1.f) * invScale;
			return RTVector3D({ mx, my, z_world });
		}
	};

	RTfloat lineWidth = RTS.lineWidth * RTS.AAfactor;

	RTVector3D s0 = ModelViewToScreen(p0);
	RTVector3D s1 = ModelViewToScreen(p1);

	//needs to be excluded, as some lines may be really far away => very costly!
	if ((s0[0] < -RTS.imageWidth || s0[0] >= 2 * RTS.imageWidth
		|| s0[1] < -RTS.imageHeight || s0[1] >= 2 * RTS.imageHeight) ||
	   (s1[0] < -RTS.imageWidth || s1[0] >= 2 * RTS.imageWidth
		|| s1[1] < -RTS.imageHeight || s1[1] >= 2 * RTS.imageHeight) ) { return; }

	RTfloat invW0 = 1.0f;
	RTfloat invW1 = 1.0f;

	if (usePerspective)
	{
		//adjust linear interpolation to correct interpolation of zDepth and color
		RTfloat w0 = dist - s0[2]; //distance from eye to p0
		RTfloat w1 = dist - s1[2]; //distance from eye to p1

		//safety for short distances (usually not relevant)
		if (w0 < 0.001f) { w0 = 0.001f; }
		if (w1 < 0.001f) { w1 = 0.001f; }

		invW0 = 1.0f / w0;
		invW1 = 1.0f / w1;
	}

	//colors divided by W for correct interpolation
	//RTVector4D colorW0 = color0 * invW0;
	//RTVector4D colorW1 = color1 * invW1;

	RTfloat dx = s1[0] - s0[0];
	RTfloat dy = s1[1] - s0[1];
	RTfloat length = std::sqrt(dx * dx + dy * dy);
	if (length < 1e-6f) { return; }

	int steps = static_cast<int>(length * 1.001f); // subpixel steps for smooth line; seems to need no more than 1
	float diameter = lineWidth / 2.001f;
	int radiusMin = int(diameter*0.5f);
	int radiusMax = std::max(0,int(std::round(diameter - int(diameter * 0.5f)) ) );
	RTfloat z = 0.f;
	RTfloat currentW = 1.f;
	for (int i = 0; i <= steps; ++i)
	{
		RTfloat t = (RTfloat)i / (RTfloat)steps;

		RTfloat x = s0[0] + dx * t;
		RTfloat y = s0[1] + dy * t;

		if (!usePerspective)
		{
			z = s0[2] + (s1[2] - s0[2]) * t; //linear approximation
		}
		else
		{
			//interpolate the reciprocal of W
			RTfloat currentInvW = (1.f - t) * invW0 + t * invW1;
			currentW = 1.0f / currentInvW;
			//perspective-correct world Z
			z = dist - currentW;
		}

		if (RTS.useClippingPlane && !isStaticObject)
		{
			//check if line is in clipping plane
			//RTVector3D p({ x,y,z });
			if (RTS.clippingPlaneNormal * (ScreenToModelView(x,y,z) - RTS.clippingPlanePoint) > 0)
			{
				continue; //skip point
			}
		}

		RTVector4D color = (1.f - t) * color0 + t * color1; //({ 0,0,0,1 }); //linear interpolation of color
		//RTVector4D color = ((1.f - t) * colorW0 + t * colorW1) * currentW; //correct color interpolation

		int maxWidth = radiusMin + radiusMax;
		for (int oy = -radiusMax; oy <= radiusMin; ++oy)
		{
			for (int ox = -radiusMax; ox <= radiusMin; ++ox)
			{
				if ((maxWidth >= 3 && (abs(ox)+abs(oy) >= maxWidth+maxWidth%2)) 
					//|| (maxWidth >= 4 && (abs(ox) + abs(oy) >= maxWidth-1)) 
					) {continue; } //avoid much thicker diagonal lines ...
				int px = int(std::round(x)) + ox;
				int py = int(std::round(y)) + oy;

				if (px < 0 || px >= RTS.imageWidth || py < 0 || py >= RTS.imageHeight) { continue; }

				Index pixelIdx = py * RTS.imageWidth + px;
				RTfloat zWithBias = z + RTS.zBiasLines;

				if ((zWithBias > zdepths[pixelIdx] && zWithBias < nearPlaneZ) || isStaticObject)
				{
					zdepths[pixelIdx] = zWithBias;
					Index iRGBA = pixelIdx * RTS.RTcolorDepth;
					RTfloat alpha = color[3];
					for (int c = 0; c < 3; ++c) // RGB channels
					{
						RTfloat existing = pixelsRGBA[iRGBA + c] / 255.f;
						RTfloat newValue = color[c] * alpha + existing * (1.0f - alpha);
						pixelsRGBA[iRGBA + c] = FloatColorToByte(newValue);
					}
				}
			}
		}
	}
}

void Raytracer::RasterizeCharacter(int charIndex, const RTVector3D& posPixels,
	const RTVector4D& color, RTfloat fontFactor, bool alwaysInFront)
{
	RTfloat originZ = posPixels[2];

	int ix = (int)posPixels[0];
	int iy = (int)posPixels[1];

	//std::cout << "ix=" << ix << ", iy=" << iy << ", f=" << fontFactor << "\n";

	// 1. Basic Image Bounds Check for the Origin
	//if (ix < 0 || ix >= RTS.imageWidth || iy < 0 || iy >= RTS.imageHeight) return;
	if (ix < -fontFactor * (RTfloat)bitmapFont.characterWidth || ix >= RTS.imageWidth ||
		iy < -fontFactor * (RTfloat)bitmapFont.characterHeight || iy >= RTS.imageHeight) { return; }

	// 2. Visibility Check (Per request: check origin only to avoid artifacts)
	// The reference code implies that larger Z values are "in front" or overwrite 
	// existing pixels (logic: if zWithBias > zdepths[i] -> draw).
	//Index originIdx = iy * RTS.imageWidth + ix;
	RTfloat zWithBias = originZ + RTS.zBiasLines; // Reuse line bias or add a specific text bias

	// If the text origin is behind what's already in the buffer, skip drawing.
	//if (!alwaysInFront && zWithBias <= RTS.zDepths[originIdx])
	//{
	//	return;
	//}

	// 3. Setup Dimensions
	const unsigned char* bitmap = fontBitmaps[charIndex-bitmapFont.characterOffset];
	int srcW = bitmapFont.characterWidth;
	int srcH = bitmapFont.characterHeight;
	int srcLineW = bitmapFont.characterByteWidth * 8; //characters are 8-bit aligned

	// Calculate scaled dimensions
	int dstW = static_cast<int>(srcW * fontFactor+1.f);
	int dstH = static_cast<int>(srcH * fontFactor+1.f);

	// 4. Rasterize the Character Bitmap
	for (int dy = 0; dy < dstH; ++dy)
	{
		for (int dx = 0; dx < dstW; ++dx)
		{
			// Calculate screen coordinates
			int px = ix + dx;
			int py = iy + dy;

			// Clip against screen boundaries
			if (px < 0 || px >= RTS.imageWidth || py < 0 || py >= RTS.imageHeight) { continue; }

			Index pixelIdx = py * RTS.imageWidth + px;
			if (!alwaysInFront && zWithBias <= RTS.zDepths[pixelIdx]) { continue; }

			// Calculate Texture Coordinates (Nearest Neighbor Mapping)
			// You could add bilinear filtering here if fontFactor is large.
			int sx = (int)(dx / fontFactor);
			int sy = (int)(dy / fontFactor);

			// Safety clamp for source coordinates
			if (sx >= srcW) { sx = srcW - 1; }
			if (sy >= srcH) { sy = srcH - 1; }

			// Read from RGBA Bitmap (4 bytes per pixel)
			int srcIdx = (sy * srcLineW + sx) * RTS.RTcolorDepth;

			// Normalize texture values to 0.0 - 1.0
			RTfloat texR = bitmap[srcIdx + 0] / 255.f;
			RTfloat texG = bitmap[srcIdx + 1] / 255.f;
			RTfloat texB = bitmap[srcIdx + 2] / 255.f;
			RTfloat texA = bitmap[srcIdx + 3] / 255.f;

			// Combine Texture Color with Input Tint Color
			// ResultAlpha = TextureAlpha * TintAlpha
			RTfloat finalAlpha = texA * color[3];

			// Optimization: Skip fully transparent pixels
			if (finalAlpha <= 0.001f) { continue; }

			// Calculate final RGB based on Texture * Tint
			RTVector4D finalColor;
			finalColor[0] = texR * color[0];
			finalColor[1] = texG * color[1];
			finalColor[2] = texB * color[2];

			// 5. Blend with Background
			Index iRGBA = pixelIdx * RTS.RTcolorDepth;

			// NOTE: We do NOT update zdepths[pixelIdx] here.
			// Since we validated the Z-position at the origin, we draw the 
			// whole character "flat" on top without modifying the depth buffer.
			// This prevents the character from occluding other geometry 
			// if the character rectangle is large.

			for (int c = 0; c < 3; ++c) // Update RGB channels
			{
				RTfloat existing = RTS.pixelsRGBA[iRGBA + c] / 255.f;
				// Standard Alpha Blending: src * alpha + dst * (1 - alpha)
				RTfloat newValue = finalColor[c] * finalAlpha + existing * (1.0f - finalAlpha);
				RTS.pixelsRGBA[iRGBA + c] = FloatColorToByte(newValue);
			}
		}
	}
}

//! draw a 0-terminated text string with fontSize, including monitor scaling factor; 
//! position p is in modelview coordinates, like in OpenGL DrawString!
//! offset is given relative to character width (X), height (Y) and scene Z-size (Z); offset is not rotated
void Raytracer::DrawString(const char* text, float fontSizeScaled, const RTVector3D& p, const RTVector3D& offset, Float4 color,
	bool transparent, bool alwaysInFront, bool isStaticObject)
{
	float fontScaling = fontSizeScaled / (float)bitmapFont.fontSize;
	fontScaling *= RTS.AAfactor / (float)RTS.imageSizeFactor;

	const RTfloat perspectiveFactor = isStaticObject ? 0 : RTS.perspectiveFactor;
	const RTfloat dist = (perspectiveFactor > perspectiveFactorLimit) ? (RTS.zoom / perspectiveFactor) : 0.0f;
	RTfloat scale = (perspectiveFactor <= perspectiveFactorLimit) ? 1 : dist / EXUstd::Maximum(dist - p[2], 1e-6f);
	fontScaling *= EXUstd::Minimum(20.f, scale);

	auto ModelViewToScreen = [&](const RTVector3D& p_world) -> RTVector3D {
		if (perspectiveFactor <= perspectiveFactorLimit) {
			//orthographic projection
			RTfloat sx = (p_world[0] / (RTS.zoom * RTS.ratio) + 1.f) * 0.5f * RTS.imageWidth;
			RTfloat sy = (p_world[1] / RTS.zoom + 1.f) * 0.5f * RTS.imageHeight;
			return RTVector3D({ sx, sy, p_world[2] });
		}
		else {
			//perspective projection: scale based on distance from eye (dist - z)
			//objects at z=0 have scale 1.0.
			RTfloat sx = (p_world[0] * scale / (RTS.zoom * RTS.ratio) + 1.f) * 0.5f * RTS.imageWidth;
			RTfloat sy = (p_world[1] * scale / RTS.zoom + 1.f) * 0.5f * RTS.imageHeight;
			return RTVector3D({ sx, sy, p_world[2] });
		}
	};

	RTVector3D posPixels = ModelViewToScreen(p);

	if (!transparent) //choose in which sense non-transparent characters are drawn
	{
		float avColor = (3.f) * (color[0] + color[1] + color[2]);
		if (avColor < 0.4) //very dark colors are not visible
		{
			color[0] = EXUstd::Minimum(1.f, color[0] + 1.f - avColor);
			color[1] = EXUstd::Minimum(1.f, color[1] + 1.f - avColor);
			color[2] = EXUstd::Minimum(1.f, color[2] + 1.f - avColor);
		}
	}

	//draw multi-line string character-by-character
	float w = (float)bitmapFont.characterWidth * fontScaling;
	float h = (float)bitmapFont.characterHeight * fontScaling;

	posPixels[0] += offset[0] * w;
	posPixels[1] += offset[1] * h;
	RTfloat zOffset = offset[2] * RTS.searchTree.GetBox().SizeZ();

	float colOff = 0;
	float lineOff = 0;

	guint updatedIndex = 0;
	gchar cUnicode = bitmapFont.GetUnicodeCharacterFromUTF8(text, updatedIndex);

	while (cUnicode != 0)
	{
		//use UTF-8 encoding: https://de.wikipedia.org/wiki/UTF-8
		//accept unicode characters: https://www.utf8-chartable.de/unicode-utf8-table.pl?unicodeinhtml=dec&htmlent=1
		if (cUnicode != (guint)'\n')
		{
			//std::cout << c << ":" << ci << "\n";
			if (cUnicode >= bitmapFont.characterOffset && cUnicode < bitmapFont.nCharacters + bitmapFont.characterOffset) //do not print control characters ...
			{
				//print character
				//std::cout << (char)cUnicode;
				RasterizeCharacter(cUnicode, Float3({ posPixels[0] + colOff, posPixels[1] + lineOff, p[2] + zOffset }), color, fontScaling, alwaysInFront);
			}
			colOff += w;
		}
		else
		{
			colOff = 0;
			lineOff -= h;
		}
		cUnicode = bitmapFont.GetUnicodeCharacterFromUTF8(text, updatedIndex);
	}
	//std::cout << "\n";

}


void Raytracer::CopyVisSettings2RTS(Index viewID)
{
	const VSettingsView& settingsView = GetSettingsView(viewID, *visSettings);

	//copy current material information from visSettings (change of material always updates visSettings!)
	basicVisualizationSystemContainer->CopyMaterialsFromVisualizationSettings();

	//Raytracing settings:
	RTS.maxReflectionDepth = EXUstd::Clamp(visSettings->raytracer.maxReflectionDepth, 0, 32);
	RTS.maxTransparencyDepth = EXUstd::Clamp(visSettings->raytracer.maxTransparencyDepth, 0, 32);
	RTS.keepWindowActive = visSettings->raytracer.keepWindowActive;
	RTS.backgroundColorReflections = ColorRGBA2RGB(visSettings->raytracer.advanced.backgroundColorReflections);
	RTS.ambientLightColor = ColorRGBA2RGB(visSettings->openGL.lightModelAmbient); //previously: raytracer.ambientLightColor);
	RTS.globalFogDensity = visSettings->raytracer.globalFogDensity;
	RTS.globalFogColor = ColorRGBA2RGB(visSettings->raytracer.globalFogColor);
	RTS.lightRadiusVariations = EXUstd::Clamp(visSettings->raytracer.lightRadiusVariations, 0, 256);
	RTS.shadowScalingFactor = EXUstd::Clamp(visSettings->raytracer.advanced.shadowScalingFactor, 0, 16);
	RTS.shadowSmoothingSteps = EXUstd::Clamp(visSettings->raytracer.advanced.shadowSmoothingSteps, 0, 32);

	RTS.verbose = visSettings->raytracer.verbose;
	RTS.numberOfThreads = EXUstd::Clamp(visSettings->raytracer.numberOfThreads, 1, RTS.maxNThreads);
	RTS.tilesPerThread = visSettings->raytracer.advanced.tilesPerThread;
	RTS.imageSizeFactor = EXUstd::Clamp(visSettings->raytracer.imageSizeFactor, 1, 16);
	RTS.zMaxSceneFactor = visSettings->openGL.zMaxSceneFactor;
	RTS.nearFarPlaneOffset = settingsView.camera.nearFarPlaneOffset;

	//RTS.zBiasLines //at raytracer

	//openGL settings:
	RTS.AAfactor = EXUstd::Clamp(visSettings->raytracer.multiSampling,1,4); // 2x supersampling; higher quality
	RTS.showLines = settingsView.scene.showLines || settingsView.scene.showFaceEdges || settingsView.scene.showMeshEdges;
	RTS.useClippingPlane =settingsView.camera.clippingPlaneNormal.GetL2NormSquared() != 0.f; //normal is set with model rotation!
	RTS.clippingPlaneDistance =settingsView.camera.clippingPlaneDistance;
	RTS.clippingPlaneColor = ColorRGBA2RGB(visSettings->openGL.advanced.clippingPlaneColor);
	RTS.useClippingPlaneColor = (Index)visSettings->openGL.advanced.clippingPlaneColor[3];
	RTS.globalMaxAlpha = settingsView.scene.facesTransparent ? visSettings->openGL.faceTransparencyGlobal : 1.f;

	//visibility settings:
	RTS.lineWidth = EXUstd::Clamp(visSettings->openGL.lineWidth,0.f,16.f);
	RTS.zBiasLines = visSettings->openGL.advanced.polygonOffset;
	RTS.perspectiveFactor = settingsView.camera.perspective;
	RTS.modelCentricView = settingsView.camera.modelCentricView;
	RTS.useGradientBackground = visSettings->general.useGradientBackground;

	RTS.backgroundColor = ColorRGBA2RGB(visSettings->general.backgroundColor);
	RTS.backgroundColorBottom = ColorRGBA2RGB(visSettings->general.backgroundColorBottom);

	for (Index lightID = 0; lightID < (Index)RTS.lights.size(); lightID++)
	{
		const VSettingsLight& settingsLight = GetSettingsLight(lightID, *visSettings);
		
		RTS.lights[lightID].active = settingsLight.enable;
		RTS.lights[lightID].shadow = settingsLight.shadow > 0;
		RTS.lights[lightID].diffuse = settingsLight.diffuse;
		RTS.lights[lightID].specular = settingsLight.specular;
		RTS.lights[lightID].constantAttenuation = settingsLight.constantAttenuation;
		RTS.lights[lightID].linearAttenuation = settingsLight.linearAttenuation;
		RTS.lights[lightID].quadraticAttenuation = settingsLight.quadraticAttenuation;
		RTS.lights[lightID].position = settingsLight.position;
		RTS.lights[lightID].lightRadius = settingsLight.lightRadius;
		RTS.lights[lightID].useCameraFrame = settingsLight.useCameraFrame;
	}

}

bool inSoftwareRenderer = false;
//! software renderer used instead of OpenGL renderer
void Raytracer::SoftwareRenderer(Index viewID, VisualizationSystemContainerBase* basicVisualizationSystemContainerInit, 
	const RenderStateMachine& stateMachine, bool storeImage, bool isCalledFromMainThread)
{
	if (inSoftwareRenderer) { std::cout << "ALREADY IN SoftwareRenderer!!!!!"; return; }
	inSoftwareRenderer = true;

	Real tStart = EXUstd::GetTimeInSeconds();

	basicVisualizationSystemContainer = basicVisualizationSystemContainerInit;
	visSettings = &basicVisualizationSystemContainer->GetVisualizationSettings();

	renderState = &basicVisualizationSystemContainer->GetRenderState(viewID);
	const VSettingsView& settingsView = GetSettingsView(viewID, *visSettings); //requires visSettings!

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//INIT
	//do this before CopyVisSettings2RTS!
	RTS.materials = basicVisualizationSystemContainer->GetGraphicsMaterialList();

	//RaytracingSettings RTS initialized at very start!
	CopyVisSettings2RTS(viewID);
	//we have to sync the size; but use renderState size if size is changed in settings!
	
	if (!PyIsRendererActive(false))
	{
		//active renderer: we use renderState, as user may have resized window; 
		//offline raytracer: use visSettings, as renderState did not get any update
		renderState->currentWindowSize[0] = GetSettingsView(viewID, *visSettings).window.renderWindowSize[0];
		renderState->currentWindowSize[1] = GetSettingsView(viewID, *visSettings).window.renderWindowSize[1];
	}
	
	RTS.isCalledFromMainThread = isCalledFromMainThread;

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//copy renderState settings
	RTS.displayScaling = renderState->displayScaling;
	RTS.zoom = EXUstd::Maximum(renderState->zoom, 1e-7f); //avoid division by zero!
	Index width = renderState->currentWindowSize[0];
	Index height = renderState->currentWindowSize[1];

	if (height == 0) { height = 1; }
	if (width == 0) { width = 1; }
	RTS.ratio = width / (RTfloat)height; //cannot be zero or div by zero!

	//RTS.modelViewRM = renderState->modelRotation.GetTransposed(); //DOES NOT INCLUDE trackMarker; row-major
	renderState->GetRotationTranslationFWithMarker(RTS.rotationView, RTS.translationView,
		basicVisualizationSystemContainer, settingsView, false);

	HomogeneousTransformationF modelView(RTS.rotationView, -RTS.translationView); //neg. translation to compensate for centerPoint shift
	RTS.modelViewRM = modelView.GetHT44();
	RTS.projectionNearMin = renderState->GetProjectionNearMin();

	RTfloat translationViewZ = RTS.translationView[2]; //used to correct Z-offset in static objects

	//std::cout << "MV=" << RTS.modelViewRM << "\n";
	//std::cout << "RV=" << RTS.rotationView << "\n";
	//std::cout << "TV=" << RTS.translationView << "\n";

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++
	//setup image size!
	RTS.imageWidth = EXUstd::Maximum(4,(RTS.AAfactor*width) / RTS.imageSizeFactor);
	RTS.imageHeight = EXUstd::Maximum(4, (RTS.AAfactor*height) / RTS.imageSizeFactor);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++
	//initialize texts if they do not exist
	if (fontBitmaps.NumberOfItems() == 0)
	{
		fontBitmaps.SetNumberOfItems(bitmapFont.nCharacters);
		for (unsigned int iChar = 0; iChar < bitmapFont.nCharacters; iChar++)
		{
			//these are the unscaled (high resolution) bitmaps (=textures) for each character:
			fontBitmaps[iChar] = bitmapFont.GetRGBAFontCharacter(iChar, true);
		}
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++
	//setup triangles and lines to draw
	graphicsData.FlushData();

	//this could be parallelized:
	//transform points and normals and compute bounding box:
	Box3DBase<RTfloat> box3D;
	Index cntWrongNormals = 0;
	static bool warnedWrongNormals = false;
	for (auto data : basicVisualizationSystemContainer->GetGraphicsDataList())
	{
		for (const GLTriangle& trig : data->glTriangles)
		{
			GLTriangle trigNew(trig);

			for (Index i = 0; i < 3; i++)
			{
				TransformVertexRM(trig.points[i], RTS.modelViewRM, trigNew.points[i]);
				trigNew.normals[i] = RTS.rotationView * trig.normals[i];
				CHECKandTHROW(!trigNew.points[i].HasInvalid(), "SoftwareRenderer: triangle points contain invalid (nan) coordinates - check your geometry!");
				box3D.Add(trigNew.points[i]);
			}
			Float3 n = EGeometry::ComputeTriangleNormalTemplate<RTfloat, std::array<Float3, 3>>(trig.points);
			if (n * trig.normals[0] < 0.) { cntWrongNormals++; }

			if ((!trig.isFiniteElement && settingsView.scene.showFaces) ||
				(trig.isFiniteElement && settingsView.scene.showMeshFaces))
			{
				graphicsData.glTriangles.Append(trigNew);
			}
			if (settingsView.scene.showFaceEdges || settingsView.scene.showMeshEdges)
			{
				if ((trigNew.isFiniteElement && settingsView.scene.showMeshEdges)
					|| (!trigNew.isFiniteElement && settingsView.scene.showFaceEdges))
				{
					for (Index i = 0; i < 3; i++)
					{
						GLLine lineNew;
						lineNew.color1 = visSettings->openGL.faceEdgesColor;
						lineNew.color2 = visSettings->openGL.faceEdgesColor;
						lineNew.point1 = trigNew.points[i];
						lineNew.point2 = trigNew.points[(i + 1) % 3];
						graphicsData.glLines.Append(lineNew);
					}
				}
			}
		}
		if (RTS.showLines)
		{
			for (const GLLine& line : data->glLines)
			{
				GLLine lineNew(line);
				TransformVertexRM(line.point1, RTS.modelViewRM, lineNew.point1);
				TransformVertexRM(line.point2, RTS.modelViewRM, lineNew.point2);
				graphicsData.glLines.Append(lineNew);
			}
		}
	}
	if (cntWrongNormals && !warnedWrongNormals && RTS.verbose)
	{
		warnedWrongNormals = true;
		PrintDelayed("********\nWARNING: Raytracer: There are "+EXUstd::ToString(cntWrongNormals)+" wrong normals in the geometry\n********");
	}

	//std::cout << "#trigs=" << graphicsData.glTriangles.NumberOfItems() << "\n";

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//add static overlay (status, Exudyn, coordinate system, contour bar, ...)
	Index nTrigsDynamic = graphicsData.glTriangles.NumberOfItems();
	GetGraphicsDataStatic(viewID, basicVisualizationSystemContainer, graphicsData, stateMachine, false);

	float offsetZ = EXUstd::Maximum(box3D.PMaxZ(), 0.f);
	for (Index j = nTrigsDynamic; j < graphicsData.glTriangles.NumberOfItems(); j++)
	{
		GLTriangle& trig = graphicsData.glTriangles[j];
		for (Index i = 0; i < 3; i++)
		{
			if (trig.itemID == itemIDstaticObjectWithoutZoff)
			{
				//should be always zero, except for trackMarker
				trig.points[i][2] -= translationViewZ; //correct Z, as we compute Z for static objects due to transformation ...
			}
			else if (RTS.perspectiveFactor == 0)
			{
				trig.points[i][2] += offsetZ * 0.999f; //we must bring these objects to the front
			}
			box3D.Add(trig.points[i]);
		}
	}
	//offsetZ = EXUstd::Maximum(box3D.PMaxZ(), 0.f); //update, if needed later

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	if (!box3D.Empty() || storeImage) //may be empty at beginning!
	{
		//std::cout << "box3D.Empty()=" << box3D.Empty() << "\n";

		if (box3D.Empty()) { box3D.Add(RTVector3D({ 0,0,0 }) ); } //add a single point
		if (box3D.SizeX() * box3D.SizeY() * box3D.SizeZ() == 0.f) { box3D.Increase(1.f); }

		box3D.InflateFactor(1.f + RTS.epsilon * 10); //inflate to avoid strange boundary effects

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// some settings which require bounding box or transformation:
		//anyway, we have some offset in the origin!
		RTS.searchTreeBoxMaxRadius = box3D.Radius(); //searchTree not initialized here!
		//RTS.maxSceneSize = EXUstd::Maximum((box3D.PMax() - box3D.PMin())[2], visSettings->general.minSceneSize); //makes problems with long cylinders starting at [0,0,0] in x-direction
		RTS.maxSceneSize = EXUstd::Maximum(2.f*(RTS.searchTreeBoxMaxRadius+box3D.Center().GetL2Norm()), visSettings->general.minSceneSize); //modified to include center displacement!

		RTS.zBiasLines = (RTfloat)visSettings->raytracer.advanced.zBiasLines * 2.f * RTS.searchTreeBoxMaxRadius; //relative to sceen size
		RTS.tolParallel = RTS.epsilon * 2.f * RTS.searchTreeBoxMaxRadius; //fixed for now
		RTS.tolParallel2 = RTS.tolParallel * RTS.tolParallel * 1e-4f; //factor 1e-4 due to small triangles in spheres!
		//avoid this, as the user may get confused by that: box3D.InflateFactor(1.002f);

		RTS.clippingPlaneNormal = GetSettingsView(viewID, *visSettings).camera.clippingPlaneNormal;
		RTS.clippingPlaneNormal.NormalizeSafe();
		RTS.clippingPlanePoint = RTS.clippingPlaneDistance * RTS.clippingPlaneNormal;
		RTS.clippingPlaneNormal = RTS.rotationView * RTS.clippingPlaneNormal;
		TransformVertexRM(RTS.clippingPlanePoint, RTS.modelViewRM, RTS.clippingPlanePoint);
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



		Index factST = EXUstd::Clamp(visSettings->raytracer.advanced.searchTreeFactor, 1, 128); //additional factor for testing ...
		Index sx, sy, sz;
		SearchTreeBase<RTfloat>::ComputeSearchTreeBinCounts(graphicsData.glTriangles.NumberOfItems()*factST, box3D, sx, sy, sz);

		//std::cout << "sizes=" << sx << "," << sy << "," << sz << "\n";

		RTS.searchTree.ResetSearchTree(2*sx,2*sy,sz, box3D);
		//RTS.hasSearchTree = true;

		for (Index i = 0; i < graphicsData.glTriangles.NumberOfItems(); ++i)
		{
			const GLTriangle& triangle = graphicsData.glTriangles[i];
			Box3DBase<RTfloat> tBox;
			for (Index i = 0; i < 3; i++) { tBox.Add(triangle.points[i]); }

			RTS.searchTree.AddItemTriangle(tBox, i, triangle.points[0], triangle.points[1], triangle.points[2]);
		}

		//std::cout << "searchTree box=" << RTS.searchTree.GetBox() << "\n";
		//std::cout << "box3D=" << box3D << "\n";
		//std::cout << "W*H*D=" << RTS.imageWidth << "," << RTS.imageHeight << "," << RTS.RTcolorDepth << "\n";

		RTS.pixelsRGBA.SetNumberOfItems(RTS.imageWidth * RTS.imageHeight * RTS.RTcolorDepth);
		RTS.pixelsRGBA.SetAll(0);
		//NOTE: zDepths are used to add lines accordingly (without fogs, etc.); 
		//      however, zDepths are measured from model-perspective; +Z is in direction pointing to eye/camera
		RTS.zDepths.SetNumberOfItems(RTS.imageWidth * RTS.imageHeight);
		RTS.zDepths.SetAll(0);

		RTVector3D rayDirection({ 0,0,-1 }); //general ray direction; in unrotated config, z-axis points to viewer
		RTfloat zOrigin = RTS.searchTree.GetBox().PMaxZ(); //always outside of box

		//RTfloat zSize = RTS.searchTree.GetBox().SizeZ();   
		//!convert 
		auto ToModelView = [&](Index x, Index y, Index width, Index height) -> RTVector3D {
			RTfloat fx = static_cast<RTfloat>(x);
			RTfloat fy = static_cast<RTfloat>(y);

			RTfloat screenX = ((2.f * fx + 1.f) / width - 1.f) * RTS.zoom * RTS.ratio;
			RTfloat screenY = ((2.f * fy + 1.f) / height - 1.f) * RTS.zoom;

			return RTVector3D({ screenX, screenY, zOrigin });
			};
		auto ToPerspectiveRay = [&](Index x, Index y, Index width, Index height, RTfloat perspectiveFactor,
			RTfloat maxSceneSize, RTfloat zoom, RTfloat zMaxSceneFactor) -> std::pair<RTVector3D, RTVector3D>
			{
				// get the screen-space coordinate at z=0 (the 'ortho' position); x/y in pixels, pointOnPlane in screen coords
				RTVector3D pointOnPlane = ToModelView(x, y, width, height);

				if (perspectiveFactor <= perspectiveFactorLimit) {
					// ORTHOGRAPHIC projection
					RTVector3D rayOrigin = pointOnPlane;
					// Start ray far enough back to see the whole scene
					rayOrigin[2] = zMaxSceneFactor * maxSceneSize;
					RTVector3D rayDir = { 0.0f, 0.0f, -1.0f };
					return { rayOrigin, rayDir };
				}
				else {
					// PERSPECTIVE projection
					RTVector3D rayOrigin;
					RTVector3D rayDir;
					RTfloat distance = EXUstd::Maximum(1e-6f, zoom / perspectiveFactor); //distance may not be zero to makeup a meaningful rayDir

					if (settingsView.camera.modelCentricView) 
					{
						// physical Zoom: move the eye back, look at the focal plane (Z=0)
						rayOrigin = { 0.0f, 0.0f, distance };

						pointOnPlane[2] = 0.0f; // target the focal plane
						rayDir = pointOnPlane - rayOrigin;
					}
					else 
					{
						// Lens Zoom: Eye stays at origin, look through the "zoom" window
						// In OpenGL camera-centric, the camera is at (0,0,0)
						rayOrigin = { 0.0f, 0.0f, 0.0f };

						// To match 'scale = zNear * perspective', the 'window' height 
						// pointOnPlane currently gives us the ortho-coords (-zoom to +zoom).
						RTfloat zNearPlane = RTS.projectionNearMin;
						if (RTS.nearFarPlaneOffset[2] == 1.f) 
						{ 
							zNearPlane = EXUstd::Maximum(RTS.nearFarPlaneOffset[0], RTS.projectionNearMin);
						}
						RTfloat scale = zNearPlane * perspectiveFactor;

						//compute ray dir directly
						rayDir = {
							pointOnPlane[0] * scale, //as rayDir is normalized, zNearPlane cancels out!!!
							pointOnPlane[1] * scale,
							-zNearPlane //plane distance
						};

						//rayDir = directionTarget - rayOrigin;
					}

					rayDir.NormalizeSafe();
					return { rayOrigin, rayDir };
				}
			};


		//DELETE: //provisional computation of rayOrigin[2]:
		//auto [rayOrigin, rayDir] = ToPerspectiveRay(0, 0, RTS.shadowWidth, RTS.shadowHeight, RTS.perspectiveFactor,
		//	RTS.maxSceneSize,
		//	RTS.zoom, RTS.zMaxSceneFactor);
		
		//used lateron for RasterizeLine:
		//NOTE: nearPlane is defined from perspective of camera (goes [0,0,-1]), however
		//      zDepths are given from perspective of model (opposite direction, +Z!)
		RTfloat nearPlaneZ = std::numeric_limits<RTfloat>::max(); //used for cutting view at screen
		if (RTS.perspectiveFactor > perspectiveFactorLimit) { nearPlaneZ = -RTS.projectionNearMin; } //negative for RasterizeLines
		if (RTS.nearFarPlaneOffset[2] == 1.f) { nearPlaneZ = -RTS.nearFarPlaneOffset[0]; } //negative nearPlane gives larger zDepth! 

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if (false)
		{
			Index numberOfTreeItems;
			float averageFill;
			Index numberOfZeros;
			Index maxFill;
			Index numberOf10average;
			RTS.searchTree.GetStatistics(numberOfTreeItems, averageFill, numberOfZeros, maxFill, numberOf10average);

			std::cout << "statistics:" << numberOfTreeItems << ", " << averageFill << ", " << numberOfZeros << ", " << maxFill << ", " << numberOf10average << "\n";
		}

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		RTS.numberOfThreadsUsed = 1;

		if (ExuThreading::TaskManager::IsRunning()) //should not happen; only if FinalizeSolver has not been called in last computation
		{
			if (RTS.verbose) { PrintDelayed("Raytracer: multi-threading: TaskManager already/still running (in solver?); do not use multithreading for solver"); }
			RTS.numberOfThreads = 1;
		}
		RTS.numberOfThreadsUsed = RTS.numberOfThreads;
		if (RTS.numberOfThreads > 1)
		{
			ExuThreading::TaskManager::SetNumThreads(RTS.numberOfThreads);

			ExuThreading::EnterTaskManager(); //this is needed in order that any ParallelFor is executed in parallel during solving

			RTS.numberOfThreadsUsed = ExuThreading::TaskManager::GetNumThreads();

			if (RTS.numberOfThreadsUsed == 1) //this reflects that taskmanager is not running!
			{
				if (RTS.verbose) { PrintDelayed("Raytracer: multi-threading failed"); }
			}
			else
			{
				if (RTS.verbose) { PrintDelayed("Raytracer: Start multi-threading with " +EXUstd::ToString(RTS.numberOfThreadsUsed)+ " threads"); }
			}
		}

		Real tStartRT = EXUstd::GetTimeInSeconds();

		Index nBoxes = RTS.numberOfThreadsUsed*RTS.tilesPerThread;

		Index taskSplit = nBoxes; //RTS.imageHeight / 10;
		//determine tiling in 2D: find grid dimensions close to square
		Index tilesX = EXUstd::Maximum(1, (Index)std::sqrt(nBoxes));
		while (nBoxes % tilesX != 0) { --tilesX; }
		//tilesX = 1;
		Index tilesY = EXUstd::Maximum(nBoxes / tilesX, 1);

		//Real complexity = ((Real)RTS.imageHeight * (Real)RTS.imageWidth * (Real)graphicsData.glTriangles.NumberOfItems()) / 
		//	EXUstd::Maximum(1., (Real)RTS.searchTree.NumberOfCells()* sqrt((Real)RTS.numberOfThreadsUsed) );
		//if (RTS.verbose) { PrintDelayed("complexity: " + EXUstd::ToString(complexity)); }

		//shadowMode: 0=normal, 1=precompute shadow (returns shadow in RTVector3D[0]), 2=use computed shadow
		RTS.preComputeShadow = RTS.lightRadiusVariations > 1 && (RTS.shadowScalingFactor * RTS.AAfactor) > 1; //if factor is <=1, precomputation makes no sense!

		if (RTS.preComputeShadow)
		{
			Index factorScaling = RTS.shadowScalingFactor * RTS.AAfactor;
			RTS.shadowWidth = RTS.imageWidth / (factorScaling);
			RTS.shadowHeight = RTS.imageHeight / (factorScaling);

			RTS.shadowValues.SetNumberOfItems(RTS.shadowWidth * RTS.shadowHeight);
			RTS.shadowValues.SetAll({ 1.f, 1.f, 1.f, 1.f});

			//! precompute shadows:
			ExuThreading::ParallelFor(nBoxes, [this, &ToPerspectiveRay,
				&tilesX, &tilesY, &tStartRT, &nBoxes](ParallelSizeType index)
				{
					Index tileWidth = RTS.shadowWidth / tilesX;
					Index tileHeight = RTS.shadowHeight / tilesY;

					//Index threadID = RTS.numberOfThreads == 1 ? 0 : ExuThreading::TaskManager::GetThreadId();
					Index tx = (Index)index % tilesX;
					Index ty = (Index)index / tilesX;

					// Compute tile bounds
					Index xStart = tx * tileWidth;
					Index xEnd = (tx == tilesX - 1) ? RTS.shadowWidth : xStart + tileWidth;

					Index yStart = ty * tileHeight;
					Index yEnd = (ty == tilesY - 1) ? RTS.shadowHeight : yStart + tileHeight;

					RTfloat zDepthDummy;
					const Index computeShadow = 1;
					for (Index y = yStart; y < yEnd; ++y)
					{
						for (Index x = xStart; x < xEnd; ++x)
						{
							RTVector3D rayOrigin0 = RTVector3D({ (2.f * x + 1.f) / RTS.shadowWidth - 1.f,
								((2.f * y + 1.f) / RTS.shadowHeight - 1.f), 0.f });

							Index iPix = (y * RTS.shadowWidth + x);
							auto [rayOrigin, rayDir] = ToPerspectiveRay(x, y, RTS.shadowWidth, RTS.shadowHeight, RTS.perspectiveFactor,
								RTS.maxSceneSize,//RTS.searchTree.GetBox().SizeZ(), 
								RTS.zoom, RTS.zMaxSceneFactor);

							//in this mode, color[3] becomes the shadow value
							ComputePixelColor(rayOrigin, rayDir, graphicsData.glTriangles, RTS, zDepthDummy, RTS.shadowValues[iPix], 0, 0, computeShadow, rayOrigin0);
						}
					}
				}, taskSplit);
			for (Index i = 0; i < RTS.shadowSmoothingSteps; i++)
			{
				SmoothImageValues(RTS.shadowValues, RTS.smoothShadowValues, RTS.shadowWidth, RTS.shadowHeight, RTS);
				RTS.shadowValues = RTS.smoothShadowValues;
			}
			if (RTS.shadowSmoothingSteps < 1) { RTS.smoothShadowValues = RTS.shadowValues; }

			ScaleImageValues(RTS.smoothShadowValues, RTS.shadowValues, RTS.shadowWidth, RTS.shadowHeight,
				RTS.imageWidth, RTS.imageHeight);

			for (Index i = 0; i < RTS.shadowScalingFactor; i++) //RTS.shadowScalingFactor*2 looks better for larger shadowScalingFactor!
			{
				SmoothImageValues(RTS.shadowValues, RTS.smoothShadowValues, RTS.imageWidth, RTS.imageHeight, RTS, false);
				RTS.shadowValues = RTS.smoothShadowValues;
			}
		}

		//! final rendering:
		//std::cout << "final rendering\n";
		//std::cout << "#trigs=" << graphicsData.glTriangles.NumberOfItems() << "\n";
		//std::cout << "RTS.preComputeShadow=" << RTS.preComputeShadow << "\n";

		ExuThreading::ParallelFor(nBoxes, [this, &ToPerspectiveRay,
			&tilesX, &tilesY, &tStartRT, &nBoxes](ParallelSizeType index)
			{
				Index threadID = RTS.numberOfThreads == 1 ? 0 : ExuThreading::TaskManager::GetThreadId();
				if (threadID == 0) //main thread
				{
					if (EXUstd::GetTimeInSeconds() - tStartRT > 0.5)
					{
						if (RTS.keepWindowActive && ! RTS.isCalledFromMainThread)
						{
							GlfwPollEvents(); //avoid strange timeouts by catching glfw window callbacks in main thread
						}
						if (RTS.verbose) //main thread
						{
							PrintDelayed("\rrendering: " + EXUstd::ToString(std::round((Real)index / (Real)nBoxes * 100.)) + "%", false, true);
						}
					}
				}
				Index tileWidth = RTS.imageWidth / tilesX;
				Index tileHeight = RTS.imageHeight / tilesY;

				Index tx = (Index)index % tilesX;
				Index ty = (Index)index / tilesX;

				// Compute tile bounds
				Index xStart = tx * tileWidth;
				Index xEnd = (tx == tilesX - 1) ? RTS.imageWidth : xStart + tileWidth;

				Index yStart = ty * tileHeight;
				Index yEnd = (ty == tilesY - 1) ? RTS.imageHeight : yStart + tileHeight;

				//Real val = 0;
				RTVector3D color;
				ShadowValue shadowValue = { 1.f, 1.f, 1.f, 1.f};
				Index shadowMode = RTS.preComputeShadow ? 2 : 0;
				for (Index y = yStart; y < yEnd; ++y)
				{
					for (Index x = xStart; x < xEnd; ++x)
					{
						RTVector3D rayOrigin0 = RTVector3D({ ((2.f * x + 1.f) / RTS.imageWidth - 1.f),
							((2.f * y + 1.f) / RTS.imageHeight - 1.f), 0.f });

						Index iPix = (y * RTS.imageWidth + x);
						Index i = iPix * RTS.RTcolorDepth;
						if (RTS.preComputeShadow)
						{
							//Index iPixShadow = ((y / (RTS.shadowScalingFactor * RTS.AAfactor)) * RTS.shadowWidth + x / (RTS.shadowScalingFactor * RTS.AAfactor));
							//shadowValue = RTS.shadowValues[iPixShadow];
							shadowValue = RTS.shadowValues[iPix];
						}
						auto [rayOrigin, rayDir] = ToPerspectiveRay(x, y, RTS.imageWidth, RTS.imageHeight, RTS.perspectiveFactor,
							RTS.maxSceneSize, RTS.zoom, RTS.zMaxSceneFactor);

						color = ComputePixelColor(rayOrigin, rayDir, graphicsData.glTriangles, RTS, RTS.zDepths[iPix], shadowValue, 0, 0, shadowMode, rayOrigin0);

						for (Index j = 0; j < 3; j++)
						{
							RTS.pixelsRGBA[i + j] = FloatColorToByte(color[j]);
						}
						RTS.pixelsRGBA[i + 3] = 255;
					}
				}
			}, taskSplit);

		if (RTS.numberOfThreadsUsed != 1 && //do not stop threads, if running in solver thread!
			ExuThreading::TaskManager::IsRunning()) 
		{
			if (RTS.verbose) { PrintDelayed("Stop multi-threading"); }
			ExuThreading::ExitTaskManager(1); // output.numberOfThreadsUsed);
			ExuThreading::TaskManager::SetNumThreads(1); //for next computation, if it is going to be serial
			RTS.numberOfThreadsUsed = 1;
		}


		//line overlay:
		if (RTS.showLines) //otherwise, requires to adapt line drawing
		{
			for (const auto& line : graphicsData.glLines)
			{
				RTVector3D p0 = line.point1;
				RTVector3D p1 = line.point2;
				Float4 color0 = line.color1;
				Float4 color1 = line.color2;

				bool isStaticObject = line.itemID == itemIDstaticObject;
				//if (RTS.perspectiveFactor == 0 || isStaticObject)
				{
					RasterizeLine(p0, p1, color0, color1, RTS.zDepths, RTS.pixelsRGBA, RTS, isStaticObject, nearPlaneZ);
				}
			}
		}

		if (visSettings->raytracer.advanced.showText)
		{
			//RTfloat scaleFactor = 2.f * RTS.zoom / (float)RTS.imageHeight; //height cannot be zero
			RTfloat zOffset = visSettings->general.textAlwaysInFront ? 0 : visSettings->general.textOffsetFactor;
			float globalFontSize = GetSettingsView(viewID, *visSettings).window.globalFontSize;
			bool alwaysInFront = true;
			RTVector3D pos;
			for (auto data : basicVisualizationSystemContainer->GetGraphicsDataList())
			{
				for (const GLText& text : data->glTexts)
				{
					TransformVertexRM(text.point, RTS.modelViewRM, pos);
					float fontSizeScaled = text.fontSize ? text.fontSize * RTS.displayScaling : globalFontSize * RTS.displayScaling;

					DrawString(text.text, fontSizeScaled,
						pos, RTVector3D({ text.offsetX , text.offsetY, zOffset }), text.color,
						!visSettings->general.textHasBackground, visSettings->general.textAlwaysInFront);
				}
			}
			for (const GLText& text : graphicsData.glTexts)
			{
				pos = text.point;
				alwaysInFront = visSettings->general.textAlwaysInFront || (text.itemID != itemIDstaticObject);
				if (text.itemID != itemIDstaticObject)
				{
					TransformVertexRM(text.point, RTS.modelViewRM, pos);
				}
				float fontSizeScaled = text.fontSize ? text.fontSize * RTS.displayScaling : globalFontSize * RTS.displayScaling;

				DrawString(text.text, fontSizeScaled,
					pos, RTVector3D({ text.offsetX , text.offsetY, zOffset }), text.color,
					!visSettings->general.textHasBackground, alwaysInFront, text.itemID == itemIDstaticObject);
			}
			//DrawString("This is a Demo 123", 12 * RTS.displayScaling, Float3({ 0,0,1 }), RTVector3D(0), Float4({ 0,0,0,1 }), true, true);
		}


		//std::cout << "lines=" << cntLines << "\n";
		Real tEnd = EXUstd::GetTimeInSeconds();
		if (RTS.verbose) 
		{
			STDstring s = "";
			s = "raytracer: timing=" + EXUstd::ToString(tEnd - tStartRT);
			if (RTS.verbose > 1)
			{
				PrintDelayed(", nTrigs" + EXUstd::ToString(graphicsData.glTriangles.NumberOfItems())
					+ ", sTree=" + EXUstd::ToString(RTS.searchTree.NumberOfCellsXYZ())
					+ ", sBox=" + EXUstd::ToString(RTS.searchTree.GetBox())
				);
			}
			PrintDelayed(s);
		}
		if (rendererDebug)
		{ 
			//std::cout << "nTrigs" << graphicsData.glTriangles.NumberOfItems() << ", box3D=" << box3D << "\n";
			PrintDelayed(" tPre=" + EXUstd::ToString(tStartRT-tStart) 
				+ "  nRT=" + EXUstd::ToString(nHits / 1e6) + "M, nRT/s=" 
				+ EXUstd::ToString(nHits / 1e6 / EXUstd::Maximum(1e-6, tEnd - tStartRT)) 
				+ "M/s");

			//without searchTree, 1 thread: 100M/S; with searchTree: 57M/s
			nHits = 0;
		}

		Index finalImageWidth = 0;
		Index finalImageHeight = 0;
		ResizableArray<unsigned char>* finalImageData = NULL;

		if (RTS.AAfactor >= 1) //multisampling requires to scale final image
		{
			Index aaImageWidth = RTS.imageWidth / RTS.AAfactor;
			Index aaImageHeight = RTS.imageHeight / RTS.AAfactor;
			RTS.finalImageAA.SetNumberOfItems(aaImageWidth * aaImageHeight * RTS.RTcolorDepth);
			RTfloat sFactor = (RTfloat)(RTS.AAfactor * RTS.AAfactor)* 255.f;

			const float kernel[3][3] = {
					{ 1.f, 2.f, 1.f },
					{ 2.f, 4.f, 2.f },
					{ 1.f, 2.f, 1.f }
				};
			const float kernelSum = 16.f;
			if (RTS.AAfactor == 3) { sFactor = kernelSum * 255.f; }

			for (Index y = 0; y < aaImageHeight; ++y)
			{
				for (Index x = 0; x < aaImageWidth; ++x)
				{
					Index loIdx = (y * aaImageWidth + x) * RTS.RTcolorDepth;
					RTfloat r = 0, g = 0, b = 0;
					for (Index dy = 0; dy < (Index)RTS.AAfactor; ++dy)
					{
						for (Index dx = 0; dx < (Index)RTS.AAfactor; ++dx)
						{
							Index sx = x * (Index)RTS.AAfactor + dx;
							Index sy = y * (Index)RTS.AAfactor + dy;
							Index hiIdx = (sy * RTS.imageWidth + sx) * RTS.RTcolorDepth;
							RTfloat weight = 1.f;
							if (RTS.AAfactor == 3) { weight = kernel[dy][dx]; }
							r += weight * (int)(RTS.pixelsRGBA[hiIdx + 0]);
							g += weight * (int)(RTS.pixelsRGBA[hiIdx + 1]);
							b += weight * (int)(RTS.pixelsRGBA[hiIdx + 2]);
						}
					}
					
					RTS.finalImageAA[loIdx + 0] = FloatColorToByte(r / sFactor);
					RTS.finalImageAA[loIdx + 1] = FloatColorToByte(g / sFactor);
					RTS.finalImageAA[loIdx + 2] = FloatColorToByte(b / sFactor);
					RTS.finalImageAA[loIdx + 3] = 255;
				}
			}
			DrawImageRGBA(RTS.finalImageAA, aaImageWidth, aaImageHeight, width, height);
			finalImageWidth = aaImageWidth;
			finalImageHeight = aaImageHeight;
			finalImageData = &RTS.finalImageAA;
		}
		else
		{
			DrawImageRGBA(RTS.pixelsRGBA, RTS.imageWidth, RTS.imageHeight, width, height);
			finalImageWidth = RTS.imageWidth;
			finalImageHeight = RTS.imageHeight;
			finalImageData = &RTS.pixelsRGBA;
		}

		if (storeImage)
		{
			//copy image into according buffer
			const Index nrChannels = 3; //for image output, only RGB

			SlimVectorBase<Index, 2>* imageSize = basicVisualizationSystemContainer->GetImageDataSize(viewID);
			(*imageSize)[0] = finalImageWidth; //only needed for SaveImageAsData
			(*imageSize)[1] = finalImageHeight;

			ResizableArray<uint8_t>* pixelImageStore = basicVisualizationSystemContainer->ImageData(viewID);
			pixelImageStore->SetNumberOfItems(finalImageWidth* finalImageHeight* nrChannels);

			for (Index i = 0; i < finalImageHeight; i++)
			{
				for (Index j = 0; j < finalImageWidth; j++)
				{
					for (Index k = 0; k < nrChannels; k++)
					{
						(*pixelImageStore)[(finalImageHeight - i - 1) * finalImageWidth * nrChannels + j * nrChannels + k] =
							(uint8_t)(*finalImageData)[i * finalImageWidth * RTS.RTcolorDepth + j * RTS.RTcolorDepth + k];
					}
				}
			}
		}
	}
	inSoftwareRenderer = false;

}




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "System/versionCpp.h"

//! write static overlay text and objects for render view
//! this is used in GlfwClient and in Raytracer and does not need GLFWClient
//! zOffset is added in GlfwClient such that it is put on top!
void GetGraphicsDataStatic(Index viewID, VisualizationSystemContainerBase* basicVisualizationSystemContainer,
	GraphicsData& graphicsData, const RenderStateMachine& stateMachine, bool flushData)
{
	VisualizationSettings* visSettings = &basicVisualizationSystemContainer->GetVisualizationSettings();
	RenderState* state = &basicVisualizationSystemContainer->GetRenderState(viewID);
	const VSettingsView& settingsView = GetSettingsView(viewID, *visSettings);

	if (flushData) { graphicsData.FlushData(); } //erase old data

	Matrix3DF rotationMV;
	Float3 translationMV;
	//get modelview transformation for this renderState
	//DELETE: state->GetRotationTranslationF(rotationMV, translationMV, false); //false=row-major
	state->GetRotationTranslationFWithMarker(rotationMV, translationMV,
		basicVisualizationSystemContainer, settingsView, false);

	Index drawCoordinateSystem = settingsView.scene.drawCoordinateSystem;

	Index itemID = itemIDstaticObject; //for raytracer, static objects are not transformed and always on top!
	float fontSize = settingsView.window.globalFontSize;
	float fontSizeScaled = fontSize * state->displayScaling; //for dimensions relative to font size; fontSize is scaled in DrawString functions later on!
	
	int width = state->currentWindowSize[0];
	int height = state->currentWindowSize[1];
	float zoom = state->zoom;

	const int textIndentPixels = 10; //left indentation for text
	const int textBottomPixels = 6; //bottom indentation for text
	Float4 textColor = visSettings->general.textColor;

	Index computationMessageNumberOfLines = 0; //compute offset for contour plot

	Index precision = EXUstd::Maximum(0, EXUstd::Minimum(visSettings->general.rendererPrecision-1, 16)); //reduce precision for smaller sceens

	// lambda function to convert float to string using captured precision and internal buffer
	auto F2Str = [&](double val) {
		const Index nCharMax = 24;
		char buf[nCharMax];
		snprintf(buf, nCharMax, "%7.*g", precision, val);
		return STDstring(buf);
		};

	//compute common coordinates in MODELVIEW:
	Float3 pTopLeft3D = basicVisualizationSystemContainer->PixelsToModelViewCoordinates((float)textIndentPixels,
		(float)(height), width, height, zoom); //fixed position, very top left window position
	Float3 pBottomRight3D = basicVisualizationSystemContainer->PixelsToModelViewCoordinates((float)width,
		(float)textBottomPixels, width, height, zoom); //fixed position, very bottom right window position
	Float3 pBottomLeft3D({ pTopLeft3D[0], pBottomRight3D[1], 0 });

	Float2 onePixel = basicVisualizationSystemContainer->PixelsToModelViewCoordinates2D(1, 1, width, height, zoom) -
		basicVisualizationSystemContainer->PixelsToModelViewCoordinates2D(0, 0, width, height, zoom);

	if (settingsView.window.showComputationInfo)
	{
		graphicsData.AddText(pTopLeft3D, textColor, "EXUDYN", fontSize * fontLargeFactor, 0, -0.95f, itemID);

		std::string message = basicVisualizationSystemContainer->GetComputationMessage(visSettings->general.showSolverInformation,
			visSettings->general.showSolutionInformation, visSettings->general.showSolverTime);
		if (visSettings->general.renderWindowString.size() != 0)
		{
			message = visSettings->general.renderWindowString + '\n' + message;
		}
		computationMessageNumberOfLines = EXUstd::Count(message, '\n');

		graphicsData.AddText(pTopLeft3D, textColor, message.c_str(), fontSize, 0, -(fontLargeFactor+0.85f), itemID);

		//+++++++++++++++++++
		//Exudyn version bottom-leftmost:
		STDstring versionString = STDstring("version ") + EXUstd::exudynVersion;
		graphicsData.AddText(pBottomRight3D, textColor, versionString.c_str(), fontSize * fontSmallFactor,
			-(float)versionString.size() - 1, 0, itemID);
	}

	//	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	STDstring statusMessage = stateMachine.rendererMessage;

	if (settingsView.window.showMouseCoordinates)
	{
		if (statusMessage.size() != 0) { statusMessage += "\n"; }

		Vector2D centerPoint({ (double)state->centerPoint[0], (double)state->centerPoint[1] });
		//float height = (float)state->currentWindowSize[1];
		//float factor = 2.f * state->zoom / height;
		//Vector2D lastPressedCoords = factor * Vector2D({ stateMachine.lastMousePressedX - 0.5 * state->currentWindowSize[0],
		//	-1. * (stateMachine.lastMousePressedY - 0.5 * state->currentWindowSize[1]) }) + centerPoint;

		Vector2D lastPressedCoords = basicVisualizationSystemContainer->PixelsToOpenGLCoordinates2D(
			stateMachine.lastMousePressedX, stateMachine.lastMousePressedY, *state);

		if (basicVisualizationSystemContainer->IsAxisAlignedView(*state) && 
			settingsView.camera.perspective == 0 &&
			settingsView.camera.trackMarker < 0)
		{
			statusMessage += STDstring("mouse=(") + F2Str(state->openGLcoordinates[0]) + "," + F2Str(state->openGLcoordinates[1]) + ")";

			if (stateMachine.hasLastMousePressed)
			{
				statusMessage += ", last=(" + F2Str(lastPressedCoords[0]) + "," + F2Str(lastPressedCoords[1]) + "), dist=" +
					F2Str((state->openGLcoordinates - lastPressedCoords).GetL2Norm());
			}
			if (!settingsView.camera.useRaytracer)
			{
				float ratio = width / (float)height;
				Float4 colorCrossline = Float4({ 0.f,0.f,0.f,1.f });
				Vector3D pCenter({ state->openGLcoordinates[0] - centerPoint[0], state->openGLcoordinates[1] - centerPoint[1], 0. });
				Vector3D dX({ 2. * (Real)(state->zoom * ratio),0.,0. });
				Vector3D dY({ 0., 2. * (Real)state->zoom,0. });
				Real offCenter = 8. / (Real)state->currentWindowSize[1]; //do not show exact center

				for (Index i = 0; i < 1+Index(stateMachine.hasLastMousePressed); i++)
				{
					if (i == 1)
					{
						colorCrossline = Float4({ 0.3f,0.9f,0.3f,1.f });
						pCenter = Vector3D({ lastPressedCoords[0] - centerPoint[0], lastPressedCoords[1] - centerPoint[1], 0. });
					}
					graphicsData.AddLine(pCenter + offCenter * dX, pCenter + dX, colorCrossline, colorCrossline, itemID);
					graphicsData.AddLine(pCenter - offCenter * dX, pCenter - dX, colorCrossline, colorCrossline, itemID);
					graphicsData.AddLine(pCenter + offCenter * dY, pCenter + dY, colorCrossline, colorCrossline, itemID);
					graphicsData.AddLine(pCenter - offCenter * dY, pCenter - dY, colorCrossline, colorCrossline, itemID);
				}
			}
		}
		else
		{
			statusMessage += STDstring("mouse coordinates only shown in axis-aligned view with perspective=0 and not tracked marker");
		}

	}
	if (settingsView.window.showRenderStateInfo)
	{
		if (statusMessage.size() != 0) { statusMessage += "\n"; }

		//includes marker information
		Vector3D rotationVector = state->GetRotationVector(basicVisualizationSystemContainer, settingsView, true);
		//translationMV = state->centerPoint if trackMarker is deactivated!
		statusMessage += STDstring("modelview: zoom=") + F2Str(state->zoom) + 
			", rot=[" + F2Str(rotationVector[0]) + "," + F2Str(rotationVector[1]) + "," + F2Str(rotationVector[2]) + "]" +
			", center=[" + F2Str(translationMV[0]) + "," + F2Str(translationMV[1]) + "]";
	}


	if (statusMessage.size() != 0)
	{
		graphicsData.AddText(pBottomLeft3D, textColor, statusMessage.c_str(), fontSize* fontSmallFactor,
			0, (float)EXUstd::Count(statusMessage, '\n'), itemID);
	}

	if (visSettings->contour.advanced.showColorBar && visSettings->contour.outputVariable != OutputVariableType::_None) //draw coordinate system
	{
		Float3 p0 = basicVisualizationSystemContainer->PixelsToModelViewCoordinates((float)textIndentPixels,
			(float)height - fontSizeScaled * (fontLargeFactor*1.5f + 1.54f*(3.f + (float)computationMessageNumberOfLines)), //values for compatibility with previous modes
			width, height, zoom, 0); //offsetZ added later!

		float dBasic = 1.6f * 2.f * fontSizeScaled * zoom / ((float)height); //field is 1.6 characters high

		float minVal = 0;
		float maxVal = 1;
		ResizableArray<GraphicsData*>* graphicsDataList = &basicVisualizationSystemContainer->GetGraphicsDataList();
		if (graphicsDataList)
		{
			minVal = graphicsDataList->GetItem(0)->GetContourCurrentMinValue();
			maxVal = graphicsDataList->GetItem(0)->GetContourCurrentMaxValue();
		}
		STDstring contourStr = STDstring("contour plot: ") + GetOutputVariableTypeString(visSettings->contour.outputVariable) +
			"\ncomponent=" + EXUstd::ToString(visSettings->contour.outputVariableComponent) +
			"\nmin=" + EXUstd::Num2String(minVal, visSettings->contour.advanced.colorBarPrecision) + ",max=" + EXUstd::Num2String(maxVal, visSettings->contour.advanced.colorBarPrecision);
		
		graphicsData.AddText(p0, textColor, contourStr.c_str(), fontSize,
			0, 0, itemID);
		p0 += Float3({ 0.f,-2.5f * dBasic,0.f });

		//now draw boxes for contour plot colors and add texts
		float n = (float)visSettings->contour.advanced.colorBarTiling;
		float alphaTransparency = (float)visSettings->contour.alphaTransparency;
		float range = maxVal - minVal;
		const float sizeX = 1.25f * dBasic;
		const float sizeY = dBasic;

		Float3 dX = Float3({ sizeX,0.f,0.f });
		Float3 dY = Float3({ 0.f,sizeY,0.f });

		for (float i = 0; i < n; i++)
		{
			float value = i / n * range + minVal;

			Float4 colorQuad = basicVisualizationSystemContainer->ColorBarColor(minVal, maxVal, value, alphaTransparency); //alpha=1
			Float4 colorLine = Float4({ 0.1f,0.1f,0.1f,1.f });

			std::array<Float3, 4> points = { p0, p0 - dY, p0 + dX - dY, p0 + dX };
			std::array<Float4, 4> colors = { colorQuad, colorQuad, colorQuad, colorQuad };
			graphicsData.AddQuad(points, colors, itemID);

			graphicsData.AddLine(p0, p0 + dX, colorLine, colorLine, itemID);
			graphicsData.AddLine(p0, p0 - dY, colorLine, colorLine, itemID);
			graphicsData.AddLine(p0 + dX, p0 + dX - dY, colorLine, colorLine, itemID);
			graphicsData.AddLine(p0 - dY, p0 + dX - dY, colorLine, colorLine, itemID);

			graphicsData.AddText(p0 + Float3({ 1.2f * sizeX,-0.85f * sizeY,0 }), textColor, 
				EXUstd::Num2String(value, visSettings->contour.advanced.colorBarPrecision).c_str(), fontSize * fontSmallFactor,
				0, (float)EXUstd::Count(statusMessage, '\n'), itemID);

			p0 += Float3({ 0.f,-sizeY,0.f });
		}
	}

	if (drawCoordinateSystem) //draw coordinate system (any settings)
	{
		float dPixels = fontSize * state->displayScaling * visSettings->general.coordinateSystemSize;
		float textOffset = 0.25; //relative to font size

		Float3 p0 = basicVisualizationSystemContainer->PixelsToModelViewCoordinates(1*dPixels, 1*dPixels, width, height, zoom);
		float d = onePixel[0] * dPixels;
		//std::cout << "d=" << d << ",rot=" << rotationMV << "\n";

		Float3 p1 = p0 + rotationMV * Float3({ d,  0.f,0.f });
		Float3 p2 = p0 + rotationMV * Float3({ 0.f,  d,0.f });
		Float3 p3 = p0 + rotationMV * Float3({ 0.f,0.f,  d });

		GLLine line;
		Float4 colorCS({ 0.3f,0.3f,0.3f,1 });

		bool cs3Dpossible = !(settingsView.camera.perspective && settingsView.camera.useRaytracer);

		if (drawCoordinateSystem == 1 ||
			(drawCoordinateSystem >= 2 && !cs3Dpossible) )
		{
			line.color1 = colorCS;
			line.color2 = colorCS;
			line.itemID = itemID;
			line.point1 = p0;
			line.point2 = p1;
			graphicsData.glLines.Append(line); //line is only visible if no faces shown
			line.point2 = p2;
			graphicsData.glLines.Append(line);
			line.point2 = p3;
			graphicsData.glLines.Append(line);
		}

		if (drawCoordinateSystem >= 2 && cs3Dpossible)
		{
			Real radius = (Real)d * 0.05;
			Index tiling = 24;
			Real factLength = 1.;
			EXUvis::DrawArrow(Vector3D({ (Real)p0[0], (Real)p0[1], (Real)p0[2] }),
				factLength * Vector3D({ (Real)(p1 - p0)[0], (Real)(p1 - p0)[1], (Real)(p1 - p0)[2] }), //make arrow a little bit longer than line to hide line
				radius, EXUvis::red, graphicsData, itemIDstaticObjectShaded, tiling);
			EXUvis::DrawArrow(Vector3D({ (Real)p0[0], (Real)p0[1], (Real)p0[2] }),
				factLength * Vector3D({ (Real)(p2 - p0)[0], (Real)(p2 - p0)[1], (Real)(p2 - p0)[2] }),
				radius, EXUvis::green, graphicsData, itemIDstaticObjectShaded, tiling);
			EXUvis::DrawArrow(Vector3D({ (Real)p0[0], (Real)p0[1], (Real)p0[2] }),
				factLength * Vector3D({ (Real)(p3 - p0)[0], (Real)(p3 - p0)[1], (Real)(p3 - p0)[2] }),
				radius, EXUvis::blue, graphicsData, itemIDstaticObjectShaded, tiling);
		}

		if (drawCoordinateSystem == 1 || drawCoordinateSystem >= 3 || (drawCoordinateSystem == 2 && !cs3Dpossible) )
		{
			graphicsData.AddText(p1, colorCS, "X(0)", fontSize, textOffset, textOffset, itemID);
			graphicsData.AddText(p2, colorCS, "Y(1)", fontSize, textOffset, textOffset, itemID);
			graphicsData.AddText(p3, colorCS, "Z(2)", fontSize, textOffset, textOffset, itemID);
		}
	}
	if (settingsView.scene.drawWorldBasis) //draw world basis coordinate system (at [0,0,0] in scene)
	{

		//compensate for center point shift:
		Vector3D p0({ -(Real)translationMV[0], -(Real)translationMV[1], -(Real)translationMV[2] });
		Matrix3D rot3D;
		rot3D.CopyFrom(rotationMV);

		EXUvis::DrawOrthonormalBasis(p0, rot3D, settingsView.scene.worldBasisSize,
			0.005* settingsView.scene.worldBasisSize, graphicsData, itemIDstaticObjectWithoutZoff);
	}

	if (visSettings->openGL.advanced.showBoundingBox && !settingsView.camera.perspective) //does not work with perspective
	{
		//draw bounding box
		//NOTE: state->boundingBox uses screen coordinates
		//NOTE: state->boundingBox is only updated on ZoomAll or at startup!

		Float4 colorBB({ 0.9f,0.2f,0.2f, 1 });

		Box3DF box = state->boundingBox;
		Float3 p0 = -translationMV + box.Center(); //compensate for center point shift
		Float3 dX = Float3({ 0.5f * box.SizeX(), 0.f, 0.f });
		Float3 dY = Float3({ 0.f, 0.5f * box.SizeY(), 0.f });
		Float3 dZ = Float3({ 0.f, 0.f, 0.5f * box.SizeZ() });

		for (Index i=0; i <= 2; i++)
		{
			RTfloat fact1 = (i != 1) ? -1.f : 1.f;
			RTfloat fact2 = (i == 0) ? -1.f : 1.f;
			
			//NOTE: only flat rectangle is visible due to hidden lines, drawn in sceen coordinates!!
			graphicsData.AddLine(p0 - dX + dY + fact1 * dZ, p0 + dX + dY + fact2 * dZ, colorBB, colorBB, itemIDstaticObjectWithoutZoff);
			graphicsData.AddLine(p0 - dX - dY + fact1 * dZ, p0 + dX - dY + fact2 * dZ, colorBB, colorBB, itemIDstaticObjectWithoutZoff);
			graphicsData.AddLine(p0 + dX - dY + fact1 * dZ, p0 + dX + dY + fact2 * dZ, colorBB, colorBB, itemIDstaticObjectWithoutZoff);
			graphicsData.AddLine(p0 - dX - dY + fact1 * dZ, p0 - dX + dY + fact2 * dZ, colorBB, colorBB, itemIDstaticObjectWithoutZoff);
		}

	}

}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



