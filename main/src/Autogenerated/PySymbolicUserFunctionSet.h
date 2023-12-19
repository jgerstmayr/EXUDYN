    //include file for PySymbolicUserFunctionSet
    //author: Johannes Gerstmayr
    //license: see Exudyn license
    //AUTO:

	//! set up general user function from dictionary:
	template<typename TItemIndex>
	void SetUserFunctionFromDict(MainSystem& mainSystem, py::dict pyObject, TItemIndex itemIndex, const STDstring& userFunctionName)
	{
		STDstring sType = itemIndex.GetTypeString();
		STDstring itemTypeName = GetItemTypeName(mainSystem, itemIndex);

		SetupUserFunction(pyObject, itemTypeName, userFunctionName);

		//now cast items to set user function
        if (itemTypeName == "GenericODE2" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex2Vector = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "GenericODE1" && userFunctionName == "rhsUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndexVector = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "KinematicTree" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex2Vector = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "FFRF" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex2Vector = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "FFRFreducedOrder" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex2Vector = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ANCFCable2D" && userFunctionName == "axialForceUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar9 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6, Real arg7, Real arg8, Real arg9, Real arg10)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ANCFCable2D" && userFunctionName == "bendingMomentUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar9 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6, Real arg7, Real arg8, Real arg9, Real arg10)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ConnectorSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar5 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ConnectorCartesianSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            vector3DmbsScalarIndexScalar4Vector3D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdVector3D arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ConnectorRigidBodySpringDamper" && userFunctionName == "springForceTorqueUserFunction")
        {            //define the user function as lambda function of this
            vector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdMatrix6D arg6, StdMatrix6D arg7, StdMatrix3D arg8, StdMatrix3D arg9, StdVector6D arg10)
                {
                    return this->EvaluateStdVector6D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ConnectorRigidBodySpringDamper" && userFunctionName == "postNewtonStepUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdVector3D arg6, StdMatrix6D arg7, StdMatrix6D arg8, StdMatrix3D arg9, StdMatrix3D arg10, StdVector6D arg11)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11);
                };
        }
        else if (itemTypeName == "ConnectorLinearSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar5 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ConnectorTorsionalSpringDamper" && userFunctionName == "springTorqueUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar5 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ConnectorCoordinateSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar5 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ConnectorCoordinateSpringDamperExt" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar11 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6, Real arg7, Real arg8, Real arg9, Real arg10, Real arg11, Real arg12)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12);
                };
        }
        else if (itemTypeName == "ConnectorCoordinate" && userFunctionName == "offsetUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ConnectorCoordinate" && userFunctionName == "offsetUserFunction_t")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ConnectorCoordinateVector" && userFunctionName == "constraintUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex2VectorBool = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3, bool arg4)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4);
                };
        }
        else if (itemTypeName == "JointGeneric" && userFunctionName == "offsetUserFunction")
        {            //define the user function as lambda function of this
            vector6DmbsScalarIndexVector6D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector6D arg2)
                {
                    return this->EvaluateStdVector6D(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "JointGeneric" && userFunctionName == "offsetUserFunction_t")
        {            //define the user function as lambda function of this
            vector6DmbsScalarIndexVector6D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector6D arg2)
                {
                    return this->EvaluateStdVector6D(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ForceVector" && userFunctionName == "loadVectorUserFunction")
        {            //define the user function as lambda function of this
            vector3DmbsScalarVector3D = [this](const MainSystem& mainSystem, Real arg0, StdVector3D arg1)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1);
                };
        }
        else if (itemTypeName == "TorqueVector" && userFunctionName == "loadVectorUserFunction")
        {            //define the user function as lambda function of this
            vector3DmbsScalarVector3D = [this](const MainSystem& mainSystem, Real arg0, StdVector3D arg1)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1);
                };
        }
        else if (itemTypeName == "MassProportional" && userFunctionName == "loadVectorUserFunction")
        {            //define the user function as lambda function of this
            vector3DmbsScalarVector3D = [this](const MainSystem& mainSystem, Real arg0, StdVector3D arg1)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1);
                };
        }
        else if (itemTypeName == "Coordinate" && userFunctionName == "loadUserFunction")
        {            //define the user function as lambda function of this
            mbsScalar2 = [this](const MainSystem& mainSystem, Real arg0, Real arg1)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1);
                };
        }
		else
		{
			PyError(STDstring("Symbolic::SetUserFunctionFromDict<") + itemTypeName + "," + userFunctionName +
				">: invalid user object type or user function type; possibly, function is not available as symbolic user function");
		}

	}


