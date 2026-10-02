    //include file for PySymbolicUserFunctionSet
    //author: Johannes Gerstmayr
    //license: see Exudyn license
    //AUTO:
//collect all kinds of user functions
public:
    std::function<bool(const MainSystem&,Real)> boolMbsScalar;
    std::function<StdVector2D(const MainSystem&,Real)> vector2DMbsScalar;
    std::function<py::object(const MainSystem&,Index)> objectGroundGraphicsDataUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)> objectGenericODE2ForceUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector)> objectGenericODE2MassMatrixUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,Real,Real)> objectGenericODE2JacobianUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,StdVector)> objectGenericODE1RhsUserFunction;
    std::function<NumpyMatrix(const MainSystem&,Real,Index,StdVector,StdVector)> objectFFRFMassMatrixUserFunction;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real)> objectANCFCable2DAxialForceUserFunction;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)> objectConnectorSpringDamperSpringForceUserFunction;
    std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)> objectConnectorCartesianSpringDamperSpringForceUserFunction;
    std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)> objectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)> objectConnectorRigidBodySpringDamperPostNewtonStepUserFunction;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)> objectConnectorCoordinateSpringDamperExtSpringForceUserFunction;
    std::function<Real(const MainSystem&,Real,Index,Real)> objectConnectorCoordinateOffsetUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector,bool)> objectConnectorCoordinateVectorConstraintUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,bool)> objectConnectorCoordinateVectorJacobianUserFunction;
    std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)> objectJointGenericOffsetUserFunction;
    std::function<StdVector3D(const MainSystem&,Real,StdVector3D)> loadForceVectorLoadVectorUserFunction;
    std::function<Real(const MainSystem&,Real,Real)> loadCoordinateLoadUserFunction;
    std::function<StdVector(const MainSystem&,Real,StdArrayIndex,StdVector,ConfigurationType)> sensorUserFunctionSensorUserFunction;

	//! set up general user function from dictionary:
	void SetUserFunctionFromDict(MainSystem& mainSystem, py::dict pyObject, const STDstring& userFunctionName, py::object itemIndex, STDstring itemTypeName)
	{
        if (itemTypeName == "None")
        {
            STDstring sType;
            Index itemNumber;
            GetItemTypeName(mainSystem, itemIndex, sType, itemTypeName, itemNumber);
        }
        else
        {
            CHECKandTHROW(itemIndex.is_none(), "SetUserFunctionFromDict: if itemTypeName is provided, itemIndex must be None", ExudynValueError);
        }

		SetupUserFunction(pyObject, itemTypeName, userFunctionName);

		//now cast items to set user function
        if (itemTypeName == "MainSystem" && userFunctionName == "preStepUserFunction")
        {            //define the user function as lambda function of this
            boolMbsScalar = [this](const MainSystem& mainSystem, Real arg0)
                {
                    return this->EvaluateBool(mainSystem, arg0);
                };
        }
        else if (itemTypeName == "MainSystem" && userFunctionName == "postStepUserFunction")
        {            //define the user function as lambda function of this
            boolMbsScalar = [this](const MainSystem& mainSystem, Real arg0)
                {
                    return this->EvaluateBool(mainSystem, arg0);
                };
        }
        else if (itemTypeName == "MainSystem" && userFunctionName == "postNewtonFunction")
        {            //define the user function as lambda function of this
            vector2DMbsScalar = [this](const MainSystem& mainSystem, Real arg0)
                {
                    return this->EvaluateStdVector2D(mainSystem, arg0);
                };
        }
        else if (itemTypeName == "ObjectGenericODE2" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2ForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectGenericODE1" && userFunctionName == "rhsUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE1RhsUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ObjectKinematicTree" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2ForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectFFRF" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2ForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectFFRFreducedOrder" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2ForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectANCFCable2D" && userFunctionName == "axialForceUserFunction")
        {            //define the user function as lambda function of this
            objectANCFCable2DAxialForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6, Real arg7, Real arg8, Real arg9, Real arg10)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ObjectANCFCable2D" && userFunctionName == "bendingMomentUserFunction")
        {            //define the user function as lambda function of this
            objectANCFCable2DAxialForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6, Real arg7, Real arg8, Real arg9, Real arg10)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ObjectConnectorSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorSpringDamperSpringForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectConnectorCartesianSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorCartesianSpringDamperSpringForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdVector3D arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectConnectorRigidBodySpringDamper" && userFunctionName == "springForceTorqueUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdMatrix6D arg6, StdMatrix6D arg7, StdMatrix3D arg8, StdMatrix3D arg9, StdVector6D arg10)
                {
                    return this->EvaluateStdVector6D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ObjectConnectorRigidBodySpringDamper" && userFunctionName == "postNewtonStepUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorRigidBodySpringDamperPostNewtonStepUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdVector3D arg6, StdMatrix6D arg7, StdMatrix6D arg8, StdMatrix3D arg9, StdMatrix3D arg10, StdVector6D arg11)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11);
                };
        }
        else if (itemTypeName == "ObjectConnectorLinearSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorSpringDamperSpringForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectConnectorTorsionalSpringDamper" && userFunctionName == "springTorqueUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorSpringDamperSpringForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinateSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorSpringDamperSpringForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinateSpringDamperExt" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorCoordinateSpringDamperExtSpringForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6, Real arg7, Real arg8, Real arg9, Real arg10, Real arg11, Real arg12)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinate" && userFunctionName == "offsetUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorCoordinateOffsetUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinate" && userFunctionName == "offsetUserFunction_t")
        {            //define the user function as lambda function of this
            objectConnectorCoordinateOffsetUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinateVector" && userFunctionName == "constraintUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorCoordinateVectorConstraintUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3, bool arg4)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4);
                };
        }
        else if (itemTypeName == "ObjectJointGeneric" && userFunctionName == "offsetUserFunction")
        {            //define the user function as lambda function of this
            objectJointGenericOffsetUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector6D arg2)
                {
                    return this->EvaluateStdVector6D(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ObjectJointGeneric" && userFunctionName == "offsetUserFunction_t")
        {            //define the user function as lambda function of this
            objectJointGenericOffsetUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector6D arg2)
                {
                    return this->EvaluateStdVector6D(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "LoadForceVector" && userFunctionName == "loadVectorUserFunction")
        {            //define the user function as lambda function of this
            loadForceVectorLoadVectorUserFunction = [this](const MainSystem& mainSystem, Real arg0, StdVector3D arg1)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1);
                };
        }
        else if (itemTypeName == "LoadTorqueVector" && userFunctionName == "loadVectorUserFunction")
        {            //define the user function as lambda function of this
            loadForceVectorLoadVectorUserFunction = [this](const MainSystem& mainSystem, Real arg0, StdVector3D arg1)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1);
                };
        }
        else if (itemTypeName == "LoadMassProportional" && userFunctionName == "loadVectorUserFunction")
        {            //define the user function as lambda function of this
            loadForceVectorLoadVectorUserFunction = [this](const MainSystem& mainSystem, Real arg0, StdVector3D arg1)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1);
                };
        }
        else if (itemTypeName == "LoadCoordinate" && userFunctionName == "loadUserFunction")
        {            //define the user function as lambda function of this
            loadCoordinateLoadUserFunction = [this](const MainSystem& mainSystem, Real arg0, Real arg1)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1);
                };
        }
		else
		{
			PyError(STDstring("Symbolic::SetUserFunctionFromDict<") + itemTypeName + "," + userFunctionName +
				">: invalid user object type or user function type; possibly, function is not available as symbolic user function",
				PyErrorType::notImplementedError);
		}

	}


	//! for specific user function type, get user function by type and name
    template<typename UFT>
    UFT GetSTDfunction() const
    {
        if constexpr (std::is_same_v<UFT, std::function<bool(const MainSystem&,Real)>>)
		{
			return boolMbsScalar;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector2D(const MainSystem&,Real)>>)
		{
			return vector2DMbsScalar;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Index)>>)
		{
			return objectGroundGraphicsDataUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)>>)
		{
			return objectGenericODE2ForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector)>>)
		{
			return objectGenericODE2MassMatrixUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,Real,Real)>>)
		{
			return objectGenericODE2JacobianUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,StdVector)>>)
		{
			return objectGenericODE1RhsUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<NumpyMatrix(const MainSystem&,Real,Index,StdVector,StdVector)>>)
		{
			return objectFFRFMassMatrixUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>)
		{
			return objectANCFCable2DAxialForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)>>)
		{
			return objectConnectorSpringDamperSpringForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)>>)
		{
			return objectConnectorCartesianSpringDamperSpringForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>)
		{
			return objectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>)
		{
			return objectConnectorRigidBodySpringDamperPostNewtonStepUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>)
		{
			return objectConnectorCoordinateSpringDamperExtSpringForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real)>>)
		{
			return objectConnectorCoordinateOffsetUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector,bool)>>)
		{
			return objectConnectorCoordinateVectorConstraintUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,bool)>>)
		{
			return objectConnectorCoordinateVectorJacobianUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)>>)
		{
			return objectJointGenericOffsetUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector3D(const MainSystem&,Real,StdVector3D)>>)
		{
			return loadForceVectorLoadVectorUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Real)>>)
		{
			return loadCoordinateLoadUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,StdArrayIndex,StdVector,ConfigurationType)>>)
		{
			return sensorUserFunctionSensorUserFunction;
		}

		return UFT(0); //will never happen, but avoids errors/warnings
    }


