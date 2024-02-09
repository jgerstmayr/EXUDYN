    //include file for PySymbolicUserFunctionSet
    //author: Johannes Gerstmayr
    //license: see Exudyn license
    //AUTO:
//collect all kinds of user functions
public:
    std::function<bool(const MainSystem&,Real)> boolMbsScalar;
    std::function<StdVector2D(const MainSystem&,Real)> vector2DMbsScalar;
    std::function<py::object(const MainSystem&,Index)> graphicsData;
    std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)> vectorMbsScalarIndex2Vector;
    std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector)> matrixContainerMbsScalarIndex2Vector;
    std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,Real,Real)> matrixContainerMbsScalarIndex2Vector2Scalar;
    std::function<StdVector(const MainSystem&,Real,Index,StdVector)> vectorMbsScalarIndexVector;
    std::function<NumpyMatrix(const MainSystem&,Real,Index,StdVector,StdVector)> matrixMbsScalarIndex2Vector;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real)> mbsScalarIndexScalar9;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)> mbsScalarIndexScalar5;
    std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)> vector3DmbsScalarIndexScalar4Vector3D;
    std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)> vector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D;
    std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)> vectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)> mbsScalarIndexScalar11;
    std::function<Real(const MainSystem&,Real,Index,Real)> mbsScalarIndexScalar;
    std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector,bool)> vectorMbsScalarIndex2VectorBool;
    std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,bool)> matrixContainerMbsScalarIndex2VectorBool;
    std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)> vector6DmbsScalarIndexVector6D;
    std::function<StdVector3D(const MainSystem&,Real,StdVector3D)> vector3DmbsScalarVector3D;
    std::function<Real(const MainSystem&,Real,Real)> mbsScalar2;
    std::function<StdVector(const MainSystem&,Real,StdArrayIndex,StdVector,ConfigurationType)> vectorMbsScalarArrayIndexVectorConfiguration;

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
            CHECKandTHROW(itemIndex.is_none(), "SetUserFunctionFromDict: if itemTypeName is provided, itemIndex must be None");
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
        else if (itemTypeName == "MainSystem" && userFunctionName == "postNewtonFunction")
        {            //define the user function as lambda function of this
            vector2DMbsScalar = [this](const MainSystem& mainSystem, Real arg0)
                {
                    return this->EvaluateStdVector2D(mainSystem, arg0);
                };
        }
        else if (itemTypeName == "ObjectGenericODE2" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex2Vector = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectGenericODE1" && userFunctionName == "rhsUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndexVector = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ObjectANCFCable2D" && userFunctionName == "axialForceUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar9 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6, Real arg7, Real arg8, Real arg9, Real arg10)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ObjectConnectorSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar5 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectConnectorCartesianSpringDamper" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            vector3DmbsScalarIndexScalar4Vector3D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdVector3D arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectConnectorRigidBodySpringDamper" && userFunctionName == "springForceTorqueUserFunction")
        {            //define the user function as lambda function of this
            vector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdMatrix6D arg6, StdMatrix6D arg7, StdMatrix3D arg8, StdMatrix3D arg9, StdVector6D arg10)
                {
                    return this->EvaluateStdVector6D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ObjectConnectorRigidBodySpringDamper" && userFunctionName == "postNewtonStepUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdVector3D arg6, StdMatrix6D arg7, StdMatrix6D arg8, StdMatrix3D arg9, StdMatrix3D arg10, StdVector6D arg11)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinateSpringDamperExt" && userFunctionName == "springForceUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar11 = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, Real arg4, Real arg5, Real arg6, Real arg7, Real arg8, Real arg9, Real arg10, Real arg11, Real arg12)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinate" && userFunctionName == "offsetUserFunction")
        {            //define the user function as lambda function of this
            mbsScalarIndexScalar = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2)
                {
                    return this->EvaluateReal(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinateVector" && userFunctionName == "constraintUserFunction")
        {            //define the user function as lambda function of this
            vectorMbsScalarIndex2VectorBool = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector arg2, StdVector arg3, bool arg4)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4);
                };
        }
        else if (itemTypeName == "ObjectJointGeneric" && userFunctionName == "offsetUserFunction")
        {            //define the user function as lambda function of this
            vector6DmbsScalarIndexVector6D = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector6D arg2)
                {
                    return this->EvaluateStdVector6D(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "LoadForceVector" && userFunctionName == "loadVectorUserFunction")
        {            //define the user function as lambda function of this
            vector3DmbsScalarVector3D = [this](const MainSystem& mainSystem, Real arg0, StdVector3D arg1)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1);
                };
        }
        else if (itemTypeName == "LoadCoordinate" && userFunctionName == "loadUserFunction")
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
			return graphicsData;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)>>)
		{
			return vectorMbsScalarIndex2Vector;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector)>>)
		{
			return matrixContainerMbsScalarIndex2Vector;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,Real,Real)>>)
		{
			return matrixContainerMbsScalarIndex2Vector2Scalar;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,StdVector)>>)
		{
			return vectorMbsScalarIndexVector;
		}
        else if constexpr (std::is_same_v<UFT, std::function<NumpyMatrix(const MainSystem&,Real,Index,StdVector,StdVector)>>)
		{
			return matrixMbsScalarIndex2Vector;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>)
		{
			return mbsScalarIndexScalar9;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)>>)
		{
			return mbsScalarIndexScalar5;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)>>)
		{
			return vector3DmbsScalarIndexScalar4Vector3D;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>)
		{
			return vector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>)
		{
			return vectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>)
		{
			return mbsScalarIndexScalar11;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real)>>)
		{
			return mbsScalarIndexScalar;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector,bool)>>)
		{
			return vectorMbsScalarIndex2VectorBool;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,bool)>>)
		{
			return matrixContainerMbsScalarIndex2VectorBool;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)>>)
		{
			return vector6DmbsScalarIndexVector6D;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector3D(const MainSystem&,Real,StdVector3D)>>)
		{
			return vector3DmbsScalarVector3D;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Real)>>)
		{
			return mbsScalar2;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,StdArrayIndex,StdVector,ConfigurationType)>>)
		{
			return vectorMbsScalarArrayIndexVectorConfiguration;
		}

		return UFT(0); //will never happen, but avoids errors/warnings
    }


