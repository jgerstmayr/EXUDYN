    //include file for PySymbolicUserFunctionSet
    //author: Johannes Gerstmayr
    //license: see Exudyn license
    //AUTO:
//collect all kinds of user functions
public:
    std::function<bool(const MainSystem&,Real)> boolMbsScalar;
    std::function<StdVector2D(const MainSystem&,Real)> vector2DMbsScalar;
    std::function<StdVector3D(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> nodeUserPositionUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> nodeUserPositionJacobianUserFunction;
    std::function<py::object(const MainSystem&,Index)> objectGroundGraphicsDataUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&)> objectGenericODE2ForceUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&)> objectGenericODE2MassMatrixUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,Real,Real)> objectGenericODE2JacobianUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const StdVector&)> objectGenericODE1RhsUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> objectGenericDAEForceUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> objectGenericDAEMassMatrixUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,bool,const StdVector&)> objectGenericDAEConstraintUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,Real,Real,Real,const StdVector&)> objectGenericDAEJacobianUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> objectGenericDAEPostNewtonStepUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> objectUserBodyForceUserFunction;
    std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> objectUserBodyPositionUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> objectUserBodyPositionJacobianUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,OutputVariableType,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> objectUserBodyOutputVariableUserFunction;
    std::function<NumpyMatrix(const MainSystem&,Real,Index,const StdVector&,const StdVector&)> objectFFRFMassMatrixUserFunction;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real)> objectANCFCable2DAxialForceUserFunction;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)> objectConnectorSpringDamperSpringForceUserFunction;
    std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)> objectConnectorCartesianSpringDamperSpringForceUserFunction;
    std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)> objectConnectorRigidBodySpringDamperSpringForceTorqueUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)> objectConnectorRigidBodySpringDamperPostNewtonStepUserFunction;
    std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)> objectConnectorCoordinateSpringDamperExtSpringForceUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const Twist&,const HomogeneousTransformation&,const Twist&,const StdVector&,const StdVector&)> objectConnectorUserForceUserFunction;
    std::function<void(const MainSystem&,Real,Index,py::object,const StdVector&,const StdVector&)> objectConnectorUserMarkerAccessUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const Twist&,const HomogeneousTransformation&,const Twist&,const StdVector&,const StdVector&,const StdVector&)> objectConnectorUserPostNewtonStepUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const HomogeneousTransformation&,const StdVector&)> objectJointUserConstraintUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,const HomogeneousTransformation&,const HomogeneousTransformation&,const StdVector&)> objectJointUserJacobianUserFunction;
    std::function<Real(const MainSystem&,Real,Index,Real)> objectConnectorCoordinateOffsetUserFunction;
    std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,bool)> objectConnectorCoordinateVectorConstraintUserFunction;
    std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,bool)> objectConnectorCoordinateVectorJacobianUserFunction;
    std::function<StdVector3D(const MainSystem&,Real,Index,Real,Real,StdVector3D,StdVector3D,Real)> objectContactSphereTriangleContactForceUserFunction;
    std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,Real,StdVector3D)> objectJournalBearingForceUserFunction;
    std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)> objectJointGenericOffsetUserFunction;
    std::function<StdVector3D(const MainSystem&,Real,StdVector3D)> loadForceVectorLoadVectorUserFunction;
    std::function<Real(const MainSystem&,Real,Real)> loadCoordinateLoadUserFunction;
    std::function<StdVector(const MainSystem&,Real,StdArrayIndex,const StdVector&,ConfigurationType)> sensorUserFunctionSensorUserFunction;

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
        else if (itemTypeName == "NodeUser" && userFunctionName == "positionUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "NodeUser" && userFunctionName == "positionJacobianUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "NodeUser" && userFunctionName == "rotationMatrixUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "NodeUser" && userFunctionName == "rotationJacobianUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "ObjectGenericODE2" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2ForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectGenericODE2" && userFunctionName == "massMatrixUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2MassMatrixUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectGenericODE2" && userFunctionName == "jacobianUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2JacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, Real arg4, Real arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "ObjectGenericODE1" && userFunctionName == "rhsUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE1RhsUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2);
                };
        }
        else if (itemTypeName == "ObjectGenericDAE" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericDAEForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectGenericDAE" && userFunctionName == "rhsUserFunction")
        {            //define the user function as lambda function of this
            objectGenericDAEForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectGenericDAE" && userFunctionName == "massMatrixUserFunction")
        {            //define the user function as lambda function of this
            objectGenericDAEMassMatrixUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectGenericDAE" && userFunctionName == "constraintUserFunction")
        {            //define the user function as lambda function of this
            objectGenericDAEConstraintUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6, bool arg7, const StdVector& arg8)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8);
                };
        }
        else if (itemTypeName == "ObjectGenericDAE" && userFunctionName == "jacobianUserFunction")
        {            //define the user function as lambda function of this
            objectGenericDAEJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6, Real arg7, Real arg8, Real arg9, const StdVector& arg10)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                };
        }
        else if (itemTypeName == "ObjectGenericDAE" && userFunctionName == "postNewtonStepUserFunction")
        {            //define the user function as lambda function of this
            objectGenericDAEPostNewtonStepUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6, const StdVector& arg7, const StdVector& arg8)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8);
                };
        }
        else if (itemTypeName == "ObjectUserBody" && userFunctionName == "massMatrixUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "ObjectUserBody" && userFunctionName == "massIntegralUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "ObjectUserBody" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectUserBodyForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "ObjectUserBody" && userFunctionName == "positionUserFunction")
        {            //define the user function as lambda function of this
            objectUserBodyPositionUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectUserBody" && userFunctionName == "positionJacobianUserFunction")
        {            //define the user function as lambda function of this
            objectUserBodyPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectUserBody" && userFunctionName == "rotationMatrixUserFunction")
        {            //define the user function as lambda function of this
            objectUserBodyPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectUserBody" && userFunctionName == "rotationJacobianUserFunction")
        {            //define the user function as lambda function of this
            objectUserBodyPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectUserBody" && userFunctionName == "outputVariableUserFunction")
        {            //define the user function as lambda function of this
            objectUserBodyOutputVariableUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, OutputVariableType arg2, StdVector3D arg3, const StdVector& arg4, const StdVector& arg5, const StdVector& arg6, const StdVector& arg7, const StdVector& arg8)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8);
                };
        }
        else if (itemTypeName == "ObjectKinematicTree" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2ForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectFFRF" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2ForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectFFRF" && userFunctionName == "massMatrixUserFunction")
        {            //define the user function as lambda function of this
            objectFFRFMassMatrixUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3)
                {
                    return this->EvaluateNumpyMatrix(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectFFRFreducedOrder" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectGenericODE2ForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3);
                };
        }
        else if (itemTypeName == "ObjectFFRFreducedOrder" && userFunctionName == "massMatrixUserFunction")
        {            //define the user function as lambda function of this
            objectFFRFMassMatrixUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3)
                {
                    return this->EvaluateNumpyMatrix(mainSystem, arg0, arg1, arg2, arg3);
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
            objectConnectorRigidBodySpringDamperPostNewtonStepUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, StdVector3D arg3, StdVector3D arg4, StdVector3D arg5, StdVector3D arg6, StdMatrix6D arg7, StdMatrix6D arg8, StdMatrix3D arg9, StdMatrix3D arg10, StdVector6D arg11)
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
        else if (itemTypeName == "ObjectConnectorUser" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorUserForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const HomogeneousTransformation& arg2, const Twist& arg3, const HomogeneousTransformation& arg4, const Twist& arg5, const StdVector& arg6, const StdVector& arg7)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7);
                };
        }
        else if (itemTypeName == "ObjectConnectorUser" && userFunctionName == "postNewtonStepUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorUserPostNewtonStepUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const HomogeneousTransformation& arg2, const Twist& arg3, const HomogeneousTransformation& arg4, const Twist& arg5, const StdVector& arg6, const StdVector& arg7, const StdVector& arg8)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8);
                };
        }
        else if (itemTypeName == "ObjectJointUser" && userFunctionName == "constraintUserFunction")
        {            //define the user function as lambda function of this
            objectJointUserConstraintUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const HomogeneousTransformation& arg2, const HomogeneousTransformation& arg3, const StdVector& arg4)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4);
                };
        }
        else if (itemTypeName == "ObjectJointUser" && userFunctionName == "jacobianUserFunction")
        {            //define the user function as lambda function of this
            objectJointUserJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const HomogeneousTransformation& arg2, const HomogeneousTransformation& arg3, const StdVector& arg4)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4);
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
            objectConnectorCoordinateVectorConstraintUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, bool arg4)
                {
                    return this->EvaluateStdVector(mainSystem, arg0, arg1, arg2, arg3, arg4);
                };
        }
        else if (itemTypeName == "ObjectConnectorCoordinateVector" && userFunctionName == "jacobianUserFunction")
        {            //define the user function as lambda function of this
            objectConnectorCoordinateVectorJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, bool arg4)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4);
                };
        }
        else if (itemTypeName == "ObjectContactSphereTriangle" && userFunctionName == "contactForceUserFunction")
        {            //define the user function as lambda function of this
            objectContactSphereTriangleContactForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, StdVector3D arg4, StdVector3D arg5, Real arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectContactSphereCylinder" && userFunctionName == "contactForceUserFunction")
        {            //define the user function as lambda function of this
            objectContactSphereTriangleContactForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, StdVector3D arg4, StdVector3D arg5, Real arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectContactCylinderCylinder" && userFunctionName == "contactForceUserFunction")
        {            //define the user function as lambda function of this
            objectContactSphereTriangleContactForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, StdVector3D arg4, StdVector3D arg5, Real arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectContactConeCone" && userFunctionName == "contactForceUserFunction")
        {            //define the user function as lambda function of this
            objectContactSphereTriangleContactForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, StdVector3D arg4, StdVector3D arg5, Real arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectContactConeTriangle" && userFunctionName == "contactForceUserFunction")
        {            //define the user function as lambda function of this
            objectContactSphereTriangleContactForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, StdVector3D arg4, StdVector3D arg5, Real arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectContactConvexConvex" && userFunctionName == "contactForceUserFunction")
        {            //define the user function as lambda function of this
            objectContactSphereTriangleContactForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, StdVector3D arg4, StdVector3D arg5, Real arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectContactPrimitiveConvex" && userFunctionName == "contactForceUserFunction")
        {            //define the user function as lambda function of this
            objectContactSphereTriangleContactForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, StdVector3D arg4, StdVector3D arg5, Real arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectContactCylinderTriangle" && userFunctionName == "contactForceUserFunction")
        {            //define the user function as lambda function of this
            objectContactSphereTriangleContactForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, Real arg2, Real arg3, StdVector3D arg4, StdVector3D arg5, Real arg6)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5, arg6);
                };
        }
        else if (itemTypeName == "ObjectJournalBearing" && userFunctionName == "forceUserFunction")
        {            //define the user function as lambda function of this
            objectJournalBearingForceUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, StdVector3D arg2, StdVector3D arg3, Real arg4, StdVector3D arg5)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
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
        else if (itemTypeName == "MarkerBodyUser" && userFunctionName == "positionUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluateStdVector3D(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "MarkerBodyUser" && userFunctionName == "positionJacobianUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "MarkerBodyUser" && userFunctionName == "rotationMatrixUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
                };
        }
        else if (itemTypeName == "MarkerBodyUser" && userFunctionName == "rotationJacobianUserFunction")
        {            //define the user function as lambda function of this
            nodeUserPositionJacobianUserFunction = [this](const MainSystem& mainSystem, Real arg0, Index arg1, const StdVector& arg2, const StdVector& arg3, const StdVector& arg4, const StdVector& arg5)
                {
                    return this->EvaluatePyObject(mainSystem, arg0, arg1, arg2, arg3, arg4, arg5);
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
        else if constexpr (std::is_same_v<UFT, std::function<StdVector3D(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return nodeUserPositionUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return nodeUserPositionJacobianUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Index)>>)
		{
			return objectGroundGraphicsDataUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&)>>)
		{
			return objectGenericODE2ForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&)>>)
		{
			return objectGenericODE2MassMatrixUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,Real,Real)>>)
		{
			return objectGenericODE2JacobianUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const StdVector&)>>)
		{
			return objectGenericODE1RhsUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return objectGenericDAEForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return objectGenericDAEMassMatrixUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,bool,const StdVector&)>>)
		{
			return objectGenericDAEConstraintUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,Real,Real,Real,const StdVector&)>>)
		{
			return objectGenericDAEJacobianUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return objectGenericDAEPostNewtonStepUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return objectUserBodyForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return objectUserBodyPositionUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return objectUserBodyPositionJacobianUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,OutputVariableType,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return objectUserBodyOutputVariableUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<NumpyMatrix(const MainSystem&,Real,Index,const StdVector&,const StdVector&)>>)
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
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>)
		{
			return objectConnectorRigidBodySpringDamperPostNewtonStepUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>)
		{
			return objectConnectorCoordinateSpringDamperExtSpringForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const Twist&,const HomogeneousTransformation&,const Twist&,const StdVector&,const StdVector&)>>)
		{
			return objectConnectorUserForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<void(const MainSystem&,Real,Index,py::object,const StdVector&,const StdVector&)>>)
		{
			return objectConnectorUserMarkerAccessUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const Twist&,const HomogeneousTransformation&,const Twist&,const StdVector&,const StdVector&,const StdVector&)>>)
		{
			return objectConnectorUserPostNewtonStepUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const HomogeneousTransformation&,const StdVector&)>>)
		{
			return objectJointUserConstraintUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,const HomogeneousTransformation&,const HomogeneousTransformation&,const StdVector&)>>)
		{
			return objectJointUserJacobianUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<Real(const MainSystem&,Real,Index,Real)>>)
		{
			return objectConnectorCoordinateOffsetUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,bool)>>)
		{
			return objectConnectorCoordinateVectorConstraintUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,bool)>>)
		{
			return objectConnectorCoordinateVectorJacobianUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector3D(const MainSystem&,Real,Index,Real,Real,StdVector3D,StdVector3D,Real)>>)
		{
			return objectContactSphereTriangleContactForceUserFunction;
		}
        else if constexpr (std::is_same_v<UFT, std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,Real,StdVector3D)>>)
		{
			return objectJournalBearingForceUserFunction;
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
        else if constexpr (std::is_same_v<UFT, std::function<StdVector(const MainSystem&,Real,StdArrayIndex,const StdVector&,ConfigurationType)>>)
		{
			return sensorUserFunctionSensorUserFunction;
		}

		return UFT(0); //will never happen, but avoids errors/warnings
    }


