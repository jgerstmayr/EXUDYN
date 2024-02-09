    //include file for PySymbolicUserFunctionTransfer
    //author: Johannes Gerstmayr
    //license: see Exudyn license
    //AUTO:
public:
    //collect all kinds of user functions
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

public:
    //! this function realizes the assignment of user function to item fully in C++, to avoid 2 x Python-casts!
    void TransferUserFunction2Item(MainSystem& mainSystem, py::object itemIndex, const STDstring& userFunctionName)
    {
		STDstring sType;
		STDstring itemTypeName;
        Index itemNumber;
		GetItemTypeName(mainSystem, itemIndex, sType, itemTypeName, itemNumber);
        //Index itemNumber = itemIndex.GetIndex();
        //STDstring sType = itemIndex.GetTypeString();
        //STDstring itemTypeName = GetItemTypeName(mainSystem, itemIndex);

        if (sType == "ObjectIndex")
        {
            CHECKandTHROW(itemNumber < mainSystem.GetCSystem().GetSystemData().GetCObjects().NumberOfItems(),
                          "Symbolic::TransferUserFunction2Item: illegal objectNumber");

            if (itemTypeName == "GenericODE2" && userFunctionName == "forceUserFunction")
            {
                CObjectGenericODE2* cItem = (CObjectGenericODE2*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().forceUserFunction.SetSymbolicUserFunction(vectorMbsScalarIndex2Vector);
            }
            else if (itemTypeName == "GenericODE1" && userFunctionName == "rhsUserFunction")
            {
                CObjectGenericODE1* cItem = (CObjectGenericODE1*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().rhsUserFunction.SetSymbolicUserFunction(vectorMbsScalarIndexVector);
            }
            else if (itemTypeName == "KinematicTree" && userFunctionName == "forceUserFunction")
            {
                CObjectKinematicTree* cItem = (CObjectKinematicTree*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().forceUserFunction.SetSymbolicUserFunction(vectorMbsScalarIndex2Vector);
            }
            else if (itemTypeName == "FFRF" && userFunctionName == "forceUserFunction")
            {
                CObjectFFRF* cItem = (CObjectFFRF*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().forceUserFunction.SetSymbolicUserFunction(vectorMbsScalarIndex2Vector);
            }
            else if (itemTypeName == "FFRFreducedOrder" && userFunctionName == "forceUserFunction")
            {
                CObjectFFRFreducedOrder* cItem = (CObjectFFRFreducedOrder*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().forceUserFunction.SetSymbolicUserFunction(vectorMbsScalarIndex2Vector);
            }
            else if (itemTypeName == "ANCFCable2D" && userFunctionName == "axialForceUserFunction")
            {
                CObjectANCFCable2D* cItem = (CObjectANCFCable2D*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().axialForceUserFunction.SetSymbolicUserFunction(mbsScalarIndexScalar9);
            }
            else if (itemTypeName == "ANCFCable2D" && userFunctionName == "bendingMomentUserFunction")
            {
                CObjectANCFCable2D* cItem = (CObjectANCFCable2D*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().bendingMomentUserFunction.SetSymbolicUserFunction(mbsScalarIndexScalar9);
            }
            else if (itemTypeName == "ConnectorSpringDamper" && userFunctionName == "springForceUserFunction")
            {
                CObjectConnectorSpringDamper* cItem = (CObjectConnectorSpringDamper*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().springForceUserFunction.SetSymbolicUserFunction(mbsScalarIndexScalar5);
            }
            else if (itemTypeName == "ConnectorCartesianSpringDamper" && userFunctionName == "springForceUserFunction")
            {
                CObjectConnectorCartesianSpringDamper* cItem = (CObjectConnectorCartesianSpringDamper*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().springForceUserFunction.SetSymbolicUserFunction(vector3DmbsScalarIndexScalar4Vector3D);
            }
            else if (itemTypeName == "ConnectorRigidBodySpringDamper" && userFunctionName == "springForceTorqueUserFunction")
            {
                CObjectConnectorRigidBodySpringDamper* cItem = (CObjectConnectorRigidBodySpringDamper*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().springForceTorqueUserFunction.SetSymbolicUserFunction(vector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D);
            }
            else if (itemTypeName == "ConnectorRigidBodySpringDamper" && userFunctionName == "postNewtonStepUserFunction")
            {
                CObjectConnectorRigidBodySpringDamper* cItem = (CObjectConnectorRigidBodySpringDamper*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().postNewtonStepUserFunction.SetSymbolicUserFunction(vectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D);
            }
            else if (itemTypeName == "ConnectorLinearSpringDamper" && userFunctionName == "springForceUserFunction")
            {
                CObjectConnectorLinearSpringDamper* cItem = (CObjectConnectorLinearSpringDamper*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().springForceUserFunction.SetSymbolicUserFunction(mbsScalarIndexScalar5);
            }
            else if (itemTypeName == "ConnectorTorsionalSpringDamper" && userFunctionName == "springTorqueUserFunction")
            {
                CObjectConnectorTorsionalSpringDamper* cItem = (CObjectConnectorTorsionalSpringDamper*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().springTorqueUserFunction.SetSymbolicUserFunction(mbsScalarIndexScalar5);
            }
            else if (itemTypeName == "ConnectorCoordinateSpringDamper" && userFunctionName == "springForceUserFunction")
            {
                CObjectConnectorCoordinateSpringDamper* cItem = (CObjectConnectorCoordinateSpringDamper*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().springForceUserFunction.SetSymbolicUserFunction(mbsScalarIndexScalar5);
            }
            else if (itemTypeName == "ConnectorCoordinateSpringDamperExt" && userFunctionName == "springForceUserFunction")
            {
                CObjectConnectorCoordinateSpringDamperExt* cItem = (CObjectConnectorCoordinateSpringDamperExt*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().springForceUserFunction.SetSymbolicUserFunction(mbsScalarIndexScalar11);
            }
            else if (itemTypeName == "ConnectorCoordinate" && userFunctionName == "offsetUserFunction")
            {
                CObjectConnectorCoordinate* cItem = (CObjectConnectorCoordinate*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().offsetUserFunction.SetSymbolicUserFunction(mbsScalarIndexScalar);
            }
            else if (itemTypeName == "ConnectorCoordinate" && userFunctionName == "offsetUserFunction_t")
            {
                CObjectConnectorCoordinate* cItem = (CObjectConnectorCoordinate*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().offsetUserFunction_t.SetSymbolicUserFunction(mbsScalarIndexScalar);
            }
            else if (itemTypeName == "ConnectorCoordinateVector" && userFunctionName == "constraintUserFunction")
            {
                CObjectConnectorCoordinateVector* cItem = (CObjectConnectorCoordinateVector*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().constraintUserFunction.SetSymbolicUserFunction(vectorMbsScalarIndex2VectorBool);
            }
            else if (itemTypeName == "JointGeneric" && userFunctionName == "offsetUserFunction")
            {
                CObjectJointGeneric* cItem = (CObjectJointGeneric*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().offsetUserFunction.SetSymbolicUserFunction(vector6DmbsScalarIndexVector6D);
            }
            else if (itemTypeName == "JointGeneric" && userFunctionName == "offsetUserFunction_t")
            {
                CObjectJointGeneric* cItem = (CObjectJointGeneric*)(mainSystem.GetCSystem().GetSystemData().GetCObjects()[itemNumber]);
                cItem->GetParameters().offsetUserFunction_t.SetSymbolicUserFunction(vector6DmbsScalarIndexVector6D);
            }

            else
            {
                PyError(STDstring("Symbolic::TransferUserFunction2Item<") + itemTypeName + "," + userFunctionName +
                        ">: invalid user object type or user function type; possibly, function is not available as symbolic user function");
            }
        }
        else if (sType == "LoadIndex")
        {
            CHECKandTHROW(itemNumber < mainSystem.GetCSystem().GetSystemData().GetCObjects().NumberOfItems(),
                          "Symbolic::TransferUserFunction2Item: illegal objectNumber");

            if (itemTypeName == "ForceVector" && userFunctionName == "loadVectorUserFunction")
            {
                CLoadForceVector* cItem = (CLoadForceVector*)(mainSystem.GetCSystem().GetSystemData().GetCLoads()[itemNumber]);
                cItem->GetParameters().loadVectorUserFunction.SetSymbolicUserFunction(vector3DmbsScalarVector3D);
            }
            else if (itemTypeName == "TorqueVector" && userFunctionName == "loadVectorUserFunction")
            {
                CLoadTorqueVector* cItem = (CLoadTorqueVector*)(mainSystem.GetCSystem().GetSystemData().GetCLoads()[itemNumber]);
                cItem->GetParameters().loadVectorUserFunction.SetSymbolicUserFunction(vector3DmbsScalarVector3D);
            }
            else if (itemTypeName == "MassProportional" && userFunctionName == "loadVectorUserFunction")
            {
                CLoadMassProportional* cItem = (CLoadMassProportional*)(mainSystem.GetCSystem().GetSystemData().GetCLoads()[itemNumber]);
                cItem->GetParameters().loadVectorUserFunction.SetSymbolicUserFunction(vector3DmbsScalarVector3D);
            }
            else if (itemTypeName == "Coordinate" && userFunctionName == "loadUserFunction")
            {
                CLoadCoordinate* cItem = (CLoadCoordinate*)(mainSystem.GetCSystem().GetSystemData().GetCLoads()[itemNumber]);
                cItem->GetParameters().loadUserFunction.SetSymbolicUserFunction(mbsScalar2);
            }

            else
            {
                PyError(STDstring("Symbolic::TransferUserFunction2Item<") + itemTypeName + "," + userFunctionName +
                        ">: invalid user object type or user function type; possibly, function is not available as symbolic user function");
            }
        }
        else
        {
            PyError(STDstring("Symbolic::TransferUserFunction2Item<") + itemTypeName + "," + userFunctionName + ">: invalid item type (must be Object or Load)");
        }
    }

