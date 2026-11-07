    //include file for PythonUserFunctionsTemplates
    //author: Johannes Gerstmayr
    //license: see Exudyn license
    //AUTO:
//included into SymbolicUtilities.h
//templates require explicit instantiations for each template because otherwise linker problems ... (alternative: include everything into .h files ...)
template class PythonUserFunctionBase<std::function<bool(const MainSystem&,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector2D(const MainSystem&,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector3D(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Index)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,Real,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,bool,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,Real,Real,Real,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,OutputVariableType,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<NumpyMatrix(const MainSystem&,Real,Index,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)>>;
template class PythonUserFunctionBase<std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const Twist&,const HomogeneousTransformation&,const Twist&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<void(const MainSystem&,Real,Index,py::object,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const Twist&,const HomogeneousTransformation&,const Twist&,const StdVector&,const StdVector&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const HomogeneousTransformation&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,const HomogeneousTransformation&,const HomogeneousTransformation&,const StdVector&)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Index,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,bool)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,bool)>>;
template class PythonUserFunctionBase<std::function<StdVector3D(const MainSystem&,Real,Index,Real,Real,StdVector3D,StdVector3D,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,Real,StdVector3D)>>;
template class PythonUserFunctionBase<std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)>>;
template class PythonUserFunctionBase<std::function<StdVector3D(const MainSystem&,Real,StdVector3D)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,StdArrayIndex,const StdVector&,ConfigurationType)>>;
template class PythonUserFunctionBase<std::function<void(const MainSystem&, Real, Index, Index)>>;
template class PythonUserFunctionBase<std::function<PyMatrixContainer(const MainSystem&, Real, Real, Real, Real)>>;

