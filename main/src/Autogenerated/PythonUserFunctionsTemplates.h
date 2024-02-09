    //include file for PythonUserFunctionsTemplates
    //author: Johannes Gerstmayr
    //license: see Exudyn license
    //AUTO:
//included into SymbolicUtilities.h
//templates require explicit instantiations for each template because otherwise linker problems ... (alternative: include everything into .h files ...)
template class PythonUserFunctionBase<std::function<bool(const MainSystem&,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector2D(const MainSystem&,Real)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Index)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,Real,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,StdVector)>>;
template class PythonUserFunctionBase<std::function<NumpyMatrix(const MainSystem&,Real,Index,StdVector,StdVector)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)>>;
template class PythonUserFunctionBase<std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Index,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector,bool)>>;
template class PythonUserFunctionBase<std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,bool)>>;
template class PythonUserFunctionBase<std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)>>;
template class PythonUserFunctionBase<std::function<StdVector3D(const MainSystem&,Real,StdVector3D)>>;
template class PythonUserFunctionBase<std::function<Real(const MainSystem&,Real,Real)>>;
template class PythonUserFunctionBase<std::function<StdVector(const MainSystem&,Real,StdArrayIndex,StdVector,ConfigurationType)>>;

