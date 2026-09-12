/** ***********************************************************************************************
* @class        PyConfig
* @brief		class for global config which can be accessed in Python
*
* @author		Gerstmayr Johannes
* @date			2025-05-21 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
*
************************************************************************************************ */
#ifndef PYCONFIG__H
#define PYCONFIG__H


class ExudynConfig
{
public: 
    ExudynConfig()
    {
        Initialize();
    }

    void Initialize()
    {
        //initialize variables here
    }

    void SetOutputPrecision(Index precision) { PySetOutputPrecision(precision); }
    Index GetOutputPrecision() const { return PyGetOutputPrecision(); }
    
    //link to global flags defined/available in Pybind_manual_classes.cpp:
    void SetLinalgPrintUsePythonFormat(Index flag) { linalgPrintUsePythonFormat = flag; }
    Index GetLinalgPrintUsePythonFormat() const { return linalgPrintUsePythonFormat; }
    void SetSuppressWarnings(Index flag) { suppressWarnings = flag; } //global flag
    Index GetSuppressWarnings() const { return suppressWarnings; }

    //! links to global outputBuffer
    void SetPrintDelayMilliSeconds(Index delayMilliSeconds) { outputBuffer.SetDelayMilliSeconds(delayMilliSeconds); }
    Index GetPrintDelayMilliSeconds() const { return outputBuffer.GetDelayMilliSeconds(); }

    void SetFlushAlways(bool flag) { outputBuffer.SetFlushAlways(flag); }
    bool GetFlushAlways() const { return outputBuffer.GetFlushAlways(); }

    bool GetWriteToFile() const { return outputBuffer.GetWriteToFile(); }
    std::string GetFileName() const { return outputBuffer.GetFileName(); }
    bool GetWriteAppend() const { return outputBuffer.GetWriteAppend(); }

    void SetWriteToConsole(bool flag) { outputBuffer.SetWriteToConsole(flag); }
    bool GetWriteToConsole() const { return outputBuffer.GetWriteToConsole(); }


    //functions
    STDstring Version(bool addDetails = false) const { return GetExudynBuildVersionString(addDetails);  };

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! to get representation:
    virtual void Print(std::ostream& os) const
    {
        os << "  Version() = " << GetExudynBuildVersionString(true) << "\n";
        os << "  suppressWarnings = " << GetSuppressWarnings() << "\n";
        os << "  outputPrecision = " << GetOutputPrecision() << "\n";
        os << "  linalgOutputFormatPython = " << GetLinalgPrintUsePythonFormat() << "\n";
        os << "  printDelayMilliSeconds = " << GetPrintDelayMilliSeconds() << "\n";
        os << "  printFlushAlways = " << GetFlushAlways() << "\n";
        os << "  printToConsole = " << GetWriteToConsole() << "\n";
        os << "  printToFile = " << GetWriteToFile() << "\n";
        os << "  printFileName = " << GetFileName() << "\n";
        os << "  printToFileAppend = " << GetWriteAppend() << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const ExudynConfig& item)
    {
        item.Print(os);
        return os;
    }

};


#endif //PYCONFIG__H

