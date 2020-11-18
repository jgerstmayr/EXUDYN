/**************************************************************************/
/* File:   exception.cpp                                                  */
/* Author: Joachim Schoeberl                                              */
/* Date:   16. Jan. 02                                                    */
/**************************************************************************/

#include "Utilities/BasicDefinitions.h" //defines Real
#ifdef USE_NGSOLVE_TASKMANAGER

#include "ngs_core.hpp"

namespace ngstd
{
  /*
  Exception :: Exception ()
  { 
    ;
  }
  */
  static mutex printexception_mutex;

  Exception :: Exception (const string & s) 
    : m_what(s)
  { 
    {
      lock_guard<mutex> guard(printexception_mutex);
      // cout << "create ngstd::Exception, m_what = " << s << endl;
    }
  }

  Exception :: Exception (const char * s) 
    : m_what(s)
  { 
    {
      lock_guard<mutex> guard(printexception_mutex);
      // cout << "create ngstd::Exception, m_what = " << s << endl;
    }
  }

  Exception :: ~Exception () 
  { ; }

  Exception & Exception :: Append (const string & s)
  { 
    m_what += s; 
    return *this;
  }

  Exception & Exception :: Append (const char * s)
  { 
    m_what += s; 
    return *this;
  }




  RangeException :: RangeException (const string & where, 
				    int ind, int imin, int imax)
    : Exception ("")
  {
    stringstream str;
    str << where << ": index " << ind << " out of range [" << imin << "," << imax << "]\n";
    Append (str.str());
  }
}
#endif //USE_NGSOLVE_TASKMANAGER
