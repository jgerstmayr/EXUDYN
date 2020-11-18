/**************************************************************************/
/* File:   bitarray.cc                                                    */
/* Autho: Joachim Schoeberl                                              */
/* Date:   01. Jun. 95                                                    */
/**************************************************************************/

/* 
   data type BitArray
*/
#include "Utilities/BasicDefinitions.h" //defines Real
#ifdef USE_NGSOLVE_TASKMANAGER

#include "ngs_core.hpp"

namespace ngstd
{
  BitArray :: BitArray ()
  {
    size = 0;
    data = NULL;
  }

  BitArray :: BitArray (size_t asize)
  {
    size = 0;
    data = NULL;
    SetSize (asize);
  }

  BitArray :: BitArray (size_t asize, LocalHeap & lh)
  {
    size = asize;
    data = new (lh) unsigned char [Addr (size)+1];
    owns_data = false;
  }
  
  BitArray :: BitArray (const BitArray & ba2)
  {
    size = 0;
    data = NULL;
    (*this) = ba2;
  }

  BitArray :: ~BitArray ()
  {
    if (owns_data)
      delete [] data;
  }

  void BitArray :: SetSize (size_t asize)
  {
    if (size == asize) return;
    if (owns_data) delete [] data;

    size = asize;
    data = new unsigned char [Addr (size)+1];
  }

  void BitArray :: Set () throw()
  {
    if (!size) return;
    for (size_t i = 0; i <= Addr (size); i++)
      data[i] = UCHAR_MAX;
  }

  void BitArray :: Clear () throw()
  {
    if (!size) return;
    for (size_t i = 0; i <= Addr (size); i++)
      data[i] = 0;
  }



  BitArray & BitArray :: Invert ()
  {
    if (!size) return *this;
    for (size_t i = 0; i <= Addr (size); i++)
      data[i] ^= 255;
    return *this;
  }

  BitArray & BitArray :: And (const BitArray & ba2)
  {
    if (!size) return *this;
    for (size_t i = 0; i <= Addr (size); i++)
      data[i] &= ba2.data[i];
        return *this;
  }


  BitArray & BitArray :: Or (const BitArray & ba2)
  {
    if (!size) return *this;
    for (size_t i = 0; i <= Addr (size); i++)
      data[i] |= ba2.data[i];
    return *this;
  }


  BitArray & BitArray :: operator= (const BitArray & ba2)
  {
    SetSize (ba2.Size());
    if (!size) 
      return *this;
    for (size_t i = 0; i <= Addr (size); i++)
      data[i] = ba2.data[i];
    return *this;
  }

  ostream & operator<<(ostream & s, const BitArray & ba)
  {
    size_t n = ba.Size();
    for (size_t i = 0; i < n; i++)
      {
	if (i % 50 == 0) s << i << ": ";
	s << int(ba[i]);
	if (i % 50 == 49) s << "\n";
      }
    s << flush;
    return s;
  }

  size_t BitArray :: NumSet () const
  {
    size_t cnt = 0;
    for (size_t i = 0; i < Size(); i++)
      if (Test(i)) cnt++;
    return cnt;
  }  
}

#endif //USE_NGSOLVE_TASKMANAGER
