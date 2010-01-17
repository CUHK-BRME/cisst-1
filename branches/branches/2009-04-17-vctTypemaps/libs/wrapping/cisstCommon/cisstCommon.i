/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: cisstCommon.i,v 1.21 2008/06/04 02:25:12 anton Exp $

  Author(s):	Anton Deguet
  Created on:   2004-10-06

  (C) Copyright 2004-2007 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/


%include "std_string.i"
%include "std_vector.i"
%include "std_map.i"
%include "std_pair.i"

%module cisstCommonPython

%include "std_streambuf.i"
%include "std_iostream.i"

// Force insertion of code related to Python iterators in cxx file
// Version 1.3.37 removed symbols starting with Py 
#if (SWIG_VERSION > 0x010336)
  %fragment("SwigPyIterator_T");
#else
  %fragment("PySwigIterator_T");
  #define SwigPyIterator PySwigIterator  
#endif

// We define __setitem__ and __getitem__
%ignore *::operator[];


%include "swigrun.i"

%header %{
    // Put header files here
    #include "cisstCommon/cisstCommon.i.h"
%}

// Generate parameter documentation for IRE
%feature("autodoc", "1");

%rename (__str__) *::ToString;
%ignore *::ToStream;
%ignore *::ToStreamRaw;
%ignore *::FromStreamRaw;
%ignore *::Serialize;
%ignore *::DeSerialize;
%ignore *::SerializeRaw;
%ignore *::DeSerializeRaw;


%ignore operator<<;

#define CISST_EXPORT
#define CISST_DEPRECATED
#define CMN_UNUSED(a) a

%ignore CMN_LOG_DETAILS;

// Typemap used so that C++ pointers on cmnGenericObject base class
// can be "casted" to the actual derived class in Python.  This is
// useful for objects stored in cmnObjectRegister.  Without the
// typemap, the user ends-up with a base class object and no derived
// feature whatsoever.
%typemap(out) cmnGenericObject * {
    if (result != NULL) {
        // create a string with a trailing "*" to retrieve the SWIG pointer type info
        std::string className = result->Services()->GetName() + " *";
        swig_type_info* typeInfo = SWIG_TypeQuery(className.c_str());
        // if the type info exists, i.e. this class has been wrapped, convert pointer
        if (typeInfo != NULL) {
            resultobj = SWIG_NewPointerObj((void*)(result), typeInfo, $owner | %newpointer_flags);
        } else {
            // fail, maybe a better fall back would be to return the base type, but this is really useless
            char buffer[256];
            sprintf(buffer, "Sorry, can't create a python object of type %s",
            className.c_str());
            PyErr_SetString(PyExc_TypeError, buffer);
            SWIG_fail;
        }
    } else {
        // Return None if object not found
        Py_INCREF(Py_None);
        resultobj = Py_None;
    }
}

// Wrap the generic object class
%include "cisstCommon/cmnGenericObject.h"
%import "cisstCommon/cmnAccessorMacros.h"

// Wrap the class register and add required code to generate python iterators
%newobject cmnClassServicesBase::Create;
%include "cisstCommon/cmnClassServicesBase.h"
%newobject cmnClassRegister::Create;
%include "cisstCommon/cmnClassRegister.h"
%include "cisstCommon/cmnClassRegisterMacros.h"
%template() std::pair<std::string, cmnClassServicesBase*>;
%template(cmnClassServicesContainer) std::map<std::string, cmnClassServicesBase*>;
%apply std::map<std::string, cmnClassServicesBase*>::const_iterator { 
    cmnClassRegister::const_iterator
};
%newobject cmnClassRegister::iterator(PyObject **PYTHON_SELF);
%extend cmnClassRegister {
    swig::SwigPyIterator* iterator(PyObject **PYTHON_SELF) {
        return swig::make_output_iterator(self->begin(), self->begin(), self->end(), *PYTHON_SELF);
    }
    %pythoncode {
        def __iter__(self):
            return self.iterator()
    }
}


// Wrap the object register and add required code to generate python iterators
%include "cisstCommon/cmnObjectRegister.h"
%template() std::pair<std::string, cmnGenericObject*>;
%template(cmnGenericObjectContainer) std::map<std::string, cmnGenericObject*>;
%apply std::map<std::string, cmnGenericObject*>::const_iterator { 
    cmnObjectRegister::const_iterator
};
%newobject cmnObjectRegister::iterator(PyObject **PYTHON_SELF);
%extend cmnObjectRegister {
    swig::SwigPyIterator* iterator(PyObject **PYTHON_SELF) {
        return swig::make_output_iterator(self->begin(), self->begin(), self->end(), *PYTHON_SELF);
    }
    %pythoncode {
        def __iter__(self):
            return self.iterator()
    }
}

// Wrap some basic types
%include "cisstCommon/cmnGenericObjectProxy.h"
%define CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(name, elementType)
// Instantiate the template
%template(name) cmnGenericObjectProxy<elementType>;
// Type addition for dynamic type checking
%{
    typedef cmnGenericObjectProxy<elementType> name;
%}
typedef cmnGenericObjectProxy<elementType> name;
%types(name *);
%enddef

CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(cmnDouble, double);
CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(cmnInt, int);
CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(cmnUInt, unsigned int);
CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(cmnShort, short);
CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(cmnUShort, unsigned short);
CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(cmnLong, long);
CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(cmnULong, unsigned long);
CMN_GENERIC_OBJECT_PROXY_INSTANTIATE(cmnBool, bool);


// Wrap stream code related to the logging system
%template() std::basic_streambuf<char,std::char_traits<char > >;
%include "cisstCommon/cmnLODMultiplexerStreambuf.h"
%template(cmnLODMultiplexerStreambufChar) cmnLODMultiplexerStreambuf<char>;
%include "cisstCommon/cmnCallbackStreambuf.h"
%template(cmnCallbackStreambufChar) cmnCallbackStreambuf<char>;
%include "cisstCommon/cmnLogger.h"

// Wrap cmnPath
%include "cisstCommon/cmnPath.h"

// Wrap and instantiate useful type traits
%include "cisstCommon/cmnTypeTraits.h"
%template(cmnTypeTraitsDouble) cmnTypeTraits<double>;
%template(cmnTypeTraitsInt) cmnTypeTraits<int>;

