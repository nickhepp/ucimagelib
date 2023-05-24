#include "gdalexception.h"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()


namespace UC {

  
/**
 *  \brief Creates an exception that captures a string describing the error
 *  		as well as a GDAL error number.
 *  
 *  \param [in] msg Message describing the error.
 *  \param [in] resetError Whether or not calling this cstor should reset
 *  		the GDAL error.
 */ 
GDALException::GDALException(const QString& msg, bool resetError)
:Exception()
{
    QString cmbndMsg = "%1. CPLError: 0x%2, %3";
    mMessage = cmbndMsg.arg(msg).arg(CPLGetLastErrorNo()).arg(CPLGetLastErrorMsg());
    mInnerEx = NULL;
    if (resetError)
        CPLErrorReset();
}


/**
 *  \brief Creates an exception that captures a string describing the error
 *  		as well as a GDAL error number.
 *  
 *  \param [in] msg Message describing the error.
 *  \param [in] innerEx An inner exception that bubbled up to cause this error.
 *  \param [in] resetError Whether or not calling this cstor should reset
 *  		the GDAL error.
 */
GDALException::GDALException(const QString& msg, Exception *innerEx, bool resetError)
:Exception()
{
    QString cmbndMsg = "%1. CPLError: 0x%2, %3";
    mMessage = cmbndMsg.arg(msg).arg(CPLGetLastErrorNo()).arg(CPLGetLastErrorMsg());
    mInnerEx = innerEx;
    if (resetError)
        CPLErrorReset();
}


}

