#ifndef GDALEXCEPTION_H
#define GDALEXCEPTION_H

#include "ucimagelib_global.h"

#include "exception.h"


namespace UC {

class IMAGELIBSHARED_EXPORT GDALException : public UC::Exception
{
public:
    GDALException(const QString& msg, bool resetError = true);

    GDALException(const QString& msg, Exception *innerEx, bool resetError = true);
};


}

#endif // GDALEXCEPTION_H
