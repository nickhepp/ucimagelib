#include "image.h"
#include "imagefuncs.h"
#include "gdalexception.h"

#include "exception.h"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()

#include <linux/types.h>
#include <inttypes.h>
#include <limits>
#include <assert.h>


namespace UC {


bool Image::mAllRegistered = false;


Image::Image(GDALDataset *srcDset,
        double srcMin,
        double srcMax,
        const char *fileName,
        GDALDataType dType,
        const char *format,
        char **options,
        double min,
        double max,
        bool destroyOnDelete)
{
    registerCheck();
    mDestroyDsetOnDelete = destroyOnDelete;
    mErrorOnDataTypeConversion = true;

    mDset = UC::Image::convertImage(
                srcDset, srcMin, srcMax,
                fileName,
                dType, format, options,
                min, max);
}


Image::Image(GDALDataset *srcDset,
        const char *fileName,
        GDALDataType dType,
        const char *format,
        bool destroyOnDelete)
{
    long srcMin, srcMax;
    long min, max;
    GDALDataType srcDType = getDataType(srcDset);

    registerCheck();
    mDestroyDsetOnDelete = destroyOnDelete;
    mErrorOnDataTypeConversion = true;

    getIntGDALDataTypeExtrema(srcDType, &srcMin, &srcMax);
    getIntGDALDataTypeExtrema(dType, &min, &max);

    mDset = UC::Image::convertImage(
                srcDset, (double)srcMin, (double)srcMax,
                fileName,
                dType, format, NULL,
                (double)min, (double)max);
}




Image::Image(GDALDataset *dSet, bool destroyOnDelete) :
    mDset(dSet), mDestroyDsetOnDelete(destroyOnDelete)
{
    registerCheck();
    mErrorOnDataTypeConversion = true;
}


Image::Image(const char *fileName, bool destroyOnDelete)
{
    registerCheck();
    mDset = (GDALDataset *)GDALOpen(fileName, GA_Update);
    if (mDset == NULL) {
        QString msg = "Error opening '%1' image.";
        throw GDALException(msg.arg(fileName));
    }
    mDestroyDsetOnDelete = destroyOnDelete;
    mErrorOnDataTypeConversion = true;
}


Image::Image(const char *format,
             const char *fileName,
             int xSize, int ySize,
             int rasterBands,
             GDALDataType dType,
             char **options,
             bool destroyOnDelete)
{
    GDALDriver *driver;

    registerCheck();
    driver = GetGDALDriverManager()->GetDriverByName(format);
    if (driver == NULL) {
        QString msg = "Error getting '%1' driver.";
        throw GDALException(msg.arg(format));
    }

    mDset = driver->Create(fileName,
                   xSize, ySize,
                   rasterBands,
                   dType,
                   options);

    if (mDset == NULL) {
        QString msg = "Error opening '%1' image.";
        throw GDALException(msg.arg(fileName));
    }
    mDestroyDsetOnDelete = destroyOnDelete;
    mErrorOnDataTypeConversion = true;
}


void Image::registerCheck()
{
    if (!mAllRegistered) {
        GDALAllRegister();
        mAllRegistered = true;
    }
}


bool Image::getErrorOnDataTypeConversions()
{
    return mErrorOnDataTypeConversion;
}


void Image::setErrorOnDataTypeConversions(bool err)
{
    mErrorOnDataTypeConversion = err;
}


void Image::dataTypeConversionCheck(GDALDataType testType)
{
    if (testType != getDataType(mDset) && mErrorOnDataTypeConversion) {
        throw UC::Exception("Performing a datatype conversion but the "
                            "ErrorOnDataTypeConversion is set.");
    }
}


GDALDataset * Image::getGDALDataset()
{
    return mDset;
}


void Image::verifyRasterBandDataType(GDALRasterBand *band, GDALDataType dType)
{
    if (band->GetRasterDataType() != dType) {
        QString msg = "Expected raster band data type '%1' but found '%2'.";
        msg = msg.arg(GDALGetDataTypeName(dType)).arg(band->GetRasterDataType());
        throw UC::Exception(msg);
    }
}


void Image::readGDTByteImageSamples(unsigned char **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_Byte);
    UC::ImageFuncs<unsigned char>::readImageSamples(mDset, GDT_Byte, true, outBuff, outBuffSize);
}


void Image::readGDTUInt16ImageSamples(uint16_t **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_UInt16);
    UC::ImageFuncs<uint16_t>::readImageSamples(mDset, GDT_UInt16, true, outBuff, outBuffSize);
}


void Image::readGDTInt16ImageSamples(int16_t **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_Int16);
    UC::ImageFuncs<int16_t>::readImageSamples(mDset, GDT_Int16, true, outBuff, outBuffSize);
}


void Image::readGDTUInt32ImageSamples(uint32_t **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_UInt32);
    UC::ImageFuncs<uint32_t>::readImageSamples(mDset, GDT_UInt32, true, outBuff, outBuffSize);
}


void Image::readGDTInt32ImageSamples(int32_t **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_Int32);
    UC::ImageFuncs<int32_t>::readImageSamples(mDset, GDT_Int32, true, outBuff, outBuffSize);
}


void Image::readGDTFloat32ImageSamples(float **outBuff,
                            long *outBuffSize)
{
    assert(sizeof(float) == 4);
    dataTypeConversionCheck(GDT_Float32);
    UC::ImageFuncs<float>::readImageSamples(mDset, GDT_Float32, true, outBuff, outBuffSize);
}


void Image::readGDTFloat64ImageSamples(double **outBuff,
                            long *outBuffSize)
{
    assert(sizeof(double) == 8);
    dataTypeConversionCheck(GDT_Float64);
    UC::ImageFuncs<double>::readImageSamples(mDset, GDT_Float64, true, outBuff, outBuffSize);
}


void Image::writeGDTByteImageSamples(unsigned char *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_Byte);
    UC::ImageFuncs<unsigned char>::writeImageSamples(mDset, GDT_Byte, true, outBuff, outBuffSize);
}


void Image::writeGDTUInt16ImageSamples(uint16_t *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_UInt16);
    UC::ImageFuncs<uint16_t>::writeImageSamples(mDset, GDT_UInt16, true, outBuff, outBuffSize);
}


void Image::writeGDTInt16ImageSamples(int16_t *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_Int16);
    UC::ImageFuncs<int16_t>::writeImageSamples(mDset, GDT_Int16, true, outBuff, outBuffSize);
}


void Image::writeGDTUInt32ImageSamples(uint32_t *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_UInt32);
    UC::ImageFuncs<uint32_t>::writeImageSamples(mDset, GDT_UInt32, true, outBuff, outBuffSize);
}


void Image::writeGDTInt32ImageSamples(int32_t *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_Int32);
    UC::ImageFuncs<int32_t>::writeImageSamples(mDset, GDT_Int32, true, outBuff, outBuffSize);
}


void Image::writeGDTFloat32ImageSamples(float *outBuff,
                            long outBuffSize)
{
    assert(sizeof(float) == 4);
    dataTypeConversionCheck(GDT_Float32);
    UC::ImageFuncs<float>::writeImageSamples(mDset, GDT_Float32, true, outBuff, outBuffSize);
}


void Image::writeGDTFloat64ImageSamples(double *outBuff,
                            long outBuffSize)
{
    assert(sizeof(double) == 8);
    dataTypeConversionCheck(GDT_Float64);
    UC::ImageFuncs<double>::writeImageSamples(mDset, GDT_Float64, true, outBuff, outBuffSize);
}


//////////////////////////////////////////////////
// read<GDALType>ImagePixel
//////////////////////////////////////////////////

void Image::readGDTByteImagePixel(int xOff, int yOff,
                           unsigned char *outBuff,
                           int rasterCount)
{
    unsigned char **buffPtr = &outBuff;
    long longRasterCount = rasterCount;
    dataTypeConversionCheck(GDT_Byte);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<unsigned char>::readImageRegion(
                                mDset,
                                GDT_Byte,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                buffPtr, &longRasterCount); //T *outBuff, long outBuffSize)
}


void Image::readGDTUInt16ImagePixel(int xOff, int yOff,
                             uint16_t *outBuff,
                             int rasterCount)
{
    uint16_t **buffPtr = &outBuff;
    long longRasterCount = rasterCount;
    dataTypeConversionCheck(GDT_UInt16);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<uint16_t>::readImageRegion(
                                mDset,
                                GDT_UInt16,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                buffPtr, &longRasterCount); //T *outBuff, long outBuffSize)
}


void Image::readGDTInt16ImagePixel(int xOff, int yOff,
                            int16_t *outBuff,
                            int rasterCount)
{
    int16_t **buffPtr = &outBuff;
    long longRasterCount = rasterCount;
    dataTypeConversionCheck(GDT_Int16);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<int16_t>::readImageRegion(
                                mDset,
                                GDT_Int16,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                buffPtr, &longRasterCount); //T *outBuff, long outBuffSize)
}


void Image::readGDTUInt32ImagePixel(int xOff, int yOff,
                             uint32_t *outBuff,
                             int rasterCount)
{
    uint32_t **buffPtr = &outBuff;
    long longRasterCount = rasterCount;
    dataTypeConversionCheck(GDT_UInt32);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<uint32_t>::readImageRegion(
                                mDset,
                                GDT_UInt32,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                buffPtr, &longRasterCount); //T *outBuff, long outBuffSize)
}


void Image::readGDTInt32ImagePixel(int xOff, int yOff,
                            int32_t *outBuff,
                            int rasterCount)
{
    int32_t **buffPtr = &outBuff;
    long longRasterCount = rasterCount;
    dataTypeConversionCheck(GDT_Int32);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<int32_t>::readImageRegion(
                                mDset,
                                GDT_Int32,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                buffPtr, &longRasterCount); //T *outBuff, long outBuffSize)
}


void Image::readGDTFloat32ImagePixel(int xOff, int yOff,
                              float *outBuff,
                              int rasterCount)
{
    float **buffPtr = &outBuff;
    long longRasterCount = rasterCount;
    assert(sizeof(float) == 4);
    dataTypeConversionCheck(GDT_Float32);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<float>::readImageRegion(
                                mDset,
                                GDT_Float32,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                buffPtr, &longRasterCount); //T *outBuff, long outBuffSize)
}


void Image::readGDTFloat64ImagePixel(int xOff, int yOff,
                              double *outBuff,
                              int rasterCount)
{
    double **buffPtr = &outBuff;
    long longRasterCount = rasterCount;
    assert(sizeof(float) == 8);
    dataTypeConversionCheck(GDT_Float64);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<double>::readImageRegion(
                                mDset,
                                GDT_Float64,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                buffPtr, &longRasterCount); //T *outBuff, long outBuffSize)
}


//////////////////////////////////////////////////
// write<GDALType>ImagePixel
//////////////////////////////////////////////////

void Image::writeGDTByteImagePixel(int xOff, int yOff,
                            unsigned char *outBuff,
                            int rasterCount)
{
    dataTypeConversionCheck(GDT_Byte);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<unsigned char>::writeImageRegion(
                                mDset,
                                GDT_Byte,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                outBuff, rasterCount); //T *outBuff, long outBuffSize)
}


void Image::writeGDTUInt16ImagePixel(int xOff, int yOff,
                              uint16_t *outBuff,
                              int rasterCount)
{
    dataTypeConversionCheck(GDT_UInt16);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<uint16_t>::writeImageRegion(
                                mDset,
                                GDT_UInt16,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                outBuff, rasterCount); //T *outBuff, long outBuffSize)
}


void Image::writeGDTInt16ImagePixel(int xOff, int yOff,
                             int16_t *outBuff,
                             int rasterCount)
{
    dataTypeConversionCheck(GDT_Int16);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<int16_t>::writeImageRegion(
                                mDset,
                                GDT_Int16,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                outBuff, rasterCount); //T *outBuff, long outBuffSize)
}


void Image::writeGDTUInt32ImagePixel(int xOff, int yOff,
                              uint32_t *outBuff,
                              int rasterCount)
{
    dataTypeConversionCheck(GDT_UInt32);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<uint32_t>::writeImageRegion(
                                mDset,
                                GDT_UInt32,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                outBuff, rasterCount); //T *outBuff, long outBuffSize)
}


void Image::writeGDTInt32ImagePixel(int xOff, int yOff,
                             int32_t *outBuff,
                             int rasterCount)
{
    dataTypeConversionCheck(GDT_Int32);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<int32_t>::writeImageRegion(
                                mDset,
                                GDT_Int32,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                outBuff, rasterCount); //T *outBuff, long outBuffSize)
}


void Image::writeGDTFloat32ImagePixel(int xOff, int yOff,
                               float *outBuff,
                               int rasterCount)
{
    assert(sizeof(float) == 4);
    dataTypeConversionCheck(GDT_Float32);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<float>::writeImageRegion(
                                mDset,
                                GDT_Float32,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                outBuff, rasterCount); //T *outBuff, long outBuffSize)
}


void Image::writeGDTFloat64ImagePixel(int xOff, int yOff,
                               double *outBuff,
                               int rasterCount)
{
    assert(sizeof(float) == 8);
    dataTypeConversionCheck(GDT_Float64);
    verifyRasterBandCount(mDset, rasterCount);
    ImageFuncs<double>::writeImageRegion(
                                mDset,
                                GDT_Float64,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                1, 1,           // int  	xSize, ySize,
                                1, 1,           // int  	bufXSize, bufYSize,
                                outBuff, rasterCount); //T *outBuff, long outBuffSize)
}


//////////////////////////////////////////////////
// read<GDALType>ImageRegion
//////////////////////////////////////////////////


void Image::readGDTByteImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            unsigned char **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_Byte);
    ImageFuncs<unsigned char>::readImageRegion(
                                mDset,
                                GDT_Byte,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTUInt16ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            uint16_t **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_UInt16);
    ImageFuncs<uint16_t>::readImageRegion(
                                mDset,
                                GDT_UInt16,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTInt16ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            int16_t **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_Int16);
    ImageFuncs<int16_t>::readImageRegion(
                                mDset,
                                GDT_Int16,      // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTUInt32ImageRegion(int xOff, int yOff,
                              int xSize, int ySize,
                             uint32_t **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_UInt32);
    ImageFuncs<uint32_t>::readImageRegion(
                                mDset,
                                GDT_UInt32,     // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTInt32ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            int32_t **outBuff,
                            long *outBuffSize)
{
    dataTypeConversionCheck(GDT_Int32);
    ImageFuncs<int32_t>::readImageRegion(
                                mDset,
                                GDT_Int32,      // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTFloat32ImageRegion(int xOff, int yOff,
                               int xSize, int ySize,
                              float **outBuff,
                            long *outBuffSize)
{
    assert(sizeof(float) == 4);
    dataTypeConversionCheck(GDT_Float32);
    ImageFuncs<float>::readImageRegion(
                                mDset,
                                GDT_Float32,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTFloat64ImageRegion(int xOff, int yOff,
                               int xSize, int ySize,
                              double **outBuff,
                            long *outBuffSize)
{
    assert(sizeof(double) == 8);
    dataTypeConversionCheck(GDT_Float64);
    ImageFuncs<double>::readImageRegion(
                                mDset,
                                GDT_Float64,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


//////////////////////////////////////////////////
// write<GDALType>ImageRegion
//////////////////////////////////////////////////

void Image::writeGDTByteImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            unsigned char *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_Byte);
    ImageFuncs<unsigned char>::writeImageRegion(
                                mDset,
                                GDT_Byte,       // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T *outBuff, long outBuffSize)
}


void Image::writeGDTUInt16ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            uint16_t *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_UInt16);
    ImageFuncs<uint16_t>::writeImageRegion(
                                mDset,
                                GDT_UInt16,     // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T *outBuff, long outBuffSize)
}


void Image::writeGDTInt16ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            int16_t *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_Int16);
    ImageFuncs<int16_t>::writeImageRegion(
                                mDset,
                                GDT_Int16,      // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T *outBuff, long outBuffSize)
}


void Image::writeGDTUInt32ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            uint32_t *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_UInt32);
    ImageFuncs<uint32_t>::writeImageRegion(
                                mDset,
                                GDT_UInt32,     // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T *outBuff, long outBuffSize)
}


void Image::writeGDTInt32ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            int32_t *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_Int32);
    ImageFuncs<int32_t>::writeImageRegion(
                                mDset,
                                GDT_Int32,      // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T *outBuff, long outBuffSize)
}


void Image::writeGDTFloat32ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            float *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_Float32);
    ImageFuncs<float>::writeImageRegion(
                                mDset,
                                GDT_Float32,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T *outBuff, long outBuffSize)
}


void Image::writeGDTFloat64ImageRegion(int xOff, int yOff,
                            int xSize, int ySize,
                            double *outBuff,
                            long outBuffSize)
{
    dataTypeConversionCheck(GDT_Float64);
    ImageFuncs<double>::writeImageRegion(
                                mDset,
                                GDT_Float64,    // GDALDataType dType,
                                xOff,           // int  	xOff,
                                yOff,           // int  	yOff,
                                xSize, ySize,   // int  	xSize, ySize,
                                xSize, ySize,   // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T *outBuff, long outBuffSize)
}


//////////////////////////////////////////////////
// read<GDALType>ImageBlock
//////////////////////////////////////////////////


void Image::readGDTByteImageBlock(int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize,
                            unsigned char **outBuff,
                            long *outBuffSize)
{
    getBlockCoordinates(xIdx, yIdx,
                            xOff, yOff,
                            xSize, ySize);

    dataTypeConversionCheck(GDT_Byte);
    ImageFuncs<unsigned char>::readImageRegion(
                                mDset,
                                GDT_Byte,       // GDALDataType dType,
                                *xOff,          // int  	xOff,
                                *yOff,          // int  	yOff,
                                *xSize, *ySize, // int  	xSize, ySize,
                                *xSize, *ySize, // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTUInt16ImageBlock(int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize,
                            uint16_t **outBuff,
                            long *outBuffSize)
{
    getBlockCoordinates(xIdx, yIdx,
                            xOff, yOff,
                            xSize, ySize);

    dataTypeConversionCheck(GDT_UInt16);
    ImageFuncs<uint16_t>::readImageRegion(
                                mDset,
                                GDT_UInt16,       // GDALDataType dType,
                                *xOff,          // int  	xOff,
                                *yOff,          // int  	yOff,
                                *xSize, *ySize, // int  	xSize, ySize,
                                *xSize, *ySize, // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTInt16ImageBlock(int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize,
                            int16_t **outBuff,
                            long *outBuffSize)
{
    getBlockCoordinates(xIdx, yIdx,
                            xOff, yOff,
                            xSize, ySize);

    dataTypeConversionCheck(GDT_Int16);
    ImageFuncs<int16_t>::readImageRegion(
                                mDset,
                                GDT_Int16,       // GDALDataType dType,
                                *xOff,          // int  	xOff,
                                *yOff,          // int  	yOff,
                                *xSize, *ySize, // int  	xSize, ySize,
                                *xSize, *ySize, // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTUInt32ImageBlock(int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize,
                            uint32_t **outBuff,
                            long *outBuffSize)
{
    getBlockCoordinates(xIdx, yIdx,
                            xOff, yOff,
                            xSize, ySize);

    dataTypeConversionCheck(GDT_UInt32);
    ImageFuncs<uint32_t>::readImageRegion(
                                mDset,
                                GDT_UInt32,       // GDALDataType dType,
                                *xOff,          // int  	xOff,
                                *yOff,          // int  	yOff,
                                *xSize, *ySize, // int  	xSize, ySize,
                                *xSize, *ySize, // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTInt32ImageBlock(int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize,
                            int32_t **outBuff,
                            long *outBuffSize)
{
    getBlockCoordinates(xIdx, yIdx,
                            xOff, yOff,
                            xSize, ySize);

    dataTypeConversionCheck(GDT_Int32);
    ImageFuncs<int32_t>::readImageRegion(
                                mDset,
                                GDT_Int32,       // GDALDataType dType,
                                *xOff,          // int  	xOff,
                                *yOff,          // int  	yOff,
                                *xSize, *ySize, // int  	xSize, ySize,
                                *xSize, *ySize, // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTFloat32ImageBlock(int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize,
                            float **outBuff,
                            long *outBuffSize)
{
    getBlockCoordinates(xIdx, yIdx,
                            xOff, yOff,
                            xSize, ySize);

    dataTypeConversionCheck(GDT_Float32);
    ImageFuncs<float>::readImageRegion(
                                mDset,
                                GDT_Float32,    // GDALDataType dType,
                                *xOff,          // int  	xOff,
                                *yOff,          // int  	yOff,
                                *xSize, *ySize, // int  	xSize, ySize,
                                *xSize, *ySize, // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


void Image::readGDTFloat64ImageBlock(int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize,
                            double **outBuff,
                            long *outBuffSize)
{
    getBlockCoordinates(xIdx, yIdx,
                            xOff, yOff,
                            xSize, ySize);

    dataTypeConversionCheck(GDT_Float64);
    ImageFuncs<double>::readImageRegion(
                                mDset,
                                GDT_Float64,       // GDALDataType dType,
                                *xOff,          // int  	xOff,
                                *yOff,          // int  	yOff,
                                *xSize, *ySize, // int  	xSize, ySize,
                                *xSize, *ySize, // int  	bufXSize, bufYSize,
                                outBuff, outBuffSize); //T **outBuff, long *outBuffSize)
}


GDALDataType Image::getDataType()
{
    return getDataType(mDset);
}


void Image::getBlockCoordinates(int xIdx, int yIdx,
                        int *xOff, int *yOff,
                        int *xSize, int *ySize)
{
    getBlockCoordinates(xIdx, yIdx,
                        xOff, yOff,
                        xSize, ySize);

}


void Image::getBlocksCount(int *wide, int *high)
{
    getBlocksCount(mDset, wide, high);
}


void Image::getBlockSize(int *xSize, int *ySize)
{
    getBlockSize(mDset, xSize, ySize);
}


void Image::getBlockCoordinates(GDALDataset *dset,
                        int xIdx, int yIdx,
                        int *xOff, int *yOff,
                        int *xSize, int *ySize)
{
    int wide, high;
    int xFullSize, yFullSize;

    getBlocksCount(dset, &wide, &high);
    getBlockSize(dset, &xFullSize, &yFullSize);

    if (xIdx < 0 || wide <= xIdx ||
            yIdx < 0 || high <= yIdx) {
        throw UC::Exception("Invalid block index.");
    }

    *xOff = xIdx * xFullSize;
    *yOff = yIdx * yFullSize;
    *xSize = dset->GetRasterXSize() - *xOff;
    *xSize = (*xSize > xFullSize) ? xFullSize : *xSize;
    *ySize = dset->GetRasterYSize() - *yOff;
    *ySize = (*ySize > yFullSize) ? yFullSize : *ySize;
}


void Image::getBlocksCount(GDALDataset *dset,
                         int *wide, int *high)
{
    int xSize, ySize;

    getBlockSize(dset, &xSize, &ySize);

    *wide = (dset->GetRasterXSize() + xSize - 1) / xSize;
    *high = (dset->GetRasterYSize() + ySize - 1) / ySize;
}


void Image::getBlockSize(GDALDataset *dset,
                         int *xSize, int *ySize)
{
    GDALRasterBand *band;

    verifySameBlockSize(dset);
    band = dset->GetRasterBand(1);
    band->GetBlockSize(xSize, ySize);
}


void Image::verifyRasterBandCount(GDALDataset *dset, int testCount)
{
    if (dset->GetRasterCount() != testCount) {
        throw UC::Exception("Raster band counts do not match.");
    }
}


void Image::verifySameBlockSize(GDALDataset *dset)
{
    int xSize, ySize, testXSize, testYSize;
    int k;
    GDALRasterBand *bandn, *band1;

    // assumed that all raster bands are same datatype
    band1 = dset->GetRasterBand(1);
    band1->GetBlockSize(&xSize, &ySize);
    for (k = 2; k <= dset->GetRasterCount(); k++) {
        bandn = dset->GetRasterBand(k);
        bandn->GetBlockSize(&testXSize, &testYSize);
        if (xSize != testXSize || ySize != testYSize) {
            throw UC::Exception("Not all raster bands have the same block size.");
        }
    }

    // made it through, no errors
}


GDALDataType Image::getDataType(GDALDataset *dset)
{
    GDALDataType dType;
    verifySameDataType(dset);
    dType = dset->GetRasterBand(1)->GetRasterDataType();
    return dType;
}


void Image::verifySameDataType(GDALDataset *dset)
{
    int k;
    GDALRasterBand *bandn, *band1;

    // assumed that all raster bands are same datatype
    band1 = dset->GetRasterBand(1);
    for (k = 2; k <= dset->GetRasterCount(); k++) {
        bandn = dset->GetRasterBand(k);
        if (band1->GetRasterDataType() != bandn->GetRasterDataType()) {
            throw UC::Exception("Not all raster bands have the same data type.");
        }
    }

    // made it through, no errors
}


void Image::verifyDataType(GDALDataset *dset, GDALDataType dType)
{
    GDALRasterBand *band1;

    Image::verifySameDataType(dset);
    band1 = dset->GetRasterBand(1);
    if (band1->GetRasterDataType() != dType) {
        QString msg = "Expected data type '%1' but found '%2'.";
        msg = msg.arg(GDALGetDataTypeName(dType))
                .arg(GDALGetDataTypeName(band1->GetRasterDataType()));
        throw UC::Exception(msg);
    }

    // made it through, no errors
}


void Image::verifySameSizeBands(GDALDataset *dset)
{
    int k;
    GDALRasterBand *bandn, *band1;

    // assumed that all raster bands are same size
    band1 = dset->GetRasterBand(1);
    for (k = 2; k <= dset->GetRasterCount(); k++) {
        bandn = dset->GetRasterBand(k);
        if (band1->GetXSize() != bandn->GetXSize() ||
                band1->GetYSize() != bandn->GetYSize()) {
            throw UC::Exception("Not all raster bands have the same size.");
        }
    }

    // made it through, no errors
}


long Image::getRasterAreaSize(GDALDataset *dset)
{
    long areaSz;
    GDALRasterBand *band1;

    Image::verifySameSizeBands(dset);
    band1 = dset->GetRasterBand(1);
    areaSz = band1->GetXSize() * band1->GetYSize();
    return areaSz;
}


long Image::getRasterBandSamplesCount(GDALDataset *dset, int nBandId)
{
    long areaSz;
    GDALRasterBand *band;

    band = dset->GetRasterBand(nBandId);
    areaSz = band->GetXSize() * band->GetYSize();
    return areaSz;
}


void Image::verifyRasterBandSamplesCount(GDALDataset *dset, int nBandId, long testSz)
{
    if (testSz < getRasterBandSamplesCount(dset, nBandId))
        throw UC::Exception("Raster band has more samples than test size.");
}


GDALDataset * Image::convertImage(GDALDataset *srcDset,
        double srcMin,
        double srcMax,
        const char *fileName,
        GDALDataType dType,
        const char *format,
        char **options,
        double min,
        double max)
{
    GDALDataset *dset;

    switch (dType) {
    case GDT_Byte:
        dset = ImageFuncs<unsigned char>::convertImage(
                    srcDset, srcMin, srcMax,
                    fileName,
                    dType, format, options,
                    (unsigned char)min, (unsigned char)max);
        break;
    case GDT_UInt16:
        dset = ImageFuncs<uint16_t>::convertImage(
                    srcDset, srcMin, srcMax,
                    fileName,
                    dType, format, options,
                    (uint16_t)min, (uint16_t)max);
        break;
    case GDT_Int16:
        dset = ImageFuncs<int16_t>::convertImage(
                    srcDset, srcMin, srcMax,
                    fileName,
                    dType, format, options,
                    (int16_t)min, (int16_t)max);
        break;
    case GDT_UInt32:
        dset = ImageFuncs<uint32_t>::convertImage(
                    srcDset, srcMin, srcMax,
                    fileName,
                    dType, format, options,
                    (uint32_t)min, (uint32_t)max);
        break;
    case GDT_Int32:
        dset = ImageFuncs<int32_t>::convertImage(
                    srcDset, srcMin, srcMax,
                    fileName,
                    dType, format, options,
                    (int32_t)min, (int32_t)max);
        break;
    case GDT_Float32:
        dset = ImageFuncs<float>::convertImage(
                    srcDset, srcMin, srcMax,
                    fileName,
                    dType, format, options,
                    (float)min, (float)max);
        break;
    case GDT_Float64:
        dset = ImageFuncs<double>::convertImage(
                    srcDset, srcMin, srcMax,
                    fileName,
                    dType, format, options,
                    (double)min, (double)max);
        break;
    default:
        throw UC::Exception("Unexpected data type.");
        break;
    }

    return dset;
}



void Image::getRasterBandSamples(GDALDataset *dset,
                           int nBandId,
                           double **outBuff,
                           long *outBuffSize)
{
    bool created;
    double *ptr;
    CPLErr err;

    //verifyRasterBandDataType(dset->GetRasterBand(nBandId), GDT_Byte);

    created = false;
    if (*outBuff == NULL) {
        created = true;
        *outBuffSize = getRasterBandSamplesCount(dset, nBandId);
        *outBuff = (double *)CPLMalloc(sizeof(double) * *outBuffSize);
    } else {
        // preallocated buffer, make sure it's big nuf
        verifyRasterBandSamplesCount(dset, nBandId, *outBuffSize);
    }
    ptr = *outBuff;
    //memset(ptr, 0, (*outBuffSize) * sizeof(double));

    err = dset->GetRasterBand(nBandId)->RasterIO(GF_Read, //GDALRWFlag eRWFlag,
                               0,0,                       //int nXOff, nYOff
                               dset->GetRasterXSize(),    //int 	nXSize
                               dset->GetRasterYSize(),    //int 	nYSize
                               (void *)ptr,               //void * 	pData
                               dset->GetRasterXSize(),    //int 	nBufXSize
                               dset->GetRasterYSize(),    //int 	nBufYSize
                               getDoubleGDTFloatType(),   //GDALDataType 	eBufType
                               0, 0); //int nPixelSpace, nLineSpace
    if (err != CE_None) {
        if (created) {
            CPLFree(ptr);
            *outBuff = NULL;
        }
        throw UC::GDALException("Error reading raster band with RasterIO().");
    }
}


void Image::getRasterBandSamples(GDALDataset *dset,
                           int nBandId,
                           unsigned char **outBuff,
                           long *outBuffSize)
{
    bool created;
    unsigned char *ptr;
    CPLErr err;

    verifyRasterBandDataType(dset->GetRasterBand(nBandId), GDT_Byte);

    created = false;
    if (*outBuff == NULL) {
        created = true;
        *outBuffSize = getRasterBandSamplesCount(dset, nBandId);
        *outBuff = (unsigned char *)CPLMalloc(*outBuffSize);
    } else {
        // preallocated buffer, make sure it's big nuf
        verifyRasterBandSamplesCount(dset, nBandId, *outBuffSize);
    }
    ptr = *outBuff;
    //memset(ptr, 0, (*outBuffSize) * 1);

    err = dset->GetRasterBand(nBandId)->RasterIO(GF_Read, //GDALRWFlag eRWFlag,
                               0,0,                       //int nXOff, nYOff
                               dset->GetRasterXSize(),    //int 	nXSize
                               dset->GetRasterYSize(),    //int 	nYSize
                               (void *)ptr,               //void * 	pData
                               dset->GetRasterXSize(),    //int 	nBufXSize
                               dset->GetRasterYSize(),    //int 	nBufYSize
                               GDT_Byte,                  //GDALDataType 	eBufType
                               0, 0); //int nPixelSpace, nLineSpace
    if (err != CE_None) {
        if (created) {
            CPLFree(ptr);
            *outBuff = NULL;
        }
        throw UC::GDALException("Error reading raster band with RasterIO().");
    }
}


long Image::getImageSamplesCount(GDALDataset *dset)
{
    long sz, areaSz;

    areaSz = Image::getRasterAreaSize(dset);
    sz = areaSz * dset->GetRasterCount();
    return sz;
}


void Image::verifyImageSamplesCount(GDALDataset *dset, long testSz)
{
    if (testSz > getImageSamplesCount(dset)) {
        throw UC::Exception("Image has more samples than test size.");
    }

    // made it through, no errors
}


void Image::getIntGDALDataTypeExtrema(GDALDataType dType, long *min, long *max)
{
    switch (dType) {
    case GDT_Byte:
        *min = std::numeric_limits<unsigned char>::min();
        *max = std::numeric_limits<unsigned char>::max();
        break;
    case GDT_UInt16:
        *min = std::numeric_limits<uint16_t>::min();
        *max = std::numeric_limits<uint16_t>::max();
        break;
    case GDT_Int16:
        *min = std::numeric_limits<int16_t>::min();
        *max = std::numeric_limits<int16_t>::max();
        break;
    case GDT_UInt32:
        *min = std::numeric_limits<uint32_t>::min();
        *max = std::numeric_limits<uint32_t>::max();
        break;
    case GDT_Int32:
        *min = std::numeric_limits<int32_t>::min();
        *max = std::numeric_limits<int32_t>::max();
        break;
    case GDT_Float32:
    case GDT_Float64:
    default:
        throw UC::Exception("Unexpected GDALDataType.");
    // do you really know the bounds of a float or double??
    //case GDT_Float32:
    //    *min = std::numeric_limits<float>::min();
    //    *max = std::numeric_limits<float>::max();
    //    break;
    //case GDT_Float64:
    //    *min = std::numeric_limits<double>::min();
    //    *max = std::numeric_limits<double>::max();
    //    break;
    }
}


GDALDataType Image::getDoubleGDTFloatType()
{
    GDALDataType dType;
    if (sizeof(double) == 8) {
        dType = GDT_Float64;
    } else {
        Q_ASSERT(sizeof(double) == 4);
        dType = GDT_Float32;
    }
    return dType;
}


void Image::freeImageSamples(void *buff)
{
    CPLFree(buff);
}


void Image::freeDptrImageSamples(void **buff)
{
    CPLFree(*buff);
    *buff = NULL;
}


void Image::remapSamples(void *inBuff, void *outBuff,
                         int remapType,
                         int dataTypeSize,
                         int samplesWidth, int samplesHeight,
                         int rasterBandCount)
{
    int inIdx, outIdx;
    int idx0, idx1, idx2;
    int idx0Max, idx1Max, idx2Max;
    int *a_in, *b_in, *c_in, *d_in, *e_in, *f_in;
    int *a_out, *b_out, *c_out, *d_out, *e_out, *f_out;

    // in MATLAB:
    // buff[idx] = (r * sampleWidth * sampleHeight) + (x * sampleHeight) + y
    // in most others
    // buff[idx] = (rasterBandCount * sampleWidth * y) + (x * rasterBandCount) + r
    // both are of form
    // buff[idx] = (a * b * c) + (d * e) + f

    if (remapType == REMAP_PXY_YXR) {
        a_in = &rasterBandCount;
        b_in = &samplesWidth;
        c_in = &idx2;
        d_in = &idx1;
        e_in = &rasterBandCount;
        f_in = &idx0;

        idx0Max = rasterBandCount;
        idx1Max = samplesWidth;
        idx2Max = samplesHeight;

        a_out = &idx0;
        b_out = &samplesWidth;
        c_out = &samplesHeight;
        d_out = &idx1;
        e_out = &samplesHeight;
        f_out = &idx2;

    } else if (remapType == REMAP_YXR_PXY) {
        a_out = &rasterBandCount;
        b_out = &samplesWidth;
        c_out = &idx2;
        d_out = &idx1;
        e_out = &rasterBandCount;
        f_out = &idx0;

        idx0Max = rasterBandCount;
        idx1Max = samplesWidth;
        idx2Max = samplesHeight;

        a_in = &idx0;
        b_in = &samplesWidth;
        c_in = &samplesHeight;
        d_in = &idx1;
        e_in = &samplesHeight;
        f_in = &idx2;

    } else {
        throw UC::Exception("Unrecognized remap type.");
    }

    for (idx2 = 0; idx2 < idx2Max; idx2++) {
        for (idx1 = 0; idx1 < idx1Max; idx1++) {
            for (idx0 = 0; idx0 < idx0Max; idx0++) {
                inIdx = ((*a_in) * (*b_in) * (*c_in)) +
                        ((*d_in) * (*e_in)) +
                        (*f_in);
                outIdx = ((*a_out) * (*b_out) * (*c_out)) +
                        ((*d_out) * (*e_out)) +
                        (*f_out);
                memcpy((char *)(outBuff) + (inIdx * dataTypeSize),
                       (char *)(inBuff) + (outIdx * dataTypeSize),
                       dataTypeSize);
            }

        }

    }
}


Image::~Image()
{
    if (mDestroyDsetOnDelete) {
        if (mDset) {
            delete(mDset);
        }
    }

}


}



// below is old code that has been replaced by imagefuncs<T>

/*


void Image::getImageSamples(GDALDataset *dset,
                            uint16_t **outBuff,
                            long *outBuffSize)
{
    imageSamplesIO(dset, GF_Read, outBuff,outBuffSize);
}


void Image::setImageSamples(GDALDataset *dset,
                            uint16_t *outBuff,
                            long outBuffSize)
{
    imageSamplesIO(dset, GF_Write, &outBuff,&outBuffSize);
}


void Image::imageSamplesIO(GDALDataset *dset,
                           GDALRWFlag rwFlag,
                           uint16_t **outBuff,
                           long *outBuffSize)
{
    bool created;
    uint16_t *ptr;
    CPLErr err;

    verifyDataType(dset, GDT_UInt16);

    created = false;
    if (*outBuff == NULL) {
        if (rwFlag == GF_Write) {
            throw UC::Exception("A buffer must be supplied when performing a write.");
        }
        created = true;
        *outBuffSize = getImageSamplesCount(dset);
        *outBuff = (uint16_t *)CPLMalloc(sizeof(uint16_t) * getImageSamplesCount(dset));
    } else {
        // preallocated buffer, make sure it's big nuf
        verifyImageSamplesCount(dset, *outBuffSize);
    }
    ptr = *outBuff;
    //memset(ptr, 0, (*outBuffSize) * 2);

    for (int r = 1; r <= dset->GetRasterCount(); r++) {
        err = dset->GetRasterBand(r)->RasterIO(rwFlag,                   //GDALRWFlag 	eRWFlag,
                             0,0,                       //int nXOff,    nYOff
                             dset->GetRasterXSize(),    //int 	nXSize
                             dset->GetRasterYSize(),    //int 	nYSize
                             ptr + r - 1, //void * 	pData
                             dset->GetRasterXSize(),    //int 	nBufXSize
                             dset->GetRasterYSize(),    //int 	nBufYSize
                             GDT_UInt16,                //GDALDataType 	eBufType
                             dset->GetRasterCount() * sizeof(uint16_t),    //int 	nPixelSpace,
                             //int 	nLineSpace,
                             dset->GetRasterXSize() * dset->GetRasterCount() * sizeof(uint16_t));
        if (err != CE_None) {
            if (created) {
                CPLFree(ptr);
                *outBuff = NULL;
            }
            throw UC::GDALException("Error reading data set with RasterIO().");
        }
    }
}


void Image::getImageSamples(GDALDataset *dset,
                           unsigned char **outBuff,
                           long *outBuffSize)
{
    bool created;
    unsigned char *ptr;
    CPLErr err;

    verifyDataType(dset, GDT_Byte);

    created = false;
    if (*outBuff == NULL) {
        created = true;
        *outBuffSize = getImageSamplesCount(dset);
        *outBuff = (unsigned char *)CPLMalloc(sizeof(unsigned char) * getImageSamplesCount(dset));
    } else {
        // preallocated buffer, make sure it's big nuf
        verifyImageSamplesCount(dset, *outBuffSize);
    }
    ptr = *outBuff;
    //memset(ptr, 0, (*outBuffSize) * 1);

    for (int r = 1; r <= dset->GetRasterCount(); r++) {
        err = dset->GetRasterBand(r)->RasterIO(GF_Read,                   //GDALRWFlag 	eRWFlag,
                             0,0,                       //int nXOff,    nYOff
                             dset->GetRasterXSize(),    //int 	nXSize
                             dset->GetRasterYSize(),    //int 	nYSize
                             ptr + r - 1,               //void * 	pData
                             dset->GetRasterXSize(),    //int 	nBufXSize
                             dset->GetRasterYSize(),    //int 	nBufYSize
                             GDT_Byte,                  //GDALDataType 	eBufType
                             dset->GetRasterCount(),    //int 	nPixelSpace,
                             dset->GetRasterXSize() * dset->GetRasterCount());                         //int 	nLineSpace,
        if (err != CE_None) {
            if (created) {
                CPLFree(ptr);
                *outBuff = NULL;
            }
            throw UC::GDALException("Error reading data set with RasterIO().");
        }


    }
}


void Image::getImageSamples(GDALDataset *dset,
                           double **outBuff,
                           long *outBuffSize)
{
    bool created;
    double *ptr;
    CPLErr err;

    //verifyDataType(dset, GDT_Byte);

    created = false;
    if (*outBuff == NULL) {
        created = true;
        *outBuffSize = getImageSamplesCount(dset);
        *outBuff = (double *)CPLMalloc(sizeof(double) * getImageSamplesCount(dset));
    } else {
        // preallocated buffer, make sure it's big nuf
        verifyImageSamplesCount(dset, *outBuffSize);
    }
    ptr = *outBuff;
    //memset(ptr, 0, (*outBuffSize) * sizeof(double));

    for (int r = 1; r <= dset->GetRasterCount(); r++) {
        err = dset->GetRasterBand(r)->RasterIO(GF_Read,                   //GDALRWFlag 	eRWFlag,
                             0,0,                       //int nXOff,    nYOff
                             dset->GetRasterXSize(),    //int 	nXSize
                             dset->GetRasterYSize(),    //int 	nYSize
                             ptr + r - 1,               //void * 	pData
                             dset->GetRasterXSize(),    //int 	nBufXSize
                             dset->GetRasterYSize(),    //int 	nBufYSize
                             getDoubleGDTFloatType(),   //GDALDataType 	eBufType
                             dset->GetRasterCount() * sizeof(double),    //int 	nPixelSpace,
                             //int 	nLineSpace,
                             dset->GetRasterXSize() * dset->GetRasterCount() * sizeof(double));
        if (err != CE_None) {
            if (created) {
                CPLFree(ptr);
                *outBuff = NULL;
            }
            throw UC::GDALException("Error reading data set with RasterIO().");
        }
    }
}
*/
