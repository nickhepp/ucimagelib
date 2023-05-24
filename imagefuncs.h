#ifndef IMAGEFUNCS_H
#define IMAGEFUNCS_H

#include "ucimagelib_global.h"

#include "image.h"
#include "gdalexception.h"

#include <inttypes.h>

#include "gdal.h"
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()

#include "exception.h"

class GDALDataset;
class GDALRasterBand;


namespace UC {


template<typename T>
class ImageFuncs
{
public:

    static GDALDataset * convertImage(GDALDataset *srcDset,
                                double srcMin,
                                double srcMax,
                                const char *fileName,
                                GDALDataType dType,
                                const char *format,
                                char **options,
                                T outMin,
                                T outMax)
    {
        GDALDriver *driver;
        double *srcSamples = NULL;
        double **srcSamplesDptr = &srcSamples;
        long srcSampleCount;
        GDALDataset *dset;
        T *outBuff;
        double outMinDbl = outMin;
        double outMaxDbl = outMax;
        double sampleDbl;

        // band number, size, type, geotransform
        driver = GetGDALDriverManager()->GetDriverByName(format);
        if (driver == NULL) {
            QString msg = "Error getting GDALDriver '%1'.";
            throw GDALException(msg.arg(format));
        }

        dset = driver->Create(fileName,
                       srcDset->GetRasterXSize(),
                       srcDset->GetRasterYSize(),
                       srcDset->GetRasterCount(),
                       dType,
                       options);
        if (dset == NULL) {
            QString msg = "Error creating new GDALDataset '%1'.";
            throw GDALException(msg.arg(fileName));
        }

        // read the samples from the source
        srcSamples = NULL;
        ImageFuncs<double>::readImageSamples(srcDset,
                         GDT_Float64,
                         false,
                         srcSamplesDptr,
                         &srcSampleCount);
        outBuff = (T *)CPLMalloc(srcSampleCount * sizeof(T));
        if (!outBuff) {
            Image::freeImageSamples(srcSamples);
            throw UC::Exception("Error allocating new sample buffer.");
        }

        // scale the samples to the new range
        for (int k = 0; k < srcSampleCount; k++) {
            // normalize to 0 to 1
            sampleDbl = (srcSamples[k] - srcMin) / (srcMax - srcMin);
            // adjust to new scale
            sampleDbl = outMinDbl + (sampleDbl * (outMaxDbl - outMinDbl));
            outBuff[k] = (T)sampleDbl;
        }
        Image::freeImageSamples(srcSamples);

        // write image samples
        try {
            ImageFuncs<T>::writeImageSamples(dset, dType, true, outBuff, srcSampleCount);
            Image::freeImageSamples(outBuff);
        } catch (...) {
            Image::freeImageSamples(outBuff);
            throw;
        }

        // set projection and metadata
        dset->SetProjection(srcDset->GetProjectionRef());
        dset->SetMetadata(srcDset->GetMetadata());

        return dset;
    }


	/**
	 *  \brief Gets the samples of a data set.
	 *  
	 *  \param [in] dset Data set that contains the samples.
	 *  \param [in,out] outBuff Buffer that will be populated with the raster
	 *  		band's samples.
	 *  		On input, set *outBuff = NULL to have the function allocate a buffer.
	 *  		On input, if *outBuff != NULL then the buffer is expected to be preallocated
	 *  		with outBuffSize number samples which should be large enough to contain the
	 *  		entire contents of the raster band.
	 *  \param [in,out] outBuffSize On input, if *outBuff == NULL the input value of *outBuffSize
	 *  		is ignored and the return value is the number of bytes
	 *  		(sizeof(sample) * number_of_samples) that were allocated for the raster band buffer.
	 *  		On input, if *outBuff != NULL then the buffer was preallocated and *outBuffSize
	 *  		contains the number of bytes for *outBuff.  In this case, the buffer must be large
	 *  		enough to contain the samples of the raster band, and the return value of
	 *  		*outBuffSize is the same as it's input value.
	 *  
	 *  \details Samples are interleaved by pixels first, columns next (x), and then by row (y).
	 *  		Allocations that occur within this function happen via CPLMalloc().
	 */	
    static void readImageSamples(GDALDataset *dset,
                                GDALDataType dType,
                                bool dataTypeMustMatch,
                                T **outBuff,
                                long *outBuffSize)
    {
        ioImageSamples(dset, dType, dataTypeMustMatch, GF_Read, outBuff, outBuffSize);
    }

	
	/**
	 *  \brief Sets the samples of a data set.
	 *  
	 *  \param [in] dset Data set to write the samples.
	 *  \param [in] outBuff Buffer that will populate the raster
	 *  		band's samples.
	 *  \param [in] outBuffSize The number of bytes in the outBuff.
	 *  
	 *  \details Samples are interleaved by pixels first, columns next (x), and then by row (y).
	 */		
    static void writeImageSamples(GDALDataset *dset,
                                GDALDataType dType,
                                bool dataTypeMustMatch,
                                T *outBuff,
                                long outBuffSize)
    {
        ioImageSamples(dset, dType, dataTypeMustMatch, GF_Write, outBuff, outBuffSize);
    }

	
	/**
	 *  \brief Gets the samples of a raster band.
	 *  
	 *  \param [in] dset Data set that contains the raster band.
	 *  \param [in] nBandId Raster band number (1 based for GDAL).
     *  \param [in] dType Data type of the data set.
	 *  \param [in,out] outBuff Buffer that will be populated with the raster
	 *  		band's samples.
	 *  		On input, set *outBuff = NULL to have the function allocate a buffer.
	 *  		On input, if *outBuff != NULL then the buffer is expected to be preallocated
	 *  		with outBuffSize number samples which should be large enough to contain the
	 *  		entire contents of the raster band.
	 *  \param [in,out] outBuffSize On input, if *outBuff == NULL the input value of *outBuffSize
	 *  		is ignored and the return value is the number of bytes
	 *  		(sizeof(sample) * number_of_samples) that were allocated for the raster band buffer.
	 *  		On input, if *outBuff != NULL then the buffer was preallocated and *outBuffSize
	 *  		contains the number of bytes for *outBuff.  In this case, the buffer must be large
	 *  		enough to contain the samples of the raster band, and the return value of
	 *  		*outBuffSize is the same as it's input value.
	 *  
	 *  \details Samples are interleaved by column (x) first and then by row (y). Allocations that
	 *  		occur within this function happen via CPLMalloc().
	 */	
    static void readRasterBandSamples(GDALDataset *dset,
                                int nBandId,
                                GDALDataType dType,
                                T **outBuff,
                                long *outBuffSize)
    {
        ioRasterBandSamples(dset, nBandId, dType, GF_Read, outBuff, outBuffSize);
    }
	

	/**
	 *  \brief Sets the samples of a raster band.
	 *  
	 *  \param [in] dset Data set that contains the raster band.
	 *  \param [in] nBandId Raster band number (1 based for GDAL).
     *  \param [in] dType Data type of the data set.
	 *  \param [in] outBuff Buffer that will populate the raster
	 *  		band's samples.
	 *  \param [in] outBuffSize The number of bytes in the outBuff.
	 *  
	 *  \details Samples are interleaved by pixels first, columns next (x), and then by row (y).
	 */		
    static void writeRasterBandSamples(GDALDataset *dset,
                                int nBandId,
                                GDALDataType dType,
                                T *outBuff,
                                long outBuffSize)
    {
        ioRasterBandSamples(dset, nBandId, dType, GF_Write, &outBuff, &outBuffSize);
    }




    static void readImageRegion(GDALDataset *dset,
                                GDALDataType dType,
                                int  	xOff,
                                int  	yOff,
                                int  	xSize,
                                int  	ySize,
                                int  	bufXSize,
                                int  	bufYSize,
                                T **outBuff,
                                long *outBuffSize)
    {
        ioImageRegion(dset, dType, GF_Read,
                        xOff, yOff,
                        xSize, ySize,
                        bufXSize, bufYSize,
                        outBuff, outBuffSize);
    }


    static void writeImageRegion(GDALDataset *dset,
                                 GDALDataType dType,
                                 int  	xOff,
                                 int  	yOff,
                                 int  	xSize,
                                 int  	ySize,
                                 int  	bufXSize,
                                 int  	bufYSize,
                                 T *outBuff,
                                 long outBuffSize)
    {
        ioImageRegion(dset, dType, GF_Write,
                        xOff, yOff,
                        xSize, ySize,
                        bufXSize, bufYSize,
                        outBuff, outBuffSize);
    }


private:	
	
	
    static void ioRasterBandSamples(GDALDataset *dset,
                               int nBandId,
                               GDALDataType dType,
                               GDALRWFlag rwFlag,
                               T **outBuff,
                               long *outBuffSize)
    {
        bool created;
        T *ptr;
        CPLErr err;

        UC::Image::verifyRasterBandDataType(dset->GetRasterBand(nBandId), dType);

        created = false;
        if (*outBuff == NULL) {
            if (rwFlag == GF_Write) {
                throw UC::Exception("A buffer must be supplied when performing a write.");
            }
            created = true;
            *outBuffSize = UC::Image::getRasterBandSamplesCount(dset, nBandId);
            *outBuff = (T *)CPLMalloc(sizeof(T) * (*outBuffSize));
        } else {
            // preallocated buffer, make sure it's big nuf
            UC::Image::verifyRasterBandSamplesCount(dset, nBandId, *outBuffSize);
        }
        ptr = *outBuff;
        //memset(ptr, 0, (*outBuffSize) * sizeof(T));

        err = dset->GetRasterBand(nBandId)->RasterIO(rwFlag, //GDALRWFlag eRWFlag,
                                   0,0,                       //int nXOff, nYOff
                                   dset->GetRasterXSize(),    //int 	nXSize
                                   dset->GetRasterYSize(),    //int 	nYSize
                                   (void *)ptr,               //void * 	pData
                                   dset->GetRasterXSize(),    //int 	nBufXSize
                                   dset->GetRasterYSize(),    //int 	nBufYSize
                                   dType,                     //GDALDataType 	eBufType
                                   0, //int nPixelSpace
                                   0); // nLineSpace
        if (err != CE_None) {
            if (created) {
                CPLFree(ptr);
                *outBuff = NULL;
            }
            throw UC::GDALException("Error reading raster band with RasterIO().");
        }
    }

	
    static void ioImageSamples(GDALDataset *dset,
                               GDALDataType dType,
                               bool dataTypeMustMatch,
                               GDALRWFlag rwFlag,
                               T **outBuff,
                               long *outBuffSize)
    {
        bool created;
        T *ptr;
        int dtypeSz = sizeof(T);

        if (dataTypeMustMatch) {
            Image::verifyDataType(dset, dType);
        }

        created = false;
        if (*outBuff == NULL) {
            if (rwFlag == GF_Write) {
                throw UC::Exception("A buffer must be supplied when performing a write.");
            }
            created = true;
            *outBuffSize = Image::getImageSamplesCount(dset);
            *outBuff = (T *)CPLMalloc(dtypeSz * (*outBuffSize));
        } else {
            // preallocated buffer, make sure it's big nuf
            Image::verifyImageSamplesCount(dset, *outBuffSize);
        }
        ptr = *outBuff;

        try {
            ioImageSamples(dset, dType, dataTypeMustMatch,
                           rwFlag,
                           ptr, *outBuffSize);
        } catch (...) {
            if (created) {
                CPLFree(ptr);
                *outBuff = NULL;
            }
            throw;
        }

    }

	
    static void ioImageSamples(GDALDataset *dset,
                               GDALDataType dType,
                               bool dataTypeMustMatch,
                               GDALRWFlag rwFlag,
                               T *outBuff,
                               long outBuffSize)
    {
        T *ptr;
        CPLErr err;
        int dtypeSz = sizeof(T);

        if (dataTypeMustMatch) {
            Image::verifyDataType(dset, dType);
        }
        Image::verifyImageSamplesCount(dset, outBuffSize);

        ptr = outBuff;
        //memset(ptr, 0, outBuffSize * dtypeSz);

        for (int r = 1; r <= dset->GetRasterCount(); r++) {
            err = dset->GetRasterBand(r)->RasterIO(rwFlag,                   //GDALRWFlag 	eRWFlag,
                                 0,0,                       //int   nXOff,    nYOff
                                 dset->GetRasterXSize(),    //int 	nXSize
                                 dset->GetRasterYSize(),    //int 	nYSize
                                 ptr + r - 1,   //void  *pData
                                 dset->GetRasterXSize(),    //int 	nBufXSize
                                 dset->GetRasterYSize(),    //int 	nBufYSize
                                 dType,                     //GDALDataType 	eBufType
                                 dset->GetRasterCount() * dtypeSz,    //int 	nPixelSpace,
                                 //int 	nLineSpace,
                                 dset->GetRasterXSize() * dset->GetRasterCount() * dtypeSz);
            if (err != CE_None) {
                throw UC::GDALException("Error reading data set with RasterIO().");
            }
        }

    }


    static void ioImageRegion(GDALDataset *dset,
                              GDALDataType dType,
                              GDALRWFlag rwFlag,
                              int  	xOff,
                              int  	yOff,
                              int  	xSize,
                              int  	ySize,
                              int  	bufXSize,
                              int  	bufYSize,
                              T **outBuff,
                              long *outBuffSize)

    {
        bool created;
        T *ptr;
        int dtypeSz = sizeof(T);
        int sampleCnt;

        Image::verifyDataType(dset, dType);
        sampleCnt = xSize * ySize * dset->GetRasterCount();

        created = false;
        if (*outBuff == NULL) {
            if (rwFlag == GF_Write) {
                throw UC::Exception("A buffer must be supplied when performing a write.");
            }
            created = true;
            *outBuffSize = sampleCnt;
            *outBuff = (T *)CPLMalloc(dtypeSz * (*outBuffSize));
        } else {
            // preallocated buffer, make sure it's big nuf
            if (*outBuffSize < sampleCnt) {
                throw UC::Exception("Buffer is not large enough to read image region.");
            }
        }
        ptr = *outBuff;

        try {
            ioImageRegion(dset, dType,
                           rwFlag,
                           xOff, yOff,
                           xSize, ySize,
                           bufXSize, bufYSize,
                           outBuff, outBuffSize);
        } catch (...) {
            if (created) {
                CPLFree(ptr);
                *outBuff = NULL;
            }
            throw;
        }

    }


    static void ioImageRegion(GDALDataset *dset,
                               GDALDataType dType,
                               GDALRWFlag rwFlag,
                               int  	xOff,
                               int  	yOff,
                               int  	xSize,
                               int  	ySize,
                               int  	bufXSize,
                               int  	bufYSize,
                               T *outBuff,
                               long outBuffSize)
    {
        T *ptr;
        CPLErr err;
        int dtypeSz = sizeof(T);
        int sampleCnt;

        Image::verifyDataType(dset, dType);

        if (xOff < 0 || xOff >= dset->GetRasterXSize() ||
                yOff < 0 || yOff >= dset->GetRasterYSize() ||
                bufXSize > dset->GetRasterXSize() ||
                bufYSize + yOff >= dset->GetRasterYSize()) {
            throw UC::Exception("Dimensions are invalid.");
        }

        sampleCnt = dset->GetRasterCount() * bufXSize * bufYSize;
        if (sampleCnt > outBuffSize) {
            throw UC::Exception("Buffer is not large enough.");
        }

        ptr = outBuff;

        for (int r = 1; r <= dset->GetRasterCount(); r++) {
            err = dset->GetRasterBand(r)->RasterIO(rwFlag,                   //GDALRWFlag 	eRWFlag,
                            xOff,
                            yOff,
                            xSize,
                            ySize,
                            ptr + r - 1 + xOff * dset->GetRasterCount() +
                            yOff * dset->GetRasterXSize(),
                            bufXSize,
                            bufYSize,
                            dType,
                            dset->GetRasterCount() * dtypeSz,    //int 	nPixelSpace,
                                 //int 	nLineSpace,
                            dset->GetRasterXSize() * dset->GetRasterCount() * dtypeSz);
            if (err != CE_None) {
                throw UC::GDALException("Error reading data set with RasterIO().");
            }
        }
    }




    // Class is meant to be static functions, not called through an instance
    ImageFuncs();
};


}

#endif // IMAGEFUNCS_H




