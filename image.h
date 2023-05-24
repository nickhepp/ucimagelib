#ifndef IMAGE_H
#define IMAGE_H

#include <inttypes.h>

#include "ucimagelib_global.h"

#include "gdal.h"


class GDALDataset;
class GDALRasterBand;
class GDALDriver;

// PXY: Pixel, x, then y
// YXR: y, x, then pixel raster bands
#define REMAP_PXY_YXR   (0)
#define REMAP_YXR_PXY   (1)


namespace UC {


class IMAGELIBSHARED_EXPORT Image
{
public:

    Image(GDALDataset *srcDset,
            double srcMin,
            double srcMax,
            const char *fileName,
            GDALDataType dType,
            const char *format,
            char **options,
            double min,
            double max,
            bool destroyOnDelete);

    Image(GDALDataset *srcDset,
            const char *fileName,
            GDALDataType dType,
            const char *format,
            bool destroyOnDelete);

    Image(GDALDataset *dSet, bool destroyOnDelete);

    Image(const char *fileName, bool destroyOnDelete);

    Image(const char *format,
                 const char *fileName,
                 int xSize, int ySize,
                 int rasterBands,
                 GDALDataType dType,
                 char **options,
                 bool destroyOnDelete);

    //Image(GDALDataset *dSet,
    //            const char *newFormat,
    //            const char *fileName,
    //            bool destroyOnDelete = true);

    virtual ~Image();

    GDALDataset * getGDALDataset();

    //////////////////////////////////////////////////
    // read<GDALType>ImageSamples
    //////////////////////////////////////////////////


    void readGDTByteImageSamples(unsigned char **outBuff,
                                long *outBuffSize);

    void readGDTUInt16ImageSamples(uint16_t **outBuff,
                                long *outBuffSize);

    void readGDTInt16ImageSamples(int16_t **outBuff,
                                long *outBuffSize);

    void readGDTUInt32ImageSamples(uint32_t **outBuff,
                                long *outBuffSize);

    void readGDTInt32ImageSamples(int32_t **outBuff,
                                long *outBuffSize);

    void readGDTFloat32ImageSamples(float **outBuff,
                                long *outBuffSize);

    void readGDTFloat64ImageSamples(double **outBuff,
                                long *outBuffSize);

    //////////////////////////////////////////////////
    // write<GDALType>ImageSamples
    //////////////////////////////////////////////////

    void writeGDTByteImageSamples(unsigned char *outBuff,
                                long outBuffSize);

    void writeGDTUInt16ImageSamples(uint16_t *outBuff,
                                long outBuffSize);

    void writeGDTInt16ImageSamples(int16_t *outBuff,
                                long outBuffSize);

    void writeGDTUInt32ImageSamples(uint32_t *outBuff,
                                long outBuffSize);

    void writeGDTInt32ImageSamples(int32_t *outBuff,
                                long outBuffSize);

    void writeGDTFloat32ImageSamples(float *outBuff,
                                long outBuffSize);

    void writeGDTFloat64ImageSamples(double *outBuff,
                                long outBuffSize);


    //////////////////////////////////////////////////
    // read<GDALType>ImagePixel
    //////////////////////////////////////////////////

    void readGDTByteImagePixel(int xOff, int yOff,
                               unsigned char *outBuff,
                               int rasterCount);

    void readGDTUInt16ImagePixel(int xOff, int yOff,
                                 uint16_t *outBuff,
                                 int rasterCount);

    void readGDTInt16ImagePixel(int xOff, int yOff,
                                int16_t *outBuff,
                                int rasterCount);

    void readGDTUInt32ImagePixel(int xOff, int yOff,
                                 uint32_t *outBuff,
                                 int rasterCount);

    void readGDTInt32ImagePixel(int xOff, int yOff,
                                int32_t *outBuff,
                                int rasterCount);

    void readGDTFloat32ImagePixel(int xOff, int yOff,
                                  float *outBuff,
                                  int rasterCount);

    void readGDTFloat64ImagePixel(int xOff, int yOff,
                                  double *outBuff,
                                  int rasterCount);

    //////////////////////////////////////////////////
    // write<GDALType>ImagePixel
    //////////////////////////////////////////////////

    void writeGDTByteImagePixel(int xOff, int yOff,
                                unsigned char *outBuff,
                                int rasterCount);

    void writeGDTUInt16ImagePixel(int xOff, int yOff,
                                  uint16_t *outBuff,
                                  int rasterCount);

    void writeGDTInt16ImagePixel(int xOff, int yOff,
                                 int16_t *outBuff,
                                 int rasterCount);

    void writeGDTUInt32ImagePixel(int xOff, int yOff,
                                  uint32_t *outBuff,
                                  int rasterCount);

    void writeGDTInt32ImagePixel(int xOff, int yOff,
                                 int32_t *outBuff,
                                 int rasterCount);

    void writeGDTFloat32ImagePixel(int xOff, int yOff,
                                   float *outBuff,
                                   int rasterCount);

    void writeGDTFloat64ImagePixel(int xOff, int yOff,
                                   double *outBuff,
                                   int rasterCount);



    //////////////////////////////////////////////////
    // read<GDALType>ImageRegion
    //////////////////////////////////////////////////


    void readGDTByteImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                unsigned char **outBuff,
                                long *outBuffSize);

    void readGDTUInt16ImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                uint16_t **outBuff,
                                long *outBuffSize);

    void readGDTInt16ImageRegion(int xOff, int yOff,
                                 int xSize, int ySize,
                                int16_t **outBuff,
                                long *outBuffSize);

    void readGDTUInt32ImageRegion(int xOff, int yOff,
                                  int xSize, int ySize,
                                 uint32_t **outBuff,
                                long *outBuffSize);

    void readGDTInt32ImageRegion(int xOff, int yOff,
                                 int xSize, int ySize,
                                int32_t **outBuff,
                                long *outBuffSize);

    void readGDTFloat32ImageRegion(int xOff, int yOff,
                                   int xSize, int ySize,
                                  float **outBuff,
                                long *outBuffSize);

    void readGDTFloat64ImageRegion(int xOff, int yOff,
                                   int xSize, int ySize,
                                  double **outBuff,
                                long *outBuffSize);

    //////////////////////////////////////////////////
    // write<GDALType>ImageRegion
    //////////////////////////////////////////////////

    void writeGDTByteImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                unsigned char *outBuff,
                                long outBuffSize);

    void writeGDTUInt16ImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                uint16_t *outBuff,
                                long outBuffSize);

    void writeGDTInt16ImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                int16_t *outBuff,
                                long outBuffSize);

    void writeGDTUInt32ImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                uint32_t *outBuff,
                                long outBuffSize);

    void writeGDTInt32ImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                int32_t *outBuff,
                                long outBuffSize);

    void writeGDTFloat32ImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                float *outBuff,
                                long outBuffSize);

    void writeGDTFloat64ImageRegion(int xOff, int yOff,
                                int xSize, int ySize,
                                double *outBuff,
                                long outBuffSize);

    //////////////////////////////////////////////////
    // read<GDALType>ImageBlock
    //////////////////////////////////////////////////

    void readGDTByteImageBlock(int xIdx, int yIdx,
                                int *xOff, int *yOff,
                                int *xSize, int *ySize,
                                unsigned char **outBuff,
                                long *outBuffSize);

    void readGDTUInt16ImageBlock(int xIdx, int yIdx,
                                int *xOff, int *yOff,
                                int *xSize, int *ySize,
                                uint16_t **outBuff,
                                long *outBuffSize);

    void readGDTInt16ImageBlock(int xIdx, int yIdx,
                                int *xOff, int *yOff,
                                int *xSize, int *ySize,
                                int16_t **outBuff,
                                long *outBuffSize);

    void readGDTUInt32ImageBlock(int xIdx, int yIdx,
                                int *xOff, int *yOff,
                                int *xSize, int *ySize,
                                uint32_t **outBuff,
                                long *outBuffSize);

    void readGDTInt32ImageBlock(int xIdx, int yIdx,
                                int *xOff, int *yOff,
                                int *xSize, int *ySize,
                                int32_t **outBuff,
                                long *outBuffSize);

    void readGDTFloat32ImageBlock(int xIdx, int yIdx,
                                int *xOff, int *yOff,
                                int *xSize, int *ySize,
                                float **outBuff,
                                long *outBuffSize);

    void readGDTFloat64ImageBlock(int xIdx, int yIdx,
                                int *xOff, int *yOff,
                                int *xSize, int *ySize,
                                double **outBuff,
                                long *outBuffSize);


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
    //void getImageSamples(GDALDataType dType,
      //                          void **outBuff,
      //                          long *outBuffSize);

    void getBlockCoordinates(int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize);

    void getBlocksCount(int *wide, int *high);

    void getBlockSize(int *xSize, int *ySize);

    bool getErrorOnDataTypeConversions();

    void setErrorOnDataTypeConversions(bool err);

    void dataTypeConversionCheck(GDALDataType testType);

    GDALDataType getDataType();

    ////////////////////////////////////////////////////////
    // Member functions
    ////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////
    // Static functions
    ////////////////////////////////////////////////////////

    static void getIntGDALDataTypeExtrema(GDALDataType dType, long *min, long *max);

    static GDALDataset * convertImage(GDALDataset *srcDset,
            double srcMin,
            double srcMax,
            const char *fileName,
            GDALDataType dType,
            const char *format,
            char **options,
            double min,
            double max);

    static void verifyRasterBandCount(GDALDataset *dset, int testCount);

    static void verifySameBlockSize(GDALDataset *dset);

    static void getBlockCoordinates(GDALDataset *dset,
                            int xIdx, int yIdx,
                            int *xOff, int *yOff,
                            int *xSize, int *ySize);

    static void getBlocksCount(GDALDataset *dset,
                            int *wide, int *high);

    static void getBlockSize(GDALDataset *dset,
                            int *xSize, int *ySize);

	/**
	 *  \brief Verifies that a given raster band has a given data type.
	 *  		The function throws an exception if the expected data type
	 *  		is not found.
	 *  
	 *  \param [in] band Raster band to test.
	 *  \param [in] dType Data type to test that 
	 *  
	 *  \throws UC::Exception
	 *  		Raster band data type does not match the expected data type.
	 */
    static void verifyRasterBandDataType(GDALRasterBand *band, GDALDataType dType);

	/**
	 *  \brief Gets the data type of the GDALDataset.
	 *  
	 *  \param [in] dset Data set to inspect.
	 *  \return Data type of the data set.
	 *  
	 *  \details Gets the data type of the GDALDataset.  This function assumes
	 *  		that all raster bands will have the same data type,
	 *  		otherwise an exception is thrown.
	 */
    static GDALDataType getDataType(GDALDataset *dset);

	/**
	 *  \brief Verifies that all raster bands of the data set have the
	 *  		same data type.
	 *  
	 *  \param [in] dset Data set to verify.
	 *  
	 *  \throws UC::Exception
	 *  		Data set does not have the same data type for all the 
	 *  		raster bands.
	 */
    static void verifySameDataType(GDALDataset *dset);

	/**
	 *  \brief Verifies all the raster bands of the data set match the
	 *  		given data type.
	 *  
	 *  \param [in] dset Data set to verify.
	 *  \param [in] dType Data type to test for.
	 *  
	 *  \throws UC::Exception
	 *  		Data set does not have the same data type for all the 
	 *  		raster bands.
	 */
    static void verifyDataType(GDALDataset *dset, GDALDataType dType);
	
	/**
	 *  \brief Verifies all raster bands of the data set have the same
	 *  		resolution.
	 *  
	 *  \param [in] dset Data set to verify.
	 *  
	 *  \throws UC::Exception
	 *  		Data set does not have the same resolution for all 
	 *  		raster bands.
	 */
    static void verifySameSizeBands(GDALDataset *dset);

	/**
	 *  \brief Gets the area (width * height) of the data set's
	 *  		resolution.
	 *  
	 *  \param [in] dset Data set from which to get the area.
	 *  \return The number of samples 1 raster band of the data set.
	 */	
    static long getRasterAreaSize(GDALDataset *dset);

	/**
	 *  \brief Gets the sample count (width * height) for a raster band.
	 *  
	 *  \param [in] dset Data set which contains the raster band.
	 *  \param [in] nBandId Raster band number (1 based for GDAL).
	 *  \return The number of samples in the raster band.
	 */	
    static long getRasterBandSamplesCount(GDALDataset *dset, int nBandId);

	/**
	 *  \brief Gets the sample count (width * height * raster_bands) for a data set.
     *  
     *  \param [in] dset Parameter_Description
     *  \return Return_Description
     *  
     *  \details Details
     */
    static long getImageSamplesCount(GDALDataset *dset);

	/**
	 *  \brief Verifies the test size is greater than or equal in size to
	 *  		the samples of the raster band.
	 *  
	 *  \param [in] dset Data set to retrieve the raster band.
	 *  \param [in] nBandId Raster band number (1 based for GDAL).
	 *  \param [in] testSz The size to verify that is equal or greater than the
	 *  		data sets resolution's area.
	 *  
	 *  \throws UC::Exception
	 *  		Raster band's area is larger than the test size.
	 */	
    static void verifyRasterBandSamplesCount(GDALDataset *dset,
                                int nBandId,
                                long testSz);

    /**
     *  \brief Verifies the test size is greater than or equal in size to
     *  	the samples of the data set.
     *  
     *  \param [in] dset Data set from which to receive the samples.
     *  \param [in] testSz The size to verify that is equal or greater than the
	 *  		data sets sample count.
	 *
	 *  \throws UC::Exception
	 *  		Raster band's area is larger than the test size.
     */
    static void verifyImageSamplesCount(GDALDataset *dset, long testSz);

    /**
     *  \brief Gets the samples of a raster band.
     *
     *  \param [in] dset Data set that contains the raster band.
     *  \param [in] nBandId Raster band number (1 based for GDAL).
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
    static void getRasterBandSamples(GDALDataset *dset,
                               int nBandId,
                               unsigned char **outBuff,
                               long *outBuffSize);

	/**
	 *  \brief Gets the samples of a raster band.
	 *  
	 *  \param [in] dset Data set that contains the raster band.
	 *  \param [in] nBandId Raster band number (1 based for GDAL).
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
    static void getRasterBandSamples(GDALDataset *dset,
                               int nBandId,
                               double **outBuff,
                               long *outBuffSize);

	/**
	 *  \brief Gets the samples of a data set.
	 *  
	 *  \param [in] dset Data set that contains the samples.
	 *  \param [in,out] outBuff Buffer that will be populated with the raster
	 *  		band's samples.
	 *  		On input, set *outBuff = NULL to have the function allocate a buffer.
	 *  		On input, if *outBuff != NULL then the buffer is expected to be preallocated
	 *  		with outBuffSize number samples which should be large enough to contain the
	 *  		entire contents of the data set.
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
    static void getImageSamples(GDALDataset *dset,
                               unsigned char **outBuff,
                               long *outBuffSize);

	/**
	 *  \brief Gets the samples of a data set.
	 *  
	 *  \param [in] dset Data set that contains the samples.
	 *  \param [in,out] outBuff Buffer that will be populated with the raster
	 *  		band's samples.
	 *  		On input, set *outBuff = NULL to have the function allocate a buffer.
	 *  		On input, if *outBuff != NULL then the buffer is expected to be preallocated
	 *  		with outBuffSize number samples which should be large enough to contain the
	 *  		entire contents of the data set.
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
    static void getImageSamples(GDALDataset *dset,
                               double **outBuff,
                               long *outBuffSize);

	/**
	 *  \brief Gets the samples of a data set.
	 *  
	 *  \param [in] dset Data set that contains the samples.
	 *  \param [in,out] outBuff Buffer that will be populated with the raster
	 *  		band's samples.
	 *  		On input, set *outBuff = NULL to have the function allocate a buffer.
	 *  		On input, if *outBuff != NULL then the buffer is expected to be preallocated
	 *  		with outBuffSize number samples which should be large enough to contain the
	 *  		entire contents of the data set.
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
    static void getImageSamples(GDALDataset *dset,
                               uint16_t **outBuff,
                               long *outBuffSize);

	/**
	 *  \brief Sets the samples of a data set.
	 *  
	 *  \param [in] dset Data set to write the samples.
	 *  \param [in] outBuff Buffer that will populate the data
	 *  		set's samples.
	 *  \param [in] outBuffSize The number of bytes in the outBuff.
	 *  
	 *  \details Samples are interleaved by pixels first, columns next (x), and then by row (y).
	 */								   
    static void setImageSamples(GDALDataset *dset,
                                uint16_t *outBuff,
                                long outBuffSize);

	static GDALDataType getDoubleGDTFloatType();

    /**
     *  \brief Frees image samples uses CPLFree().
     *  
     *  \param [in] buff buffer to free
     */
    static void freeImageSamples(void *buff);

    /**
     *  \brief Frees image samples uses CPLFree() and sets the buffer pointer to NULL.
     *  
     *  \param [in] buff buffer to free and set to NULL
     */
    static void freeDptrImageSamples(void **buff);

	/**
     *  \brief Function remaps samples from changing the order of color planes,
     *  		columns, and rows.  This is useful for going back and forth
     *  		between MATLAB image sample packing (first by row, then by column,
     *  		then by color plane) and more traditional raster image scanning
     *  		(first by color plane, then by column, then by row).
     *  
     *  \param [in] inBuff Original image buffer samples to remap
     *  \param [in] outBuff Remapped image buffer samples
     *  \param [in] remapType Type of remapping to perform.
     *			Valid values are:				  				// PXY: Pixel, x, then y
	 *			REMAP_PXY_YXR -	by pixel, column, and row converted to row, column, and raster band
	 *			REMAP_YXR_PXY - by row, column, and raster band converted to pixel, column, and row
	 * 			Note YXR is MATLAB oriented and PXY is traditional raster oriented.  
     *  \param [in] dataTypeSize Size of the data type in bytes
     *  \param [in] samplesWidth Horizontal resolution of the image
     *  \param [in] samplesHeight Vertical resolution of the image
     *  \param [in] rasterBandCount Number of raster bands in a sample
     *  
     *  \details Function assumes all color planes are sampled at the same resolution.
     */
    static void remapSamples(void *inBuff, void *outBuff,
                             int remapType,
                             int dataTypeSize,
                             int samplesWidth, int samplesHeight,
                             int rasterBandCount);

private:


    /////////////////////////////////////////////////////////////
    // member variables
    /////////////////////////////////////////////////////////////

    GDALDataset *mDset;

    bool mDestroyDsetOnDelete;

    bool mErrorOnDataTypeConversion;


    /////////////////////////////////////////////////////////////
    // static methods
    /////////////////////////////////////////////////////////////

    static bool mAllRegistered;

    static void registerCheck();

    static void imageSamplesIO(GDALDataset *dset,
                               GDALRWFlag rwFlag,
                               uint16_t **outBuff,
                               long *outBuffSize);

    static GDALDriver * getGDALDriverByName(const char *name);



};


}


#endif // IMAGE_H
