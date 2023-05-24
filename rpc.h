#ifndef RPC_H
#define RPC_H


#include "ucimagelib_global.h"
#include "rpc_types.h"


#include <QString>


class GDALDataset;
class GDALRasterBand;

#define RPC_VERBOSE (1)


namespace UC {


class IMAGELIBSHARED_EXPORT Rpc
{
public:

    Rpc();

    Rpc(const char *fileName);

    Rpc(const QString &fileName);

    Rpc(GDALDataset *dset);

    const RpcStruct * getRpcStruct();

    static Rpc * rpcFromGDALMetadata(const QString &metadata);

    static double evaluateRpc20TermPoly(double poly[20],
                                        double L,
                                        double P,
                                        double H);

    static void evaluateRpcPartialAtPLH(double poly[20],
                                        double L,
                                        double P,
                                        double H,
                                        double result[3]);

    static int evaluateRpcPartialAtPLHAndCompB(
            RpcStruct *rpcs,
            double L, double P, double H,
            ImagePos *ls, double **B);

    static void groundToImage(GroundPos gpos, RpcStruct *rpcs, ImagePos *impos);

    static void imageToGroundAtH(ImagePos impos,
                                 RpcStruct *rpcs,
                                 double H, GroundPos *gpos);

#if RPC_VERBOSE
    static void printStruct(RpcStruct *rpc);
#endif


private:

#if RPC_VERBOSE

    static void printIntVal(RpcStruct *rpc,
                          const char* valName,
                          int val,
                          unsigned int setBit);

    static void printDblVal(RpcStruct *rpc,
                          const char* valName,
                          double val,
                          unsigned int setBit);

    static void printDblArrayVals(RpcStruct *rpc,
                          const char* valName,
                          double vals[RPC_COEFFS],
                          unsigned int setBit);

#endif

    void commonFileInit(const char *fileName);

    void commonGDALMetadataInit(const QString &metatdata);

    void readRpcGDALInfo(const QString &rpcText);

    void readRpcFile(const QString &rpcText);

    void findIntValue(const QString &rpcText,
                           const QString &valName,
                           const QString &valNameSep,
                           int *val,
                           unsigned int setBit);

    void findDblValue(const QString &rpcText,
                          const QString &valName,
                          const QString &valNameSep,
                          double *val,
                          unsigned int setBit);

    void findDblCoeffValues(const QString &rpcText,
                            const QString &valName,
                            const QString &valNameSep,
                            double val[],
                            unsigned int setBit);

    void findGDALDblCoeffValues(const QString &rpcText,
                            const QString &valName,
                            const QString &valNameSep,
                            double val[],
                            unsigned int setBit);

    RpcStruct mRpc;

    static QString getValString(const QString &rpcText,
                                const QString &valName,
                                const QString &valNameSep);

    static QString getGDALCoeffsValString(const QString &rpcText,
                          const QString &valName,
                          const QString &valNameSep);

};


}


#endif // RPC_H
