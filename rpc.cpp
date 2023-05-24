
#include "rpc.h"
#include "image.h"

#include "exception.h"
#include "fileio.h"

#include "gdal_priv.h"

#include <assert.h>
#include <linux/types.h>
#include <string.h>
#include <stdio.h>

#include <QFile>
#include <QTextStream>
#include <QStringList>


namespace UC {


Rpc::Rpc()
{
    INIT_RPC(&mRpc);
}


Rpc::Rpc(const char *fileName)
{
    INIT_RPC(&mRpc);
    commonFileInit(fileName);
}


Rpc::Rpc(const QString &fileName)
{
    INIT_RPC(&mRpc);
    commonFileInit(fileName.toLatin1().constData());
}


Rpc::Rpc(GDALDataset *dset)
{
    QString metatdata;

    // this needs to be implimented
    throw UC::Exception("Rpc from cstor GDALDataset needs to be implimented.");
    // warns be gone
    dset = dset;

    commonGDALMetadataInit(metatdata);
}


Rpc * Rpc::rpcFromGDALMetadata(const QString &metadata)
{
    Rpc *rpc = new Rpc();
    rpc->commonGDALMetadataInit(metadata);
    return rpc;
}


void Rpc::commonGDALMetadataInit(const QString &metatdata)
{
    readRpcGDALInfo(metatdata);
}


void Rpc::commonFileInit(const char *fileName)
{
    QString rpcText = UC::FileIO::getFileText(fileName);
    readRpcFile(rpcText);
}


QString Rpc::getValString(const QString &rpcText,
                      const QString &valName,
                      const QString &valNameSep)
{
    int idx;
    int sepIdx;
    QString valStr = "";
    int valStartIdx;
    int valEndIdx;

    // make sure
    idx = rpcText.indexOf(valName);
    if (idx != -1) {
        sepIdx = rpcText.indexOf(valNameSep, idx + 1);
        if (sepIdx != -1) {
            idx = sepIdx;

            valStartIdx = -1;
            do {
                idx++;
                if (idx < rpcText.length() && !rpcText[idx].isSpace()) {
                    valStartIdx = idx;
                }
            } while (valStartIdx == -1 && idx < rpcText.length());

            valEndIdx = -1;
            do {
                idx++;
                if (idx < rpcText.length() && rpcText[idx].isSpace()) {
                    valEndIdx = idx;
                }
            } while (valEndIdx == -1 && idx < rpcText.length());

            valStr = rpcText.mid(valStartIdx, valEndIdx - valStartIdx);
        }
    }

    return valStr;
}


QString Rpc::getGDALCoeffsValString(const QString &rpcText,
                      const QString &valName,
                      const QString &valNameSep)
{
    int idx;
    int sepIdx;
    QString valStr = "";
    int valStartIdx;
    int valEndIdx;

    // make sure
    idx = rpcText.indexOf(valName);
    if (idx != -1) {
        sepIdx = rpcText.indexOf(valNameSep, idx + 1);
        if (sepIdx != -1) {
            idx = sepIdx;

            valStartIdx = -1;
            do {
                idx++;
                if (idx < rpcText.length() && !rpcText[idx].isSpace()) {
                    valStartIdx = idx;
                }
            } while (valStartIdx == -1 && idx < rpcText.length());

            valEndIdx = -1;
            do {
                idx++;
                if (idx < rpcText.length() && rpcText[idx] == '\n') {
                    valEndIdx = idx;
                }
            } while (valEndIdx == -1 && idx < rpcText.length());
            if (valEndIdx == -1) {
                valEndIdx = rpcText.length() - 1;
            }
            valStr = rpcText.mid(valStartIdx, valEndIdx - valStartIdx);
        }
    }

    return valStr;
}


void Rpc::findIntValue(const QString &rpcText,
                       const QString &valName,
                       const QString &valNameSep,
                       int *val,
                       unsigned int setBit)
{
    bool ok;
    QString valStr;
    double dblVal;

    valStr = getValString(rpcText, valName, valNameSep);
    if (!valStr.isEmpty()) {
        dblVal = valStr.toDouble(&ok);
        //*val = valStr.toInt(&ok);
        *val = (int)dblVal;
        if (ok) {
            mRpc.set_bits |= setBit;
        }
    }
}


void Rpc::findDblValue(const QString &rpcText,
                       const QString &valName,
                       const QString &valNameSep,
                       double *val,
                       unsigned int setBit)
{
    bool ok;
    QString valStr;

    valStr = getValString(rpcText, valName, valNameSep);
    if (!valStr.isEmpty()) {
        *val = valStr.toDouble(&ok);
        if (ok) {
            mRpc.set_bits |= setBit;
        }
    }
}


void Rpc::findDblCoeffValues(const QString &rpcText,
                        const QString &valName,
                        const QString &valNameSep,
                        double val[],
                        unsigned int setBit)
{
    bool ok;
    QString valStr;
    QString coeffValName;
    int k;

    valStr = "temp";
    k = 0;

    while (k < RPC_COEFFS && !valStr.isEmpty()) {
        coeffValName = "%1_%2";
        coeffValName = coeffValName.arg(valName).arg(k + 1);
        valStr = getValString(rpcText, coeffValName, valNameSep);
        if (!valStr.isEmpty()) {
            val[k] = valStr.toDouble(&ok);
            if (ok) {
                mRpc.set_bits |= setBit;
            }
        }
        k++;
    }

}


void Rpc::findGDALDblCoeffValues(const QString &rpcText,
                                 const QString &valName,
                                 const QString &valNameSep,
                                 double val[],
                                 unsigned int setBit)
{
    bool ok;
    QString valStr;
    int k;
    QStringList splits;

    valStr = getGDALCoeffsValString(rpcText, valName, valNameSep);
    k = 0;

    splits = valStr.split(QChar(' '), QString::SkipEmptyParts);

    while (k < RPC_COEFFS && !valStr.isEmpty()) {
        val[k] = splits[k].toDouble(&ok);
        if (!ok) {
            QString msg = "Error parsing '%1' value string '%2'.";
            throw UC::Exception(msg.arg(valName).arg(splits[k]));
        }
        k++;
    }

     if (splits.count() == RPC_COEFFS) {
         mRpc.set_bits |= setBit;
     }
}


void Rpc::readRpcFile(const QString &rpcText)
{

    QString valNameSep = ":";

    findDblValue(rpcText, RPC_ERR_BIAS_STR, valNameSep,
                 &mRpc.err_bias, RPC_ERR_BIAS_SET);

    findDblValue(rpcText, RPC_ERR_RAND_STR, valNameSep,
                 &mRpc.err_rand, RPC_ERR_RAND_SET);

    findIntValue(rpcText, RPC_LINE_OFF_STR, valNameSep,
                 &mRpc.line_off, RPC_LINE_OFF_SET);

    findIntValue(rpcText, RPC_SAMPLE_OFF_STR, valNameSep,
                 &mRpc.sample_off, RPC_SAMPLE_OFF_SET);

    findDblValue(rpcText, RPC_LAT_OFF_STR, valNameSep,
                 &mRpc.lat_off, RPC_LAT_OFF_SET);

    findDblValue(rpcText, RPC_LON_OFF_STR, valNameSep,
                 &mRpc.lon_off, RPC_LON_OFF_SET);

    findIntValue(rpcText, RPC_HGT_OFF_STR, valNameSep,
                 &mRpc.hgt_off, RPC_HGT_OFF_SET);

    findIntValue(rpcText, RPC_LINE_SCALE_STR, valNameSep,
                 &mRpc.line_scale, RPC_LINE_SCALE_SET);

    findIntValue(rpcText, RPC_SAMPLE_SCALE_STR, valNameSep,
                 &mRpc.sample_scale, RPC_SAMPLE_SCALE_SET);

    findDblValue(rpcText, RPC_LAT_SCALE_STR, valNameSep,
                 &mRpc.lat_scale, RPC_LAT_SCALE_SET);

    findDblValue(rpcText, RPC_LON_SCALE_STR, valNameSep,
                 &mRpc.lon_scale, RPC_LON_SCALE_SET);

    findIntValue(rpcText, RPC_HGT_SCALE_STR, valNameSep,
                 &mRpc.hgt_scale, RPC_HGT_SCALE_SET);

    findDblCoeffValues(rpcText, LINE_NUM_COEFF_STR, valNameSep,
                    mRpc.line_num_coeff, LINE_NUM_COEFF_SET);

    findDblCoeffValues(rpcText, LINE_DEN_COEFF_STR, valNameSep,
                    mRpc.line_den_coeff, LINE_DEN_COEFF_SET);

    findDblCoeffValues(rpcText, SAMPLE_NUM_COEFF_STR, valNameSep,
                    mRpc.sample_num_coeff, SAMPLE_NUM_COEFF_SET);

    findDblCoeffValues(rpcText, SAMPLE_DEN_COEFF_STR, valNameSep,
                    mRpc.sample_den_coeff, SAMPLE_DEN_COEFF_SET);

#if RPC_VERBOSE
    printStruct(&mRpc);
#endif // RPC_VERBOSE

}


void Rpc::readRpcGDALInfo(const QString &rpcText)
{

    QString valNameSep = "=";

    findDblValue(rpcText, RPC_ERR_BIAS_STR, valNameSep,
                 &mRpc.err_bias, RPC_ERR_BIAS_SET);

    findDblValue(rpcText, RPC_ERR_RAND_STR, valNameSep,
                 &mRpc.err_rand, RPC_ERR_RAND_SET);

    findIntValue(rpcText, RPC_LINE_OFF_STR, valNameSep,
                 &mRpc.line_off, RPC_LINE_OFF_SET);

    findIntValue(rpcText, RPC_SAMPLE_OFF_STR, valNameSep,
                 &mRpc.sample_off, RPC_SAMPLE_OFF_SET);

    findDblValue(rpcText, RPC_LAT_OFF_STR, valNameSep,
                 &mRpc.lat_off, RPC_LAT_OFF_SET);

    findDblValue(rpcText, RPC_LON_OFF_STR, valNameSep,
                 &mRpc.lon_off, RPC_LON_OFF_SET);

    findIntValue(rpcText, RPC_HGT_OFF_STR, valNameSep,
                 &mRpc.hgt_off, RPC_HGT_OFF_SET);

    findIntValue(rpcText, RPC_LINE_SCALE_STR, valNameSep,
                 &mRpc.line_scale, RPC_LINE_SCALE_SET);

    findIntValue(rpcText, RPC_SAMPLE_SCALE_STR, valNameSep,
                 &mRpc.sample_scale, RPC_SAMPLE_SCALE_SET);

    findDblValue(rpcText, RPC_LAT_SCALE_STR, valNameSep,
                 &mRpc.lat_scale, RPC_LAT_SCALE_SET);

    findDblValue(rpcText, RPC_LON_SCALE_STR, valNameSep,
                 &mRpc.lon_scale, RPC_LON_SCALE_SET);

    findIntValue(rpcText, RPC_HGT_SCALE_STR, valNameSep,
                 &mRpc.hgt_scale, RPC_HGT_SCALE_SET);

    findGDALDblCoeffValues(rpcText, LINE_NUM_COEFF_STR, valNameSep,
                    mRpc.line_num_coeff, LINE_NUM_COEFF_SET);

    findGDALDblCoeffValues(rpcText, LINE_DEN_COEFF_STR, valNameSep,
                    mRpc.line_den_coeff, LINE_DEN_COEFF_SET);

    findGDALDblCoeffValues(rpcText, SAMPLE_NUM_COEFF_STR, valNameSep,
                    mRpc.sample_num_coeff, SAMPLE_NUM_COEFF_SET);

    findGDALDblCoeffValues(rpcText, SAMPLE_DEN_COEFF_STR, valNameSep,
                    mRpc.sample_den_coeff, SAMPLE_DEN_COEFF_SET);

#if RPC_VERBOSE
    printStruct(&mRpc);
#endif // RPC_VERBOSE

}


const RpcStruct * Rpc::getRpcStruct()
{
    return &mRpc;
}


#if RPC_VERBOSE


void Rpc::printIntVal(RpcStruct *rpc,
                      const char* valName,
                      int val,
                      unsigned int setBit)
{
    QString msg = "%1 = %2\r\n";
    if (setBit & rpc->set_bits) {
        msg = msg.arg(valName).arg(val);
    } else {
        msg = msg.arg(valName).arg("NOT SET");
    }
    printf(msg.toLatin1().data());
}


void Rpc::printDblVal(RpcStruct *rpc,
                      const char* valName,
                      double val,
                      unsigned int setBit)
{
    QString msg = "%1 = %2\r\n";
    if (setBit & rpc->set_bits) {
        msg = msg.arg(valName).arg(val);
    } else {
        msg = msg.arg(valName).arg("NOT SET");
    }
    printf(msg.toLatin1().data());
}


void Rpc::printDblArrayVals(RpcStruct *rpc,
                      const char* valName,
                      double vals[RPC_COEFFS],
                      unsigned int setBit)
{
    int k;

    if (setBit & rpc->set_bits) {
        for (k = 0; k < RPC_COEFFS; k++) {
            QString msg = "%1[%2] = %3\r\n";
            msg = msg.arg(valName).arg(k).arg(vals[k]);
            printf(msg.toLatin1().data());
        }
    } else {
        QString msg = "%1 = %2\r\n";
        msg = msg.arg(valName).arg("NOT SET");
        printf(msg.toLatin1().data());
    }
}


void Rpc::printStruct(RpcStruct *rpc)
{

    printDblVal(rpc,
                RPC_ERR_BIAS_STR,
                rpc->err_bias,
                RPC_ERR_BIAS_SET);

    printDblVal(rpc,
                RPC_ERR_RAND_STR,
                rpc->err_rand,
                RPC_ERR_RAND_SET);

    printIntVal(rpc,
                RPC_LINE_OFF_STR,
                rpc->line_off,
                RPC_LINE_OFF_SET);

    printIntVal(rpc,
                RPC_SAMPLE_OFF_STR,
                rpc->sample_off,
                RPC_SAMPLE_OFF_SET);

    printDblVal(rpc,
                RPC_LAT_OFF_STR,
                rpc->lat_off,
                RPC_LAT_OFF_SET);

    printDblVal(rpc,
                RPC_LON_OFF_STR,
                rpc->lon_off,
                RPC_LON_OFF_SET);

    printIntVal(rpc,
                RPC_HGT_OFF_STR,
                rpc->hgt_off,
                RPC_HGT_OFF_SET);

    printIntVal(rpc,
                RPC_LINE_SCALE_STR,
                rpc->line_scale,
                RPC_LINE_SCALE_SET);

    printIntVal(rpc,
                RPC_SAMPLE_SCALE_STR,
                rpc->sample_scale,
                RPC_SAMPLE_SCALE_SET);

    printDblVal(rpc,
                RPC_LAT_SCALE_STR,
                rpc->lat_scale,
                RPC_LAT_SCALE_SET);

    printDblVal(rpc,
                RPC_LON_SCALE_STR,
                rpc->lon_scale,
                RPC_LON_SCALE_SET);

    printIntVal(rpc,
                RPC_HGT_SCALE_STR,
                rpc->hgt_scale,
                RPC_HGT_SCALE_SET);

    printDblArrayVals(rpc,
                      LINE_NUM_COEFF_STR,
                      rpc->line_num_coeff,
                      LINE_NUM_COEFF_SET);

    printDblArrayVals(rpc,
                      LINE_DEN_COEFF_STR,
                      rpc->line_den_coeff,
                      LINE_DEN_COEFF_SET);

    printDblArrayVals(rpc,
                      SAMPLE_NUM_COEFF_STR,
                      rpc->sample_num_coeff,
                      SAMPLE_NUM_COEFF_SET);

    printDblArrayVals(rpc,
                      SAMPLE_DEN_COEFF_STR,
                      rpc->sample_den_coeff,
                      SAMPLE_DEN_COEFF_SET);
}


#endif //RPC_VERBOSE


void matrix_transpose(double **a, double **b, int nr, int nc)
{
    assert(0); // fill this function out
    // warns be gone
    a = a;
    b = b;
    nr = nr;
    nc = nc;
}

int matrix_multiply(double **a, double **b, int anr, int anc, int bnr, int bnc, double **c)
{
    assert(0); // fill this function out
    // warns be gone
    a = a;
    b = b;
    anr = anr;
    anc = anc;
    bnr = bnr;
    bnc = bnc;
    c = c;

}

double matrix_determinant(double **matrix, int size)
{
    assert(0); // fill this function out
    // warns be gone
    matrix = matrix;
    size = size;
}


void matrix_inverse(double **mat, int dim, double **result)
{
    assert(0); // fill this function out
    // warns be gone
    mat = mat;
    dim = dim;
    result = result;
}// Numerical Recipes in C


void matrix_inverse_2(double **matrix, int size, double **Result)
{
    assert(0); // fill this function out
    // warns be gone
    matrix = matrix;
    size = size;
    Result = Result;
}// Cofactor method


int matrix_inverse_3x3(double **mat, double **result)
{
    assert(0); // fill this function out
    // warns be gone
    mat = mat;
    result = result;

}


#define MAX_ITERATIONS 10
#define CONVERGENCE_TOLERANCE (1.0e-20)

void Rpc::imageToGroundAtH(ImagePos impos, RpcStruct *rpcs, double H, GroundPos *gpos)
{
    H = H;  // warns be gone
   GroundPos init_pos, norm_gpt;
   int i, j;
   int iteration = 0;
   int status;
//   double Bmat[2][3];
//   double BTmat[3][2];
//   double Wgtmat[2][2];
//   double BTWmat[3][2];
//   double Nsinglemat[3][3], Nmat[3][3], Ninv[3][3];
//   double Vsinglemat[3][1], Vmat[3][1];
//   double Deltamat[3][1];
//   double vector[2][1];
   double **Bmat;
   double **BTmat;
   double **BTWmat;
   double **Wgtmat;
   double **Nsinglemat, **Nmat, **Ninv;
   double **Vsinglemat, **Vmat;
   double **Deltamat;
   double **vector;

   ImagePos img_pos, newimg_pos, img_resid;
   double variance;
   double deltamag;
   int Converged;

printf("rpcImageToGround:\n");
printf("   input:  line,sample = %.10lf  %.10lf\n", impos.l, impos.s);
//rpcPrint(rpcs);
fflush(stdout);
   /* allocate memory for the arrays */
   vector = (double **)calloc(2, sizeof(double *));
   for(i = 0; i < 3; i++)
   {
      vector[i] = (double *)calloc(1, sizeof(double));
   }
   Vsinglemat = (double **)calloc(3, sizeof(double *));
   Vmat       = (double **)calloc(3, sizeof(double *));
   Deltamat   = (double **)calloc(3, sizeof(double *));
   for(i = 0; i < 3; i++)
   {
      Vsinglemat[i] = (double *)calloc(1, sizeof(double));
      Vmat[i]       = (double *)calloc(1, sizeof(double));
      Deltamat[i]   = (double *)calloc(1, sizeof(double));
   }
   Wgtmat = (double **)calloc(2, sizeof(double *));
   for(i = 0; i < 2; i++)
   {
      Wgtmat[i] = (double *)calloc(2, sizeof(double));
   }
   Bmat = (double **)calloc(2, sizeof(double *));
   for(i = 0; i < 2; i++)
   {
      Bmat[i] = (double *)calloc(3, sizeof(double));
   }

   BTmat  = (double **)calloc(3, sizeof(double *));
   BTWmat = (double **)calloc(3, sizeof(double *));
   for(i = 0; i < 3; i++)
   {
      BTmat[i]  = (double *)calloc(2, sizeof(double));
      BTWmat[i] = (double *)calloc(2, sizeof(double));
   }

   Nsinglemat = (double **)calloc(3, sizeof(double *));
   Nmat       = (double **)calloc(3, sizeof(double *));
   Ninv       = (double **)calloc(3, sizeof(double *));
   for(i = 0; i < 3; i++)
   {
      Nsinglemat[i] = (double *)calloc(3, sizeof(double));
      Nmat[i]       = (double *)calloc(3, sizeof(double));
      Ninv[i]       = (double *)calloc(3, sizeof(double));
   }

   /* use the RPC reference ground point as the initial guess */
   /*   recall that x = lon and y = lat */
   init_pos.x = 18.89;
   init_pos.y = -33.66;
   init_pos.z = 133.0;
//   init_pos.x = rpcs->lon_off;
//   init_pos.y = rpcs->lat_off;
//   init_pos.z = (double)rpcs->hgt_off;
//   init_pos.z = H;
printf("   initial ground position:  lat,lon,hgt = %.10lf  %.10lf  %.3lf\n",
       init_pos.y, init_pos.x, init_pos.z);
fflush(stdout);

   variance  = 0.0;
   Converged = 0;
   Wgtmat[0][0] = Wgtmat[1][1] = 1.0;
   Wgtmat[0][1] = Wgtmat[1][0] = 0.0;

   while(iteration <= MAX_ITERATIONS && !Converged)
   {
printf("----------------------------------\n");
printf("       ITERATION = %d\n", iteration);
printf("----------------------------------\n");
fflush(stdout);
      /* normalize the ground point */
      norm_gpt.y = rpcs->lat_scale * (init_pos.y - rpcs->lat_off);                 // P
      norm_gpt.x = rpcs->lon_scale * (init_pos.x - rpcs->lon_off);                 // L
      norm_gpt.z = (double)rpcs->hgt_scale * (init_pos.z - (double)rpcs->hgt_off); // H

printf("   lat:  %.10lf  %.10lf\n", rpcs->lat_scale, rpcs->lat_off);
printf("   lon:  %.10lf  %.10lf\n", rpcs->lon_scale, rpcs->lon_off);
printf("   hgt:  %d   %d\n",  rpcs->hgt_scale, rpcs->hgt_off);
printf("   normalized position:  P = %.10lf  L = %.10lf  H = %.3lf\n",
        norm_gpt.y, norm_gpt.x, norm_gpt.z);
fflush(stdout);
      status = 0;
      img_pos.l = img_pos.s = 0.0;
      newimg_pos.l = newimg_pos.s = 0.0;
      status = evaluateRpcPartialAtPLHAndCompB(
                  rpcs, norm_gpt.y, norm_gpt.x, norm_gpt.z, &img_pos, Bmat);
printf("  after evaluate_RPC_partial_at_PLH_and_comp_B:\n");
printf("      image position:  %.10lf  %.10lf\n", img_pos.l, img_pos.s);
fflush(stdout);
      if(status != 0)
      {
         printf("error computing ground coordinates for line = %lf  sample = %lf\n", impos.l, impos.s);
         fflush(stdout);
         return;
      }

      /* de-normalize the B matrix and the computed image coordinates */
      Bmat[0][0] = Bmat[0][0] * rpcs->lat_scale / (double)rpcs->line_scale;
      Bmat[0][1] = Bmat[0][1] * rpcs->lon_scale / (double)rpcs->line_scale;
      Bmat[0][2] = Bmat[0][2] * (double)rpcs->hgt_scale / (double)rpcs->line_scale;
      Bmat[1][0] = Bmat[1][0] * rpcs->lat_scale / (double)rpcs->sample_scale;
      Bmat[1][1] = Bmat[1][1] * rpcs->lon_scale / (double)rpcs->sample_scale;
      Bmat[1][2] = Bmat[1][2] * (double)rpcs->hgt_scale / (double)rpcs->sample_scale;
      newimg_pos.l = (img_pos.l / (double)rpcs->line_scale) + (double)rpcs->line_off;
      newimg_pos.s = (img_pos.s / (double)rpcs->sample_scale) + (double)rpcs->sample_off;
printf("   newimg line,sample = %.10lf  %.10lf\n", newimg_pos.l, newimg_pos.s);
fflush(stdout);
for(i = 0; i < 2; i++)
   printf("   Bmat[%d][0] = %.10lf  Bmat[%d][1] = %.10lf  Bmat[%d][2] = %.10lf\n", i, Bmat[i][0], i, Bmat[i][1], i, Bmat[i][2]);
fflush(stdout);

      /* prepare for least squares */
      vector[0][0] = impos.l - newimg_pos.l;
      vector[1][0] = impos.s - newimg_pos.s;

      /* compute residuals */
      img_resid.l = -vector[0][0];
      img_resid.s = -vector[1][0];

      /* compute new variance value */
      variance += ((vector[0][0] * vector[0][0]) + (vector[1][0] * vector[1][0]));

      matrix_transpose(Bmat, BTmat, 2, 3);
for(i = 0; i < 3; i++)
   printf("   BTmat[%d][0] = %.10lf  BTmat[%d][1] = %.10lf\n",
              i, BTmat[i][0], i, BTmat[i][1]);
fflush(stdout);
      status = matrix_multiply(BTmat, Wgtmat, 3, 2, 2, 2, BTWmat);
      if(status != 0)
      {
         printf("error in matrix multiply\n");
         fflush(stdout);
         return;
      }
for(i = 0; i < 2; i++)
   printf("   BTWmat[%d][0] = %.10lf  BTWmat[%d][1] = %.10lf\n",
              i, BTWmat[i][0], i, BTWmat[i][1]);
fflush(stdout);
      status = matrix_multiply(BTWmat, Bmat, 3, 2, 2, 3, Nsinglemat);
      if(status != 0)
      {
         printf("error in matrix multiply\n");
         fflush(stdout);
         return;
      }
for(i = 0; i < 3; i++)
   printf("   Nsinglemat[%d][0] = %.10lf  Nsinglemat[%d][1] = %.10lf  Nsinglemat[%d][2] = %.10lf\n",
              i, Nsinglemat[i][0], i, Nsinglemat[i][1], i, Nsinglemat[i][2]);
fflush(stdout);
      status = matrix_multiply(BTWmat, vector, 3, 2, 2, 1, Vsinglemat);
      if(status != 0)
      {
         printf("error in matrix multiply\n");
         fflush(stdout);
         return;
      }
for(i = 0; i < 3; i++)
   printf("   Vsinglemat[%d][0] = %.10lf\n", i, Vsinglemat[i][0]);
fflush(stdout);
      for(i = 0; i < 3; i++)
      {
         for(j = 0; j < 3; j++)
         {
            Nmat[i][j] += Nsinglemat[i][j];
         }
      }
      for(i = 0; i < 3; i++)
      {
         Vmat[i][0] += Vsinglemat[i][0];
      }
      if(iteration == (MAX_ITERATIONS - 1))
      {
         Converged = 1;
         continue;
      }
      /* since we're only using one image,
       *     zero out the last row and col and set last element to 1.0
       */
      Nmat[0][2] = Nmat[2][0] = 0.0;
      Nmat[1][2] = Nmat[2][1] = 0.0;
      Nmat[2][2] = 1.0;
      Vmat[2][0] = 0.0;

for(i = 0; i < 3; i++)
   printf("   Nmat[%d][0] = %.3e  Nmat[%d][1] = %.3e  Nmat[%d][2] = %.3e\n",
              i, Nmat[i][0], i, Nmat[i][1], i, Nmat[i][2]);
fflush(stdout);
status =  matrix_inverse_3x3(Nmat, Ninv);
if(status != 0)
{
   printf("problem computing matrix inverse\n");
   fflush(stdout);
   return;
}
//      matrix_inverse(Nmat, 3, Ninv);
for(i = 0; i < 3; i++)
   printf("   Ninv[%d][0] = %.3e  Ninv[%d][1] = %.3e  Ninv[%d][2] = %.3e\n",
              i, Ninv[i][0], i, Ninv[i][1], i, Ninv[i][2]);
fflush(stdout);
printf("*****************************************\n");
printf(" Just for kicks - multiply Nmat and Ninv\n");
printf("*****************************************\n");
double **Kickmat, **Kinv;;
Kickmat = (double **)calloc(3, sizeof(double *));
Kinv    = (double **)calloc(3, sizeof(double *));
for(i = 0; i < 3; i++)
{
   Kickmat[i] = (double *)calloc(3, sizeof(double));
   Kinv[i]    = (double *)calloc(3, sizeof(double));
}
status = matrix_multiply(Nmat, Ninv, 3, 3, 3, 3, Kickmat);
if(status != 0)
{
   printf("error in matrix multiply\n");
   fflush(stdout);
   return;
}
for(i = 0; i < 3; i++)
   printf("   Kickmat[%d][0] = %.3e  Kickmat[%d][1] = %.3e  Kickmat[%d][2] = %.3e\n",
              i, Kickmat[i][0], i, Kickmat[i][1], i, Kickmat[i][2]);
fflush(stdout);
for(i = 0; i < 3; i++)
   for(j = 0; j < 3; j++)
      Kickmat[i][j] = 0.0;
status =  matrix_inverse_3x3(Nmat, Kinv);
if(status != 0)
{
   printf("problem computing matrix inverse\n");
   fflush(stdout);
   return;
}
for(i = 0; i < 3; i++)
   printf("   Kinv[%d][0] = %.3e  Kinv[%d][1] = %.3e  Kinv[%d][2] = %.3e\n",
              i, Kinv[i][0], i, Kinv[i][1], i, Kinv[i][2]);
fflush(stdout);
status = matrix_multiply(Nmat, Kinv, 3, 3, 3, 3, Kickmat);
if(status != 0)
{
   printf("error in matrix multiply\n");
   fflush(stdout);
   return;
}
for(i = 0; i < 3; i++)
   printf("   Kickmat[%d][0] = %.3e  Kickmat[%d][1] = %.3e  Kickmat[%d][2] = %.3e\n",
              i, Kickmat[i][0], i, Kickmat[i][1], i, Kickmat[i][2]);
fflush(stdout);

for(i = 0; i < 3; i++) free(Kickmat[i]);
for(i = 0; i < 3; i++) free(Kinv[i]);
free(Kickmat);
free(Kinv);
printf("*****************************************\n");
printf("            End of kicks\n");
printf("*****************************************\n");
for(i = 0; i < 3; i++)
   printf("   Vmat[%d][0] = %.10lf\n", i, Vmat[i][0]);
fflush(stdout);
      status = matrix_multiply(Ninv, Vmat, 3, 3, 3, 1, Deltamat);
      if(status != 0)
      {
         printf("error in matrix multiply\n");
         fflush(stdout);
         return;
      }
for(i = 0; i < 3; i++)
   printf("   Deltamat[%d][0] = %.10lf\n", i, Deltamat[i][0]);
fflush(stdout);

      /* compute magnitude of the delta vector */
      deltamag = 0.0;
      for(i = 0; i < 3; i++)
      {
         deltamag += (Deltamat[i][0] * Deltamat[i][0]);
      }

      /* add the deltas back in to the geo position and try again */
printf("   Current pos:  %.10lf  %.10lf  %.10lf\n", init_pos.y, init_pos.x, init_pos.z);
fflush(stdout);
      init_pos.y += Deltamat[0][0];
      init_pos.x += Deltamat[1][0];
      init_pos.z += Deltamat[2][0];
printf("   Adjustments:  %.10lf  %.10lf  %.10lf\n", Deltamat[0][0], Deltamat[1][0], Deltamat[2][0]);
printf("   New pos:      %.10lf  %.10lf  %.10lf\n", init_pos.y, init_pos.x, init_pos.z);
printf("   deltamag = %.10lf\n", deltamag);
fflush(stdout);

      /* test to see if convergence criteria is met */
      if(deltamag < CONVERGENCE_TOLERANCE)
      {
         iteration = MAX_ITERATIONS;
         Converged = 1;
      }
      iteration++;

   } /* end while(iteration) */

   if(!Converged)
   {
      printf("!!! Warning !!! Solution didn't converge\n");
      fflush(stdout);
   }

   /* set the ground point */
   gpos->x = init_pos.x;
   gpos->y = init_pos.y;
   gpos->z = init_pos.z;

   /* free the memory */
   for(i = 0; i < 2; i++) free(Wgtmat[i]);
   free(Wgtmat);
   for(i = 0; i < 3; i++) free(Bmat[i]);
   free(Bmat);
   for(i = 0; i < 2; i++)
   {
      free(BTmat[i]);
      free(BTWmat[i]);
   }
   free(BTmat);
   free(BTWmat);
   for(i = 0; i < 3; i++)
   {
      free(Nsinglemat[i]);
      free(Nmat[i]);
      free(Ninv[i]);
   }
   free(Nsinglemat);
   free(Nmat);
   free(Ninv);
   for(i = 0; i < 3; i++)
   {
      free(Vsinglemat[i]);
      free(Vmat[i]);
      free(Deltamat[i]);
   }
   free(Vsinglemat);
   free(Vmat);
   free(Deltamat);
   for(i = 0; i < 2; i++) free(vector[i]);
   free(vector);

   return;
}






void Rpc::groundToImage(GroundPos gpos, RpcStruct *rpcs, ImagePos *impos)
{
   double normLat, normLon, normHgt;
   double linenum, lineden, samplenum, sampleden;

   /* normalize the ground position */
   normLon = (gpos.x - rpcs->lon_off) / rpcs->lon_scale;
   normLat = (gpos.y - rpcs->lat_off) / rpcs->lat_scale;
   normHgt = (gpos.z - (double)rpcs->hgt_off) / (double)rpcs->hgt_scale;

   /* compute line numerator */
   linenum = evaluateRpc20TermPoly(rpcs->line_num_coeff, normLon, normLat, normHgt);

   /* compute line denominator */
   lineden = evaluateRpc20TermPoly(rpcs->line_den_coeff, normLon, normLat, normHgt);

   /* compute sample numerator */
   samplenum = evaluateRpc20TermPoly(rpcs->sample_num_coeff, normLon, normLat, normHgt);

   /* compute sample denominator */
   sampleden = evaluateRpc20TermPoly(rpcs->sample_den_coeff, normLon, normLat, normHgt);

   /* compute line,sample */
   impos->l = ((linenum / lineden) * (double)rpcs->line_scale) + (double)rpcs->line_off;
   impos->s = ((samplenum / sampleden) * (double)rpcs->sample_scale) + (double)rpcs->sample_off;

   return;
}


/* Evaluate a 20-term polynomial given (P,L,H)
 * This polynomial is either the numerator or denominator
 * of the RPC00B model.
 */
double Rpc::evaluateRpc20TermPoly(double poly[20], double L, double P, double H)
{
   double val;

   val = (poly[0] + (poly[5] * L * H) + (poly[10] * L * P * H) +
         (poly[15] * P * P * P) + (poly[1] * L) + (poly[6] * P * H) +
         (poly[11] * L * L * L) + (poly[16] * P * H * H) + (poly[2] * P) +
         (poly[7] * L * L) + (poly[12] * P * P * L) + (poly[17] * L * L * H) +
         (poly[3] * H) + (poly[8] * P * P) + (poly[13] * L * H * H) +
         (poly[18] * P * P * H) + (poly[4] * P * L) + (poly[9] * H * H) +
         (poly[14] * P * L * L) + (poly[19] * H * H * H));

   return(val);
}


/* Evaluate the partial derivative of the RPCs (numerator or denominator)
 * wrt (P,L,H) at the given position Phi, Lambda, Height
 */
void Rpc::evaluateRpcPartialAtPLH(double poly[20], double L, double P, double H, double result[3])
{
   /* evaluate partial wrt L */
   result[0] = (poly[1] + (poly[4] * P) + (poly[5] * H) + (2.0 * poly[7] * L) +
               (poly[10] * P * H) + (3.0 * poly[11] * L * L) + (poly[12] * P * P) +
               (poly[13] * H * H) + (2.0 * poly[14] * L * P) + (2.0 * poly[17] * L * H));
   /* evaluate partial wrt P */
   result[1] = (poly[2] + (poly[4] * L) + (poly[6] * H) + (2.0 * poly[8] * P) +
               (poly[10] * L * H) + (2.0 * poly[12]* L * P) + (poly[14] * L * L) +
               (3.0 * poly[15] * P * P) + (poly[16] * H * H) + (2.0 * poly[18] * P * H));
   /* evaluate partial wrt H */
   result[2] = (poly[3] + (poly[5] * L) + (poly[6] * P) + (2.0 * poly[9] * H) +
               (poly[10] * L * P) + (2.0 * poly[13] * L * H) + (2.0 * poly[16] * P * H) +
               (poly[17] * L * L) + (poly[18] * P * P) + (3.0 * poly[19] * H * H));
   return;
}


/* Evaluate the partial derivative of the RPCs wrt (P,L,H) at the given
 * at the given position Phi, Lambda, Height and return the X,Y value
 * Also, while we're here, compute the 'B' matrix
 */
int Rpc::evaluateRpcPartialAtPLHAndCompB(
                RpcStruct *rpcs,
                double L, double P, double H,
                ImagePos *ls, double **B)
{
   int col;
   double linen, lined, lined_sq;
   double samplen, sampled, sampled_sq;
   double partial_num_line[3], partial_den_line[3];
   double partial_num_sample[3], partial_den_sample[3];

printf("         P,L,H           = %.10lf  %.10lf  %.10lf\n", P, L, H);
fflush(stdout);

   /* compute l,s at the normalized ground position (PLH) */
   linen   = evaluateRpc20TermPoly(rpcs->line_num_coeff, L, P, H);
   lined   = evaluateRpc20TermPoly(rpcs->line_den_coeff, L, P, H);
   samplen = evaluateRpc20TermPoly(rpcs->sample_num_coeff, L, P, H);
   sampled = evaluateRpc20TermPoly(rpcs->sample_den_coeff, L, P, H);

   if(lined == 0.0)
   {
      printf("error:  line denominator = 0\n");
      fflush(stdout);
      return(-1);
   }
   if(sampled == 0.0)
   {
      printf("error:  sample denominator = 0\n");
      fflush(stdout);
      return(-1);
   }
   /* compute line,sample */
   ls->l = linen / lined;
   ls->s = samplen / sampled;
printf("         linen,lined     = %.10lf  %.10lf\n", linen, lined);
printf("         samplen,sampled = %.10lf  %.10lf\n", samplen, sampled);
printf("         line,sample     = %.10lf  %.10lf\n", ls->l, ls->s);
fflush(stdout);

   /* compute partials of the nums and dens at P,L,H */
   evaluateRpcPartialAtPLH(rpcs->line_num_coeff, L, P, H, partial_num_line);
   evaluateRpcPartialAtPLH(rpcs->line_den_coeff, L, P, H, partial_den_line);
   evaluateRpcPartialAtPLH(rpcs->sample_num_coeff, L, P, H, partial_num_sample);
   evaluateRpcPartialAtPLH(rpcs->sample_den_coeff, L, P, H, partial_den_sample);

printf("         partial linen   = %.10lf  %.10lf  %.10lf\n", partial_num_line[0], partial_num_line[1], partial_num_line[2]);
printf("         partial lined   = %.10lf  %.10lf  %.10lf\n", partial_den_line[0], partial_den_line[1], partial_den_line[2]);
printf("         partial samplen = %.10lf  %.10lf  %.10lf\n", partial_num_sample[0], partial_num_sample[1], partial_num_sample[2]);
printf("         partial sampled = %.10lf  %.10lf  %.10lf\n", partial_den_sample[0], partial_den_sample[1], partial_den_sample[2]);
fflush(stdout);
   /* compute the B matrix using the quotient rule:
    *       (f/g)'(x) = [g(x)*f'(x) - f(x)*g'(x)] / [g(x)]^2
    *
    *    since our order is LPH and we need PLH - switch here
    */
   lined_sq   = lined * lined;
   sampled_sq = sampled * sampled;
   B[0][0] = (((lined   * partial_num_line[1])   - (linen   * partial_den_line[1]))   / (lined_sq));
   B[0][1] = (((lined   * partial_num_line[0])   - (linen   * partial_den_line[0]))   / (lined_sq));
   B[0][2] = (((lined   * partial_num_line[2])   - (linen   * partial_den_line[2]))   / (lined_sq));
   B[1][0] = (((sampled * partial_num_sample[1]) - (samplen * partial_den_sample[1])) / (sampled_sq));
   B[1][1] = (((sampled * partial_num_sample[0]) - (samplen * partial_den_sample[0])) / (sampled_sq));
   B[1][2] = (((sampled * partial_num_sample[2]) - (samplen * partial_den_sample[2])) / (sampled_sq));
for(col = 0; col < 2; col++)
{
   printf("         B[%d][0] = %.10lf   B[%d][1] = %.10lf  B[%d][2] = %.10lf\n", col, B[col][0], col, B[col][1], col, B[col][2]);
   fflush(stdout);
}
   return(0);
}




}


