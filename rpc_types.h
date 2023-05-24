#ifndef RPC_TYPES_H
#define RPC_TYPES_H

namespace UC {


#define INIT_RPC(rpc)       memset(rpc, 0, sizeof(rpc_t))

typedef struct rpc_t
{
    double err_bias;
#define RPC_ERR_BIAS_SET        (0x0001)
#define RPC_ERR_BIAS_STR        "ERR_BIAS"

    double err_rand;
#define RPC_ERR_RAND_SET        (0x0002)
#define RPC_ERR_RAND_STR        "ERR_RAND"

    int line_off;
#define RPC_LINE_OFF_SET        (0x0004)
#define RPC_LINE_OFF_STR        "LINE_OFF"

    int sample_off;
#define RPC_SAMPLE_OFF_SET      (0x0008)
#define RPC_SAMPLE_OFF_STR      "SAMP_OFF"

    double lat_off;
#define RPC_LAT_OFF_SET         (0x0010)
#define RPC_LAT_OFF_STR         "LAT_OFF"

    double lon_off;
#define RPC_LON_OFF_SET         (0x0020)
#define RPC_LON_OFF_STR         "LONG_OFF"

    int hgt_off;
#define RPC_HGT_OFF_SET         (0x0040)
#define RPC_HGT_OFF_STR         "HEIGHT_OFF"

    int line_scale;
#define RPC_LINE_SCALE_SET      (0x0080)
#define RPC_LINE_SCALE_STR      "LINE_SCALE"

    int sample_scale;
#define RPC_SAMPLE_SCALE_SET    (0x0100)
#define RPC_SAMPLE_SCALE_STR    "SAMP_SCALE"

    double lat_scale;
#define RPC_LAT_SCALE_SET       (0x0200)
#define RPC_LAT_SCALE_STR       "LAT_SCALE"

    double lon_scale;
#define RPC_LON_SCALE_SET       (0x0400)
#define RPC_LON_SCALE_STR       "LONG_SCALE"

    int hgt_scale;
#define RPC_HGT_SCALE_SET       (0x0800)
#define RPC_HGT_SCALE_STR       "HEIGHT_SCALE"

#define RPC_COEFFS      (20)

    double line_num_coeff[RPC_COEFFS];
#define LINE_NUM_COEFF_SET      (0x1000)
#define LINE_NUM_COEFF_STR      "LINE_NUM_COEFF"

    double line_den_coeff[RPC_COEFFS];
#define LINE_DEN_COEFF_SET      (0x2000)
#define LINE_DEN_COEFF_STR      "LINE_DEN_COEFF"

    double sample_num_coeff[RPC_COEFFS];
#define SAMPLE_NUM_COEFF_SET    (0x4000)
#define SAMPLE_NUM_COEFF_STR    "SAMP_NUM_COEFF"

    double sample_den_coeff[RPC_COEFFS];
#define SAMPLE_DEN_COEFF_SET    (0x8000)
#define SAMPLE_DEN_COEFF_STR    "SAMP_DEN_COEFF"

    unsigned int set_bits;

#define RPC_INT_STR_VALS    {RPC_LINE_OFF_STR, RPC_SAMPLE_OFF_STR, \
                            RPC_HGT_OFF_STR, RPC_LINE_SCALE_STR, \
                            RPC_SAMPLE_SCALE_STR, RPC_HGT_SCALE_STR, \
                            NULL}

#define RPC_INT_SET_VALS    {RPC_LINE_OFF_SET, RPC_SAMPLE_OFF_SET, \
                            RPC_HGT_OFF_SET, RPC_LINE_SCALE_SET, \
                            RPC_SAMPLE_SCALE_SET, RPC_HGT_SCALE_SET}


#define RPC_DBL_STR_VALS    {RPC_ERR_BIAS_STR, RPC_ERR_RAND_STR, \
                            RPC_LAT_OFF_STR, RPC_LON_OFF_STR, \
                            RPC_LAT_SCALE_STR, RPC_LON_SCALE_STR, \
                            NULL}

#define RPC_DBL_SET_VALS    {RPC_ERR_BIAS_SET, RPC_ERR_RAND_SET, \
                            RPC_LAT_OFF_SET, RPC_LON_OFF_SET, \
                            RPC_LAT_SCALE_SET, RPC_LON_SCALE_SET}


#define RPC_DBL_ARY_STR_VALS {LINE_NUM_COEFF_STR, SAMPLE_NUM_COEFF_STR, \
                           SAMPLE_DEN_COEFF_STR, NULL}

#define RPC_DBL_ARY_SET_VALS {LINE_NUM_COEFF_SET, SAMPLE_NUM_COEFF_SET, \
                           SAMPLE_DEN_COEFF_SET}

} RpcStruct;


typedef struct gpos_t
{
   double x;  /* lat */
   double y;  /* lon */
   double z;  /* hgt */
} GroundPos;


typedef struct ipos_t
{
   double l;  /* line */
   double s;  /* sample */
} ImagePos;




}



#endif // RPC_TYPES_H
