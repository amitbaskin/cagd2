#pragma once

#include "cagd.h"

#define K_NOT_USED     -1
#define EPSILON        1e-9
#define HUGE_DOUBLE    1e9
#define DEFAULT_DOUBLE -HUGE_DOUBLE

int scale_not_zero( double scale );

int get_scale_inv_or_zero( double scale, double *rp_res );

int scale_div_vec_2d( double denom, CAGD_POINT *rp_out );
int scale_div_vec( double denom, CAGD_POINT *rp_out );

int double_cmp( double scale_1, double scale_2 );

int vec_not_zero( const CAGD_POINT *p_vec );

void copy_vec( const CAGD_POINT *p_in, CAGD_POINT *rp_out );

double vec_len( const CAGD_POINT *p_vec );
double vec_len_2d( const CAGD_POINT *p_vec );

void scale_vec_2d( double scale, CAGD_POINT *p_vec );
void scale_vec( double scale, CAGD_POINT *p_vec );

int normalize_vec( CAGD_POINT *p_vec );

double multiply_vecs( const CAGD_POINT *p_v1, const CAGD_POINT *p_v2 );

void cross_vecs( const CAGD_POINT *p_v1,
                 const CAGD_POINT *p_v2,
                 CAGD_POINT *rp_out );

void diff_vecs( const CAGD_POINT *p_v1,
                const CAGD_POINT *p_v2,
                CAGD_POINT *rp_out );

void add_vecs( const CAGD_POINT *p_v1,
               const CAGD_POINT *p_v2,
               CAGD_POINT *rp_out );

void diff_vecs_2d( const CAGD_POINT *p_v1,
                   const CAGD_POINT *p_v2,
                   CAGD_POINT *rp_out );

void add_vecs_2d( const CAGD_POINT *p_v1,
                  const CAGD_POINT *p_v2,
                  CAGD_POINT *rp_out );

int normalize_vec_2d( CAGD_POINT *p_vec );

void rotate_vec( double      angle,
                 CAGD_POINT *p_in,
                 CAGD_POINT *p_rot,
                 CAGD_POINT *p_out );
