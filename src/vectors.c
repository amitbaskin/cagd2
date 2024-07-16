#include <stdio.h>
#include <math.h>
#include "vectors.h"


/******************************************************************************
* scale_not_zero
******************************************************************************/
int scale_not_zero( double scale )
{
  return fabs( scale ) > EPSILON;
}


/******************************************************************************
* get_scale_inv_or_zero
******************************************************************************/
int get_scale_inv_or_zero( double scale, double *rp_res )
{
  int is_error = !scale_not_zero( scale );

  *rp_res = is_error == TRUE ? 0.0 : 1 / scale;

  return is_error;
}


/******************************************************************************
* scale_div_vec
******************************************************************************/
int scale_div_vec( double denom, CAGD_POINT *rp_out )
{
  double res = K_NOT_USED;

  int is_error = get_scale_inv_or_zero( denom, &res );

  scale_vec( res, rp_out );

  return is_error;
}


/******************************************************************************
* normalize_vec
******************************************************************************/
int normalize_vec( CAGD_POINT *p_vec )
{
  return scale_div_vec( vec_len( p_vec ), p_vec );
}


/******************************************************************************
* double_cmp
******************************************************************************/
int double_cmp( double scale_1, double scale_2 )
{
  double diff = scale_1 - scale_2;

  int is_zero = !scale_not_zero( diff );

  return is_zero ? 0 : diff > EPSILON ? 1 : -1;
}


/******************************************************************************
* is_scale_initialized
******************************************************************************/
int is_scale_initialized( double scale )
{
  return double_cmp( scale, -HUGE_DOUBLE );
}


/******************************************************************************
* vec_not_zero
******************************************************************************/
int vec_not_zero( const CAGD_POINT *p_vec )
{
  int status = FALSE;

  status = status || scale_not_zero( p_vec->x );
  status = status || scale_not_zero( p_vec->y );
  status = status || scale_not_zero( p_vec->z );

  return status;
}


/******************************************************************************
* copy_vec
******************************************************************************/
void copy_vec( const CAGD_POINT *p_in, CAGD_POINT *rp_out )
{
  rp_out->x = p_in->x;
  rp_out->y = p_in->y;
  rp_out->z = p_in->z;
}


/******************************************************************************
* vec_len
******************************************************************************/
double vec_len( const CAGD_POINT *p_vec )
{
  double sum = p_vec->x * p_vec->x +
               p_vec->y * p_vec->y +
               p_vec->z * p_vec->z;;

  return sqrt( sum );
}


/******************************************************************************
* scale_vec
******************************************************************************/
void scale_vec( double scale, CAGD_POINT *p_vec )
{
  p_vec->x *= scale;
  p_vec->y *= scale;
  p_vec->z *= scale;
}


/******************************************************************************
* diff_vecs
******************************************************************************/
void diff_vecs( const CAGD_POINT *p_v1,
                const CAGD_POINT *p_v2,
                CAGD_POINT       *rp_out )
{
  rp_out->x = p_v1->x - p_v2->x;
  rp_out->y = p_v1->y - p_v2->y;
  rp_out->z = p_v1->z - p_v2->z;
}


/******************************************************************************
* add_vecs
******************************************************************************/
void add_vecs( const CAGD_POINT *p_v1,
               const CAGD_POINT *p_v2,
               CAGD_POINT       *rp_out )
{
  rp_out->x = p_v1->x + p_v2->x;
  rp_out->y = p_v1->y + p_v2->y;
  rp_out->z = p_v1->z + p_v2->z;
}


/******************************************************************************
* multiply_vecs
******************************************************************************/

double multiply_vecs( const CAGD_POINT *p_v1, const CAGD_POINT *p_v2 )
{
  return p_v1->x * p_v2->x + p_v1->y * p_v2->y + p_v1->z * p_v2->z;
}


/******************************************************************************
* cross_vecs
******************************************************************************/
void cross_vecs( const CAGD_POINT *p_v1,
                 const CAGD_POINT *p_v2,
                 CAGD_POINT       *rp_out )
{
  /*
  |ii|jj|kk|
  |ax|ay|az|
  |bx|by|bz|
  */
  rp_out->x = p_v1->y * p_v2->z - p_v1->z * p_v2->y;
  rp_out->y = p_v1->z * p_v2->x - p_v1->x * p_v2->z;
  rp_out->z = p_v1->x * p_v2->y - p_v1->y * p_v2->x;
}


/******************************************************************************
* rotate_vec
******************************************************************************/
void rotate_vec( double      angle,
                 CAGD_POINT *p_in,
                 CAGD_POINT *p_rot,
                 CAGD_POINT *p_out )
{
  double cos_a = cos( angle );
  double sin_a = sin( angle );

  double ux = p_rot->x;
  double uy = p_rot->y;
  double uz = p_rot->z;

  double mat[ 3 ][ 3 ] =
  {
    { cos_a + ux * ux * ( 1 - cos( angle )             ),
      ux * uy * ( 1 - cos( angle ) ) - uz * sin( angle ),
      ux * uz * ( 1 - cos( angle ) ) + uy * sin( angle ) },

    { uy * ux * ( 1 - cos( angle ) ) + uz * sin( angle ),
      cos( angle ) + uy * uy * ( 1 - cos( angle )      ),
      uy * uz * ( 1 - cos( angle ) ) - ux * sin( angle ) },

    { uz * ux * ( 1 - cos( angle ) ) - uy * sin( angle ),
      uz * uy * ( 1 - cos( angle ) ) + ux * sin( angle ),
      cos( angle ) + uz * uz * ( 1 - cos( angle )      ) }
  };

  p_out->x = mat[ 0 ][ 0 ] * p_in->x +
             mat[ 0 ][ 1 ] * p_in->y +
             mat[ 0 ][ 2 ] * p_in->z;

  p_out->y = mat[ 1 ][ 0 ] * p_in->x +
             mat[ 1 ][ 1 ] * p_in->y +
             mat[ 1 ][ 2 ] * p_in->z;

  p_out->z = mat[ 2 ][ 0 ] * p_in->x +
             mat[ 2 ][ 1 ] * p_in->y +
             mat[ 2 ][ 2 ] * p_in->z;
}
