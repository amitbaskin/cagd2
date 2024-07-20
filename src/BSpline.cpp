#include <stdio.h>
#include "BSpline.h"
#include <algorithm>
#include <iostream>
#include <options.h>
#include <color.h>
#include <vectors.h>

void BSpline::show_crv() const
{
  double normalized_num_samps = get_default_num_steps() /
                                u_vec_[ u_vec_.size() - 1 ];

  double seg_ids_num = seg_ids_.size();

  cagdSetColor( color_[ 0 ], color_[ 1 ], color_[ 2 ] );

  for( size_t u_idx = 0; u_idx < u_vec_.size() - 1; ++u_idx )
  {
    double delta  = u_vec_[ u_idx + 1 ] - u_vec_[ u_idx ];
    int num_samps = ( int )( delta * normalized_num_samps );
    double jump   = 1.0 / num_samps;

    auto pnts = new CAGD_POINT[ num_samps ];

    if( pnts != NULL )
    {
      for( int samp_idx = 0; samp_idx < num_samps; ++samp_idx )
      {
        double param = u_vec_[ u_idx ] + jump * samp_idx;

        if( double_cmp( param, u_vec_[ u_idx + 1 ] ) > 0 )
        {
          param = u_vec_[ u_idx + 1 ];
          pnts[ samp_idx ] = evaluate( param );
          break;
        }
        else
          pnts[ samp_idx ] = evaluate( param );
      }

      if( u_idx < seg_ids_num )
        cagdReusePolyline( seg_ids_[ u_idx ], pnts, num_samps );
      else
      {
        int seg_id = cagdAddPolyline( pnts, num_samps );
        seg_ids_.push_back( seg_id );
        map_seg_to_crv( seg_id, ( Curve * )this );
      }
    }

    delete[] pnts;
  }

  set_default_color();
}

void BSpline::print() const
{
  printf( "Curve crv_type: bspline\n" );
  printf( "order_: %u\n", order_ );
  printf( "Number of knots_: %u\n", knots_.size() );

  if( !knots_.empty() )
  {
    printf( "knots_: " );

    for( const double &knot : knots_ )
      printf( "%f ", knot );

    printf( "\n" );
  }

  Curve::print();
}

// Utility method to find the knot span index
int BSpline::findKnotSpan( double t ) const
{
  int n = ctrl_pnts_.size() - 1;
  int p = order_ - 1;

  // Special case when t is at the end of the knot vector
  if( t >= knots_[ n + 1 ] )
  {
    return n;
  }

  // Binary search to find the knot span
  int low = p;
  int high = n + 1;
  int mid = ( low + high ) / 2;

  while( t < knots_[ mid ] || t >= knots_[ mid + 1 ] )
  {
    if( t < knots_[ mid ] )
    {
      high = mid;
    }
    else
    {
      low = mid;
    }
    mid = ( low + high ) / 2;
  }

  return mid;
}

// Utility method to evaluate basis functions
void BSpline::evaluateBasisFunctions( int span, double t, double *N ) const
{
  int p = order_ - 1;
  double_vec left( p + 1 );
  double_vec right( p + 1 );

  N[ 0 ] = 1.0;
  for( int j = 1; j <= p; ++j )
  {
    left[ j ] = t - knots_[ span + 1 - j ];
    right[ j ] = knots_[ span + j ] - t;
    double saved = 0.0;
    for( int r = 0; r < j; ++r )
    {
      double temp = N[ r ] / ( right[ r + 1 ] + left[ j - r ] );
      N[ r ] = saved + right[ r + 1 ] * temp;
      saved = left[ j - r ] * temp;
    }
    N[ j ] = saved;
  }
}

// Method to evaluate the B-spline at parameter t
CAGD_POINT BSpline::evaluate( double param ) const
{
  int pp = order_ - 1;
  int nn = ctrl_pnts_.size() - 1;

  if( param < knots_[ pp ] || param > knots_[ nn + 1 ] )
  {
    throw std::out_of_range( "Parameter t is out of range." );
  }

  int span = findKnotSpan( param );
  double *NN = new double[ order_ + 1 ];
  evaluateBasisFunctions( span, param, NN );

  CAGD_POINT CC = { 0.0, 0.0, 0.0 };

  for( int j = 0; j <= pp; ++j )
  {
    CC.x += NN[ j ] * ctrl_pnts_[ span - pp + j ].x;
    CC.y += NN[ j ] * ctrl_pnts_[ span - pp + j ].y;
  }

  delete[] NN;

  return CC;
}
