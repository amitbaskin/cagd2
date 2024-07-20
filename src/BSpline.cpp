#include <stdio.h>
#include "BSpline.h"
#include <algorithm>
#include <iostream>
#include <options.h>
#include <color.h>

void BSpline::show_crv() const
{
  double normalized_num_samps = get_default_num_steps() /
                                u_vec_[ u_vec_.size() - 1 ];

  double seg_ids_num = seg_ids_.size();

  cagdSetColor( color_[ 0 ], color_[ 1 ], color_[ 2 ] );

  for( size_t i = 0; i < u_vec_.size() - 1; ++i )
  {
    double delta  = u_vec_[ i + 1 ] - u_vec_[ i ];
    int num_samps = ( int )( delta * normalized_num_samps );
    double jump   = 1.0 / normalized_num_samps;

    auto pnts = new CAGD_POINT[ num_samps ];

    if( pnts != NULL )
    {
      for( int i = 0; i < num_samps; ++i )
        pnts[ i ] = evaluate( jump * i );

      if( i < seg_ids_num && seg_ids_[ i ] == K_NOT_USED )
        seg_ids_[ i ] = cagdAddPolyline( pnts, num_samps );
      else
        cagdReusePolyline( seg_ids_[ i ], pnts, num_samps );

      set_default_color();
    }

    delete[] pnts;
  }
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
CAGD_POINT BSpline::evaluate( double t ) const
{
  int p = order_ - 1;
  int n = ctrl_pnts_.size() - 1;

  if( t < knots_[ p ] || t > knots_[ n + 1 ] )
  {
    throw std::out_of_range( "Parameter t is out of range." );
  }

  int span = findKnotSpan( t );
  double *N = new double[ order_ ];
  evaluateBasisFunctions( span, t, N );

  CAGD_POINT C = { 0.0, 0.0, 0.0 };
  for( int j = 0; j <= p; ++j )
  {
    C.x += N[ j ] * ctrl_pnts_[ span - p + j ].x;
    C.y += N[ j ] * ctrl_pnts_[ span - p + j ].y;
  }

  delete[] N;
  return C;
}
