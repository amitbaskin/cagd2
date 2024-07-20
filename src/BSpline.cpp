#include <stdio.h>
#include "BSpline.h"
#include <algorithm>
#include <iostream>

void BSpline::print()
{
  printf( "Curve crv_type: bspline\n" );
  printf( "Order: %u\n", order );

  printf( "Number of knots: %u\n", knots.size() );

  if( !knots.empty() )
  {
    printf( "Knots: " );
    for( const double &knot : knots )
    {
      printf( "%f ", knot );
    }

    printf( "\n" );
  }

  printf( "Control Points:\n" );

  for( const CAGD_POINT &pt : ctrl_pnts )
  {
    printf( "(%f, %f", pt.x, pt.y );
    printf( ", %f", pt.z );
    printf( ")\n" );
  }

  printf( "\n\n\n" );
}

// Utility method to find the knot span index
int BSpline::findKnotSpan( double t ) const
{
  int n = ctrl_pnts.size() - 1;
  int p = order - 1;

  // Special case when t is at the end of the knot vector
  if( t >= knots[ n + 1 ] )
  {
    return n;
  }

  // Binary search to find the knot span
  int low = p;
  int high = n + 1;
  int mid = ( low + high ) / 2;

  while( t < knots[ mid ] || t >= knots[ mid + 1 ] )
  {
    if( t < knots[ mid ] )
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
  int p = order - 1;
  double_vec left( p + 1 );
  double_vec right( p + 1 );

  N[ 0 ] = 1.0;
  for( int j = 1; j <= p; ++j )
  {
    left[ j ] = t - knots[ span + 1 - j ];
    right[ j ] = knots[ span + j ] - t;
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
  int p = order - 1;
  int n = ctrl_pnts.size() - 1;

  if( t < knots[ p ] || t > knots[ n + 1 ] )
  {
    throw std::out_of_range( "Parameter t is out of range." );
  }

  int span = findKnotSpan( t );
  double *N = new double[ order ];
  evaluateBasisFunctions( span, t, N );

  CAGD_POINT C = { 0.0, 0.0, 0.0 };
  for( int j = 0; j <= p; ++j )
  {
    C.x += N[ j ] * ctrl_pnts[ span - p + j ].x;
    C.y += N[ j ] * ctrl_pnts[ span - p + j ].y;
  }

  delete[] N;
  return C;
}
