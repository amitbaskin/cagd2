#include "BezierCurve.h"
#include <cmath> // For std::pow

// Constructor
BezierCurve::BezierCurve( const point_vec &points ) : control_points( points )
{
// Compute the base matrix when initializing
  computeBaseMatrix();
}

// Helper function to compute binomial coefficient C(n, k)
int BezierCurve::binomialCoefficient( int n, int k ) const
{
  if( k > n - k )
    k = n - k;

  int c = 1;
  for( int i = 0; i < k; ++i )
  {
    c = c * ( n - i ) / ( i + 1 );
  }
  return c;
}

// Computes the Bernstein base matrix for the Bezier curve
void BezierCurve::computeBaseMatrix()
{
  int n = control_points.size() - 1; // Degree of the Bezier curve
  base_matrix_cache.resize( n + 1 );

  for( int i = 0; i <= n; ++i )
  {
    base_matrix_cache[ i ].resize( n + 1 );
  }

  // Populate the base matrix cache
  for( int i = 0; i <= n; ++i )
  {
    for( int j = 0; j <= i; ++j )
    {
      base_matrix_cache[ i ][ j ] = binomialCoefficient( n, i ) * std::pow( 1.0, n - i ) * std::pow( 0.0, i );
    }
  }
}

// Evaluates the Bezier curve at parameter t
CAGD_POINT BezierCurve::evaluate( GLdouble t ) const
{
  int n = control_points.size() - 1;
  CAGD_POINT point = { 0.0, 0.0, 0.0 };

  for( int i = 0; i <= n; ++i )
  {
    GLdouble blend = binomialCoefficient( n, i ) * std::pow( 1 - t, n - i ) * std::pow( t, i );
    point.x += blend * control_points[ i ].x;
    point.y += blend * control_points[ i ].y;
  }

  return point;
}
