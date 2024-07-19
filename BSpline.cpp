// BSpline.cpp
#include "BSpline.h"
#include <algorithm> // For std::lower_bound

BSpline::BSpline( const point_vec &controlPoints, const double_vec &knots, int order )
  : controlPoints( controlPoints ), knots( knots ), order( order )
{}

double BSpline::basisFunction( int i, int k, double t ) const
{
  if( k == 0 )
  {
    return ( t >= knots[ i ] && t < knots[ i + 1 ] ) ? 1.0 : 0.0;
  }
  else
  {
    double denom1 = knots[ i + k ] - knots[ i ];
    double denom2 = knots[ i + k + 1 ] - knots[ i + 1 ];
    double coeff1 = denom1 != 0.0 ? ( t - knots[ i ] ) / denom1 : 0.0;
    double coeff2 = denom2 != 0.0 ? ( knots[ i + k + 1 ] - t ) / denom2 : 0.0;
    return ( coeff1 * basisFunction( i, k - 1, t ) ) + ( coeff2 * basisFunction( i + 1, k - 1, t ) );
  }
}

int BSpline::findKnotSpan( double t ) const
{
// Binary search to find the knot span index
  auto it = std::lower_bound( knots.begin(), knots.end(), t );
  return std::distance( knots.begin(), it ) - 1;
}

CAGD_POINT BSpline::evaluate( double t ) const
{
  int n = controlPoints.size() - 1;
  int p = order - 1;
  int span = findKnotSpan( t );

  CAGD_POINT result = { 0.0, 0.0, 0.0 }; // Assuming CAGD_POINT is a 3D point (x, y, z)

  for( int i = 0; i <= p; ++i )
  {
    double coeff = basisFunction( span - p + i, p, t );
    result.x += coeff * controlPoints[ span - p + i ].x;
    result.y += coeff * controlPoints[ span - p + i ].y;
    result.z += coeff * controlPoints[ span - p + i ].z;
  }

  return result;
}
