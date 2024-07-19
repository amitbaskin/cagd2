// BSpline.cpp
#include "BSpline.h"
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <sstream>

// Constructor
BSpline::BSpline( const point_vec &controlPoints, const double_vec &knots, UINT order )
  : controlPoints( controlPoints ), knots( knots ), order( order ), degree( order - 1 )
{
  if( controlPoints.size() < 2 || knots.size() < controlPoints.size() + degree + 1 )
  {
    throw std::invalid_argument( "Invalid control points or knots vector" );
  }
}

// Evaluate the B-spline at parameter t
CAGD_POINT BSpline::evaluate( double t ) const
{
// Check the cache first
  auto it = cache.find( t );
  if( it != cache.end() )
  {
    return it->second;
  }

  CAGD_POINT result = computePoint( t );

  // Cache the result
  cache[ t ] = result;

  return result;
}

// Compute the B-spline point at parameter t using basis functions
CAGD_POINT BSpline::computePoint( double t ) const
{
  CAGD_POINT result = { 0.0, 0.0, 0.0 };
  for( size_t i = 0; i < controlPoints.size(); ++i )
  {
    double basisVal = basisFunction( i, degree, t );
    CAGD_POINT basisPoint = pointMultiply( controlPoints[ i ], basisVal );
    result = pointAdd( result, basisPoint );
  }
  return result;
}

// Compute the B-spline basis function
double BSpline::basisFunction( size_t i, UINT k, double t ) const
{
  if( k == 0 )
  {
    return ( t >= knots[ i ] && t < knots[ i + 1 ] ) ? 1.0 : 0.0;
  }
  else
  {
    double denom1 = knots[ i + k ] - knots[ i ];
    double denom2 = knots[ i + k + 1 ] - knots[ i + 1 ];
    double term1 = 0.0;
    double term2 = 0.0;

    if( denom1 != 0.0 )
    {
      term1 = ( t - knots[ i ] ) / denom1 * basisFunction( i, k - 1, t );
    }
    if( denom2 != 0.0 )
    {
      term2 = ( knots[ i + k + 1 ] - t ) / denom2 * basisFunction( i + 1, k - 1, t );
    }

    return term1 + term2;
  }
}

// Multiply a point by a scalar
CAGD_POINT BSpline::pointMultiply( const CAGD_POINT &p, double scalar ) const
{
  return { p.x * scalar, p.y * scalar, p.z * scalar };
}

// Add two points
CAGD_POINT BSpline::pointAdd( const CAGD_POINT &p1, const CAGD_POINT &p2 ) const
{
  return { p1.x + p2.x, p1.y + p2.y, p1.z + p2.z };
}
