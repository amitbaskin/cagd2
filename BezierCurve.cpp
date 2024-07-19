#include "BezierCurve.h"
#include <cmath> // For std::pow
#include <stdexcept> // For std::out_of_range

BezierCurve::BezierCurve( const point_vec &controlPoints )
  : controlPoints( controlPoints )
{
  computeBasisMatrix(); // Compute the basis matrix initially
}

CAGD_POINT BezierCurve::evaluate( double t ) const
{
  int n = controlPoints.size() - 1;
  CAGD_POINT point = { 0.0, 0.0, 0.0 };

  for( int i = 0; i <= n; ++i )
  {
    double B = bernsteinPolynomial( i, n, t );
    point.x += B * controlPoints[ i ].x;
    point.y += B * controlPoints[ i ].y;
    point.z += B * controlPoints[ i ].z;
  }

  return point;
}

void BezierCurve::updateControlPoint( int index, const CAGD_POINT &newPoint )
{
  if( index >= 0 && index < static_cast< int >( controlPoints.size() ) )
  {
    controlPoints[ index ] = newPoint;
    computeBasisMatrix(); // Recompute the basis matrix if control points change
  }
  else
  {
    throw std::out_of_range( "Control point index out of range." );
  }
}

double BezierCurve::binomialCoefficient( int n, int k ) const
{
  if( k > n ) return 0;
  double result = 1;
  for( int i = 0; i < k; ++i )
  {
    result *= ( n - i );
    result /= ( i + 1 );
  }
  return result;
}

double BezierCurve::bernsteinPolynomial( int i, int n, double t ) const
{
  int key = i * 100000 + n; // Create a simple key (adjust multiplier as needed)
  auto it = basisMatrixCache.find( key );
  if( it != basisMatrixCache.end() )
  {
    return it->second;
  }

  double result = binomialCoefficient( n, i ) * std::pow( t, i ) * std::pow( 1 - t, n - i );
  basisMatrixCache[ key ] = result;
  return result;
}

void BezierCurve::computeBasisMatrix() const
{
  basisMatrixCache.clear();
  int n = controlPoints.size() - 1;
  for( int i = 0; i <= n; ++i )
  {
    for( double t = 0.0; t <= 1.0; t += 0.01 )
    { // Example resolution for t
      bernsteinPolynomial( i, n, t );
    }
  }
}
