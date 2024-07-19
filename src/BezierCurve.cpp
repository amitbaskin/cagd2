#include <vector>
#include <stdexcept>
#include "BezierCurve.h"
#include <cmath>


// Helper function to calculate binomial coefficients
GLdouble binomialCoefficient( int n, int k )
{
  if( k > n || k < 0 ) return 0;
  GLdouble result = 1;
  for( int i = 0; i < k; ++i )
  {
    result *= ( n - i );
    result /= ( i + 1 );
  }
  return result;
}

void BezierCurve::calculateMatrixM( std::vector<std::vector<GLdouble>> &M ) const
{
  int n = control_points.size() - 1;
  M.resize( n + 1, std::vector<GLdouble>( n + 1, 0.0 ) );

  // Populate matrix M with correct coefficients
  for( int i = 0; i <= n; ++i )
  {
    for( int j = 0; j <= i; ++j )
    {
      M[ i ][ j ] = binomialCoefficient( n, i ) * binomialCoefficient( i, j ) * std::pow( -1, i - j );
    }
  }
}

// Compute the matrix-vector product M * P and cache the result
void BezierCurve::computeMP() const
{
  int n = control_points.size() - 1;
  MP_cache.resize( n + 1 );

  // Construct the Bernstein basis matrix M
  std::vector<std::vector<GLdouble>> base_matrix;
  calculateMatrixM( base_matrix );

  // Compute M * P
  for( int i = 0; i <= n; ++i )
  {
    MP_cache[ i ].x = 0.0;
    MP_cache[ i ].y = 0.0;
    for( int j = 0; j <= n; ++j )
    {
      MP_cache[ i ].x += base_matrix[ i ][ j ] * control_points[ j ].x;
      MP_cache[ i ].y += base_matrix[ i ][ j ] * control_points[ j ].y;
    }
  }
}

// Constructor for BezierCurve
BezierCurve::BezierCurve( const std::vector<CAGD_POINT> &points )
  : control_points( points )
{
  computeMP(); // Compute MP on initialization
}

// Evaluate the Bézier curve at parameter t
CAGD_POINT BezierCurve::evaluate( GLdouble t ) const
{
  int n = control_points.size() - 1;

  // Construct vector T
  std::vector<GLdouble> T( n + 1 );
  GLdouble t_pow = 1.0;
  for( int i = 0; i <= n; ++i )
  {
    T[ i ] = t_pow;
    t_pow *= t;
  }

  // Ensure MP is cached
  if( MP_cache.empty() )
  {
    computeMP();
  }

  // Compute the final result using T * MP
  CAGD_POINT point = { 0.0, 0.0, 0.0 };

  for( int i = 0; i <= n; ++i )
  {
    point.x += T[ i ] * MP_cache[ i ].x;
    point.y += T[ i ] * MP_cache[ i ].y;
  }

  return point;
}

// Add a control point and recache the MP
void BezierCurve::addControlPoint( const CAGD_POINT &new_point )
{
  control_points.push_back( new_point );
  computeMP();
}

// Remove a control point and recache the MP
void BezierCurve::removeControlPoint( size_t index )
{
  if( index >= control_points.size() )
  {
    throw std::out_of_range( "Index out of range." );
  }
  control_points.erase( control_points.begin() + index );
  computeMP();
}

// Update a control point and recache the MP
void BezierCurve::updateControlPoint( size_t index, const CAGD_POINT &new_point )
{
  if( index >= control_points.size() )
  {
    throw std::out_of_range( "Index out of range." );
  }
  control_points[ index ] = new_point;
  computeMP();
}
