#include <vector>
#include <stdexcept>
#include "Bezier.h"
#include <cmath>

void Bezier::print()
{
  printf( "Curve crv_type: Bezier\n" );
  printf( "Order: %u\n", order );
  printf( "Control Points:\n" );

  for( const CAGD_POINT &pt : ctrl_pnts )
  {
    printf( "(%f, %f", pt.x, pt.y );
    printf( ", %f", pt.z );
    printf( ")\n" );
  }

  printf( "\n\n\n" );
}

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

void Bezier::calculateMatrixM( std::vector<std::vector<GLdouble>> &M ) const
{
  int n = ctrl_pnts.size() - 1;
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
void Bezier::computeMP() const
{
  int n = ctrl_pnts.size() - 1;
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
      MP_cache[ i ].x += base_matrix[ i ][ j ] * ctrl_pnts[ j ].x;
      MP_cache[ i ].y += base_matrix[ i ][ j ] * ctrl_pnts[ j ].y;
    }
  }
}

// Constructor for Bezier
Bezier::Bezier( const std::vector<CAGD_POINT> &points )
  : ctrl_pnts( points )
{
  computeMP(); // Compute MP on initialization
}

// Evaluate the Bézier Curve at parameter t
CAGD_POINT Bezier::evaluate( GLdouble t ) const
{
  int n = ctrl_pnts.size() - 1;

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
void Bezier::addControlPoint( const CAGD_POINT &new_point )
{
  ctrl_pnts.push_back( new_point );
  computeMP();
}

// Remove a control point and recache the MP
void Bezier::removeControlPoint( size_t index )
{
  if( index >= ctrl_pnts.size() )
  {
    throw std::out_of_range( "Index out of range." );
  }
  ctrl_pnts.erase( ctrl_pnts.begin() + index );
  computeMP();
}

// Update a control point and recache the MP
void Bezier::updateControlPoint( size_t index, const CAGD_POINT &new_point )
{
  if( index >= ctrl_pnts.size() )
  {
    throw std::out_of_range( "Index out of range." );
  }
  ctrl_pnts[ index ] = new_point;
  computeMP();
}
