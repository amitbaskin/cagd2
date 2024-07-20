#include <vector>
#include <stdexcept>
#include "Bezier.h"
#include <cmath>
#include "options.h"
#include "color.h"

void Bezier::print() const
{
  printf( "Curve crv_type: Bezier\n" );
  Curve::print();
}

void Bezier::show_crv() const
{
  cagdSetColor( color_[ 0 ], color_[ 1 ], color_[ 2 ] );

  unsigned int def_num_steps = get_default_num_steps();

  double jump = 1.0 / ( double )def_num_steps;

  auto pnts = new CAGD_POINT[ def_num_steps ];

  if( pnts != NULL )
  {
    for( unsigned int i = 0; i < def_num_steps; ++i )
      pnts[ i ] = evaluate( jump * i );

    if( seg_ids_[ 0 ] == K_NOT_USED )
      seg_ids_[ 0 ] = cagdAddPolyline( pnts, def_num_steps );
    else
      cagdReusePolyline( seg_ids_[ 0 ], pnts, def_num_steps );

    set_default_color();
  }

  delete[] pnts;
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
  int n = ctrl_pnts_.size() - 1;
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
  int n = ctrl_pnts_.size() - 1;
  MP_cache_.resize( n + 1 );

  // Construct the Bernstein basis matrix M
  std::vector<std::vector<GLdouble>> base_matrix;
  calculateMatrixM( base_matrix );

  // Compute M * P
  for( int i = 0; i <= n; ++i )
  {
    MP_cache_[ i ].x = 0.0;
    MP_cache_[ i ].y = 0.0;
    for( int j = 0; j <= n; ++j )
    {
      MP_cache_[ i ].x += base_matrix[ i ][ j ] * ctrl_pnts_[ j ].x;
      MP_cache_[ i ].y += base_matrix[ i ][ j ] * ctrl_pnts_[ j ].y;
    }
  }
}

CAGD_POINT Bezier::evaluate( GLdouble t ) const
{
  int n = ctrl_pnts_.size() - 1;

  // Construct vector T
  std::vector<GLdouble> T( n + 1 );
  GLdouble t_pow = 1.0;
  for( int i = 0; i <= n; ++i )
  {
    T[ i ] = t_pow;
    t_pow *= t;
  }

  // Ensure MP is cached
  if( MP_cache_.empty() )
  {
    computeMP();
  }

  // Compute the final result using T * MP
  CAGD_POINT point = { 0.0, 0.0, 0.0 };

  for( int i = 0; i <= n; ++i )
  {
    point.x += T[ i ] * MP_cache_[ i ].x;
    point.y += T[ i ] * MP_cache_[ i ].y;
  }

  return point;
}

// Add a control point and recache the MP
void Bezier::addControlPoint( const CAGD_POINT &new_point )
{
  ctrl_pnts_.push_back( new_point );
  computeMP();
}

// Remove a control point and recache the MP
void Bezier::removeControlPoint( size_t index )
{
  if( index >= ctrl_pnts_.size() )
  {
    throw std::out_of_range( "Index out of range." );
  }
  ctrl_pnts_.erase( ctrl_pnts_.begin() + index );
  computeMP();
}

// Update a control point and recache the MP
void Bezier::updateControlPoint( size_t index, const CAGD_POINT &new_point )
{
  if( index >= ctrl_pnts_.size() )
  {
    throw std::out_of_range( "Index out of range." );
  }
  ctrl_pnts_[ index ] = new_point;
  computeMP();
}
