#include <vector>
#include <stdexcept>
#include "Bezier.h"
#include <cmath>

#include "options.h"
#include "color.h"
#include "crv_utils.h"

/******************************************************************************
* Bezier::print
******************************************************************************/
void Bezier::print() const
{
  printf( "Curve crv_type: Bezier\n" );
  Curve::print();
}

/******************************************************************************
* Bezier::show_crv
******************************************************************************/
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

    if( seg_ids_.size() > 0 )
      cagdReusePolyline( seg_ids_[ 0 ], pnts, def_num_steps );
    else
    {
      int seg_id = cagdAddPolyline( pnts, def_num_steps );
      seg_ids_.push_back( seg_id );
      map_seg_to_crv( seg_id, ( Curve * )this );
    }

    set_default_color();
  }

  delete[] pnts;
}

/******************************************************************************
* Bezier::binomialCoefficient
******************************************************************************/
GLdouble binomialCoefficient( int n, int k )
{
  if( k > n || k < 0 ) return 0;

  int result = 1;

  for( int i = 0; i < k; ++i )
  {
    result *= ( n - i );
    result /= ( i + 1 );
  }
  return result;
}

/******************************************************************************
* Bezier::calculateMatrixM
******************************************************************************/
void Bezier::calculateMatrixM( std::vector<std::vector<GLdouble>> &M ) const
{
  int n = ctrl_pnts_.size() - 1;
  M.resize( n + 1, std::vector<GLdouble>( n + 1, 0.0 ) );

  for( int i = 0; i <= n; ++i )
  {
    for( int j = 0; j <= i; ++j )
    {
      M[ i ][ j ] = binomialCoefficient( n, i ) *
        binomialCoefficient( i, j ) * std::pow( -1, i - j );
    }
  }
}

/******************************************************************************
* Bezier::computeMP
******************************************************************************/
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

/******************************************************************************
* Bezier::evaluate
******************************************************************************/
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
