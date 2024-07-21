#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <options.h>
#include <color.h>
#include <vectors.h>

#include "BSpline.h"
#include "crv_utils.h"

/******************************************************************************
* BSpline::rmv_ctrl_pnt
******************************************************************************/
void BSpline::rmv_ctrl_pnt( int idx )
{
  // TODO - how to update the knot vector accordingly???

  show_crv( idx, CtrlOp::RMV );
}

/******************************************************************************
* Bezier::add_ctrl_pnt
******************************************************************************/
void BSpline::add_ctrl_pnt( CAGD_POINT &ctrl_pnt, int idx )
{
  Curve::add_ctrl_pnt( ctrl_pnt, idx );

  // TODO - how to update the knot vector accordingly???

  show_crv( idx, CtrlOp::ADD );
}

/******************************************************************************
* BSpline::show_crv
******************************************************************************/
void BSpline::show_crv( int chg_ctrl_idx, CtrlOp op ) const
{
  // TODO how to calc affected segments when add / rmv ctrl point???

  if( chg_ctrl_idx != K_NOT_USED )
  {
    std::vector< int > u_vec_idxs = findAffectedSegments( chg_ctrl_idx );
    show_crv_helper( u_vec_idxs );
  }
  else
  {
    std::vector< int > u_vec_idxs;

    for( size_t i = 0; i < u_vec_.size(); ++i )
      u_vec_idxs.push_back( i );

    show_crv_helper( u_vec_idxs );
  }
}

/******************************************************************************
* BSpline::show_crv_helper
******************************************************************************/
void BSpline::show_crv_helper( std::vector< int > u_vec_idxs ) const
{
  double normalized_num_samps = get_default_num_steps() /
    u_vec_[ u_vec_.size() - 1 ];

  double seg_ids_num = seg_ids_.size();

  cagdSetColor( color_[ 0 ], color_[ 1 ], color_[ 2 ] );

  for( size_t i = 0; i < u_vec_idxs.size() - 1; ++i )
  {
    double delta = u_vec_[ u_vec_idxs[ i + 1 ] ] - u_vec_[ u_vec_idxs[ i ] ];

    int num_samps = ( int )( delta * normalized_num_samps );
    double jump = 1.0 / num_samps;

    auto pnts = new CAGD_POINT[ num_samps ];

    if( pnts != NULL )
    {
      for( int samp_idx = 0; samp_idx < num_samps; ++samp_idx )
      {
        double param = u_vec_[ u_vec_idxs[ i ] ] + jump * samp_idx;

        if( double_cmp( param, u_vec_[ u_vec_idxs[ i + 1 ] ] ) > 0 )
        {
          param = u_vec_[ u_vec_idxs[ i + 1 ] ];
          pnts[ samp_idx ] = evaluate( param );
          break;
        }
        else
          pnts[ samp_idx ] = evaluate( param );
      }

      if( u_vec_idxs[ i ] < seg_ids_num )
        cagdReusePolyline( seg_ids_[ u_vec_idxs[ i ] ], pnts, num_samps );
      else
      {
        int seg_id = cagdAddPolyline( pnts, num_samps );
        seg_ids_.push_back( seg_id );
        map_seg_to_crv( seg_id, ( Curve * )this );
      }
    }

    delete[] pnts;
  }
}

/******************************************************************************
* BSpline::print
******************************************************************************/
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

/******************************************************************************
* BSpline::findKnotSpan
******************************************************************************/
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

/******************************************************************************
* BSpline::evaluateBasisFunctions
******************************************************************************/
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

/******************************************************************************
* BSpline::evaluate
******************************************************************************/
CAGD_POINT BSpline::evaluate( double param ) const
{
  int pp = order_ - 1;
  int nn = ctrl_pnts_.size() - 1;

  if( param < knots_[ pp ] || param > knots_[ nn + 1 ] )
  {
    throw std::out_of_range( "Parameter t is out of range." );
  }

  int span = findKnotSpan( param );
  double *NN = new double[ order_ + 1 ];
  evaluateBasisFunctions( span, param, NN );

  CAGD_POINT CC = { 0.0, 0.0, 0.0 };

  double weight_sum = 0.0;

  for( int j = 0; j <= pp; ++j )
  {
    double wNN = NN[ j ] * ctrl_pnts_[ span - pp + j ].z;
    CC.x += wNN * ctrl_pnts_[ span - pp + j ].x;
    CC.y += wNN * ctrl_pnts_[ span - pp + j ].y;
    weight_sum += wNN;
  }

  if( double_cmp( weight_sum, 0.0 ) != 0 )
  {
    CC.x /= weight_sum;
    CC.y /= weight_sum;
  }

  delete[] NN;

  return CC;
}

/******************************************************************************
* BSpline::findAffectedSegments
******************************************************************************/
std::vector<int> BSpline::findAffectedSegments( int controlPointIndex ) const
{
  std::vector<int> affectedSegments;
  int degree = order_ - 1;

  // Ensure the control point index is within valid range
  if( ( size_t )controlPointIndex < 0 ||
      ( size_t )controlPointIndex >= ctrl_pnts_.size() )
  {
    throw std::out_of_range( "Control point index is out of range." );
  }

  // Knot span influenced by the control point P_i
  double T_i = knots_[ controlPointIndex ];
  double T_i_k_plus_1 = knots_[ controlPointIndex + degree + 1 ];

  // Iterate over the unique knot values in u_vec_
  for( size_t i = 0; i < u_vec_.size() - 1; ++i )
  {
    if( u_vec_[ i ] >= T_i && u_vec_[ i ] < T_i_k_plus_1 )
    {
      affectedSegments.push_back( i );
    }
  }

  return affectedSegments;
}