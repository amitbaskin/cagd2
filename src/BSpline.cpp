#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <options.h>
#include <color.h>
#include <vectors.h>
#include <cmath>

#include "BSpline.h"
#include "Bezier.h"
#include "crv_utils.h"


/******************************************************************************
* BSpline::connectC0_bezier
******************************************************************************/
void BSpline::connectC0_bezier( const Bezier *other )
{
  ctrl_pnts_.back() = other->ctrl_pnts_.front();

  int degree = order_ - 1;
  int knotCount = knots_.size() - 1;
  for( int i = 1; i <= degree; ++i )
  {
    knots_[ knotCount - i ] = knots_[ knotCount - degree - 1 ];
  }
}

/******************************************************************************
* BSpline::connectC1_bezier
******************************************************************************/
void BSpline::connectC1_bezier( const Bezier *other )
{
  connectC0_bezier( other );

  if( ctrl_pnts_.size() > 1 && other->ctrl_pnts_.size() > 1 )
  {
    CAGD_POINT startPointOther = other->ctrl_pnts_.front();
    CAGD_POINT secondCtrlPointOther = other->ctrl_pnts_[ 1 ];

    ctrl_pnts_[ ctrl_pnts_.size() - 2 ] = {
        startPointOther.x - ( secondCtrlPointOther.x - startPointOther.x ),
        startPointOther.y - ( secondCtrlPointOther.y - startPointOther.y ),
        startPointOther.z - ( secondCtrlPointOther.z - startPointOther.z )
    };
  }
}

/******************************************************************************
* BSpline::connectG1_bezier
******************************************************************************/
void BSpline::connectG1_bezier( const Bezier *other )
{
  connectC0_bezier( other );

  if( ctrl_pnts_.size() > 1 && other->ctrl_pnts_.size() > 1 )
  {
    CAGD_POINT startPointOther = other->ctrl_pnts_.front();
    CAGD_POINT secondCtrlPointOther = other->ctrl_pnts_[ 1 ];

    double dxOther = secondCtrlPointOther.x - startPointOther.x;
    double dyOther = secondCtrlPointOther.y - startPointOther.y;
    double dzOther = secondCtrlPointOther.z - startPointOther.z;

    double lengthOther = sqrt( dxOther * dxOther +
                               dyOther * dyOther +
                               dzOther * dzOther );

    if( lengthOther != 0 )
    {
      dxOther /= lengthOther;
      dyOther /= lengthOther;
      dzOther /= lengthOther;
    }

    CAGD_POINT endPointThis = ctrl_pnts_.back();

    ctrl_pnts_[ ctrl_pnts_.size() - 2 ] =
    {
      endPointThis.x - dxOther,
      endPointThis.y - dyOther,
      endPointThis.z - dzOther
    };
  }
}

/******************************************************************************
* BSpline::connectC0_bspline
******************************************************************************/
void BSpline::connectC0_bspline( const BSpline *other )
{
  double tEndThis = knots_.back();
  CAGD_POINT endPointThis = evaluate( tEndThis );

  double tStartOther = other->knots_.front();
  CAGD_POINT startPointOther = other->evaluate( tStartOther );

  ctrl_pnts_.back() = startPointOther;

  int degree = order_ - 1;
  int knotCount = knots_.size() - 1;

  for( int i = 1; i <= degree; ++i )
    knots_[ knotCount - i ] = knots_[ knotCount - degree - 1 ];
}

/******************************************************************************
* BSpline::connectC1_bspline
******************************************************************************/
void BSpline::connectC1_bspline( const BSpline *other )
{
  connectC0_bspline( other );

  if( ctrl_pnts_.size() > 1 && other->ctrl_pnts_.size() > 1 )
  {
    CAGD_POINT startPointOther = other->ctrl_pnts_.front();
    CAGD_POINT secondCtrlPointOther = other->ctrl_pnts_[ 1 ];

    double dxOther = secondCtrlPointOther.x - startPointOther.x;
    double dyOther = secondCtrlPointOther.y - startPointOther.y;
    double dzOther = secondCtrlPointOther.z - startPointOther.z;

    ctrl_pnts_[ ctrl_pnts_.size() - 2 ] =
    {
      startPointOther.x - dxOther,
      startPointOther.y - dyOther,
      startPointOther.z - dzOther
    };
  }
}

/******************************************************************************
* BSpline::connectG1_bspline
******************************************************************************/
void BSpline::connectG1_bspline( const BSpline *other )
{
  connectC0_bspline( other );

  if( ctrl_pnts_.size() > 1 && other->ctrl_pnts_.size() > 1 )
  {
    CAGD_POINT startPointOther = other->ctrl_pnts_.front();
    CAGD_POINT secondCtrlPointOther = other->ctrl_pnts_[ 1 ];

    double dxOther = secondCtrlPointOther.x - startPointOther.x;
    double dyOther = secondCtrlPointOther.y - startPointOther.y;
    double dzOther = secondCtrlPointOther.z - startPointOther.z;

    double lengthOther = sqrt( dxOther * dxOther +
                               dyOther * dyOther +
                               dzOther * dzOther );

    if( lengthOther != 0 )
    {
      dxOther /= lengthOther;
      dyOther /= lengthOther;
      dzOther /= lengthOther;
    }

    CAGD_POINT endPointThis = ctrl_pnts_.back();

    ctrl_pnts_[ ctrl_pnts_.size() - 2 ] =
    {
      endPointThis.x - dxOther,
      endPointThis.y - dyOther,
      endPointThis.z - dzOther
    };
  }
}

/******************************************************************************
* BSpline::rmv_ctrl_pnt
******************************************************************************/
void BSpline::rmv_ctrl_pnt( int idx )
{
  Curve::rmv_ctrl_pnt( idx );

  if( is_uni_ )
  {
    makeUniformKnotVector();

    if( is_open_ )
      makeOpenKnotVector();

    show_crv( idx, CtrlOp::RMV );
  }
}

/******************************************************************************
* Bezier::add_ctrl_pnt
******************************************************************************/
void BSpline::add_ctrl_pnt( CAGD_POINT &ctrl_pnt, int idx )
{
  Curve::add_ctrl_pnt( ctrl_pnt, idx );

  if( is_uni_ )
  {
    makeUniformKnotVector();

    if( is_open_ )
      makeOpenKnotVector();

    show_crv( idx, CtrlOp::ADD );
  }
}

/******************************************************************************
* BSpline::update_u_vec
******************************************************************************/
void BSpline::update_u_vec()
{
  size_t knots_num = knots_.size();
  double prev_knot = knots_.front();
  double cur_knot = prev_knot;

  u_vec_.clear();

  for( size_t i = 1; i < knots_num; ++i )
  {
    cur_knot = knots_[ i ];

    if( double_cmp( cur_knot, prev_knot ) > 0 )
    {
      prev_knot = cur_knot;
      u_vec_.push_back( cur_knot );
    }
  }
}

/******************************************************************************
* BSpline::makeUniformKnotVector
******************************************************************************/
void BSpline::makeUniformKnotVector()
{
  int numControlPoints = ctrl_pnts_.size();
  int degree = order_ - 1;

  double minKnotValue = DEF_START_DOM;
  double maxKnotValue = DEF_END_DOM;

  double prev_knot = -HUGE_DOUBLE;
  double cur_knot = -HUGE_DOUBLE;

  if( knots_.size() > 1 )
  {
    double minKnotValue = knots_.front();
    double maxKnotValue = knots_.back();
  }

  knots_.clear();

  double range = maxKnotValue - minKnotValue;

  for( int i = 0; i <= numControlPoints + degree; ++i )
  {
    cur_knot = minKnotValue + ( range * i ) /
               ( ( double )numControlPoints + ( double )degree );

    knots_.push_back( cur_knot );
  }

  update_u_vec();
}

/******************************************************************************
* Bezier::makeOpenKnotVector
******************************************************************************/
void BSpline::makeOpenKnotVector()
{
  int numControlPoints = ctrl_pnts_.size();
  int degree = order_ - 1;

  double minKnotValue = knots_.front();
  double maxKnotValue = knots_.back();

  if( knots_.empty() )
    knots_.resize( numControlPoints + order_ );

  for( int i = 0; i <= degree; ++i )
    knots_[ i ] = minKnotValue;

  for( int i = numControlPoints; i < numControlPoints + order_; ++i )
    knots_[ i ] = maxKnotValue;

  update_u_vec();
}

/******************************************************************************
* BSpline::show_crv
******************************************************************************/
void BSpline::show_crv( int chg_ctrl_idx, CtrlOp op ) const
{
  if( false && chg_ctrl_idx != K_NOT_USED )
  {
    std::vector< int > u_vec_idxs = findAffectedSegments( chg_ctrl_idx );
    show_crv_helper( u_vec_idxs );
  }
  else
  {
    /*std::vector< int > u_vec_idxs;

    for( size_t i = 0; i < u_vec_.size(); ++i )
      u_vec_idxs.push_back( i );

    show_crv_helper( u_vec_idxs );*/

    size_t num_steps = get_default_num_steps();
    auto pnts = new CAGD_POINT[ num_steps ];

    if( pnts != NULL )
    {
      double min_val = knots_[ order_ -1 ];
      double max_val = knots_[ ctrl_pnts_.size() ];
      double delta = max_val - min_val;
      double jump = 1.0 / ( double )( num_steps - 1 );

      cagdSetColor( color_[ 0 ], color_[ 1 ], color_[ 2 ] );

      for( size_t i = 0; i < num_steps; ++i )
      {
        double param = min_val + delta * jump * i;
        pnts[ i ] = evaluate( param );
      }

      if( seg_ids_.size() > 0 )
        cagdReusePolyline( seg_ids_[ 0 ], pnts, num_steps );
      else
      {
        int seg_id = cagdAddPolyline( pnts, num_steps );
        seg_ids_.push_back( seg_id );
        map_seg_to_crv( seg_id, ( Curve * )this );
      }

      delete[] pnts;
    }
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

    seg_ids_.resize( u_vec_idxs.size() - 1 );

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
  int degree = order_ - 1;

  if( param < knots_[ degree ] || param > knots_[ ctrl_pnts_.size() ] )
    throw std::out_of_range( "Parameter t is out of range." );

  int span = findKnotSpan( param );
  double *NN = new double[ order_ + 1 ];
  evaluateBasisFunctions( span, param, NN );

  CAGD_POINT CC = { 0.0, 0.0, 0.0 };

  double weight_sum = 0.0;

  for( int j = 0; j <= degree; ++j )
  {
    double wNN = NN[ j ] * ctrl_pnts_[ span - degree + j ].z;
    CC.x += wNN * ctrl_pnts_[ span - degree + j ].x;
    CC.y += wNN * ctrl_pnts_[ span - degree + j ].y;
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

  double T_i = knots_[ controlPointIndex ];
  double T_i_k_plus_1 = knots_[ controlPointIndex + degree + 1 ];

  for( size_t i = 0; i < u_vec_.size() - 1; ++i )
  {
    if( u_vec_[ i ] >= T_i && u_vec_[ i ] < T_i_k_plus_1 )
      affectedSegments.push_back( i );
  }

  return affectedSegments;
}

/******************************************************************************
* BSpline::interpolate
******************************************************************************/
CAGD_POINT BSpline::interpolate( const CAGD_POINT &P1, const CAGD_POINT &P2,
                                 double t ) const
{
  CAGD_POINT result;
  result.x = P1.x + t * ( P2.x - P1.x );
  result.y = P1.y + t * ( P2.y - P1.y );
  return result;
}

/******************************************************************************
* BSpline::distance
******************************************************************************/
double BSpline::distance( const CAGD_POINT &P1, const CAGD_POINT &P2 ) const
{
  return std::sqrt( std::pow( P2.x - P1.x, 2 ) + std::pow( P2.y - P1.y, 2 ) );
}

/******************************************************************************
* BSpline::ensureNonDecreasingKnotVector
******************************************************************************/
void BSpline::ensureNonDecreasingKnotVector()
{
  for( size_t i = 1; i < knots_.size(); ++i )
  {
    if( knots_[ i ] < knots_[ i - 1 ] )
      std::swap( knots_[ i ], knots_[ i - 1 ] );
  }
}

/******************************************************************************
* BSpline::updateUniqueKnotsAndMultiplicity
******************************************************************************/
void BSpline::updateUniqueKnotsAndMultiplicity()
{
  u_vec_.clear();
  multiplicity_.clear();

  for( double knot : knots_ )
  {
    if( u_vec_.empty() || u_vec_.back() != knot )
    {
      u_vec_.push_back( knot );
      multiplicity_.push_back( 1 );
    }
    else
      multiplicity_.back()++;
  }
}

/******************************************************************************
* BSpline::addKnot
******************************************************************************/
void BSpline::addKnot( double new_knot )
{
  auto it = std::lower_bound( knots_.begin(), knots_.end(), new_knot );
  int insert_pos = it - knots_.begin();
  knots_.insert( it, new_knot );
  ensureNonDecreasingKnotVector();
  updateUniqueKnotsAndMultiplicity();

  point_vec new_ctrl_pnts;
  int p = order_ - 1;

  for( int i = 0; i <= insert_pos - p - 1; ++i )
    new_ctrl_pnts.push_back( ctrl_pnts_[ i ] );

  for( int i = insert_pos - p; i <= insert_pos - 1; ++i )
  {
    double alpha = ( new_knot - knots_[ i ] ) / ( knots_[ i + p + 1 ] - knots_[ i ] );
    CAGD_POINT new_point;
    new_point.x = ( 1.0 - alpha ) * ctrl_pnts_[ i - 1 ].x + alpha * ctrl_pnts_[ i ].x;
    new_point.y = ( 1.0 - alpha ) * ctrl_pnts_[ i - 1 ].y + alpha * ctrl_pnts_[ i ].y;
    new_ctrl_pnts.push_back( new_point );
  }

  for( size_t i = insert_pos; i < ctrl_pnts_.size(); ++i )
    new_ctrl_pnts.push_back( ctrl_pnts_[ i ] );

  ctrl_pnts_ = new_ctrl_pnts;
}

/******************************************************************************
* BSpline::updateKnot
******************************************************************************/
void BSpline::updateKnot( int knot_idx, double new_param )
{
  if( ( size_t )knot_idx >= knots_.size() )
    throw std::runtime_error( "wrong knot idx" );

  knots_[ knot_idx ] = new_param;
  ensureNonDecreasingKnotVector();
  updateUniqueKnotsAndMultiplicity();
}

/******************************************************************************
* BSpline::rmvKnot
******************************************************************************/
void BSpline::rmvKnot( int knot_idx )
{
  double knot = knots_[ knot_idx ];

  if( ( size_t )knot_idx >= knots_.size() )
    throw std::runtime_error( "wrong knot idx" );

  knots_.erase( knots_.begin() + knot_idx );

  updateUniqueKnotsAndMultiplicity();

  point_vec new_ctrl_pnts;
  int p = order_ - 1;

  for( int i = 0; i <= knot_idx - p - 1; ++i )
    new_ctrl_pnts.push_back( ctrl_pnts_[ i ] );

  for( int i = knot_idx - p; i <= knot_idx - 1; ++i )
  {
    double alpha = ( knot - knots_[ i ] ) / ( knots_[ i + p + 1 ] - knots_[ i ] );
    CAGD_POINT new_point;
    new_point.x = ( ctrl_pnts_[ i ].x - ( 1.0 - alpha ) * ctrl_pnts_[ i - 1 ].x ) / alpha;
    new_point.y = ( ctrl_pnts_[ i ].y - ( 1.0 - alpha ) * ctrl_pnts_[ i - 1 ].y ) / alpha;
    new_ctrl_pnts.push_back( new_point );
  }

  for( size_t i = knot_idx; i < ctrl_pnts_.size(); ++i )
    new_ctrl_pnts.push_back( ctrl_pnts_[ i ] );

  ctrl_pnts_ = new_ctrl_pnts;
}
