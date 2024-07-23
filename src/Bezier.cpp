#include <vector>
#include <stdexcept>
#include "Bezier.h"
#include <cmath>

#include "options.h"
#include "color.h"
#include "crv_utils.h"
#include <vectors.h>
#include <BSpline.h>


/******************************************************************************
* Bezier::print
******************************************************************************/
void Bezier::print() const
{
  printf( "Curve crv_type: Bezier\n" );
  Curve::print();
}

/******************************************************************************
* Bezier::rmv_ctrl_pnt
******************************************************************************/
void Bezier::rmv_ctrl_pnt( int idx )
{
  Curve::rmv_ctrl_pnt( idx );
}

/******************************************************************************
* Bezier::add_ctrl_pnt
******************************************************************************/
void Bezier::add_ctrl_pnt( const CAGD_POINT &ctrl_pnt, int idx )
{
  Curve::add_ctrl_pnt( ctrl_pnt, idx );
}

/******************************************************************************
* Bezier::connectC0_bezier
******************************************************************************/
void Bezier::connectC0_bezier( const Bezier *other )
{
  ctrl_pnts_[ ctrl_pnts_.size() - 1 ] = other->ctrl_pnts_.front();
}

/******************************************************************************
* Bezier::connectC1_bezier
******************************************************************************/
void Bezier::connectC1_bezier( const Bezier *other )
{
  connectC0_bezier( other );

  if( ctrl_pnts_.size() > 1 && other->ctrl_pnts_.size() > 1 )
  {
    CAGD_POINT secondLastCtrlPntThis = ctrl_pnts_[ ctrl_pnts_.size() - 2 ];
    CAGD_POINT firstCtrlPntOther = other->ctrl_pnts_.front();
    CAGD_POINT secondCtrlPntOther = other->ctrl_pnts_[ 1 ];

    CAGD_POINT tangentDirection =
    {
      secondCtrlPntOther.x - firstCtrlPntOther.x,
      secondCtrlPntOther.y - firstCtrlPntOther.y,
      secondCtrlPntOther.z - firstCtrlPntOther.z
    };

    CAGD_POINT new_pnt = { firstCtrlPntOther.x - tangentDirection.x,
                           firstCtrlPntOther.y - tangentDirection.y,
                           firstCtrlPntOther.z - tangentDirection.z };

    ctrl_pnts_[ ctrl_pnts_.size() - 2 ] = new_pnt;
  }
}

/******************************************************************************
* Bezier::connectG1_bezier
******************************************************************************/
void Bezier::connectG1_bezier( const Bezier *other )
{
  connectC0_bezier( other );

  if( ctrl_pnts_.size() > 1 && other->ctrl_pnts_.size() > 1 )
  {
    CAGD_POINT firstCtrlPntOther = other->ctrl_pnts_.front();
    CAGD_POINT secondCtrlPntOther = other->ctrl_pnts_[ 1 ];

    CAGD_POINT lastCtrlPntThis = ctrl_pnts_.back();

    double dxOther = secondCtrlPntOther.x - firstCtrlPntOther.x;
    double dyOther = secondCtrlPntOther.y - firstCtrlPntOther.y;
    double dzOther = secondCtrlPntOther.z - firstCtrlPntOther.z;

    double lengthOther = sqrt( dxOther * dxOther +
                               dyOther * dyOther +
                               dzOther * dzOther );

    if( lengthOther != 0 )
    {
      dxOther /= lengthOther;
      dyOther /= lengthOther;
      dzOther /= lengthOther;
    }

    CAGD_POINT new_pnt = { lastCtrlPntThis.x - dxOther,
                           lastCtrlPntThis.y - dyOther,
                           lastCtrlPntThis.z - dzOther };

    ctrl_pnts_[ ctrl_pnts_.size() - 2 ] = new_pnt;
  }
}

/******************************************************************************
* Bezier::connectC0_bspline
******************************************************************************/
void Bezier::connectC0_bspline( const BSpline *bspline )
{
  CAGD_POINT startPoint = bspline->evaluate( bspline->knots_.front() );
  ctrl_pnts_[ ctrl_pnts_.size() - 1 ] = startPoint;
}

/******************************************************************************
* Bezier::connectC1_bspline
******************************************************************************/
void Bezier::connectC1_bspline( const BSpline *bspline )
{
  CAGD_POINT startPoint = bspline->evaluate( bspline->knots_.front() );
  ctrl_pnts_[ ctrl_pnts_.size() - 1 ] = startPoint;

  if( ctrl_pnts_.size() > 1 && bspline->ctrl_pnts_.size() > 1 )
  {
    CAGD_POINT secondCtrlPntBSpline = bspline->ctrl_pnts_[ 1 ];

    double dxBSpline = secondCtrlPntBSpline.x - startPoint.x;
    double dyBSpline = secondCtrlPntBSpline.y - startPoint.y;
    double dzBSpline = secondCtrlPntBSpline.z - startPoint.z;

    CAGD_POINT new_pnt = { startPoint.x - dxBSpline,
                           startPoint.y - dyBSpline,
                           startPoint.z - dzBSpline };

    ctrl_pnts_[ ctrl_pnts_.size() - 2 ] = new_pnt;
  }
}

/******************************************************************************
* Bezier::connectG1_bspline
******************************************************************************/
void Bezier::connectG1_bspline( const BSpline *bspline )
{
  connectC0_bspline( bspline );

  if( ctrl_pnts_.size() > 1 && bspline->ctrl_pnts_.size() > 1 )
  {
    CAGD_POINT startPoint = bspline->ctrl_pnts_.front();
    CAGD_POINT secondCtrlPntBSpline = bspline->ctrl_pnts_[ 1 ];

    double dxBSpline = secondCtrlPntBSpline.x - startPoint.x;
    double dyBSpline = secondCtrlPntBSpline.y - startPoint.y;
    double dzBSpline = secondCtrlPntBSpline.z - startPoint.z;

    double lengthBSpline = sqrt( dxBSpline * dxBSpline +
                                 dyBSpline * dyBSpline +
                                 dzBSpline * dzBSpline );

    if( lengthBSpline != 0 )
    {
      dxBSpline /= lengthBSpline;
      dyBSpline /= lengthBSpline;
      dzBSpline /= lengthBSpline;
    }

    CAGD_POINT lastCtrlPntThis = ctrl_pnts_.back();

    CAGD_POINT new_pnt = { lastCtrlPntThis.x - dxBSpline,
                           lastCtrlPntThis.y - dyBSpline,
                           lastCtrlPntThis.z - dzBSpline };

    ctrl_pnts_[ ctrl_pnts_.size() - 2 ] = new_pnt;
  }
}

/******************************************************************************
* Bezier::show_crv
******************************************************************************/
void Bezier::show_crv( int chg_ctrl_idx, CtrlOp ) const
{
  MP_cache_.clear();

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
    double weight_sum = 0.0;

    MP_cache_[ i ].x = 0.0;
    MP_cache_[ i ].y = 0.0;

    for( int j = 0; j <= n; ++j )
    {
      double w_base = base_matrix[ i ][ j ]/* * ctrl_pnts_[ j ].z*/;
      MP_cache_[ i ].x += w_base * ctrl_pnts_[ j ].x;
      MP_cache_[ i ].y += w_base * ctrl_pnts_[ j ].y;
      /*weight_sum += w_base;*/
    }

    /*if( double_cmp( weight_sum, 0.0 ) != 0 )
    {
      MP_cache_[ i ].x /= weight_sum;
      MP_cache_[ i ].y /= weight_sum;
    }*/
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
    computeMP();

  // Compute the final result using T * MP
  CAGD_POINT point = { 0.0, 0.0, 0.0 };

  for( int i = 0; i <= n; ++i )
  {
    point.x += T[ i ] * MP_cache_[ i ].x;
    point.y += T[ i ] * MP_cache_[ i ].y;
  }

  return point;
}