#include "Curve.h"
#include "crv_utils.h"
#include "options.h"
#include "color.h"

/******************************************************************************
* Curve::Curve
******************************************************************************/
Curve::Curve() : order_( 0 )
{
  const unsigned char *curve_color = get_curve_color();

  color_[ 0 ] = curve_color[ 0 ];
  color_[ 1 ] = curve_color[ 1 ];
  color_[ 2 ] = curve_color[ 2 ];
}

/******************************************************************************
* Curve::Curve
******************************************************************************/
Curve::Curve( int order, point_vec ctrl_pnts ) :
  order_( order ),
  ctrl_pnts_( ctrl_pnts )
{
  const unsigned char *curve_color = get_curve_color();

  color_[ 0 ] = curve_color[ 0 ];
  color_[ 1 ] = curve_color[ 1 ];
  color_[ 2 ] = curve_color[ 2 ];
}

/******************************************************************************
* Curve::rmv_ctrl_pnt
******************************************************************************/
void Curve::rmv_ctrl_pnt( int idx )
{
  erase_pnt_to_crv_ctrl( pnt_ids_[ idx ] );

  pnt_ids_.erase( pnt_ids_.begin() + idx );
  ctrl_pnts_.erase( ctrl_pnts_.begin() + idx );

  show_ctrl_poly();
}

/******************************************************************************
* Curve::add_ctrl_pnt
******************************************************************************/
void Curve::add_ctrl_pnt( CAGD_POINT &ctrl_pnt, int idx )
{
  int pnt_id = cagdAddPoint( &ctrl_pnt );
  map_pnt_to_crv_ctrl( pnt_id, this, idx );
  pnt_ids_.insert( pnt_ids_ .begin() + idx, pnt_id );

  ctrl_pnts_.insert( ctrl_pnts_.begin() + idx, ctrl_pnt );

  show_ctrl_poly();
}

/******************************************************************************
* Curve::print
******************************************************************************/
void Curve::print() const
{
  printf( "Order: %u\n", order_ );
  printf( "Control Points:\n" );

  for( const CAGD_POINT &pt : ctrl_pnts_ )
  {
    printf( "(%f, %f", pt.x, pt.y );
    printf( ", %f", pt.z );
    printf( ")\n" );
  }

  printf( "\n\n\n" );
}

/******************************************************************************
* Curve::show_ctrl_poly
******************************************************************************/
void Curve::show_ctrl_poly()
{
  size_t cur_pnts_num = ctrl_pnts_.size();
  size_t old_pnts_num = pnt_ids_.size();
  bool reuse_ids = old_pnts_num >= cur_pnts_num;

  if( old_pnts_num > cur_pnts_num )
  {
    for( size_t i = cur_pnts_num; i < old_pnts_num; ++i )
    {
      cagdFreeSegment( pnt_ids_[ i ] );
      erase_pnt_to_crv_ctrl( pnt_ids_[ i ] );

      if( i < old_pnts_num )
      {
        cagdFreeSegment( poly_seg_ids_[ i ] );
        erase_ctrl_seg_to_pnts( poly_seg_ids_[ i ] );
      }
    }

    pnt_ids_.resize( cur_pnts_num );
  }

  if( cur_pnts_num > 1 )
  {
    CAGD_POINT prev_pnt = { ctrl_pnts_[ 0 ].x, ctrl_pnts_[ 0 ].y, 0.0 };
    CAGD_POINT cur_pnt;

    if( reuse_ids )
    {
      set_norm_color();
      cagdReusePoint( pnt_ids_[ 0 ], &prev_pnt );
    }
    else
    {
      int pnt_id = cagdAddPoint( &prev_pnt );
      pnt_ids_.push_back( pnt_id );
      map_pnt_to_crv_ctrl( pnt_id, this, 0 );
    }


    for( size_t i = 1; i < cur_pnts_num; ++i )
    {
      cur_pnt = { ctrl_pnts_[ i ].x, ctrl_pnts_[ i ].y, 0.0 };
      CAGD_POINT pnts[ 2 ] = { prev_pnt, cur_pnt };
      set_norm_color();

      if( reuse_ids )
      {
        cagdReusePoint( pnt_ids_[ i ], &cur_pnt );
        set_bi_color();
        cagdReusePolyline( poly_seg_ids_[ i - 1 ], pnts, 2 );
      }
      else
      {
        int pnt_id = cagdAddPoint( &cur_pnt );

        pnt_ids_.push_back( pnt_id );
        map_pnt_to_crv_ctrl( pnt_id, this, i );

        set_bi_color();
        int poly_seg_id = cagdAddPolyline( pnts, 2 );

        map_ctrl_seg_to_pnts( poly_seg_id, pnt_ids_[ i - 1 ], pnt_id );
        poly_seg_ids_.push_back( poly_seg_id );
      }

      prev_pnt = cur_pnt;
    }
  }
}

/******************************************************************************
* Curve::add_ctrl_pnt_from_str
******************************************************************************/
void Curve::add_ctrl_pnt_from_str( std::istringstream &line )
{
  CAGD_POINT point;

  while( line >> point.x >> point.y >> point.z )
  {
    point.x /= point.z;
    point.y /= point.z;
    ctrl_pnts_.push_back( point );
  }
}
