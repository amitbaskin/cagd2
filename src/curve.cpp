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
  erase_pnt_to_crv( pnt_ids_[ idx ] );
  cagdFreeSegment( pnt_ids_[ idx ] );
  pnt_ids_.erase( pnt_ids_.begin() + idx );
  ctrl_pnts_.erase( ctrl_pnts_.begin() + idx );
}

/******************************************************************************
* Curve::add_ctrl_pnt
******************************************************************************/
void Curve::add_ctrl_pnt( const CAGD_POINT &ctrl_pnt, int idx )
{
  set_norm_color();
  int pnt_id = cagdAddPoint( &ctrl_pnt );

  map_pnt_to_crv( pnt_id, this );

  pnt_ids_.insert( pnt_ids_ .begin() + idx, pnt_id );

  ctrl_pnts_.insert( ctrl_pnts_.begin() + idx, ctrl_pnt );
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
* Curve::clean_ctrl_poly
******************************************************************************/
void Curve::clean_ctrl_poly()
{
  for( size_t i = 0; i < poly_seg_ids_.size(); ++i )
  {
    erase_ctrl_seg_to_pnts( poly_seg_ids_[ i ] );
    erase_ctrl_seg_to_pnts( poly_seg_ids_[ i ] );
    cagdFreeSegment( poly_seg_ids_[ i ] );
  }

  poly_seg_ids_.clear();
}

/******************************************************************************
* Curve::show_ctrl_poly
******************************************************************************/
void Curve::show_ctrl_poly()
{
  clean_ctrl_poly();

  size_t cur_pnts_num = ctrl_pnts_.size();

  if( cur_pnts_num > 0 )
  {
    CAGD_POINT prev_pnt = { ctrl_pnts_[ 0 ].x, ctrl_pnts_[ 0 ].y, 0.0 };
    CAGD_POINT cur_pnt;

    if( pnt_ids_.size() > 0 )
    {
      set_norm_color();
      cagdReusePoint( pnt_ids_[ 0 ], &prev_pnt );
    }
    else
    {
      set_norm_color();
      int pnt_id = cagdAddPoint( &prev_pnt );
      pnt_ids_.push_back( pnt_id );
      map_pnt_to_crv( pnt_id, this );
    }

    for( size_t i = 1; i < cur_pnts_num; ++i )
    {
      cur_pnt = { ctrl_pnts_[ i ].x, ctrl_pnts_[ i ].y, 0.0 };
      CAGD_POINT pnts[ 2 ] = { prev_pnt, cur_pnt };
      set_norm_color();

      int pnt_id = K_NOT_USED;

      if( i < pnt_ids_.size() )
      {
        pnt_id = pnt_ids_[ i ];
        cagdReusePoint( pnt_ids_[ i ], &cur_pnt );
      }
      else
      {
        pnt_id = cagdAddPoint( &cur_pnt );
        pnt_ids_.push_back( pnt_id );
        map_pnt_to_crv( pnt_id, this );
      }

      set_bi_color();
      int poly_seg_id = cagdAddPolyline( pnts, 2 );
      map_ctrl_seg_to_pnts( poly_seg_id, pnt_ids_[ i - 1 ], pnt_id );
      poly_seg_ids_.push_back( poly_seg_id );

      prev_pnt = cur_pnt;
    }
  }

  pnt_ids_.resize( cur_pnts_num );
}

/******************************************************************************
* Curve::get_pnt_id_idx
******************************************************************************/
int Curve::get_pnt_id_idx( int pnt_id )
{
  auto it = std::find( pnt_ids_.begin(), pnt_ids_.end(), pnt_id );

  if( it != pnt_ids_.end() )
    return std::distance( pnt_ids_.begin(), it );
  else
    return K_NOT_USED;
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

/******************************************************************************
* Curve::update_weight
******************************************************************************/
void Curve::update_weight( int pnt_idx, double val )
{
  if( ( size_t )pnt_idx >= ctrl_pnts_.size() )
    throw std::runtime_error( "wrong ctrl pnt idx" );

  ctrl_pnts_[ pnt_idx ].z = val;
}

/******************************************************************************
* Curve::change_color
******************************************************************************/
void Curve::change_color( BYTE red, BYTE green, BYTE blue )
{
  for( auto seg_id : seg_ids_ )
    cagdSetSegmentColor( seg_id, red, green, blue );

  cagdRedraw();
}
