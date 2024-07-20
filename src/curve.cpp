#include "Curve.h"
#include "crv_utils.h"
#include "options.h"
#include "color.h"

/******************************************************************************
* Curve::Curve
******************************************************************************/
Curve::Curve() : order_( 0 ), poly_seg_id_( K_NOT_USED )
{
  const unsigned char *curve_color = get_curve_color();

  color_[ 0 ] = curve_color[ 0 ];
  color_[ 1 ] = curve_color[ 1 ];
  color_[ 2 ] = curve_color[ 2 ];
}

/******************************************************************************
* Curve::Curve
******************************************************************************/
Curve::Curve( int order_, point_vec ctrl_pnts_ ) :
  poly_seg_id_( K_NOT_USED ),
  order_( order_ ),
  ctrl_pnts_( ctrl_pnts_ )
{
  const unsigned char *curve_color = get_curve_color();

  color_[ 0 ] = curve_color[ 0 ];
  color_[ 1 ] = curve_color[ 1 ];
  color_[ 2 ] = curve_color[ 2 ];
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
  size_t num_pts = ctrl_pnts_.size();

  if( num_pts > 1 )
  {
    auto pnts = new CAGD_POINT[ num_pts ];

    if( pnts != nullptr )
    {
      set_norm_color();

      for( size_t i = 0; i < num_pts; i++ )
      {
        if( ctrl_pnts_[ i ].z == 0 )
        {
          print_error( "control point can't have weight 0" );
          return;
        }

        pnts[ i ] = { ctrl_pnts_[ i ].x/* / curve_data->ctrl_pts[i].z*/,
                      ctrl_pnts_[ i ].y/* / curve_data->ctrl_pts[i].z*/,
                      0.0 };

        if( pnt_ids_.size() < num_pts )
        {
          int pnt_id = cagdAddPoint( &pnts[ i ] );
          pnt_ids_.push_back( pnt_id );
          map_pnt_to_crv_ctrl( pnt_id, this, i );
        }
        else
          cagdReusePoint( pnt_ids_[ i ], &pnts[ i ] );
      }
    }

    set_bi_color();

    if( poly_seg_id_ == K_NOT_USED )
      poly_seg_id_ = cagdAddPolyline( pnts, num_pts );
    else
      cagdReusePolyline( poly_seg_id_, pnts, num_pts );

    delete[] pnts;
  }
}

/******************************************************************************
* Curve::add_ctrl_pnt
******************************************************************************/
void Curve::add_ctrl_pnt( std::istringstream &line )
{
  CAGD_POINT point;

  while( line >> point.x >> point.y >> point.z )
  {
    point.x /= point.z;
    point.y /= point.z;
    ctrl_pnts_.push_back( point );
  }
}