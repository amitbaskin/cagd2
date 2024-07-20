#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "Curve.h"
#include "color.h"
#include "vectors.h"
#include "options.h"
#include "BSpline.h"
#include "Bezier.h"

std::vector< Curve * > cur_curves;

Curve::Curve() : order_( 0 ), poly_seg_id_( K_NOT_USED )
{
  const unsigned char *curve_color = get_curve_color();

  color_[ 0 ] = curve_color[ 0 ];
  color_[ 1 ] = curve_color[ 1 ];
  color_[ 2 ] = curve_color[ 2 ];
}

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
* print_err
******************************************************************************/
void print_err( char *str )
{
  errno = EPERM;
  perror( str );
  cagdSetHelpText( str );
  cagdShowHelp();
}

/******************************************************************************
* rtrim  trim from start (in place)
******************************************************************************/
static inline void ltrim( std::string &s )
{
  s.erase( s.begin(), std::find_if( s.begin(), s.end(), []( unsigned char ch )
  {
    return !std::isspace( ch );
  } ) );
}

/******************************************************************************
* rtrim  trim from end (in place)
******************************************************************************/
static inline void rtrim( std::string &s )
{
  s.erase( std::find_if( s.rbegin(), s.rend(), []( unsigned char ch )
  {
    return !std::isspace( ch );
  } ).base(), s.end() );
}

/******************************************************************************
* trim
******************************************************************************/
static inline void trim( std::string &s )
{
  ltrim( s );
  rtrim( s );
}

/******************************************************************************
* skip_blank_lines_and_comments
******************************************************************************/
void skip_blank_lines_and_comments( std::ifstream &file )
{
  std::streampos lastPos = file.tellg();
  std::string line;
  while( std::getline( file, line ) )
  {
    ltrim( line );
    if( line.empty() || line[0] == '#' )
    {
      lastPos = file.tellg();
      continue;
    }

    file.seekg( lastPos );
    break;
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

/******************************************************************************
* parse_file
******************************************************************************/
size_t parse_file( const std::string &filePath )
{
  size_t first_new_idx = cur_curves.size();

  std::ifstream file( filePath, std::ios::binary );

  if( !file.is_open() )
  {
    print_err( "Error opening file" );
    return first_new_idx;
  }

  while( file )
  {
    int order = 0;

    Curve *p_curve;

    skip_blank_lines_and_comments( file );

    std::string line;
    if( !std::getline( file, line ) && !file.eof() )
    {
      print_err( "Error reading file" );
      break;
    }

    ltrim( line );

    if( line.empty() )
      continue;

    std::istringstream issOrder( line );
    if( !( issOrder >> order ) )
    {
      print_err( "Error reading order" );
      continue;
    }

    skip_blank_lines_and_comments( file );

    if( !std::getline( file, line ) )
    {
      print_err( "Error reading file" );
      break;
    }

    if( line.find( "knots" ) != std::string::npos )
    {
      p_curve = new BSpline();
      BSpline *p_bspline = ( BSpline * )p_curve;
      p_bspline->order_ = order;

      size_t pos = line.find( "[" );
      size_t posEnd = line.find( "]" );

      if( pos != std::string::npos && posEnd != std::string::npos && pos < posEnd )
      {
        std::string numKnotsStr = line.substr( pos + 1, posEnd - pos - 1 );
        trim( numKnotsStr );
        size_t numKnots = std::stoi( numKnotsStr );

        std::string after_equal_sign = line.substr( posEnd + 3 );
        ltrim( after_equal_sign );

        std::istringstream issLine( after_equal_sign );

        double prev_knot = -HUGE_DOUBLE;
        double knot = -HUGE_DOUBLE;

        while( issLine >> knot )
        {
          p_bspline->knots_.push_back( knot );

          if( double_cmp( knot, prev_knot ) > 0 )
          {
            prev_knot = knot;
            p_bspline->u_vec_.push_back( knot );
          }

          if( p_bspline->knots_.size() == numKnots )
          {
            break;
          }
        }

        while( p_bspline->knots_.size() < numKnots &&
               std::getline( file, line ) )
        {
          ltrim( line );

          if( line.empty() || line[ 0 ] == '#' )
          {
            continue;
          }

          std::istringstream issLine( line );
          double knot;

          while( issLine >> knot )
          {
            p_bspline->knots_.push_back( knot );

            if( double_cmp( knot, prev_knot ) > 0 )
            {
              prev_knot = knot;
              p_bspline->u_vec_.push_back( knot );
            }

            if( p_bspline->knots_.size() == numKnots )
            {
              break;
            }
          }
        }
      }
      else
      {
        print_err( "Error parsing knots_ size" );
        delete p_curve;
        continue;
      }
    }
    else if( order > 0 )
    {
      p_curve = new Bezier();
      Bezier *p_bezier = ( Bezier * )p_curve;
      p_bezier->order_ = order;

      std::istringstream issctrl_pnts( line );
      p_curve->add_ctrl_pnt( issctrl_pnts );

      if( !p_curve->is_miss_ctrl_pnts() )
        break;
    }
    else
    {
      print_err( "Error in file format - no knots and no order" );
      break;
    }

    while( std::getline( file, line ) )
    {
      ltrim( line );

      if( line.empty() || line[ 0 ] == '#' )
      {
        continue;
      }

      std::istringstream issctrl_pnts( line );
      p_curve->add_ctrl_pnt( issctrl_pnts );

      if( !p_curve->is_miss_ctrl_pnts() )
        break;
    }

    if( IS_DEBUG )
      p_curve->print();

    cur_curves.push_back( p_curve );
  }

  file.close();

  return first_new_idx;
}

/******************************************************************************
* clean_all_curves
******************************************************************************/
void clean_all_curves()
{
  clean_cur_curves_vec();

  cagdFreeAllSegments();
  cagdRedraw();
}

/******************************************************************************
* clean_cur_curves_vec
******************************************************************************/
void clean_cur_curves_vec()
{
  for( auto crv : cur_curves )
    delete crv;

  cur_curves.clear();
}

/******************************************************************************
* load_curves
******************************************************************************/
void load_curves( int dummy1, int dummy2, void *p_data )
{
  char *file_path = ( char * )p_data;
  std::string file_str = file_path;
  size_t first_new_idx = parse_file( file_str );

  if( first_new_idx < cur_curves.size() )
  {
    for( size_t i = first_new_idx; i < cur_curves.size(); ++i )
    {
      cur_curves[ i ]->show_ctrl_poly();
      cur_curves[ i ]->show_crv();
    }

    cagdRedraw();
  }
}

/******************************************************************************
* Curve::show_ctrl_poly
******************************************************************************/
void Curve::show_ctrl_poly()
{
  int num_pts = ctrl_pnts_.size();

  if( num_pts > 1 )
  {
    auto pnts = new CAGD_POINT[ num_pts ];

    if( pnts != nullptr )
    {
      set_norm_color();

      for( int i = 0; i < num_pts; i++ )
      {
        if( ctrl_pnts_[i].z == 0 )
        {
          print_err( "control point can't have weight 0" );
          return;
        }

        pnts[i] = { ctrl_pnts_[i].x/* / curve_data->ctrl_pts[i].z*/,
                    ctrl_pnts_[i].y/* / curve_data->ctrl_pts[i].z*/,
                    0 };

        cagdAddPoint( &pnts[i] );
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
* redraw_all_curves
******************************************************************************/
void redraw_all_curves()
{
  for( auto p_crv : cur_curves )
    p_crv->show_crv();

  cagdRedraw();
}
