#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>

#include "vectors.h"
#include "options.h"
#include "BSpline.h"
#include "Bezier.h"
#include "crv_utils.h"


std::vector< Curve * > cur_curves;
std::map< int, Curve * > seg_to_crv;
std::map< int, std::tuple< Curve *, int > > pnt_to_crv_ctrl;

size_t parse_file( const std::string &filePath );


/******************************************************************************
* redraw_all_curves
******************************************************************************/
void redraw_all_curves()
{
  for( auto p_crv : cur_curves )
    p_crv->show_crv();

  cagdRedraw();
}

/******************************************************************************
* map_seg_to_crv
******************************************************************************/
void map_seg_to_crv( int seg_id, Curve *p_curve )
{
  seg_to_crv[ seg_id ] = p_curve;
}

/******************************************************************************
* map_pnt_to_crv_ctrl
******************************************************************************/
void map_pnt_to_crv_ctrl( int pnt_id, Curve *p_curve, int ctrl_idx )
{
  pnt_to_crv_ctrl[ pnt_id ] = std::make_tuple( p_curve, ctrl_idx );
}

/******************************************************************************
* get_pnt_crv_ctrl
******************************************************************************/
std::tuple< Curve *, int > get_pnt_crv_ctrl( int pnt_id )
{
  if( pnt_to_crv_ctrl.find( pnt_id ) != pnt_to_crv_ctrl.end() )
    return pnt_to_crv_ctrl[ pnt_id ];
  else
    return std::make_tuple( nullptr, -1 );
}

/******************************************************************************
* update_ctrl_pnt
******************************************************************************/
void update_ctrl_pnt( int pnt_id, int new_x, int new_y )
{
  std::tuple< Curve *, int > crv_ctrl_idx = get_pnt_crv_ctrl( pnt_id );

  auto p_curve = std::get< 0 >( crv_ctrl_idx );
  auto ctrl_idx = std::get< 1 >( crv_ctrl_idx );

  if( p_curve != nullptr )
  {
    p_curve->ctrl_pnts_[ ctrl_idx ].x = new_x;
    p_curve->ctrl_pnts_[ ctrl_idx ].y = new_y;
    p_curve->show_ctrl_poly();
    p_curve->show_crv( ctrl_idx );
    cagdRedraw();
  }
}

/******************************************************************************
* get_seg_crv
******************************************************************************/
Curve *get_seg_crv( int seg_id )
{
  if( seg_to_crv.find( seg_id ) != seg_to_crv.end() )
    return seg_to_crv[ seg_id ];
  else
    return nullptr;
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
* ltrim
******************************************************************************/
static inline void ltrim( std::string &ss )
{
  ss.erase( ss.begin(), std::find_if( ss.begin(), ss.end(),
                                      []( unsigned char ch )
                                      {
                                        return !std::isspace( ch );
                                      } ) );
}

/******************************************************************************
* rtrim  trim from end (in place)
******************************************************************************/
static inline void rtrim( std::string &ss )
{
  ss.erase( std::find_if( ss.rbegin(), ss.rend(), []( unsigned char ch )
                          {
                            return !std::isspace( ch );
                          } ).base(), ss.end() );
}

/******************************************************************************
* trim
******************************************************************************/
static inline void trim( std::string &ss )
{
  ltrim( ss );
  rtrim( ss );
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
    if( line.empty() || line[ 0 ] == '#' )
    {
      lastPos = file.tellg();
      continue;
    }

    file.seekg( lastPos );
    break;
  }
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
