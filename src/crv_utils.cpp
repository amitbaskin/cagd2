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
std::map< int, std::tuple< int, int > > ctrl_seg_to_pnts;
active_ctrl_pt_data active_drag_pt = { K_NOT_USED, { 0, 0 }, true };

void print_error( const std::string &message );
static inline void ltrim( std::string &str );
static inline void rtrim( std::string &str );
static inline void trim( std::string &str );
void skip_blank_and_comment_lines( std::ifstream &file );
void clean_all_curves();
void clean_current_curves();
bool parse_order_from_line( const std::string &line, int &order );
void parse_knots_from_line( const std::string &line, BSpline *bspline );

bool extract_knots_information( const std::string &line,
                                BSpline *bspline,
                                size_t &numKnots );

bool parse_remaining_knots( std::ifstream &file,
                            BSpline *bspline,
                            size_t numKnots );

BSpline *create_bspline( int order,
                         const std::string &line,
                         std::ifstream &file );

Bezier *create_bezier( int order, const std::string &line );
Curve *create_curve( int order, const std::string &line, std::ifstream &file );
bool add_control_points( std::ifstream &file, Curve *curve );
size_t parse_file( const std::string &filePath );

/******************************************************************************
* get_active_pt_id
******************************************************************************/
int get_active_pt_id()
{
  return active_drag_pt.active_pt_id;
}

/******************************************************************************
* set_active_pt_id
******************************************************************************/
void set_active_pt_id( int id )
{
  active_drag_pt.active_pt_id = id;
}

/******************************************************************************
* get_active_pt_last_pos
******************************************************************************/
int *get_active_pt_last_pos()
{
  return active_drag_pt.last_pos;
}

/******************************************************************************
* set_active_pt_last_pos
******************************************************************************/
void set_active_pt_last_pos( int pos[2] )
{
  active_drag_pt.last_pos[0] = pos[0];
  active_drag_pt.last_pos[1] = pos[1];
}

/******************************************************************************
* get_active_pt_is_first_move
******************************************************************************/
bool get_active_pt_is_first_move()
{
  return active_drag_pt.first_move;
}

/******************************************************************************
* set_active_pt_last_pos
******************************************************************************/
void set_active_pt_is_first_move( bool value )
{
  active_drag_pt.first_move = value;
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
* map_ctrl_seg_to_pnts
******************************************************************************/
void map_ctrl_seg_to_pnts( int seg_id, int pnt1, int pnt2 )
{
  ctrl_seg_to_pnts[ seg_id ] = std::make_tuple( pnt1, pnt2 );
}

/******************************************************************************
* erase_pnt_to_crv_ctrl
******************************************************************************/
void erase_pnt_to_crv_ctrl( int pnt_id )
{
  pnt_to_crv_ctrl.erase( pnt_id );
}

/******************************************************************************
* erase_ctrl_seg_to_pnts
******************************************************************************/
void erase_ctrl_seg_to_pnts( int seg_id )
{
  ctrl_seg_to_pnts.erase( seg_id );
}

/******************************************************************************
* get_pnt_crv_ctrl
******************************************************************************/
std::tuple< Curve *, int > get_pnt_crv_ctrl( int pnt_id )
{
  if( pnt_to_crv_ctrl.find( pnt_id ) != pnt_to_crv_ctrl.end() )
    return pnt_to_crv_ctrl[ pnt_id ];
  else
    return std::make_tuple( nullptr, K_NOT_USED );
}

/******************************************************************************
* get_ctrl_seg_pnts
******************************************************************************/
std::tuple< int, int > get_ctrl_seg_pnts( int seg_id )
{
  if( ctrl_seg_to_pnts.find( seg_id ) != ctrl_seg_to_pnts.end() )
    return ctrl_seg_to_pnts[ seg_id ];
  else
    return std::make_tuple( K_NOT_USED, K_NOT_USED );
}

/******************************************************************************
* update_ctrl_pnt
******************************************************************************/
void update_ctrl_pnt( int pnt_id, double new_x, double new_y )
{
  std::tuple< Curve *, int > crv_ctrl_idx = get_pnt_crv_ctrl( pnt_id );

  auto p_curve = std::get< 0 >( crv_ctrl_idx );
  auto ctrl_idx = std::get< 1 >( crv_ctrl_idx );

  if( p_curve != nullptr )
  {
    double old_pos_x = p_curve->ctrl_pnts_[ ctrl_idx ].x;
    double old_pos_y = p_curve->ctrl_pnts_[ ctrl_idx ].y;

    p_curve->ctrl_pnts_[ ctrl_idx ].x = new_x;
    p_curve->ctrl_pnts_[ ctrl_idx ].y = new_y;

    p_curve->show_ctrl_poly();
    p_curve->show_crv( ctrl_idx );
    cagdRedraw();
  }
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
* calculate_ctrl_pnt_updated_pos
******************************************************************************/
void calculate_ctrl_pnt_updated_pos( int pnt_id, int dx, int dy,
                                     double &new_x, double &new_y )
{
  std::tuple< Curve *, int > crv_ctrl_idx = get_pnt_crv_ctrl( pnt_id );

  auto p_curve = std::get< 0 >( crv_ctrl_idx );
  auto ctrl_idx = std::get< 1 >( crv_ctrl_idx );

  if( p_curve != nullptr )
  {
    double move_vec[ 2 ];
    cagdGetMoveVec( dx, dy, move_vec[ 0 ], move_vec[ 1 ] );

    new_x = p_curve->ctrl_pnts_[ ctrl_idx ].x + move_vec[ 0 ];
    new_y = p_curve->ctrl_pnts_[ ctrl_idx ].y + move_vec[ 1 ];
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
* print_error
******************************************************************************/
void print_error( const std::string &message )
{
  errno = EPERM;
  perror( message.c_str() );
  cagdSetHelpText( message.c_str() );
  cagdShowHelp();
}

/******************************************************************************
* ltrim
******************************************************************************/
static inline void ltrim( std::string &str )
{
  str.erase( str.begin(),
             std::find_if( str.begin(),
                           str.end(),
                           []( unsigned char ch )
                           {
                             return !std::isspace( ch );
                           } ) );
}

/******************************************************************************
* rtrim
******************************************************************************/
static inline void rtrim( std::string &str )
{
  str.erase( std::find_if( str.rbegin(), str.rend(), []( unsigned char ch )
                           {
                             return !std::isspace( ch );
                           } ).base(), str.end() );
}

/******************************************************************************
* trim
******************************************************************************/
static inline void trim( std::string &str )
{
  ltrim( str );
  rtrim( str );
}

/******************************************************************************
* skip_blank_and_comment_lines
******************************************************************************/
void skip_blank_and_comment_lines( std::ifstream &file )
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
* clean_current_curves
******************************************************************************/
void clean_current_curves()
{
  for( auto curve : cur_curves )
    delete curve;

  cur_curves.clear();
}

/******************************************************************************
* clean_all_curves
******************************************************************************/
void clean_all_curves()
{
  clean_current_curves();
  cagdFreeAllSegments();
  cagdRedraw();
}

/******************************************************************************
* parse_order_from_line
******************************************************************************/
bool parse_order_from_line( const std::string &line, int &order )
{
  std::istringstream iss( line );
  if( !( iss >> order ) )
  {
    print_error( "Error reading order" );
    return false;
  }
  return true;
}

/******************************************************************************
* parse_knots_from_line
******************************************************************************/
void parse_knots_from_line( const std::string &line, BSpline *bspline )
{
  std::istringstream iss( line );
  double prev_knot = -HUGE_DOUBLE;
  double knot = -HUGE_DOUBLE;

  while( iss >> knot )
  {
    bspline->knots_.push_back( knot );
    if( double_cmp( knot, prev_knot ) > 0 )
    {
      prev_knot = knot;
      bspline->u_vec_.push_back( knot );
    }
  }
}

/******************************************************************************
* extract_knots_information
******************************************************************************/
bool extract_knots_information( const std::string &line,
                                BSpline *bspline,
                                size_t &numKnots )
{
  size_t pos = line.find( "[" );
  size_t posEnd = line.find( "]" );
  if( pos != std::string::npos && posEnd != std::string::npos && pos < posEnd )
  {
    std::string numKnotsStr = line.substr( pos + 1, posEnd - pos - 1 );
    trim( numKnotsStr );
    numKnots = std::stoi( numKnotsStr );

    std::string after_equal_sign = line.substr( posEnd + 3 );
    ltrim( after_equal_sign );
    parse_knots_from_line( after_equal_sign, bspline );
    return true;
  }
  else
  {
    print_error( "Error parsing knots size" );
    return false;
  }
}

/******************************************************************************
* parse_remaining_knots
******************************************************************************/
bool parse_remaining_knots( std::ifstream &file,
                            BSpline *bspline,
                            size_t numKnots )
{
  std::string line;
  while( bspline->knots_.size() < numKnots && std::getline( file, line ) )
  {
    ltrim( line );
    if( line.empty() || line[ 0 ] == '#' )
    {
      continue;
    }
    parse_knots_from_line( line, bspline );
  }

  if( bspline->knots_.size() != numKnots )
  {
    print_error( "Error parsing knots size" );
    return false;
  }
  return true;
}

/******************************************************************************
* create_bspline
******************************************************************************/
BSpline *create_bspline( int order,
                         const std::string &line,
                         std::ifstream &file )
{
  auto bspline = new BSpline();
  bspline->order_ = order;

  size_t numKnots;
  if( !extract_knots_information( line, bspline, numKnots ) )
  {
    delete bspline;
    return nullptr;
  }

  if( !parse_remaining_knots( file, bspline, numKnots ) )
  {
    delete bspline;
    return nullptr;
  }

  return bspline;
}

/******************************************************************************
* create_bezier
******************************************************************************/
Bezier *create_bezier( int order, const std::string &line )
{
  auto bezier = new Bezier();
  bezier->order_ = order;

  std::istringstream iss( line );
  bezier->add_ctrl_pnt_from_str( iss );

  return bezier;
}

/******************************************************************************
* create_curve
******************************************************************************/
Curve *create_curve( int order, const std::string &line, std::ifstream &file )
{
  if( line.find( "knots" ) != std::string::npos )
  {
    return create_bspline( order, line, file );
  }
  else if( order > 0 )
  {
    return create_bezier( order, line );
  }

  print_error( "Error in file format - no knots and no order" );
  return nullptr;
}

/******************************************************************************
* add_control_points
******************************************************************************/
bool add_control_points( std::ifstream &file, Curve *curve )
{
  std::string line;
  while( std::getline( file, line ) )
  {
    ltrim( line );
    if( line.empty() || line[ 0 ] == '#' )
    {
      continue;
    }

    std::istringstream iss( line );
    curve->add_ctrl_pnt_from_str( iss );

    if( !curve->is_miss_ctrl_pnts() )
      return true;
  }
  return false;
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
    print_error( "Error opening file" );
    return first_new_idx;
  }

  while( file )
  {
    int order = 0;

    skip_blank_and_comment_lines( file );

    std::string line;
    if( !std::getline( file, line ) && !file.eof() )
    {
      print_error( "Error reading file" );
      break;
    }

    ltrim( line );
    if( line.empty() )
      continue;

    if( !parse_order_from_line( line, order ) )
      continue;

    skip_blank_and_comment_lines( file );

    if( !std::getline( file, line ) )
    {
      print_error( "Error reading file" );
      break;
    }

    auto curve = create_curve( order, line, file );
    if( !curve )
      continue;

    if( !add_control_points( file, curve ) )
    {
      delete curve;
      continue;
    }

    if( IS_DEBUG )
      curve->print();

    cur_curves.push_back( curve );
  }

  file.close();
  return first_new_idx;
}