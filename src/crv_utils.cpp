#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <stdexcept>

#include "vectors.h"
#include "options.h"
#include "BSpline.h"
#include "Bezier.h"
#include "crv_utils.h"

std::vector< Curve * > cur_curves;
std::map< int, Curve * > seg_to_crv;
std::map< int, Curve * > pnt_to_crv;
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
* map_pnt_to_crv
******************************************************************************/
void map_pnt_to_crv( int pnt_id, Curve *p_curve )
{
  pnt_to_crv[ pnt_id ] = p_curve;
}

/******************************************************************************
* map_ctrl_seg_to_pnts
******************************************************************************/
void map_ctrl_seg_to_pnts( int seg_id, int pnt1, int pnt2 )
{
  ctrl_seg_to_pnts[ seg_id ] = std::make_tuple( pnt1, pnt2 );
}

/******************************************************************************
* erase_pnt_to_crv
******************************************************************************/
void erase_pnt_to_crv( int pnt_id )
{
  pnt_to_crv.erase( pnt_id );
}

/******************************************************************************
* erase_ctrl_seg_to_pnts
******************************************************************************/
void erase_ctrl_seg_to_pnts( int seg_id )
{
  ctrl_seg_to_pnts.erase( seg_id );
}

/******************************************************************************
* erase_seg_to_crv
******************************************************************************/
void erase_seg_to_crv( int seg_id )
{
  seg_to_crv.erase( seg_id );
}

/******************************************************************************
* get_pnt_crv
******************************************************************************/
Curve *get_pnt_crv( int pnt_id )
{
  if( pnt_to_crv.find( pnt_id ) != pnt_to_crv.end() )
    return pnt_to_crv[ pnt_id ];
  else
    return nullptr;
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
* update_weight_callback
******************************************************************************/
void update_weight_callback( int seg_id, int pnt_idx, double val )
{
  try
  {
    Curve *p_crv = nullptr;
    CurveType crv_type = get_crv( seg_id, &p_crv );

    if( crv_type == CurveType::NONE )
    {
      throw std::runtime_error( "wrong crv type" );
    }

    p_crv->update_weight( pnt_idx, val );
    cagdRedraw();
  }
  catch( const std::runtime_error &err )
  {
    throw err;
  }
}

/******************************************************************************
* get_crv_type
******************************************************************************/
CurveType get_crv_type( Curve *p_crv )
{
  BSpline *p_bspline = dynamic_cast< BSpline * >( p_crv );

  if( p_bspline != nullptr )
    return CurveType::BSPLINE;

  Bezier *p_bezier = dynamic_cast< Bezier * >( p_crv );

  if( p_bezier != nullptr )
    return CurveType::BEZIER;
  else
    throw std::runtime_error( "bad map for seg id to crv" );
}

/******************************************************************************
* get_crv
******************************************************************************/
CurveType get_crv( int seg_id, Curve **rp_crv )
{
  *rp_crv = get_seg_crv( seg_id );

  if( *rp_crv == nullptr )
    throw std::runtime_error( "bad map for seg id to crv" );

  BSpline *p_bspline = dynamic_cast< BSpline * >( *rp_crv );

  if( p_bspline != nullptr )
    return CurveType::BSPLINE;

  Bezier *p_bezier = dynamic_cast< Bezier * >( *rp_crv );

  if( p_bezier != nullptr )
    return CurveType::BEZIER;
  else
    throw std::runtime_error( "bad map for seg id to crv" );
}

/******************************************************************************
* connect_crv_callback
******************************************************************************/
void connect_crv_callback( int seg_id_1, int seg_id_2, ConnType conn )
{
  try
  {
    Curve *p_crv_1 = nullptr;
    Curve *p_crv_2 = nullptr;
    CurveType crv_type_1 = get_crv( seg_id_1, &p_crv_1 );
    CurveType crv_type_2 = get_crv( seg_id_2, &p_crv_2 );

    switch( conn )
    {
      case ConnType::C0:
      {
        if( crv_type_2 == CurveType::BSPLINE )
          p_crv_1->connectC0_bspline( ( BSpline * )p_crv_2 );
        else if( crv_type_2 == CurveType::BEZIER )
          p_crv_1->connectC0_bezier( ( Bezier * )p_crv_2 );
        else
        {
          throw std::runtime_error( "invalid crv type" );
        }
        break;
      }

      case ConnType::C1:
      {
        if( crv_type_2 == CurveType::BSPLINE )
          p_crv_1->connectC1_bspline( ( BSpline * )p_crv_2 );
        else if( crv_type_2 == CurveType::BEZIER )
          p_crv_1->connectC1_bezier( ( Bezier * )p_crv_2 );
        else
        {
          throw std::runtime_error( "invalid crv type" );
        }
        break;
      }

      case ConnType::G1:
      {
        if( crv_type_2 == CurveType::BSPLINE )
          p_crv_1->connectG1_bspline( ( BSpline * )p_crv_2 );
        else if( crv_type_2 == CurveType::BEZIER )
          p_crv_1->connectG1_bezier( ( Bezier * )p_crv_2 );
        else
        {
          throw std::runtime_error( "invalid crv type" );
        }
        break;
      }
      default:
      {
        throw std::runtime_error( "invalid conn type" );
      }
    }

    p_crv_1->show_ctrl_poly();
    p_crv_1->show_crv();
    cagdRedraw();

    if( crv_type_1 == CurveType::BEZIER )
    {
      if( crv_type_2 == CurveType::BEZIER )
        createBSplineFromBezierCurves( ( Bezier * )p_crv_1, ( Bezier * )p_crv_2 );
    }
    else
    {
      if( crv_type_2 == CurveType::BSPLINE )
        createBSplineFromBSplines( ( BSpline * )p_crv_1, ( BSpline * )p_crv_2 );
    }
  }
  catch( const std::runtime_error &err )
  {
    throw err;
  }
}

/******************************************************************************
* createBSplineFromBeziers
******************************************************************************/
BSpline *createBSplineFromBezierCurves( Bezier *bezier1, Bezier *bezier2 )
{
  point_vec combined_ctrl_pnts;
  combined_ctrl_pnts.insert( combined_ctrl_pnts.end(), bezier1->ctrl_pnts_.begin(), bezier1->ctrl_pnts_.end() - 1 );
  combined_ctrl_pnts.insert( combined_ctrl_pnts.end(), bezier2->ctrl_pnts_.begin(), bezier2->ctrl_pnts_.end() );

  int order = bezier1->order_;

  std::vector<double> knots;
  int n = combined_ctrl_pnts.size();

  for( int i = 0; i < order; ++i ) knots.push_back( 0.0 );
  for( int i = 1; i < n - order + 1; ++i ) knots.push_back( static_cast< double >( i ) / ( n - order + 1 ) );
  for( int i = 0; i < order; ++i ) knots.push_back( 1.0 );

  free_crv( bezier1 );
  free_crv( bezier2 );

  auto new_bspline = new BSpline( order, combined_ctrl_pnts, knots );
  register_crv( new_bspline );

  return new_bspline;
}

/******************************************************************************
* createBSplineFromBSplines
******************************************************************************/
BSpline *createBSplineFromBSplines( BSpline *bspline1, BSpline *bspline2 )
{
  point_vec combined_ctrl_pnts;
  combined_ctrl_pnts.insert( combined_ctrl_pnts.end(), bspline1->ctrl_pnts_.begin(), bspline1->ctrl_pnts_.end() - 1 );
  combined_ctrl_pnts.insert( combined_ctrl_pnts.end(), bspline2->ctrl_pnts_.begin(), bspline2->ctrl_pnts_.end() );

  int order = bspline1->order_;

  std::vector<double> knots;
  int n = combined_ctrl_pnts.size();

  for( int i = 0; i < order; ++i ) knots.push_back( 0.0 );
  for( int i = 1; i < n - order + 1; ++i ) knots.push_back( static_cast< double >( i ) / ( n - order + 1 ) );
  for( int i = 0; i < order; ++i ) knots.push_back( 1.0 );

  free_crv( bspline1 );
  free_crv( bspline2 );

  auto new_bspline = new BSpline( order, combined_ctrl_pnts, knots );
  register_crv( new_bspline );

  return new BSpline( order, combined_ctrl_pnts, knots );
}

/******************************************************************************
* make_open_callback
******************************************************************************/
void make_open_callback( int seg_id )
{
  try
  {
    Curve *p_crv = nullptr;
    CurveType crv_type = get_crv( seg_id, &p_crv );

    if( crv_type != CurveType::BSPLINE )
    {
      throw std::runtime_error( "wrong crv type" );
    }

    ( ( BSpline * )p_crv )->makeOpenKnotVector();
  }
  catch( const std::runtime_error &err )
  {
    throw err;
  }
}

/******************************************************************************
* make_uni_callback
******************************************************************************/
void make_uni_callback( int seg_id )
{
  try
  {
    Curve *p_crv = nullptr;
    CurveType crv_type = get_crv( seg_id, &p_crv );

    if( crv_type != CurveType::BSPLINE )
    {
      throw std::runtime_error( "wrong crv type" );
    }

    ( ( BSpline * )p_crv )->makeUniformKnotVector();
  }
  catch( const std::runtime_error &err )
  {
    throw err;
  }
}

/******************************************************************************
* update_knot_callback
******************************************************************************/
void update_knot_callback( int seg_id, int knot_idx, double new_val )
{
  try
  {
    Curve *p_crv = nullptr;
    CurveType crv_type = get_crv( seg_id, &p_crv );

    if( crv_type != CurveType::BSPLINE )
    {
      throw std::runtime_error( "wrong crv type" );
    }

    ( ( BSpline * )p_crv )->updateKnot( knot_idx, new_val );
  }
  catch( const std::runtime_error &err )
  {
    throw err;
  }
}

/******************************************************************************
* add_knot_callback
******************************************************************************/
void add_knot_callback( int seg_id, double val )
{
  try
  {
    Curve *p_crv = nullptr;
    CurveType crv_type = get_crv( seg_id, &p_crv );

    if( crv_type != CurveType::BSPLINE )
    {
      throw std::runtime_error( "wrong crv type" );
    }

    ( ( BSpline * )p_crv )->addKnot( val );
  }
  catch( const std::runtime_error &err )
  {
    throw err;
  }
}

/******************************************************************************
* rmv_knot_callback
******************************************************************************/
void rmv_knot_callback( int seg_id, int knot_idx )
{
  try
  {
    Curve *p_crv = nullptr;
    CurveType crv_type = get_crv( seg_id, &p_crv );

    if( crv_type != CurveType::BSPLINE )
    {
      throw std::runtime_error( "wrong crv type" );
    }

    ( ( BSpline * )p_crv )->rmvKnot( knot_idx );
  }
  catch( const std::runtime_error &err )
  {
    throw err;
  }
}

/******************************************************************************
* update_ctrl_pnt_callback
******************************************************************************/
void update_ctrl_pnt_callback( int pnt_id, double new_x, double new_y )
{
  Curve *p_crv = get_pnt_crv( pnt_id );

  if( p_crv != nullptr )
  {
    int pnt_idx = p_crv->get_pnt_id_idx( pnt_id );

    if( pnt_idx == K_NOT_USED )
    {
      throw std::runtime_error( "bad pnt id" );
    }

    p_crv->ctrl_pnts_[ pnt_idx ].x = new_x;
    p_crv->ctrl_pnts_[ pnt_idx ].y = new_y;
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
    cagdRedraw();
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
* register_crv
******************************************************************************/
void register_crv( Curve *p_crv )
{
  p_crv->show_crv();
  p_crv->show_ctrl_poly();
  cur_curves.push_back( p_crv );
}

/******************************************************************************
* free_crv
******************************************************************************/
void free_crv( Curve *p_crv )
{
  erase_seg_to_crv( p_crv->seg_ids_[ 0 ] );
  cagdFreeSegment( p_crv->seg_ids_[ 0 ] );

  for( int i = 0; i < p_crv->poly_seg_ids_.size(); ++i )
  {
    erase_seg_to_crv( p_crv->poly_seg_ids_[ i ] );
    cagdFreeSegment( p_crv->poly_seg_ids_[ i ] );
  }

  for( int i = 0; i < p_crv->pnt_ids_.size(); ++i )
  {
    erase_pnt_to_crv( p_crv->pnt_ids_[ i ] );
    erase_ctrl_seg_to_pnts( p_crv->pnt_ids_[ i ] );
    cagdFreeSegment( p_crv->pnt_ids_[ i ] );
  }

  auto it = std::find( cur_curves.begin(), cur_curves.end(), p_crv );

  if( it != cur_curves.end() )
    cur_curves.erase( it );

  delete p_crv;
}

/******************************************************************************
* clean_current_curves
******************************************************************************/
void clean_current_curves()
{
  for( auto curve : cur_curves )
    free_crv( curve );

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

    register_crv( curve );
  }

  file.close();
  return first_new_idx;
}
