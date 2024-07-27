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
#include <algorithm>

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
void update_weight_callback( int pt_id, int pnt_idx, double val )
{
  Curve *p_crv = nullptr;
  p_crv = get_pnt_crv( pt_id );

  if( p_crv != nullptr )
  {
    p_crv->update_weight( pnt_idx, val );
    p_crv->show_crv();
    cagdRedraw();
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
    return CurveType::NONE;

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
bool connect_crv_callback( int seg_id_1, int seg_id_2, ConnType conn )
{
  try
  {
    Curve *p_crv_1 = nullptr;
    Curve *p_crv_2 = nullptr;
    CurveType crv_type_1 = get_crv( seg_id_1, &p_crv_1 );
    CurveType crv_type_2 = get_crv( seg_id_2, &p_crv_2 );

    if( crv_type_2 == CurveType::NONE )
      return false;

    /*createBSplineFromCurves( p_crv_1, p_crv_2, conn );
    cagdRedraw();
    return true;*/
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

    /*if( crv_type_1 == CurveType::BEZIER )
    {
      if( crv_type_2 == CurveType::BEZIER )
        createBSplineFromBezierCurves( ( Bezier * )p_crv_1, ( Bezier * )p_crv_2 );
    }
    else
    {
      if( crv_type_2 == CurveType::BSPLINE )
        createBSplineFromBSplines( ( BSpline * )p_crv_1, ( BSpline * )p_crv_2 );
    }*/
  }
  catch( const std::runtime_error &err )
  {
    throw err;
  }

  //cagdRedraw();
  return true;
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
* computeTangent
******************************************************************************/
CAGD_POINT computeTangent( const Curve *curve, double param )
{
  const double h = 1e-5; // Small delta for numerical differentiation

  double domainStart = curve->get_dom_start();
  double domainEnd = curve->get_dom_end();

  double t1 = max( domainStart, param - h );
  double t2 = min( domainEnd, param + h );

  CAGD_POINT p1 = curve->evaluate( t1 );
  CAGD_POINT p2 = curve->evaluate( t2 );

  CAGD_POINT tangent;
  diff_vecs_2d( &p2, &p1, &tangent );
  normalize_vec_2d( &tangent );

  return tangent;
}

/******************************************************************************
* constructKnotVector
******************************************************************************/
std::vector<double> constructKnotVector( const Curve *crv1, const Curve *crv2, ConnType continuityType )
{
  if( crv1 == nullptr || crv2 == nullptr )
    throw std::runtime_error( "bad crvs to connect" );

  std::vector<double> knots1;
  std::vector<double> knots2;

  // Extract knots from the first curve
  if( const Bezier *b1 = dynamic_cast< const Bezier * >( crv1 ) )
  {
    knots1.resize( b1->ctrl_pnts_.size() + b1->order_, 0.0 );
    std::fill( knots1.begin() + b1->ctrl_pnts_.size(), knots1.end(), 1.0 );
  }
  else if( const BSpline *bs1 = dynamic_cast< const BSpline * >( crv1 ) )
  {
    knots1 = bs1->knots_;
  }

  // Extract knots from the second curve
  if( const Bezier *b2 = dynamic_cast< const Bezier * >( crv2 ) )
  {
    knots2.resize( b2->ctrl_pnts_.size() + b2->order_, 0.0 );
    std::fill( knots2.begin() + b2->ctrl_pnts_.size(), knots2.end(), 1.0 );
  }
  else if( const BSpline *bs2 = dynamic_cast< const BSpline * >( crv2 ) )
  {
    knots2 = bs2->knots_;
  }

  double maxKnot1 = *std::max_element( knots1.begin(), knots1.end() );

  // Adjust knots2 to match the maximum value of knots1
  for( double &knot : knots2 )
  {
    knot += maxKnot1 + 1;
  }

  int min_order = min( crv1->order_, crv2->order_ );
  int order = max( crv1->order_, crv2->order_ );
  double ratio = ( double )min_order / ( double )order;

  int num1 = min_order / 2;
  int num2 = min_order - num1;

  // Merge the two knot vectors
  std::vector<double> newKnotVector;
  newKnotVector.insert( newKnotVector.end(), knots1.begin(), knots1.end() - num1 );
  newKnotVector.insert( newKnotVector.end(), knots2.begin() + num2, knots2.end() );

  return newKnotVector;
}

/******************************************************************************
* adjustControlPointsForContinuity
******************************************************************************/
void adjustControlPointsForContinuity( Curve *crv1, Curve *crv2, ConnType continuityType )
{
  if( continuityType == ConnType::C1 )
  {
    // Ensure C1 continuity: The first curve's end tangent should match the second curve's start tangent
    CAGD_POINT tangent1 = computeTangent( crv1, crv1->get_dom_end() );
    CAGD_POINT tangent2 = computeTangent( crv2, crv2->get_dom_start() );

    // Compute the control points adjustment for C1 continuity
    CAGD_POINT end1 = crv1->ctrl_pnts_.back();
    CAGD_POINT start2 = crv2->ctrl_pnts_.front();

    CAGD_POINT tangentAdjustment;
    diff_vecs_2d( &tangent1, &tangent2, &tangentAdjustment );
    add_vecs_2d( &end1, &tangentAdjustment, &crv2->ctrl_pnts_.front() );
  }
  else if( continuityType == ConnType::G1 )
  {
    // Ensure G1 continuity: The tangent vectors at the connection point should be collinear
    CAGD_POINT tangent1 = computeTangent( crv1, crv1->get_dom_end() );
    CAGD_POINT tangent2 = computeTangent( crv2, crv2->get_dom_start() );

    // Normalize tangents
    normalize_vec_2d( &tangent1 );
    normalize_vec_2d( &tangent2 );

    // Adjust control points for G1 continuity
    CAGD_POINT end1 = crv1->ctrl_pnts_.back();
    CAGD_POINT start2 = crv2->ctrl_pnts_.front();

    CAGD_POINT adjustedStart2;
    CAGD_POINT tangentDifference;
    diff_vecs_2d( &tangent2, &tangent1, &tangentDifference );
    double dotProduct = tangent1.x * tangent2.x + tangent1.y * tangent2.y;

    // Adjust the start control point of crv2 to align with the tangent of crv1
    if( dotProduct > 0 )
    {
      add_vecs_2d( &end1, &tangent1, &adjustedStart2 );
      add_vecs_2d( &adjustedStart2, &tangentDifference, &crv2->ctrl_pnts_.front() );
    }
  }
}

/******************************************************************************
* createBSplineFromCurves
******************************************************************************/
BSpline *createBSplineFromCurves( Curve *crv1, Curve *crv2, ConnType continuityType )
{
  if( !crv1 || !crv2 )
    throw std::invalid_argument( "Curve pointers must not be null" );

  std::vector<double> knots1;
  std::vector<double> knots2;

  // Extract knots from the first curve
  if( const Bezier *b1 = dynamic_cast< const Bezier * >( crv1 ) )
  {
    knots1.resize( b1->ctrl_pnts_.size() + b1->order_, 0.0 );
    std::fill( knots1.begin() + b1->ctrl_pnts_.size(), knots1.end(), 1.0 );
  }
  else if( const BSpline *bs1 = dynamic_cast< const BSpline * >( crv1 ) )
  {
    knots1 = bs1->knots_;
  }

  // Extract knots from the second curve
  if( const Bezier *b2 = dynamic_cast< const Bezier * >( crv2 ) )
  {
    knots2.resize( b2->ctrl_pnts_.size() + b2->order_, 0.0 );
    std::fill( knots2.begin() + b2->ctrl_pnts_.size(), knots2.end(), 1.0 );
  }
  else if( const BSpline *bs2 = dynamic_cast< const BSpline * >( crv2 ) )
  {
    knots2 = bs2->knots_;
  }

  // Prepare control points for the new B-spline
  std::vector<CAGD_POINT> combinedCtrlPoints = crv1->ctrl_pnts_;
  combinedCtrlPoints.insert( combinedCtrlPoints.end(), crv2->ctrl_pnts_.begin(), crv2->ctrl_pnts_.end() );

  // Adjust control points for continuity
  Curve *mutableCrv1 = const_cast< Curve * >( crv1 );
  Curve *mutableCrv2 = const_cast< Curve * >( crv2 );
  adjustControlPointsForContinuity( mutableCrv1, mutableCrv2, continuityType );

  //// Construct the knot vector using the helper function
  //std::vector<double> knotVector = constructKnotVector( crv1, crv2, continuityType );

  // Determine the order of the new B-spline
  int order1 = crv1->order_;
  int order2 = crv2->order_;
  int order = max( order1, order2 );

  // Create the new B-spline curve
  BSpline *newBSpline = new BSpline();
  newBSpline->order_ = order;
  newBSpline->ctrl_pnts_ = combinedCtrlPoints;
  //newBSpline->knots_ = knotVector;
  newBSpline->makeUniformKnotVector( true, knots2[ crv2->order_ - 1 ], knots2[ crv2->ctrl_pnts_.size() ] );

  double add_knot = knots2[ crv2->ctrl_pnts_.size() ];

  for( size_t i = 0; i < knots2.size(); ++i )
    knots2[ i ] += add_knot;

  int diff = newBSpline->knots_.size() - knots2.size();

  for( size_t i = 0; i < knots2.size(); ++i )
    newBSpline->knots_[ i + diff ] = knots2[ i ];

  // Set the spline as open or uniform based on its structure (for simplicity, assuming uniform here)
  newBSpline->is_open_ = false;
  newBSpline->is_uni_ = false;

  free_crv( crv1 );
  free_crv( crv2 );
  register_crv( newBSpline );

  return newBSpline;
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
* show_add_curve_help_text
******************************************************************************/
void show_add_curve_help_text()
{
  std::string text = "Left click on screen to add a control point.\n"
                     "Click on middle mouse button to exit Add Curve Mode";
  cagdSetHelpText( text.c_str() );
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
  if( p_crv != nullptr )
  {
    remove_crv_data( p_crv );

    erase_crv_from_cur_crvs( p_crv );

    delete p_crv;
    p_crv = nullptr;
  }
}

/******************************************************************************
* remove_crv_data
******************************************************************************/
void remove_crv_data( Curve *p_crv )
{
  if( p_crv != nullptr )
  {
    if( p_crv->seg_ids_.size() > 0 )
    {
      erase_seg_to_crv( p_crv->seg_ids_[0] );
      cagdFreeSegment( p_crv->seg_ids_[0] );
    }

    for( size_t i = 0; i < p_crv->poly_seg_ids_.size(); ++i )
    {
      erase_seg_to_crv( p_crv->poly_seg_ids_[i] );
      cagdFreeSegment( p_crv->poly_seg_ids_[i] );
    }

    for( size_t i = 0; i < p_crv->pnt_ids_.size(); ++i )
    {
      erase_pnt_to_crv( p_crv->pnt_ids_[i] );
      erase_ctrl_seg_to_pnts( p_crv->pnt_ids_[i] );
      cagdFreeSegment( p_crv->pnt_ids_[i] );
    }
  }
}

/******************************************************************************
* erase_crv_from_cur_crvs
******************************************************************************/
void erase_crv_from_cur_crvs( Curve *p_curve )
{
  auto it = std::find( cur_curves.begin(), cur_curves.end(), p_curve );

  if( it != cur_curves.end() )
    cur_curves.erase( it );
}

/******************************************************************************
* clean_current_curves
******************************************************************************/
void clean_current_curves()
{
  auto it = cur_curves.begin();
  while( it != cur_curves.end() )
  {
    remove_crv_data( *it );
    delete *it;
    *it = nullptr;
    it = cur_curves.erase( it );
  }

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
