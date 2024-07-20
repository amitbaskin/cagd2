#include "Curve.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "color.h"

extern int num_samples;
extern bool show_control_polyline;
extern unsigned int curve_color[3];

std::vector<Curve *> cur_curves;

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
* print_debug_curve_data
******************************************************************************/
void print_debug_curve_data( Curve *curveData )
{
  char *curve_type = "none";
  switch( curveData->type )
  {
  case curve_type::CURVE_TYPE_BEZIER:
    curve_type = "bezier";
    break;
  case curve_type::CURVE_TYPE_BSPLINE:
    curve_type = "bspline";
    break;
  default:
    break;
  }
  printf( "Curve Type: %d (%s)\n", curveData->type, curve_type );
  printf( "Order: %u\n", curveData->order );

  printf( "Number of knots: %u\n", curveData->knots.size() );
  if( !curveData->knots.empty() )
  {
    printf( "Knots: " );
    for( const double &knot : curveData->knots )
    {
      printf( "%f ", knot );
    }
    printf( "\n" );
  }

  printf( "Control Points:\n" );
  for( const CAGD_POINT &pt : curveData->ctrl_pts )
  {
    printf( "(%f, %f", pt.x, pt.y );
    printf( ", %f", pt.z );
    printf( ")\n" );
  }

  printf( "\n\n\n" );
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
      lastPos = file.tellg(); // Record the position before reading the next line
      continue; // Skip blank lines and comments
    }
    // If a valid line is found, rewind to the position before the last read line
    file.seekg( lastPos );
    break; // Exit the loop once a valid line is found
  }
}


/******************************************************************************
* parse_file
******************************************************************************/
void parse_file( const std::string &filePath )
{
  std::ifstream file( filePath, std::ios::binary ); // Open file in binary mode
  if( !file.is_open() )
  {
    print_err( "Error opening file" );
    return;
  }

  while( file )
  {
    Curve *curveData = new Curve();
    curveData->type = curve_type::CURVE_TYPE_NONE;
    curveData->order = 0;
    curveData->bs_crv = nullptr; // Assuming bs_crv and bz_crv are not needed for this example
    curveData->bz_crv = nullptr;
    curveData->color[ 0 ] = curve_color[ 0 ];
    curveData->color[ 1 ] = curve_color[ 1 ];
    curveData->color[ 2 ] = curve_color[ 2 ];
    curveData->poly_id = K_NOT_USED;
    curveData->curve_id = K_NOT_USED;

    // Read order
    skip_blank_lines_and_comments( file );
    std::string line;
    if( !std::getline( file, line ) )
    {
      if( !file.eof() )
      {
        print_err( "Error reading file" );
      }
      delete curveData;
      break;
    }

    ltrim( line );
    std::istringstream issOrder( line );
    if( !( issOrder >> curveData->order ) )
    {
      print_err( "Error reading order" );
      delete curveData;
      continue;
    }

    // Check if the next line contains "knots"
    skip_blank_lines_and_comments( file );
    if( !std::getline( file, line ) )
    {
      if( !file.eof() )
      {
        print_err( "Error reading file" );
      }
      delete curveData;
      break;
    }

    if( line.find( "knots" ) != std::string::npos )
    {
      curveData->type = curve_type::CURVE_TYPE_BSPLINE;

      // Extract the number of knots from the line
      size_t pos = line.find( "[" );
      size_t posEnd = line.find( "]" );
      if( pos != std::string::npos && posEnd != std::string::npos && pos < posEnd )
      {
        std::string numKnotsStr = line.substr( pos + 1, posEnd - pos - 1 );
        trim( numKnotsStr );
        int numKnots = std::stoi( numKnotsStr );

        // Extract the knots from subsequent lines until a blank or comment line
        std::string after_equal_sign = line.substr( posEnd + 3 );
        ltrim( after_equal_sign );

        std::istringstream issLine( after_equal_sign ); // Start reading after "= "
        double knot;

        while( issLine >> knot )
        { // reading knots after "= "
          curveData->knots.push_back( knot );
          if( curveData->knots.size() == numKnots )
          {
            break;
          }
        }

        while( curveData->knots.size() < numKnots && std::getline( file, line ) )
        {
          ltrim( line );
          if( line.empty() || line[ 0 ] == '#' )
          {
            continue; // Skip blank lines and comments
          }

          std::istringstream issLine( line );
          double knot;

          while( issLine >> knot )
          { // reading knots after "= "
            curveData->knots.push_back( knot );
            if( curveData->knots.size() == numKnots )
            {
              break;
            }
          }
        }
      }
      else
      {
        print_err( "Error parsing knots size" );
        delete curveData;
        continue;
      }
    }
    else if( curveData->order > 0 )
    {
      curveData->type = curve_type::CURVE_TYPE_BEZIER;
    }
    else
    {
      delete curveData;
      break;
    }

    ltrim( line );
    if( line.empty() || line[ 0 ] == '#' )
    {
      continue; // Skip blank lines and comments
    }

    std::istringstream issControlPoints( line );
    CAGD_POINT point;
    while( issControlPoints >> point.x >> point.y >> point.z )
    {
      point.x /= point.z;
      point.y /= point.z;
      curveData->ctrl_pts.push_back( point );
    }

    if( curveData->type == curve_type::CURVE_TYPE_BEZIER )
    {
      if( curveData->ctrl_pts.size() == curveData->order )
        break;
    }
    else
    {
      if( curveData->ctrl_pts.size() == curveData->knots.size() - curveData->order )
        break;
    }

    // Read control points if not already parsed
    while( std::getline( file, line ) )
    {
      ltrim( line );
      if( line.empty() || line[ 0 ] == '#' )
      {
        continue; // Skip blank lines and comments
      }

      std::istringstream issControlPoints( line );
      CAGD_POINT point;
      while( issControlPoints >> point.x >> point.y >> point.z )
      {
        point.x /= point.z;
        point.y /= point.z;
        curveData->ctrl_pts.push_back( point );
      }

      if( curveData->type == curve_type::CURVE_TYPE_BEZIER )
      {
        if( curveData->ctrl_pts.size() == curveData->order )
          break;
      }
      else
      {
        if( curveData->ctrl_pts.size() == curveData->knots.size() - curveData->order )
          break;
      }
    }

    if( IS_DEBUG )
    {
      print_debug_curve_data( curveData );
    }

    cur_curves.push_back( curveData );
  }

  file.close(); // Close the file
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
}

/******************************************************************************
* load_curve
******************************************************************************/
void load_curve( int dummy1, int dummy2, void *p_data )
{
  char *file_path = ( char * )p_data;
  std::string file_str = file_path;
  parse_file( file_str );

  for( int i = 0; i < cur_curves.size(); ++i )
  {
    auto curve_data = cur_curves[ i ];

    if( curve_data != nullptr )
    {
      if( curve_data->type == curve_type::CURVE_TYPE_BEZIER )
      {
        curve_data->bz_crv = new BezierCurve( curve_data->ctrl_pts );
      }
      else if( curve_data->type == curve_type::CURVE_TYPE_BSPLINE )
      {
        curve_data->bs_crv = new BSpline( curve_data->ctrl_pts,
                                          curve_data->knots,
                                          curve_data->order );
      }

      show_curve( curve_data, true );
    }
  }
}

/******************************************************************************
* save_curve
******************************************************************************/
void save_curve( int dummy1, int dummy2, void *p_data )
{
  // TODO
}

/******************************************************************************
* get_jump_sample_val
******************************************************************************/
static double get_jump_sample_val( double start, double end, int num_pnts )
{
  return ( end - start ) / ( ( double )num_pnts - 1 );
}

/******************************************************************************
* show_curve
******************************************************************************/
void show_curve( Curve *curve_data, bool redraw_ctrl_polyline )
{
  if( show_control_polyline && redraw_ctrl_polyline )
  {
    show_ctrl_pts_polyline( curve_data );
  }

  show_curve_helper( curve_data );

  cagdRedraw();
}

/******************************************************************************
* show_ctrl_pts_polyline
******************************************************************************/
void show_ctrl_pts_polyline( Curve *curve_data )
{
  int num_pts = curve_data->ctrl_pts.size();

  if( num_pts > 1 )
  {
    CAGD_POINT *pnts = ( CAGD_POINT * )malloc( sizeof( CAGD_POINT ) *
                                               num_pts );

    if( pnts != nullptr )
    {
      set_norm_color();

      for( int i = 0; i < num_pts; i++ )
      {
        if( curve_data->ctrl_pts[i].z == 0 )
        {
          print_err( "control point can't have weight 0" );
          return;
        }

        pnts[i] = { curve_data->ctrl_pts[i].x/* / curve_data->ctrl_pts[i].z*/,
                    curve_data->ctrl_pts[i].y/* / curve_data->ctrl_pts[i].z*/,
                    0 };

        cagdAddPoint( &pnts[i] );
      }
    }

    set_bi_color();
    if( curve_data->poly_id == K_NOT_USED )
      curve_data->poly_id = cagdAddPolyline( pnts, num_pts );
    else
      cagdReusePolyline( curve_data->poly_id, pnts, num_pts );

    free( pnts );
  }
}
CAGD_POINT Curve::evaluate( double param )
{
  if( type == curve_type::CURVE_TYPE_BEZIER )
    return bz_crv->evaluate( param );
  else
    return bs_crv->evaluate( param );
}

double Curve::get_end_param()
{
  if( type == curve_type::CURVE_TYPE_BEZIER )
    return 1.0;
  else
    return knots[ knots.size()-1 ];
}

/******************************************************************************
* show_curve_helper
******************************************************************************/
void show_curve_helper( Curve *curve_data )
{
  CAGD_POINT *pnts = ( CAGD_POINT * )malloc( sizeof( CAGD_POINT ) *
                                             num_samples );
  if( pnts != NULL )
  {
    double start = 0.0;
    double end = curve_data->get_end_param();

    double jump = get_jump_sample_val( start,
                                       end,
                                       num_samples );

    for( int i = 0; i < num_samples; i++ )
    {
      double param = 0 + jump * i;

      pnts[ i ] = curve_data->evaluate( param );
    }

    cagdSetColor( curve_data->color[0], curve_data->color[1], curve_data->color[2] );

    if( curve_data->curve_id == K_NOT_USED )
      curve_data->curve_id = cagdAddPolyline( pnts, num_samples );
    else
      cagdReusePolyline( curve_data->curve_id, pnts, num_samples );

    set_default_color();
  }

  free( pnts );
}

/******************************************************************************
* show_bspline_curve
******************************************************************************/
void show_bspline_curve( Curve *curve_data )
{

}

/******************************************************************************
* redraw_all_curves
******************************************************************************/
void redraw_all_curves()
{
  for( auto p_crv : cur_curves )
  {
    show_curve( p_crv, false );
  }
}