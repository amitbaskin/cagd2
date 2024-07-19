#include "curve.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

std::vector<curve *> cur_curves;


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
void print_debug_curve_data( curve *curveData )
{
  char *curve_type = "none";
  switch( curveData->type )
  {
  case CURVE_TYPE_BEZIER:
    curve_type = "bezier";
    break;
  case CURVE_TYPE_BSPLINE:
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
curve *parse_file( const std::string &filePath )
{
  std::ifstream file( filePath, std::ios::binary ); // Open file in binary mode
  if( !file.is_open() )
  {
    print_err( "Error opening file" );
    return nullptr;
  }

  curve *curveData = new curve;
  curveData->type = CURVE_TYPE_NONE;
  curveData->order = 0;

  // Read order
  skip_blank_lines_and_comments( file );
  std::string line;
  if( !std::getline( file, line ) )
  {
    print_err( "Error reading file" );
    delete curveData;
    return nullptr;
  }

  ltrim( line );
  std::istringstream issOrder( line );
  if( !( issOrder >> curveData->order ) )
  {
    print_err( "Error reading order" );
    delete curveData;
    return nullptr;
  }

  // Check if the next line contains "knots"
  skip_blank_lines_and_comments( file );
  if( !std::getline( file, line ) )
  {
    print_err( "Error reading file" );
    delete curveData;
    return nullptr;
  }

  if( line.find( "knots" ) != std::string::npos )
  {
    curveData->type = CURVE_TYPE_BSPLINE;

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
      while( issLine >> knot ) // reading knots after "= "
      {
        curveData->knots.push_back( knot );
        if( curveData->knots.size() == numKnots )
        {
          break;
        }
      }

      while( std::getline( file, line ) )
      {
        ltrim( line );

        if( line.empty() || line[0] == '#' )
        {
          if( curveData->knots.size() != numKnots )
            continue; // still looking for knots in the next line
          else
            break;
        }

        std::istringstream issLine( line );
        while( issLine >> knot )
        {
          curveData->knots.push_back( knot );
          if( curveData->knots.size() == numKnots )
          {
            break;
          }
        }

        if( curveData->knots.size() == numKnots )
        {
          break;
        }
      }
    }
    else
    {
      print_err( "Error parsing knots size" );
      delete curveData;
      return nullptr;
    }
  }
  else
  {
    curveData->type = CURVE_TYPE_BEZIER;
    // Parse control points directly from this line
    std::istringstream issControlPoints( line );
    CAGD_POINT point;
    while( issControlPoints >> point.x >> point.y >> point.z )
    {
      curveData->ctrl_pts.push_back( point );
    }
  }

  // Read control points if not already parsed
  while( std::getline( file, line ) )
  {
    ltrim( line );
    if( line.empty() || line[0] == '#' )
    {
      continue; // Skip blank lines and comments
    }

    std::istringstream issControlPoints( line );
    CAGD_POINT point;
    while( issControlPoints >> point.x >> point.y >> point.z )
    {
      curveData->ctrl_pts.push_back( point );
    }
  }

  file.close(); // Close the file

  if( IS_DEBUG )
    print_debug_curve_data( curveData );

  return curveData;
}

/******************************************************************************
* free_curve_data
******************************************************************************/
void free_curve_data( curve *curveData )
{
  if( curveData )
  {
    delete curveData;
  }
}

/******************************************************************************
* load_curve
******************************************************************************/
void load_curve( int dummy1, int dummy2, void *p_data )
{
  char *file_path = ( char * )p_data;
  std::string file_str = file_path;
  curve *curve_data = parse_file( file_str );

  if( curve_data != nullptr )
    cur_curves.push_back( curve_data );

  //TODO: display_curve
}

/******************************************************************************
* save_curve
******************************************************************************/
void save_curve( int dummy1, int dummy2, void *p_data )
{
  // TODO
}