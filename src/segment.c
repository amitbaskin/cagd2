/************************************************************************
* Copyright (C) 1996, by Dmitri Bassarab.				*
* Send bugs and/or frames to /dev/null or dima@cs.technion.ac.il	*
*************************************************************************/
#include "cagd.h"
#include "internal.h"

typedef struct
{
  UINT        crv_type;
  BOOL        visible;
  UINT        length;
  PSTR        text;
  GLubyte     color_[ 3 ];
  CAGD_POINT *where;
} SEGMENT;

static GLubyte color_[] = { 255, 255, 255 };
static UINT nSegments = 0;
static SEGMENT *list = NULL;

static BOOL valid( UINT id )
{
  return ( id < nSegments ) && ( list[ id ].crv_type != CAGD_SEGMENT_UNUSED );
}

void cagdSetColor( BYTE red, BYTE green, BYTE blue )
{
  color_[ 0 ] = red;
  color_[ 1 ] = green;
  color_[ 2 ] = blue;
}

void cagdGetColor( BYTE *red, BYTE *green, BYTE *blue )
{
  *red = color_[ 0 ];
  *green = color_[ 1 ];
  *blue = color_[ 2 ];
}

BOOL cagdSetSegmentColor( UINT id, BYTE red, BYTE green, BYTE blue )
{
  SEGMENT *segment;
  if( !valid( id ) )
    return FALSE;
  segment = &list[ id ];
  segment->color_[ 0 ] = red;
  segment->color_[ 1 ] = green;
  segment->color_[ 2 ] = blue;
  return TRUE;
}

BOOL cagdGetSegmentColor( UINT id, BYTE *red, BYTE *green, BYTE *blue )
{
  SEGMENT *segment;
  if( !valid( id ) )
    return FALSE;
  segment = &list[ id ];
  *red = segment->color_[ 0 ];
  *green = segment->color_[ 1 ];
  *blue = segment->color_[ 2 ];
  return TRUE;
}

static UINT findUnused()
{
  UINT id;
  for( id = 1; id < nSegments; id++ )
    if( list[ id ].crv_type == CAGD_SEGMENT_UNUSED )
      break;
  if( nSegments <= id )
  {
    SEGMENT *tmp = ( SEGMENT * )realloc( list, sizeof( SEGMENT ) * ( nSegments += 20 ) );

    if( tmp != NULL )
    {
      list = tmp;

      for( id = nSegments - 20; id < nSegments; id++ )
      {
        SEGMENT *segment = &list[ id ];
        segment->crv_type = CAGD_SEGMENT_UNUSED;
        segment->visible = FALSE;
        segment->length = 0;
        segment->text = NULL;
        segment->where = NULL;
      }
    }

    return findUnused();
  }
  return id;
}

UINT cagdAddPoint( const CAGD_POINT *where )
{
  UINT id = findUnused();
  SEGMENT *segment = &list[ id ];
  segment->crv_type = CAGD_SEGMENT_POINT;
  segment->visible = TRUE;
  memcpy( segment->color_, color_, sizeof( GLubyte ) * 3 );
  segment->where = ( CAGD_POINT * )malloc( sizeof( CAGD_POINT ) );

  if( segment->where != NULL )
  {
    *segment->where = *where;
    segment->length = 1;
  }

  return id;
}

BOOL cagdReusePoint( UINT id, const CAGD_POINT *where )
{
  if( !valid( id ) )
    return FALSE;
  if( list[ id ].crv_type != CAGD_SEGMENT_POINT )
    return FALSE;
  *list[ id ].where = *where;
  return TRUE;
}

UINT cagdAddText( const CAGD_POINT *where, PCSTR text )
{
  UINT id = findUnused();
  SEGMENT *segment = &list[ id ];
  if( !text )
    return 0;
  segment->crv_type = CAGD_SEGMENT_TEXT;
  segment->visible = TRUE;
  memcpy( segment->color_, color_, sizeof( GLubyte ) * 3 );
  segment->where = ( CAGD_POINT * )malloc( sizeof( CAGD_POINT ) );

  if( segment->where != NULL )
  {
    *segment->where = *where;
    if( !text )
      text = "";
    segment->text = _strdup( text );
    segment->length = strlen( text );
  }

  return id;
}

BOOL cagdReuseText( UINT id, const CAGD_POINT *where, PCSTR text )
{
  SEGMENT *segment;
  if( !text )
    return FALSE;
  if( !valid( id ) )
    return FALSE;
  segment = &list[ id ];
  if( segment->crv_type != CAGD_SEGMENT_TEXT )
    return FALSE;
  *segment->where = *where;
  free( segment->text );
  segment->text = _strdup( text );
  segment->length = strlen( text );
  return TRUE;
}

BOOL cagdGetText( UINT id, PSTR text )
{
  SEGMENT *segment;
  if( !valid( id ) )
    return FALSE;
  segment = &list[ id ];
  if( segment->crv_type != CAGD_SEGMENT_TEXT )
    return FALSE;
  strcpy( text, segment->text );
  return TRUE;
}

UINT cagdAddPolyline( const CAGD_POINT *where, UINT length )
{
  UINT id = findUnused();
  SEGMENT *segment = &list[ id ];
  if( length < 2 )
    return 0;
  segment->crv_type = CAGD_SEGMENT_POLYLINE;
  segment->visible = TRUE;
  memcpy( segment->color_, color_, sizeof( GLubyte ) * 3 );
  segment->where = ( CAGD_POINT * )malloc( sizeof( CAGD_POINT ) * length );

  if( segment->where != NULL )
  {
    memcpy( segment->where, where, sizeof( CAGD_POINT ) * length );
    segment->length = length;
  }

  return id;
}

BOOL cagdReusePolyline( UINT id, const CAGD_POINT *where, UINT length )
{
  SEGMENT *segment;
  if( length < 2 )
    return 0;
  if( !valid( id ) )
    return FALSE;
  segment = &list[ id ];
  if( segment->crv_type != CAGD_SEGMENT_POLYLINE )
    return FALSE;
  free( segment->where );
  segment->where = ( CAGD_POINT * )malloc( sizeof( CAGD_POINT ) * length );

  if( segment->where != NULL )
  {
    memcpy( segment->where, where, sizeof( CAGD_POINT ) * length );
    segment->length = length;
  }

  return TRUE;
}

BOOL cagdGetVertex( UINT id, UINT vertex, CAGD_POINT *where )
{
  SEGMENT *segment;
  if( !valid( id ) )
    return FALSE;
  segment = &list[ id ];
  if( segment->crv_type != CAGD_SEGMENT_POLYLINE )
    return FALSE;
  if( segment->length <= vertex )
    return FALSE;
  memcpy( where, &segment->where[ vertex ], sizeof( CAGD_POINT ) );
  return TRUE;
}

BOOL cagdSetVertex( UINT id, UINT vertex, const CAGD_POINT *where )
{
  SEGMENT *segment;
  if( !valid( id ) )
    return FALSE;
  segment = &list[ id ];
  if( segment->crv_type != CAGD_SEGMENT_POLYLINE )
    return FALSE;
  if( segment->length <= vertex )
    return FALSE;
  memcpy( &segment->where[ vertex ], where, sizeof( CAGD_POINT ) );
  return TRUE;
}

BOOL cagdShowSegment( UINT id )
{
  if( !valid( id ) )
    return FALSE;
  list[ id ].visible = TRUE;
  return TRUE;
}

BOOL cagdHideSegment( UINT id )
{
  if( !valid( id ) )
    return FALSE;
  list[ id ].visible = FALSE;
  return TRUE;
}

BOOL cagdIsSegmentVisible( UINT id )
{
  if( !valid( id ) )
    return FALSE;
  return list[ id ].visible;
}

BOOL cagdFreeSegment( UINT id )
{
  SEGMENT *segment;
  if( !valid( id ) )
    return FALSE;
  segment = &list[ id ];
  if( segment->crv_type == CAGD_SEGMENT_TEXT )
    free( segment->text );
  segment->crv_type = CAGD_SEGMENT_UNUSED;
  free( segment->where );
  return TRUE;
}

void cagdFreeAllSegments()
{
  UINT id;
  for( id = 1; id < nSegments; id++ )
    cagdFreeSegment( id );
}

UINT cagdGetSegmentType( UINT id )
{
  if( id < nSegments )
    return list[ id ].crv_type;
  return CAGD_SEGMENT_UNUSED;
}

UINT cagdGetSegmentLength( UINT id )
{
  if( !valid( id ) )
    return 0;
  return list[ id ].length;
}

BOOL cagdGetSegmentLocation( UINT id, CAGD_POINT *where )
{
  UINT length = 1;
  SEGMENT *segment;
  if( !valid( id ) )
    return 0;
  segment = &list[ id ];
  if( segment->crv_type == CAGD_SEGMENT_POLYLINE )
    length = segment->length;
  memcpy( where, segment->where, sizeof( CAGD_POINT ) * length );
  return TRUE;
}

UINT cagdGetNearestVertex( UINT id, int x, int y )
{
  UINT i, minI = 0;
  int X, Y, d, minD;
  SEGMENT *segment;
  if( !valid( id ) )
    return 0;
  segment = &list[ id ];
  if( segment->crv_type != CAGD_SEGMENT_POLYLINE )
    return 0;
  for( i = 0; i < segment->length; i++ )
  {
    if( !cagdToWindow( &segment->where[ i ], &X, &Y ) )
      continue;
    d = ( X - x ) * ( X - x ) + ( Y - y ) * ( Y - y );
    if( i == 0 )
      minD = d;
    if( d < minD )
    {
      minD = d;
      minI = i;
    }
  }
  return ++minI;
}

void drawSegments( GLenum mode )
{
  UINT id, i;
  glClear( GL_COLOR_BUFFER_BIT );
  if( mode == GL_SELECT )
  {
    glInitNames();
    glPushName( 0 );
  }
  for( id = 1; id < nSegments; id++ )
  {
    SEGMENT *segment = &list[ id ];
    if( segment->crv_type == CAGD_SEGMENT_UNUSED )
      continue;
    if( !segment->visible )
      continue;
    if( mode == GL_SELECT )
      glLoadName( id );
    glColor3ubv( segment->color_ );
    switch( segment->crv_type )
    {
    case CAGD_SEGMENT_POINT:
      glBegin( GL_POINTS );
      glVertex3dv( ( GLdouble * )segment->where );
      glEnd();
      break;
    case CAGD_SEGMENT_POLYLINE:
      glBegin( GL_LINE_STRIP );
      for( i = 0; i < segment->length; i++ )
        glVertex3dv( ( GLdouble * )&segment->where[ i ] );
      glEnd();
      break;
    case CAGD_SEGMENT_TEXT:
      if( mode == GL_SELECT )
        break;
      glRasterPos3dv( ( GLdouble * )segment->where );
      auxDrawStr( segment->text );
      break;
    }
  }
}