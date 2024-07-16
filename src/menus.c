#include <cagd.h>
#include <stdio.h>
#include "menus.h"
#include "resource.h"
#include "color.h"

char myBuffer[BUFSIZ];
UINT myText;

extern void myMessage( PSTR title, PSTR message, UINT type );

/******************************************************************************
* init_menus
******************************************************************************/
void init_menus()
{
  HMENU op_menu = CreatePopupMenu(); // options
  HMENU curve_menu = CreatePopupMenu(); // curve

  // Curve
  AppendMenu( curve_menu, MF_STRING, CAGD_RESET, "new menu" );
  //AppendMenu( op_menu, MF_STRING, CAGD_SHOW_EVOLUTE_MENU, "Show Evolute Curve" );
  //AppendMenu( op_menu, MF_SEPARATOR, 0, NULL );

  // Options
  AppendMenu( op_menu, MF_STRING, CAGD_RESET, "new menu" );


  // adding to cagd
  cagdAppendMenu( curve_menu, "Curve" );
  cagdAppendMenu( op_menu, "Options" );

  // register callback to handle all menus
  cagdRegisterCallback( CAGD_MENU, menu_callbacks, NULL );

  // register callback to handle lmb on cur_curve
  cagdRegisterCallback( CAGD_LBUTTONUP, left_mouse_click_cb, NULL );
}

/******************************************************************************
* myDialogProc REFINEMENT DIALOG
******************************************************************************/
LRESULT CALLBACK myDialogProc( HWND hDialog, UINT message, WPARAM wParam, LPARAM lParam )
{
  if( message != WM_COMMAND )
    return FALSE;
  switch( LOWORD( wParam ) )
  {
  case IDOK:
    GetDlgItemText( hDialog, IDC_EDIT, myBuffer, sizeof( myBuffer ) );
    EndDialog( hDialog, TRUE );
    return TRUE;
  case IDCANCEL:
    EndDialog( hDialog, FALSE );
    return TRUE;
  default:
    return FALSE;
  }
}

/******************************************************************************
* menu_callbacks
******************************************************************************/
void menu_callbacks( int id, int unUsed, PVOID userData )
{
  int is_error = 0;

  switch( id )
  {
  
  }

}

/******************************************************************************
* left_mouse_click_cb
******************************************************************************/
void left_mouse_click_cb( int x, int y, PVOID userData )
{
  CAGD_POINT p = { 0.0,0.0,0.0 };
  cagdHideSegment( myText = cagdAddText( &p, "" ) );

  UINT id;
  int v;
  for( cagdPick( x, y ); id = cagdPickNext();)
    if( cagdGetSegmentType( id ) == CAGD_SEGMENT_POLYLINE )
      break;
  if( id )
  {
    if( v = cagdGetNearestVertex( id, x, y ) )
    {
      cagdGetVertex( id, --v, &p );
      //double param = get_param_from_segment_number( v );

    }
  }
  cagdRedraw();
}