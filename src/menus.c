#include <cagd.h>
#include <stdio.h>
#include "menus.h"
#include "resource.h"
#include "color.h"

char samples_buffer[BUFSIZ];
char default_deg_buffer[BUFSIZ];
UINT myText2;

extern void myMessage( PSTR title, PSTR message, UINT type );
extern int num_samples;
extern unsigned int default_degree;

/******************************************************************************
* init_menus
******************************************************************************/
void init_menus()
{
  HMENU op_menu = CreatePopupMenu(); // options
  HMENU curve_menu = CreatePopupMenu(); // curve

  // Curve
  AppendMenu( curve_menu, MF_STRING, CAGD_SETTINGS, "new menu" );
  //AppendMenu( op_menu, MF_STRING, CAGD_SHOW_EVOLUTE_MENU, "Show Evolute Curve" );
  //AppendMenu( op_menu, MF_SEPARATOR, 0, NULL );

  // Options
  AppendMenu( op_menu, MF_STRING, CAGD_SETTINGS, "Settings" );


  // adding to cagd
  cagdAppendMenu( curve_menu, "Curve" );
  cagdAppendMenu( op_menu, "Options" );

  // register callback to handle all menus
  cagdRegisterCallback( CAGD_MENU, menu_callbacks, NULL );

  // register callback to handle lmb on cur_curve
  cagdRegisterCallback( CAGD_LBUTTONUP, left_mouse_click_cb, NULL );
}

/******************************************************************************
* myDialogProc SETTINGS DIALOG
******************************************************************************/
LRESULT CALLBACK SettingsDialogProc( HWND hDialog, UINT message, WPARAM wParam, LPARAM lParam )
{
  switch( message )
  {
  case WM_INITDIALOG:
    SetDlgItemInt( hDialog, IDC_SAMPLES, num_samples, FALSE );
    SetDlgItemInt( hDialog, IDC_DEF_DEGREE, default_degree, FALSE );
    break;


  case WM_COMMAND:
    switch( LOWORD( wParam ) )
    {
    case IDOK:
      GetDlgItemText( hDialog, IDC_SAMPLES, samples_buffer, sizeof( samples_buffer ) );
      GetDlgItemText( hDialog, IDC_DEF_DEGREE, default_deg_buffer, sizeof( default_deg_buffer ) );
      EndDialog( hDialog, TRUE );
      return TRUE;
    case IDCANCEL:
      EndDialog( hDialog, FALSE );
      return TRUE;
    default:
      return FALSE;
    }
  break;

  default:
    return FALSE;
    break;
  }

  return FALSE;
}

/******************************************************************************
* menu_callbacks
******************************************************************************/
void menu_callbacks( int id, int unUsed, PVOID userData )
{
  int is_error = 0;

  switch( id )
  {
  case CAGD_SETTINGS:
    handle_settings_menu();
    break;
  }

}

/******************************************************************************
* handle_settings_menu
******************************************************************************/
void handle_settings_menu()
{
  if( DialogBox( cagdGetModule(),
      MAKEINTRESOURCE( IDD_SETTINGS ),
      cagdGetWindow(),
      ( DLGPROC )SettingsDialogProc ) )
  {

  }
}

/******************************************************************************
* left_mouse_click_cb
******************************************************************************/
void left_mouse_click_cb( int x, int y, PVOID userData )
{
  CAGD_POINT p = { 0.0,0.0,0.0 };
  cagdHideSegment( myText2 = cagdAddText( &p, "" ) );

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