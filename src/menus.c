#include <cagd.h>
#include <stdio.h>
#include "menus.h"
#include "resource.h"
#include "color.h"
#include "options.h"

char buffer1[ BUFSIZ ];
char buffer2[ BUFSIZ ];
char buffer3[ BUFSIZ ];
UINT myText2;

Curve *active_rmb_curve = nullptr;
std::tuple< Curve *, int > active_crv_ctrl_pt = std::make_tuple( nullptr, -1 );

extern void myMessage( PSTR title, PSTR message, UINT crv_type );

/******************************************************************************
* init_menus
******************************************************************************/
void init_menus()
{
  HMENU op_menu = CreatePopupMenu(); // options
  HMENU curve_menu = CreatePopupMenu(); // Curve

  // Curve
  AppendMenu( curve_menu, MF_STRING, CAGD_CURVE_COLOR, "Default Color" );
  //AppendMenu( op_menu, MF_STRING, CAGD_SHOW_EVOLUTE_MENU, "Show Evolute Curve" );
  //AppendMenu( op_menu, MF_SEPARATOR, 0, NULL );

  // Options
  AppendMenu( op_menu, MF_STRING, CAGD_SETTINGS, "Settings" );
  AppendMenu( op_menu, MF_SEPARATOR, 0, NULL );
  AppendMenu( op_menu, MF_STRING, CAGD_CLEAN_ALL, "Clean all" );

  // adding to cagd
  cagdAppendMenu( curve_menu, "Curve" );
  cagdAppendMenu( op_menu, "Options" );

  // register callback to handle all menus
  cagdRegisterCallback( CAGD_MENU, menu_callbacks, NULL );

  // register callback to handle lmb on cur_curve
  cagdRegisterCallback( CAGD_LBUTTONDOWN, lmb_down_cb, NULL );
  cagdRegisterCallback( CAGD_LBUTTONUP, lmb_up_cb, NULL );
  cagdRegisterCallback( CAGD_RBUTTONUP, rmb_up_cb, NULL );
}

/******************************************************************************
* SettingsDialogProc SETTINGS DIALOG
******************************************************************************/
LRESULT CALLBACK SettingsDialogProc( HWND hDialog, UINT message, WPARAM wParam, LPARAM lParam )
{
  switch( message )
  {
  case WM_INITDIALOG:
    SetDlgItemInt( hDialog, IDC_SAMPLES, get_default_num_steps(), FALSE );
    SetDlgItemInt( hDialog, IDC_DEF_DEGREE, get_def_degree(), FALSE );
    break;

  case WM_COMMAND:
    switch( LOWORD( wParam ) )
    {
    case IDOK:
      GetDlgItemText( hDialog, IDC_SAMPLES, buffer1, sizeof( buffer1 ) );
      GetDlgItemText( hDialog, IDC_DEF_DEGREE, buffer2, sizeof( buffer2 ) );
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
* CurveColorDialogProc Curve color_ DIALOG
******************************************************************************/
LRESULT CALLBACK CurveColorDialogProc( HWND hDialog, UINT message, WPARAM wParam, LPARAM lParam )
{
  const unsigned char *curve_color = get_curve_color();

  switch( message )
  {
  case WM_INITDIALOG:
    SetDlgItemInt( hDialog, IDC_RED, curve_color[ 0 ], FALSE );
    SetDlgItemInt( hDialog, IDC_GREEN, curve_color[ 1 ], FALSE );
    SetDlgItemInt( hDialog, IDC_BLUE, curve_color[ 2 ], FALSE );
    break;

  case WM_COMMAND:
    switch( LOWORD( wParam ) )
    {
    case IDOK:
      GetDlgItemText( hDialog, IDC_RED, buffer1, sizeof( buffer1 ) );
      GetDlgItemText( hDialog, IDC_GREEN, buffer2, sizeof( buffer2 ) );
      GetDlgItemText( hDialog, IDC_BLUE, buffer3, sizeof( buffer3 ) );
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

  case CAGD_CLEAN_ALL:
    handle_clean_all_menu();
    break;

  case CAGD_CURVE_COLOR:
    handle_curve_color_menu();
    break;
  case CAGD_REMOVE_CURVE:
    handle_rmb_remove_curve();
    break;
  }
}

/******************************************************************************
* handle_rmb_remove_curve
******************************************************************************/
void handle_rmb_remove_curve()
{
  if( active_rmb_curve != nullptr )
  {
    //active_rmb_curve->remove(); TODO if have time. not in PDF
    active_rmb_curve = nullptr;
    cagdRedraw();
  }
}

/******************************************************************************
* handle_curve_color_menu
******************************************************************************/
void handle_curve_color_menu()
{
  const unsigned char *curve_color = get_curve_color();

  if( DialogBox( cagdGetModule(),
                 MAKEINTRESOURCE( IDD_COLOR ),
                 cagdGetWindow(),
                 ( DLGPROC )CurveColorDialogProc ) )
  {
    GLubyte new_colors[ 3 ];
    if( sscanf( buffer1, "%hhu", &new_colors[ 0 ] ) == 1 &&
        sscanf( buffer2, "%hhu", &new_colors[ 1 ] ) == 1 &&
        sscanf( buffer3, "%hhu", &new_colors[ 2 ] ) == 1 )
    {
      set_curve_color( new_colors );
    }
    else
      print_error( "Invalid RGB values" );
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
    unsigned int new_samples = 0;
    unsigned int def_deg = 0;

    if( sscanf( buffer1, "%d", &new_samples ) == 1 &&
        sscanf( buffer2, "%du", &def_deg ) == 1 )
    {
      set_default_num_steps( new_samples );
      set_def_degree( def_deg );

      redraw_all_curves();
    }
    else
      print_error( "Invalid input" );
  }
}

/******************************************************************************
* handle_clean_all_menu
******************************************************************************/
void handle_clean_all_menu()
{
  clean_all_curves();
}

/******************************************************************************
* lmb_down_cb
******************************************************************************/
void lmb_down_cb( int x, int y, PVOID userData )
{
  UINT id;

  for( cagdPick( x, y ); id = cagdPickNext();)
  {
    if( cagdGetSegmentType( id ) == CAGD_SEGMENT_POINT )
      break;
  }

  if( id )
    set_active_pt_id( id );

  cagdRedraw();
}

/******************************************************************************
* lmb_up_cb
******************************************************************************/
void lmb_up_cb( int x, int y, PVOID userData )
{
  set_active_pt_id( K_NOT_USED );
  set_active_pt_is_first_move( true );
}

/******************************************************************************
* rmb_up_cb
******************************************************************************/
void rmb_up_cb( int x, int y, PVOID userData )
{
  UINT id;
  UINT pt_id = K_NOT_USED;
  UINT polyline_id = K_NOT_USED;
  Curve *crv = nullptr;
  std::tuple< Curve *, int > crv_pnt_ctrl;

  for( cagdPick( x, y ); id = cagdPickNext();)
  {
    if( cagdGetSegmentType( id ) == CAGD_SEGMENT_POINT )
      pt_id = id;
    if( cagdGetSegmentType( id ) == CAGD_SEGMENT_POLYLINE )
      polyline_id = id;

    if( pt_id != K_NOT_USED && polyline_id != K_NOT_USED )
      break;
  }

  if( pt_id != K_NOT_USED )
    crv_pnt_ctrl = get_pnt_crv_ctrl( id );

  if( polyline_id != K_NOT_USED )
    crv = get_seg_crv( polyline_id );

  if( std::get< 0 >( crv_pnt_ctrl ) != nullptr )
  {
    active_crv_ctrl_pt = crv_pnt_ctrl;
    show_rmb_on_ctrl_pt_menu( x, y );
  }
  else if( crv != nullptr )
  {
    active_rmb_curve = crv;
    show_rmb_on_curve_menu( x, y );
  }
  else
    show_no_selection_rmb_menu( x, y );

}

/******************************************************************************
* show_rmb_on_curve_menu
******************************************************************************/
void show_rmb_on_curve_menu( int x, int y )
{
  HWND hWnd = auxGetHWND();
  POINT pt;
  pt.x = x;
  pt.y = y;
  ClientToScreen( hWnd, &pt );

  HMENU rmb_menu = CreatePopupMenu();
  AppendMenu( rmb_menu, MF_STRING, CAGD_REMOVE_CURVE, TEXT( "Remove Curve" ) );

  TrackPopupMenu( rmb_menu, TPM_RIGHTBUTTON, pt.x, pt.y, 0, hWnd, NULL );
  DestroyMenu( rmb_menu );
}

/******************************************************************************
* show_rmb_on_ctrl_pt_menu
******************************************************************************/
void show_rmb_on_ctrl_pt_menu( int x, int y )
{
  HWND hWnd = auxGetHWND();
  POINT pt;
  pt.x = x;
  pt.y = y;
  ClientToScreen( hWnd, &pt );

  HMENU rmb_menu = CreatePopupMenu();
  AppendMenu( rmb_menu, MF_STRING, CAGD_CLEAN_ALL, TEXT( "Remove Point" ) );

  TrackPopupMenu( rmb_menu, TPM_RIGHTBUTTON, pt.x, pt.y, 0, hWnd, NULL );
  DestroyMenu( rmb_menu );
}

/******************************************************************************
* handle_no_selection_rmb
******************************************************************************/
void show_no_selection_rmb_menu( int x, int y )
{
  HWND hWnd = auxGetHWND();
  POINT pt;
  pt.x = x;
  pt.y = y;
  ClientToScreen( hWnd, &pt );

  HMENU rmb_menu = CreatePopupMenu();
  AppendMenu( rmb_menu, MF_STRING, CAGD_CLEAN_ALL, TEXT( "Item 1" ) );
  AppendMenu( rmb_menu, MF_SEPARATOR, 0, NULL );
  AppendMenu( rmb_menu, MF_STRING, CAGD_CLEAN_ALL, TEXT( "Clean All" ) );

  TrackPopupMenu( rmb_menu, TPM_RIGHTBUTTON, pt.x, pt.y, 0, hWnd, NULL );
  DestroyMenu( rmb_menu );
}