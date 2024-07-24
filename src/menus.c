#include <cagd.h>
#include <stdio.h>
#include "menus.h"
#include "resource.h"
#include "color.h"
#include "options.h"
#include <vectors.h>
#include "crv_utils.h"

char buffer1[ BUFSIZ ];
char buffer2[ BUFSIZ ];
char buffer3[ BUFSIZ ];
UINT myText2;

ConnType conn = ConnType::NONE;
Curve *active_rmb_curve = nullptr;
int active_pnt_id = K_NOT_USED;
int active_rmb_ctrl_polyline = K_NOT_USED;
int cur_rmb_screen_pick[ 2 ] = { K_NOT_USED, K_NOT_USED };
int hilited_pt_id = K_NOT_USED;

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
  cagdRegisterCallback( CAGD_MOUSEMOVE, mouse_move_cb, NULL );
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

  case CAGD_REMOVE_CTRL_PT:
    handle_rmb_remove_ctrl_pt();
    break;

  case CAGD_INSERT_CTRL_PT:
    handle_rmb_insert_ctrl_pt();
    break;

  case CAGD_CONNECT_C0:
    handle_rmb_connect_c0();
    break;
  case CAGD_CONNECT_C1:
    handle_rmb_connect_c1();
    break;
  case CAGD_CONNECT_G1:
    handle_rmb_connect_g1();
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
* handle_rmb_insert_ctrl_pt
******************************************************************************/
void handle_rmb_insert_ctrl_pt()
{
  Curve *p_curve = active_rmb_curve;

  std::tuple< int, int > end_pnts =
    get_ctrl_seg_pnts( active_rmb_ctrl_polyline );

  auto ctrl_idx = p_curve->get_pnt_id_idx( std::get< 1 >( end_pnts ) );

  if( p_curve != nullptr &&
      ctrl_idx != K_NOT_USED &&
      active_rmb_ctrl_polyline != K_NOT_USED )
  {
    Bezier *bz_crv = dynamic_cast< Bezier * >( p_curve );
    BSpline *bs_crv = dynamic_cast< BSpline * >( p_curve );

    CAGD_POINT pick_pnt = screen_to_world_coord( cur_rmb_screen_pick[ 0 ],
                                                 cur_rmb_screen_pick[ 1 ] );

    if( bz_crv != nullptr )
    {
      bz_crv->add_ctrl_pnt( pick_pnt, ctrl_idx );
      bz_crv->show_ctrl_poly();
      bz_crv->show_crv();
    }
    else if( bs_crv != nullptr )
    {
      bs_crv->add_ctrl_pnt( pick_pnt, ctrl_idx );
      bs_crv->show_ctrl_poly();
      bs_crv->show_crv();
    }
    else
      print_error( "Error removing control point" );

    cagdRedraw();
  }

  clean_active_rmb_data();
}

/******************************************************************************
* handle_rmb_connect_c0
******************************************************************************/
void handle_rmb_connect_c0()
{
  conn = ConnType::C0;
  active_rmb_curve->change_color( 255, 165, 0 );
}

/******************************************************************************
* handle_rmb_connect_c1
******************************************************************************/
void handle_rmb_connect_c1()
{
  conn = ConnType::C1;
  active_rmb_curve->change_color( 255, 165, 0 );
}

/******************************************************************************
* handle_rmb_connect_g1
******************************************************************************/
void handle_rmb_connect_g1()
{
  conn = ConnType::G1;
  active_rmb_curve->change_color( 255, 165, 0 );
}

/******************************************************************************
* handle_rmb_remove_ctrl_pt
******************************************************************************/
void handle_rmb_remove_ctrl_pt()
{
  Curve *p_curve = active_rmb_curve;
  auto ctrl_idx = p_curve->get_pnt_id_idx( active_pnt_id );

  if( p_curve != nullptr && ctrl_idx != K_NOT_USED )
  {
    Bezier *bz_crv = dynamic_cast< Bezier * >( p_curve );
    BSpline *bs_crv = dynamic_cast< BSpline * >( p_curve );

    if( bz_crv != nullptr )
    {
      bz_crv->rmv_ctrl_pnt( ctrl_idx );
      bz_crv->show_ctrl_poly();
      bz_crv->show_crv();
    }
    else if( bs_crv != nullptr )
    {
      bs_crv->rmv_ctrl_pnt( ctrl_idx );
      bs_crv->show_ctrl_poly();
      bs_crv->show_crv();
    }
    else
      print_error( "Error removing control point" );

    cagdRedraw();
  }
  else
    print_error( "Error removing control point" );

  clean_active_rmb_data();
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
}

/******************************************************************************
* lmb_up_cb
******************************************************************************/
void lmb_up_cb( int x, int y, PVOID userData )
{
  set_active_pt_id( K_NOT_USED );

  if( conn != ConnType::NONE )
  {
    int sec_seg_id = get_crv_by_pick( x, y );

    if( sec_seg_id != 0 )
    {
      active_rmb_curve->change_color( 255, 0, 0 );
      connect_crv_callback( active_rmb_curve->seg_ids_[ 0 ], sec_seg_id, conn  );
      conn = ConnType::NONE;
    }
  }
}

/******************************************************************************
* get_crv_by_pick
******************************************************************************/
UINT get_crv_by_pick( int x, int y )
{
  UINT id = 0;
  UINT seg_id = 0;

  for( cagdPick( x, y ); id = cagdPickNext(); )
  {
    if( cagdGetSegmentType( id ) == CAGD_SEGMENT_POLYLINE )
    {
      seg_id = id;
      break;
    }
  }

  return seg_id;
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
  std::tuple< int, int > neibor_pts = std::make_tuple( K_NOT_USED, K_NOT_USED );

  for( cagdPick( x, y ); id = cagdPickNext();)
  {
    if( cagdGetSegmentType( id ) == CAGD_SEGMENT_POINT )
      pt_id = id;
    if( cagdGetSegmentType( id ) == CAGD_SEGMENT_POLYLINE )
      polyline_id = id;

    if( pt_id != K_NOT_USED && polyline_id != K_NOT_USED )
      break;
  }

  if( polyline_id != K_NOT_USED )
  {
    crv = get_seg_crv( polyline_id );
    neibor_pts = get_ctrl_seg_pnts( polyline_id );
  }

  if( pt_id != K_NOT_USED ) // RMB on ctrl pt
  {
    active_pnt_id = pt_id;
    active_rmb_curve = get_pnt_crv( pt_id );
    show_rmb_on_ctrl_pt_menu( x, y );
  }
  else if( crv != nullptr ) // RMB on curve
  {
    active_rmb_curve = crv;
    show_rmb_on_curve_menu( x, y );
  }
  else if( std::get< 0 >( neibor_pts ) != K_NOT_USED && // RMB on control points polyline
           std::get< 1 >( neibor_pts ) != K_NOT_USED )
  {
    active_rmb_curve = get_pnt_crv( std::get< 1 >( neibor_pts ) );
    active_rmb_ctrl_polyline = polyline_id;
    cur_rmb_screen_pick[0] = x;
    cur_rmb_screen_pick[1] = y;
    show_rmb_on_ctrl_polyline_menu( x, y );
  }
  else
    show_no_selection_rmb_menu( x, y );

}

/******************************************************************************
* show_rmb_on_ctrl_polyline_menu
******************************************************************************/
void show_rmb_on_ctrl_polyline_menu( int x, int y )
{
  HWND hWnd = auxGetHWND();
  POINT pt;
  pt.x = x;
  pt.y = y;
  ClientToScreen( hWnd, &pt );

  HMENU rmb_menu = CreatePopupMenu();
  AppendMenu( rmb_menu, MF_STRING, CAGD_INSERT_CTRL_PT, TEXT( "Insert Control Point" ) );

  TrackPopupMenu( rmb_menu, TPM_RIGHTBUTTON, pt.x, pt.y, 0, hWnd, NULL );
  DestroyMenu( rmb_menu );
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
  AppendMenu( rmb_menu, MF_STRING, CAGD_CONNECT_C0, TEXT( "Connect C0" ) );
  AppendMenu( rmb_menu, MF_STRING, CAGD_CONNECT_C1, TEXT( "Connect C1" ) );
  AppendMenu( rmb_menu, MF_STRING, CAGD_CONNECT_G1, TEXT( "Connect G1" ) );

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
  AppendMenu( rmb_menu, MF_STRING, CAGD_REMOVE_CTRL_PT, TEXT( "Remove Control Point" ) );

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
  AppendMenu( rmb_menu, MF_STRING, CAGD_SETTINGS, TEXT( "Settings" ) );
  AppendMenu( rmb_menu, MF_SEPARATOR, 0, NULL );
  AppendMenu( rmb_menu, MF_STRING, CAGD_CLEAN_ALL, TEXT( "Clean All" ) );

  TrackPopupMenu( rmb_menu, TPM_RIGHTBUTTON, pt.x, pt.y, 0, hWnd, NULL );
  DestroyMenu( rmb_menu );
}

/******************************************************************************
* clean_active_rmb_data
******************************************************************************/
void clean_active_rmb_data()
{
  active_rmb_curve = nullptr;
  active_rmb_ctrl_polyline = K_NOT_USED;
  cur_rmb_screen_pick[0] = K_NOT_USED;
}

/******************************************************************************
* mouse_move_cb
******************************************************************************/
void mouse_move_cb( int x, int y, PVOID userData )
{
  int id = K_NOT_USED;

  for( cagdPick( x, y ); id = cagdPickNext();)
  {
    if( cagdGetSegmentType( id ) == CAGD_SEGMENT_POINT )
      break;
  }

  if( id > 0 )
  {
    cagdSetSegmentColor( id, 255, 255, 0 );
    cagdRedraw();
    hilited_pt_id = id;
  }
  else
  {
    if( hilited_pt_id != K_NOT_USED )
    {
      cagdSetSegmentColor( hilited_pt_id, 0, 255, 0 );
      cagdRedraw();
    }

    hilited_pt_id = K_NOT_USED;
  }
}