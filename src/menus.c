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

char knots_buf[ BUFSIZ ];

UINT myText2;

HMENU g_op_menu = nullptr;

ConnType conn = ConnType::NONE;
Curve *active_rmb_curve = nullptr;
int active_polyline_id;
int active_pnt_id = K_NOT_USED;
int active_rmb_ctrl_polyline = K_NOT_USED;
int cur_rmb_screen_pick[ 2 ] = { K_NOT_USED, K_NOT_USED };
int hilited_pt_id = K_NOT_USED;
bool add_bezier_is_active = false;
Bezier *add_bezier_active_crv = nullptr;
bool add_bspline_is_active = false;
BSpline *add_bspline_active_crv = nullptr;
Curve *active_lmb_curve = nullptr;
CAGD_POINT lmb_pnt = { 0 };

extern void myMessage( PSTR title, PSTR message, UINT crv_type );

/******************************************************************************
* reset_active
******************************************************************************/
void reset_active()
{
  conn = ConnType::NONE;
  active_rmb_curve = nullptr;
  active_pnt_id = K_NOT_USED;
  active_rmb_ctrl_polyline = K_NOT_USED;
  cur_rmb_screen_pick[ 0 ] = K_NOT_USED;
  cur_rmb_screen_pick[ 1 ] = K_NOT_USED;
  hilited_pt_id = K_NOT_USED;
  add_bezier_is_active = false;
  add_bezier_active_crv = nullptr;
  add_bspline_is_active = false;
  add_bspline_active_crv = nullptr;
  active_lmb_curve = nullptr;
  lmb_pnt = { 0 };
}

/******************************************************************************
* init_menus
******************************************************************************/
void init_menus()
{
  HMENU op_menu = CreatePopupMenu(); // options
  HMENU curve_menu = CreatePopupMenu(); // Curve

  g_op_menu = op_menu;

  // Curve
  AppendMenu( curve_menu, MF_STRING, CAGD_CURVE_COLOR, "Default Color" );

  // Options
  AppendMenu( op_menu, MF_STRING, CAGD_SETTINGS, "Settings" );
  AppendMenu( op_menu, MF_STRING, CAGD_HIDE_CTRL_POLYS, "Hide Control Polylines" );
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
  cagdRegisterCallback( CAGD_MBUTTONUP, mmb_up_cb, NULL );
}

/******************************************************************************
* KnotsDialogProc KNOTS DIALOG
******************************************************************************/
LRESULT CALLBACK KnotsDialogProc( HWND hDialog, UINT message, WPARAM wParam, LPARAM lParam )
{
  BSpline *p_bspline = ( BSpline * )active_rmb_curve;

  switch( message )
  {
  case WM_INITDIALOG:
  {
    const char *str = p_bspline->getKnotsDescription();
    SetDlgItemTextA( hDialog, IDC_KNOTS, str );
    free( ( void * )str );
    break;
  }

  case WM_COMMAND:
    switch( LOWORD( wParam ) )
    {
    case IDOK:
      GetDlgItemText( hDialog, IDC_KNOTS, knots_buf, sizeof( knots_buf ) );
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
* AddKnotDialogProc INSERT KNOT DIALOG
******************************************************************************/
LRESULT CALLBACK AddKnotDialogProc( HWND hDialog, UINT message, WPARAM wParam, LPARAM lParam )
{
  switch( message )
  {
  case WM_COMMAND:
    switch( LOWORD( wParam ) )
    {
    case IDOK:
      GetDlgItemText( hDialog, IDC_INSERT_KNOT, buffer1, sizeof( buffer1 ) );
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
* WeightDialogProc CHANGE WEIGHT DIALOG
******************************************************************************/
LRESULT CALLBACK WeightDialogProc( HWND hDialog, UINT message, WPARAM wParam, LPARAM lParam )
{
  Curve *p_curve = active_rmb_curve;
  auto ctrl_idx = p_curve->get_pnt_id_idx( active_pnt_id );

  switch( message )
  {
  case WM_INITDIALOG:
    char buffer[50];
    // Convert the double to a string
    sprintf_s( buffer, sizeof( buffer ), "%lf", p_curve->ctrl_pnts_[ctrl_idx].z );
    // Set the text of the dialog item
    SetDlgItemText( hDialog, IDC_CHANGE_WEIGHT, buffer );
    break;

  case WM_COMMAND:
    switch( LOWORD( wParam ) )
    {
    case IDOK:
      GetDlgItemText( hDialog, IDC_CHANGE_WEIGHT, buffer1, sizeof( buffer1 ) );
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

  case CAGD_ADD_BEZIER_CURVE:
    add_bezier_is_active = true;
    add_bspline_is_active = false;
    handle_add_curve_menu();
    break;

  case CAGD_ADD_BSPLINE_CURVE:
    add_bspline_is_active = true;
    add_bezier_is_active = false;
    handle_add_curve_menu();
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

  case CAGD_PREPEND_CTRL_PT:
    handle_rmb_prepend_ctrl_pt();
    break;

  case CAGD_APPEND_CTRL_PT:
    handle_rmb_append_ctrl_pt();
    break;

  case CAGD_CHANGE_WEIGHT:
    handle_change_weight_menu();
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
  case CAGD_MOD_KNOTS:
    handle_rmb_mod_knots();
    break;
  case CAGD_ADD_KNOT:
    handle_rmb_add_knot_menu();
    break;
  case CAGD_OPEN_KNOTS:
    handle_rmb_open_knots();
    break;
  case CAGD_UNI_KNOTS:
    handle_rmb_uni_knots();
    break;
  case CAGD_RMV_OPEN_KNOTS:
    handle_rmb_rmv_open_knots();
    break;
  case CAGD_RMV_UNI_KNOTS:
    handle_rmb_rmv_uni_knots();
    break;
  case CAGD_HIDE_CTRL_POLYS:
    handle_hide_ctrl_polys_menu();
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
    free_crv( active_rmb_curve );
    cagdRedraw();
  }

  clean_active_rmb_data();
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
* handle_rmb_prepend_ctrl_pt
******************************************************************************/
void handle_rmb_prepend_ctrl_pt()
{
  Curve *p_curve = active_rmb_curve;

  if( p_curve != nullptr )
  {
    Bezier *bz_crv = dynamic_cast< Bezier * >( p_curve );
    BSpline *bs_crv = dynamic_cast< BSpline * >( p_curve );

    CAGD_POINT new_pt_loc;
    CAGD_POINT location_vec;
    CAGD_POINT p0 = p_curve->ctrl_pnts_[0];
    CAGD_POINT p1 = p_curve->ctrl_pnts_[1];
    diff_vecs( &p0, &p1, &location_vec );

    add_vecs( &p0, &location_vec, &new_pt_loc );

    if( bz_crv != nullptr )
    {
      bz_crv->add_ctrl_pnt( new_pt_loc, 0 );
      bz_crv->show_ctrl_poly();
      bz_crv->show_crv();
    }
    else if( bs_crv != nullptr )
    {
      bs_crv->add_ctrl_pnt( new_pt_loc, 0 ); // not working good
      bs_crv->show_ctrl_poly();
      bs_crv->show_crv();
    }

    cagdRedraw();
  }

  clean_active_rmb_data();
}

/******************************************************************************
* handle_rmb_append_ctrl_pt
******************************************************************************/
void handle_rmb_append_ctrl_pt()
{
  Curve *p_curve = active_rmb_curve;

  if( p_curve != nullptr )
  {
    Bezier *bz_crv = dynamic_cast< Bezier * >( p_curve );
    BSpline *bs_crv = dynamic_cast< BSpline * >( p_curve );

    int num_ctrl_pts = p_curve->ctrl_pnts_.size();

    if( num_ctrl_pts > 1 )
    {
      CAGD_POINT new_pt_loc;
      CAGD_POINT location_vec;
      CAGD_POINT p0 = p_curve->ctrl_pnts_[num_ctrl_pts - 1];
      CAGD_POINT p1 = p_curve->ctrl_pnts_[num_ctrl_pts - 2];
      diff_vecs( &p0, &p1, &location_vec );

      add_vecs( &p0, &location_vec, &new_pt_loc );

      if( bz_crv != nullptr )
      {
        bz_crv->add_ctrl_pnt( new_pt_loc, num_ctrl_pts );
        bz_crv->show_ctrl_poly();
        bz_crv->show_crv();
      }
      else if( bs_crv != nullptr )
      {
        bs_crv->add_ctrl_pnt( new_pt_loc, num_ctrl_pts ); // not working good
        bs_crv->show_ctrl_poly();
        bs_crv->show_crv();
      }

      cagdRedraw();
    }
    else
      print_error( "Can't append control point with less than 2 existing" );
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

    hilited_pt_id = K_NOT_USED;

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
* handle_hide_ctrl_polys_menu
******************************************************************************/
void handle_hide_ctrl_polys_menu()
{
  toggle_check_menu( g_op_menu, CAGD_HIDE_CTRL_POLYS );
  set_hide_ctrl_polys( !get_hide_ctrl_polys() );

  if( get_hide_ctrl_polys() )
  {
    hide_all_ctrl_polys();
  }
  else
  {
    show_all_ctrl_polys();
  }
}

/******************************************************************************
* handle_rmb_rmv_uni_knots
******************************************************************************/
void handle_rmb_rmv_uni_knots()
{
  BSpline *p_bspline = ( BSpline * )active_rmb_curve;
  p_bspline->is_uni_ = false;
}

/******************************************************************************
* handle_rmb_rmv_open_knots
******************************************************************************/
void handle_rmb_rmv_open_knots()
{
  BSpline *p_bspline = ( BSpline * )active_rmb_curve;
  p_bspline->is_open_ = false;
}

/******************************************************************************
* handle_rmb_open_knots
******************************************************************************/
void handle_rmb_open_knots()
{
  BSpline *p_bspline = ( BSpline * )active_rmb_curve;
  p_bspline->makeOpenKnotVector();
}

/******************************************************************************
* handle_rmb_uni_knots
******************************************************************************/
void handle_rmb_uni_knots()
{
  BSpline *p_bspline = ( BSpline * )active_rmb_curve;
  p_bspline->makeUniformKnotVector();
}

/******************************************************************************
* handle_rmb_mod_knots
******************************************************************************/
void handle_rmb_mod_knots()
{
  BSpline *p_bspline = ( BSpline * )active_rmb_curve;

  if( DialogBox( cagdGetModule(),
                 MAKEINTRESOURCE( IDD_KNOTS ),
                 cagdGetWindow(),
                 ( DLGPROC )KnotsDialogProc ) )
  {
    p_bspline->parseKnotsDescription( knots_buf );

    if( p_bspline->knots_.size() != p_bspline->ctrl_pnts_.size() + p_bspline->order_ )
    {
      print_error( "Please make sure the number of knots is\n"
                   "the number of control points plus the order." );
    }
    else
    {
      p_bspline->show_crv();
      cagdRedraw();
    }
  }
}

/******************************************************************************
* handle_rmb_add_knot_menu
******************************************************************************/
void handle_rmb_add_knot_menu()
{
  if( DialogBox( cagdGetModule(),
      MAKEINTRESOURCE( IDD_INSERT_KNOT ),
      cagdGetWindow(),
      ( DLGPROC )AddKnotDialogProc ) )
  {
    BSpline *p_bspline = ( BSpline * )active_rmb_curve;
    double knot_value;

    if( sscanf( buffer1, "%lf", &knot_value ) == 1 )
    {
      p_bspline->insertKnot( knot_value );
    }
    else
      print_error( "Invalid input" );
  }
}

/******************************************************************************
* handle_change_weight_menu
******************************************************************************/
void handle_change_weight_menu()
{
  if( DialogBox( cagdGetModule(),
      MAKEINTRESOURCE( IDD_CHANGE_WEIGHT ),
      cagdGetWindow(),
      ( DLGPROC )WeightDialogProc ) )
  {
    double new_weight = 1;

    if( sscanf( buffer1, "%lf", &new_weight ) == 1 && new_weight > 0 )
    {
      Curve *p_curve = active_rmb_curve;
      auto ctrl_idx = p_curve->get_pnt_id_idx( active_pnt_id );

      if( ctrl_idx > 0 )
        update_weight_callback( active_pnt_id, ctrl_idx, new_weight );
    }
    else
      print_error( "Invalid input" );
  }

  clean_active_rmb_data();
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
  reset_active();
}

/******************************************************************************
* handle_add_curve_menu
******************************************************************************/
void handle_add_curve_menu()
{
  int sc_x = cur_rmb_screen_pick[0];
  int sc_y = cur_rmb_screen_pick[1];
  CAGD_POINT pt0 = screen_to_world_coord( sc_x, sc_y );

  if( add_bezier_is_active )
  {
    Bezier *p_bezier = new Bezier();
    p_bezier->order_ = 0;
    p_bezier->add_ctrl_pnt( pt0, 0 );
    register_crv( p_bezier );
    add_bezier_active_crv = p_bezier;
    p_bezier->change_color( 204, 102, 0 );
  }
  else if( add_bspline_is_active )
  {
    BSpline *p_bspline = new BSpline();
    p_bspline->order_ = get_def_degree();
    p_bspline->add_ctrl_pnt( pt0, 0 );
    register_crv( p_bspline );
    add_bspline_active_crv = p_bspline;
    p_bspline->change_color( 204, 102, 0 );
  }

  cagdRedraw();

  if( !IS_DEBUG )
    show_add_curve_help_text();
}

/******************************************************************************
* lmb_down_cb
******************************************************************************/
void lmb_down_cb( int x, int y, PVOID userData )
{
  if( add_bezier_is_active || add_bspline_is_active )
  {
    return;
  }

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

  if( add_bezier_is_active && add_bezier_active_crv != nullptr )
  {
    CAGD_POINT wc_p = screen_to_world_coord( x, y );
    int num_ctrl_pts = add_bezier_active_crv->ctrl_pnts_.size();
    add_bezier_active_crv->add_ctrl_pnt( wc_p, num_ctrl_pts );
    add_bezier_active_crv->show_ctrl_poly();
    add_bezier_active_crv->show_crv();
    cagdRedraw();
  }
  else if( add_bspline_is_active && add_bspline_active_crv != nullptr )
  {
    CAGD_POINT wc_p = screen_to_world_coord( x, y );
    int num_ctrl_pts = add_bspline_active_crv->ctrl_pnts_.size();
    add_bspline_active_crv->add_ctrl_pnt( wc_p, num_ctrl_pts );
    add_bspline_active_crv->makeUniformKnotVector( true, 0.0, 1.0 );
    add_bspline_active_crv->makeOpenKnotVector();
    add_bspline_active_crv->show_ctrl_poly();
    add_bspline_active_crv->show_crv();
    cagdRedraw();
  }
  else if( conn != ConnType::NONE )
  {
    int sec_seg_id = get_crv_by_pick( x, y );

    if( sec_seg_id != 0 )
    {
      if( get_crv_type( active_rmb_curve ) != CurveType::BSPLINE ||
          active_rmb_curve->seg_ids_[ 0 ] != sec_seg_id )
      {
        bool res = connect_crv_callback( active_rmb_curve->seg_ids_[ 0 ], sec_seg_id, conn );

        if( res )
        {
          active_rmb_curve->change_color( 255, 0, 0 );
          conn = ConnType::NONE;
          active_rmb_curve = NULL;
        }
      }
      else
        print_error( "Please avoid trying to connect a bspline to itself." );
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
  if( add_bezier_is_active || add_bspline_is_active )
  {
    return;
  }

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

  cur_rmb_screen_pick[0] = x;
  cur_rmb_screen_pick[1] = y;

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
    active_polyline_id = polyline_id;
    active_rmb_curve = crv;
    show_rmb_on_curve_menu( x, y );
  }
  else if( std::get< 0 >( neibor_pts ) != K_NOT_USED && // RMB on control points polyline
           std::get< 1 >( neibor_pts ) != K_NOT_USED )
  {
    active_rmb_curve = get_pnt_crv( std::get< 1 >( neibor_pts ) );
    active_rmb_ctrl_polyline = polyline_id;
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
  AppendMenu( rmb_menu, MF_STRING, CAGD_PREPEND_CTRL_PT, TEXT( "Prepend Control Point" ) );
  AppendMenu( rmb_menu, MF_STRING, CAGD_APPEND_CTRL_PT, TEXT( "Append Control Point" ) );

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

  CurveType crv_type = get_crv_type( active_rmb_curve );

  HMENU rmb_menu = CreatePopupMenu();
  AppendMenu( rmb_menu, MF_STRING, CAGD_REMOVE_CURVE, TEXT( "Remove Curve" ) );
  AppendMenu( rmb_menu, MF_STRING, CAGD_SAVE_CURVE, TEXT( "Save Curve" ) );
  AppendMenu( rmb_menu, MF_SEPARATOR, 0, NULL );
  AppendMenu( rmb_menu, MF_STRING, CAGD_CONNECT_C0, TEXT( "Connect C0" ) );
  AppendMenu( rmb_menu, MF_STRING, CAGD_CONNECT_C1, TEXT( "Connect C1" ) );
  AppendMenu( rmb_menu, MF_STRING, CAGD_CONNECT_G1, TEXT( "Connect G1" ) );

  if( crv_type == CurveType::BSPLINE )
  {
    AppendMenu( rmb_menu, MF_SEPARATOR, 0, NULL );
    AppendMenu( rmb_menu, MF_STRING, CAGD_MOD_KNOTS, TEXT( "Modify Knots" ) );
    AppendMenu( rmb_menu, MF_STRING, CAGD_ADD_KNOT, TEXT( "Insert Knot" ) );

    BSpline *bspline = ( BSpline * )active_rmb_curve;

    if( bspline->is_open_ )
      AppendMenu( rmb_menu, MF_STRING, CAGD_RMV_OPEN_KNOTS, TEXT( "Close Knots" ) );
    else
      AppendMenu( rmb_menu, MF_STRING, CAGD_OPEN_KNOTS, TEXT( "Open Knots" ) );

    if( bspline->is_uni_ )
      AppendMenu( rmb_menu, MF_STRING, CAGD_RMV_UNI_KNOTS, TEXT( "Disunify Knots" ) );
    else
      AppendMenu( rmb_menu, MF_STRING, CAGD_UNI_KNOTS, TEXT( "Unify Knots" ) );
  }

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
  AppendMenu( rmb_menu, MF_SEPARATOR, 0, NULL );
  AppendMenu( rmb_menu, MF_STRING, CAGD_CHANGE_WEIGHT, TEXT( "Change Weight" ) );

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
  AppendMenu( rmb_menu, MF_STRING, CAGD_ADD_BEZIER_CURVE, TEXT( "Add Bezier Curve" ) );

  // not working currently
  AppendMenu( rmb_menu, MF_STRING, CAGD_ADD_BSPLINE_CURVE, TEXT( "Add BSpline Curve" ) );

  AppendMenu( rmb_menu, MF_SEPARATOR, 0, NULL );
  AppendMenu( rmb_menu, MF_STRING, CAGD_SETTINGS, TEXT( "Settings" ) );

  AppendMenu( rmb_menu, MF_STRING, CAGD_HIDE_CTRL_POLYS, "Hide Control Polylines" );
  CheckMenuItem( rmb_menu, CAGD_HIDE_CTRL_POLYS, get_hide_ctrl_polys() ? MF_CHECKED : MF_UNCHECKED );

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
  cur_rmb_screen_pick[1] = K_NOT_USED;
  hilited_pt_id = K_NOT_USED;
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

/******************************************************************************
* mmb_up_cb (Middle mouse button up)
******************************************************************************/
void mmb_up_cb( int x, int y, PVOID userData )
{
  if( add_bezier_is_active && add_bezier_active_crv != nullptr )
  {
    BYTE def_color[3];
    get_curve_color( &def_color[0], &def_color[1], &def_color[2] );
    add_bezier_active_crv->change_color( def_color[0], def_color[1], def_color[2] );
  }

  if( add_bspline_is_active && add_bspline_active_crv != nullptr )
  {
    BYTE def_color[3];
    get_curve_color( &def_color[0], &def_color[1], &def_color[2] );
    add_bspline_active_crv->change_color( def_color[0], def_color[1], def_color[2] );
  }

  add_bezier_is_active = false;
  add_bspline_is_active = false;

  add_bezier_active_crv = nullptr;
  add_bspline_active_crv = nullptr;
}

/******************************************************************************
* toggle_check_menu
******************************************************************************/
void toggle_check_menu( HMENU main_menu, UINT sub_menu_id )
{
  if( GetMenuState( main_menu, sub_menu_id, MF_BYCOMMAND ) & MF_CHECKED )
    CheckMenuItem( main_menu, sub_menu_id, MF_UNCHECKED );
  else
    CheckMenuItem( main_menu, sub_menu_id, MF_CHECKED );
}