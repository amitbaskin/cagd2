#pragma once

#include "crv_utils.h"
#include "Bezier.h"
#include "BSpline.h"

void init_menus();

UINT get_crv_by_pick( int x, int y );

void menu_callbacks( int id, int unUsed, PVOID userData );
void toggle_check_menu( HMENU main_menu, UINT sub_menu_id );

void lmb_down_cb( int x, int y, PVOID userData );
void lmb_up_cb( int x, int y, PVOID userData );

void rmb_up_cb( int x, int y, PVOID userData );

void mouse_move_cb( int x, int y, PVOID userData );

void mmb_up_cb( int x, int y, PVOID userData );

void handle_rmb_rmv_open_knots();
void handle_rmb_rmv_uni_knots();
void handle_rmb_open_knots();
void handle_rmb_uni_knots();
void handle_rmb_mod_knots();
void handle_rmb_add_knot_menu();
void handle_settings_menu();
void handle_clean_all_menu();
void handle_hide_ctrl_polys_menu();
void handle_add_curve_menu();
void handle_curve_color_menu();
void handle_rmb_remove_curve();
void handle_rmb_remove_ctrl_pt();
void handle_rmb_insert_ctrl_pt();
void handle_rmb_prepend_ctrl_pt();
void handle_rmb_append_ctrl_pt();
void handle_change_weight_menu();
void handle_rmb_connect_c0();
void handle_rmb_connect_c1();
void handle_rmb_connect_g1();

void show_rmb_on_ctrl_polyline_menu( int x, int y );
void show_rmb_on_curve_menu( int x, int y );
void show_rmb_on_ctrl_pt_menu( int x, int y );
void show_no_selection_rmb_menu( int x, int y );

void clean_active_rmb_data();
