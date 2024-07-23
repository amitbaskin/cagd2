#pragma once

#include "crv_utils.h"
#include "Bezier.h"
#include "BSpline.h"

void init_menus();

void menu_callbacks( int id, int unUsed, PVOID userData );

void lmb_down_cb( int x, int y, PVOID userData );
void lmb_up_cb( int x, int y, PVOID userData );

void rmb_up_cb( int x, int y, PVOID userData );

void handle_settings_menu();
void handle_clean_all_menu();
void handle_curve_color_menu();
void handle_rmb_remove_curve();
void handle_rmb_remove_ctrl_pt();
void handle_rmb_insert_ctrl_pt();

void show_rmb_on_ctrl_polyline_menu( int x, int y );
void show_rmb_on_curve_menu( int x, int y );
void show_rmb_on_ctrl_pt_menu( int x, int y );
void show_no_selection_rmb_menu( int x, int y );

void clean_active_rmb_data();