#pragma once



void init_menus();

void menu_callbacks( int id, int unUsed, PVOID userData );

void lmb_down_cb( int x, int y, PVOID userData );
void lmb_up_cb( int x, int y, PVOID userData );

void handle_settings_menu();
void handle_clean_all_menu();
void handle_curve_color_menu();