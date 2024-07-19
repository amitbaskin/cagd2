#pragma once



void init_menus();

void menu_callbacks( int id, int unUsed, PVOID userData );

void left_mouse_click_cb( int x, int y, PVOID userData );

void handle_settings_menu();