#pragma once

#include <string>
#include <map>

#define K_NOT_USED -1

class Curve;

typedef struct
{
  int active_pt_id;
  int last_pos[2]; // in screen coordinates
  bool first_move;

} active_ctrl_pt_data; // for lmb drag of ctrl pt

void load_curves( int dummy1, int dummy2, void *p_data );

void clean_all_curves();

void print_error( const std::string &message );

void redraw_all_curves();

void map_seg_to_crv( int seg_id, Curve *p_curve );
void map_pnt_to_crv_ctrl( int pnt_id, Curve *p_curve, int ctrl_idx );
void map_ctrl_seg_to_pnts( int seg_id, int pnt1, int pnt2 );

void erase_pnt_to_crv_ctrl( int pnt_id );
void erase_ctrl_seg_to_pnts( int pnt_id );

std::tuple< Curve *, int > get_pnt_crv_ctrl( int pnt_id );
std::tuple< int, int > get_ctrl_seg_pnts( int seg_id );
Curve *get_seg_crv( int seg_id );

int get_active_pt_id();
void set_active_pt_id( int id );

int *get_active_pt_last_pos();
void set_active_pt_last_pos( int pos[2] );

bool get_active_pt_is_first_move();
void set_active_pt_is_first_move( bool value );

void update_ctrl_pnt( int pnt_id, double new_x, double new_y );
void calculate_ctrl_pnt_updated_pos( int pnt_id, int dx, int dy,
                                     double &new_x, double &new_y );
