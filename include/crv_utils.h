#pragma once

#include <string>
#include <map>

#define K_NOT_USED -1

class Curve;

void load_curves( int dummy1, int dummy2, void *p_data );

void clean_all_curves();

void print_error( const std::string &message );

void redraw_all_curves();

void map_seg_to_crv( int seg_id, Curve *p_curve );
void map_pnt_to_crv_ctrl( int pnt_id, Curve *p_curve, int ctrl_idx );

int get_active_pt_id();
void set_active_pt_id( int id );

std::tuple< Curve *, int > get_pnt_crv_ctrl( int pnt_id );
Curve *get_seg_crv( int seg_id );

void update_ctrl_pnt( int pnt_id, double new_x, double new_y );
void calculate_ctrl_pnt_updated_pos( int pnt_id, int dx, int dy, 
                                     double &new_x, double &new_y );
