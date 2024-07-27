#pragma once

#include <string>
#include <map>
#include <vector>

#define K_NOT_USED -1

class Curve;
class Bezier;
class BSpline;

typedef struct
{
  int active_pt_id;
  int last_pos[2]; // in screen coordinates
  bool first_move;

} active_ctrl_pt_data; // for lmb drag of ctrl pt


enum class CurveType
{
  NONE = 0,
  BEZIER = 1,
  BSPLINE = 2
};

enum class ConnType
{
  NONE = 0,
  C0 = 1,
  C1 = 2,
  G1 = 3
};

void save_all_curves( int seg_crv, int dummy2, void *p_data );
void save_curve( int seg_crv, int is_close, void *p_data );
void load_curves( int dummy1, int dummy2, void *p_data );

void register_crv( Curve *p_crv );
void free_crv( Curve *p_crv );
void remove_crv_data( Curve *p_crv );
void clean_all_curves();

void print_error( const std::string &message );
void show_add_curve_help_text();

void redraw_all_curves();
void hide_all_ctrl_polys();
void show_all_ctrl_polys();

void map_seg_to_crv( int seg_id, Curve *p_curve );
void map_pnt_to_crv( int pnt_id, Curve *p_curve );
void map_ctrl_seg_to_pnts( int seg_id, int pnt1, int pnt2 );

void erase_seg_to_crv( int seg_id );
void erase_pnt_to_crv( int pnt_id );
void erase_ctrl_seg_to_pnts( int pnt_id );
void erase_crv_from_cur_crvs( Curve *p_curve );

Curve *get_pnt_crv( int pnt_id );
std::tuple< int, int > get_ctrl_seg_pnts( int seg_id );
Curve *get_seg_crv( int seg_id );

int get_active_pt_id();
void set_active_pt_id( int id );

int *get_active_pt_last_pos();
void set_active_pt_last_pos( int pos[2] );

bool get_active_pt_is_first_move();
void set_active_pt_is_first_move( bool value );

CurveType get_crv_type( Curve *p_crv );
CurveType get_crv( int seg_id, Curve **rp_crv );

void update_weight_callback( int pt_id, int pnt_idx, double val );
void make_open_callback( int seg_id );
void make_uni_callback( int seg_id );
void update_knot_callback( int seg_id, int knot_idx, double new_val );
void add_knot_callback( int seg_id, double val );
void rmv_knot_callback( int seg_id, int knot_idx );


void update_ctrl_pnt_callback( int pnt_id, double new_x, double new_y );
bool connect_crv_callback( int seg_id_1, int seg_id_2, ConnType type );
BSpline *createBSplineFromBezierCurves( Bezier *bezier1, Bezier *bezier2 );
BSpline *createBSplineFromBSplines( BSpline *bspline1, BSpline *bspline2 );
BSpline *createBSplineFromCurves( Curve *crv1, Curve *crv2, ConnType continuityType );
