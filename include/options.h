#pragma once

#define NUM_SAMPS 2000

// user controlled through GUI
double frenet_anim_speed = 50; // higher number - slower animation.
int num_samples = NUM_SAMPS;

// for code
int frenet_anim_running = 0;
int frenet_anim_smoothness = NUM_SAMPS;
HMENU g_anim_settings_menu = NULL;
