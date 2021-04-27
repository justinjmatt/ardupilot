#include "mode.h"
#include "Plane.h"

void ModeTraining::update()
{
////////// Modified 4/22/2021 - Justin Matt //////////

	// Plane should be in manual control
	plane.training_manual_roll = true;
    plane.nav_roll_cd = 0;
	plane.training_manual_pitch = true;
	plane.nav_pitch_cd = 0;
	
	if (plane.sweep_axis == 0) {
		plane.do_sweep = false;
	} else {
		plane.do_sweep = true; 
	}
	
	// Get constant minimum frequency time
	if (plane.f_const) {
		plane.t_start = 1000/plane.f_min_hz; // ms
		plane.t_sweep_end = plane.t_rec - plane.t_start;
	} else {
			plane.t_start = 0;
			plane.t_sweep_end = plane.t_rec;
	}
	
	// Get time variables
	if (plane.t_in_mode == 0) {
		// Set t0 to current time
		plane.t0_sweep = millis();
		plane.theta_sweep = 0;
		plane.t_last = 0;
	}
	
	plane.t_in_mode = millis() - plane.t0_sweep;
	plane.t_current = millis();
	plane.sweep_time_step = plane.t_current - plane.t_last;
	plane.t_last = plane.t_current;
		
	
	// Main sweep code
	if (plane.t_in_mode < plane.t_rec && plane.do_sweep == true) {
		plane.sweep_active = true;
		if (plane.t_in_mode < plane.t_start) {
			plane.t_sweep = 0;
			plane.k_sweep = 0;
		} else {
			plane.t_sweep = plane.t_in_mode - plane.t_start;
			plane.k_sweep = 0.0187*(expf(4.0*(plane.t_sweep*0.001f)/plane.t_sweep_end)-1.0);
		}
		plane.omega_rps = (plane.f_min_hz + plane.k_sweep*(plane.f_max_hz - plane.f_min_hz))*2*3.14159;
		plane.theta_sweep += plane.omega_rps*plane.sweep_time_step*1000.0f; // rad
		plane.u_sweep = sinf(plane.theta_sweep);
		if (plane.t_in_mode < plane.t_fadein) {
			plane.u_sweep = plane.u_sweep*plane.t_in_mode/plane.t_fadein;
		}
		if (plane.t_in_mode > plane.t_rec - plane.t_fadeout) {
			plane.u_sweep = plane.u_sweep*(plane.t_rec - plane.t_in_mode)/plane.t_fadeout;
		}
		
		// Hold attitude of decoupled axis
		if (plane.sweep_att_hold == true) {
			if (plane.sweep_axis == 1 || plane.sweep_axis == 3) {
				// Hold pitch at zero when doing roll or yaw sweeps.
				plane.training_manual_pitch = false;
				plane.nav_pitch_cd = 0; 
			}
			if (plane.sweep_axis == 2) {
				// Hold roll at zero when doing pitch sweeps.
				plane.training_manual_roll = false;
				plane.nav_roll_cd = 0; 
			}
		}
	}
    else { // If sweep isn't active, behave normally. 
	    plane.sweep_active = false; // End of modifications.
	
		plane.training_manual_roll = false;
		plane.training_manual_pitch = false;
		plane.update_load_factor();

		// if the roll is past the set roll limit, then
		// we set target roll to the limit
		if (plane.ahrs.roll_sensor >= plane.roll_limit_cd) {
			plane.nav_roll_cd = plane.roll_limit_cd;
		} else if (plane.ahrs.roll_sensor <= -plane.roll_limit_cd) {
			plane.nav_roll_cd = -plane.roll_limit_cd;
		} else {
			plane.training_manual_roll = true;
			plane.nav_roll_cd = 0;
		}

		// if the pitch is past the set pitch limits, then
		// we set target pitch to the limit
		if (plane.ahrs.pitch_sensor >= plane.aparm.pitch_limit_max_cd) {
			plane.nav_pitch_cd = plane.aparm.pitch_limit_max_cd;
		} else if (plane.ahrs.pitch_sensor <= plane.pitch_limit_min_cd) {
			plane.nav_pitch_cd = plane.pitch_limit_min_cd;
		} else {
			plane.training_manual_pitch = true;
			plane.nav_pitch_cd = 0;
		}
		if (plane.fly_inverted()) {
			plane.nav_pitch_cd = -plane.nav_pitch_cd;
		}
	}
}
