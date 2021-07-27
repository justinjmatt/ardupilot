#include "mode.h"
#include "Plane.h"

void ModeTraining::update()
{
////////// Modified 4/22/2021 - Justin Matt //////////

	// Get parameters
	plane.sweep_axis  = plane.g2.jsysid_axis;
	plane.sweep_type  = plane.g2.jsysid_type;
	plane.sweep_amp   = 0.01f*plane.g2.jsysid_amp;
	plane.t_rec       = 1000.0*plane.g2.jsysid_t_rec;
	plane.f_min_hz    = plane.g2.jsysid_f_min_hz;
	plane.f_max_hz    = plane.g2.jsysid_f_max_hz;
	plane.t_fadein    = 1000.0*plane.g2.jsysid_t_fadein;
	float jrandom_number;
	
	if (plane.sweep_axis != 1 && 
		plane.sweep_axis != 2 && 
		plane.sweep_axis != 3 && 
		plane.sweep_axis != 4 && 
		plane.sweep_axis != 5 &&
		plane.sweep_axis != 6 &&
		plane.sweep_axis != 7 &&
		plane.sweep_axis != 10) {
		plane.do_sweep = false;
	} else if (plane.sweep_type != 0 &&
			   plane.sweep_type != 1) {
		plane.do_sweep = false;
	} else {
		plane.do_sweep = true; 
	}
	
	// Get constant minimum frequency time
	if (plane.f_const) {
		plane.t_start = 1000/plane.f_min_hz; // ms
		plane.t_sweep_end = plane.t_rec - plane.t_start; // ms
	} else {
			plane.t_start = 0;
			plane.t_sweep_end = plane.t_rec;
	}
	
	// Get time variables
	if (plane.t0_sweep == 0) {
		// Set t0 to current time
		plane.t0_sweep = millis();
		plane.t_last = millis();
		plane.theta_sweep = 0;
		plane.sweep_noise_y = 0;
		plane.bangbang_pos_aileron = false;
		plane.bangbang_neg_aileron = false;
		plane.bangbang_ail_time = -1;
	}
	
	plane.t_in_mode = millis() - plane.t0_sweep;
	plane.t_current = millis();
	plane.sweep_time_step = plane.t_current - plane.t_last;
	plane.t_last = plane.t_current;

	// Main sweep code
	if (plane.t_in_mode < plane.t_rec && plane.do_sweep == true && plane.sweep_type == 1) {
		
		plane.sweep_active = true;
		
		// Add control to off-axis if desired
		if (plane.sweep_axis == 4 || 
			plane.sweep_axis == 6 ||
			plane.sweep_axis == 10) {
			// Add pitch control during roll or yaw sweep
			plane.training_manual_pitch = false;
			plane.training_manual_roll = true;
			
		} else if (plane.sweep_axis == 5) {
			// Add roll control during pitch sweep
			plane.training_manual_pitch = true;
			plane.training_manual_roll = false;
		} else {
			plane.training_manual_roll = true;
			plane.training_manual_pitch = true;
		}
		plane.nav_pitch_cd = 190; // cdeg, trim pitch = 1.9 deg
		plane.nav_roll_cd = 0;
		
		if (plane.t_in_mode < plane.t_start) {
			plane.t_sweep = 0;
			plane.k_sweep = 0;
		} else {
			plane.t_sweep = plane.t_in_mode - plane.t_start;
			plane.k_sweep = 0.0187*(expf(4.0*plane.t_sweep/plane.t_sweep_end)-1.0);
		}
		plane.omega_rps = (plane.f_min_hz + plane.k_sweep*(plane.f_max_hz - plane.f_min_hz))*2*3.14159;
		plane.theta_sweep += plane.omega_rps*plane.sweep_time_step*0.001f; // rad
		// Add noise to sweep. Lowpass at 20 rps
		jrandom_number = ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
		plane.sweep_noise_u = 1.5*2.0*450.0*plane.sweep_amp*jrandom_number;
		plane.alpha_LP = expf(-20.0*plane.sweep_time_step*0.001f);
		plane.sweep_noise_y = plane.alpha_LP*plane.sweep_noise_y + (1-plane.alpha_LP)*plane.sweep_noise_u;
		// Get sweep input
		// Add variable frequency
		if (plane.sweep_axis == 1 || plane.sweep_axis == 4 {
			// decrease magnitude at low frequencies for roll sweeps
			if (plane.omega_rps < 4.5) {
				plane.sweep_amp = plane.sweep_amp*2.0/3.0;
			} else { 
					if ((plane.omega_rps >= 4.5) && (plane.omega_rps < 5.0)) {
					plane.sweep_amp = plane.sweep_amp*(2.0 + (plane.omega_rps - 4.5)/0.5)/3.0;
					}
			}
		}
		plane.u_sweep = 4500.0*plane.sweep_amp*sinf(plane.theta_sweep) + plane.sweep_noise_y; // units - notational centi-degrees
		if (plane.sweep_axis == 7 || plane.sweep_axis == 10) {
			// bang bang controller
			float j_roll_rate = ahrs.get_gyro().x; // rad/s
			float j_roll_angle = ahrs.roll_sensor; // cdeg
			if (plane.bangbang_pos_aileron) {
				if (plane.bangbang_ail_time < 300) {
					plane.u_sweep += 200;
				}
			}
					
				
			if (j_roll_rate > 0 && j_roll_angle > 1000) {
				// add negative aileron
				bangbang_neg_aileron = true;
				bangbang_pos_aileron = false;
				bangbang_ail_time = t_current;
				
			}
			if (j_roll_rate < 0 && j_roll_angle < 1000) {
				// add positive aileron
				bangbang_pos_aileron = true;
				bangbang_neg_aileron = false;
				bangbang_ail_time = t_current;
			}
			
		} 
		if (plane.t_in_mode < plane.t_fadein) {
			plane.u_sweep = plane.u_sweep*plane.t_in_mode/plane.t_fadein;
		}
		else if (plane.t_in_mode > plane.t_rec - plane.t_fadeout) {
			plane.u_sweep = plane.u_sweep*(plane.t_rec - plane.t_in_mode)/plane.t_fadeout;
		}
	}
	else if (plane.t_in_mode < plane.t_rec && plane.do_sweep == true && plane.sweep_type == 0) {
		// set up controls for manual sweep
		plane.sweep_active = true;
		plane.u_sweep = 0;
		// Plane should be in manual control
		plane.training_manual_roll = true;
		plane.nav_roll_cd = 0;
		plane.training_manual_pitch = true;
		plane.nav_pitch_cd = 0;
	}
	else { // If sweep isn't active, behave normally. 
	    plane.sweep_active = false; // End of modifications.
		plane.u_sweep = 0;
		
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
