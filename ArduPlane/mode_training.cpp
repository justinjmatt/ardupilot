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
	plane.j_trim_pitch = 100*plane.g2.jsysid_theta0;
	
	if (plane.sweep_axis != 1 && 
		plane.sweep_axis != 2 && 
		plane.sweep_axis != 3 && 
		plane.sweep_axis != 4 && 
		plane.sweep_axis != 5 &&
		plane.sweep_axis != 6) {
		plane.do_sweep = false;
	} else if (plane.sweep_type != 0 &&
			   plane.sweep_type != 1 &&
			   plane.sweep_type != 3) {
		plane.do_sweep = false;
	} else if (plane.override_sweep) {
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
	}
	
	plane.t_in_mode = millis() - plane.t0_sweep;
	plane.t_current = millis();
	plane.sweep_time_step = plane.t_current - plane.t_last;
	plane.t_last = plane.t_current;

	// Main sweep code
	if (plane.t_in_mode < plane.t_rec && plane.do_sweep == true && plane.sweep_type == 1) {
		plane.sweep_active = true;
		plane.frequency_sweep();
		// If roll or pitch angle goes past limit abort the sweep
		if (plane.ahrs.pitch_sensor > plane.aparm.pitch_limit_max_cd || 
			plane.ahrs.pitch_sensor < plane.pitch_limit_min_cd || 
			plane.ahrs.roll_sensor > plane.roll_limit_cd) {
			plane.override_sweep = true;
			plane.do_sweep = false;
			plane.sweep_active = false;
			plane.set_mode(plane.mode_fbwa, ModeReason::ABORT_SWEEP);
		}
		else { 
			plane.override_sweep = false;
		}
	} else if ((plane.t_in_mode < plane.t_rec) && (plane.do_sweep == true) && (plane.sweep_type == 3)) {
		// do automated doublets
		plane.sweep_active = true;
		plane.auto_doublet();
	/*} else if (plane.override_sweep) {
		// If sweep override was triggered, stabilize aircraft 
		plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
		plane.update_load_factor();
		float pitch_input = plane.channel_pitch->norm_input();
		if (pitch_input > 0) {
			plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max_cd;
		} else {
			plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min_cd);
		} */		
	} else { // If sweep isn't active, behave normally. 
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

void Plane::frequency_sweep() {
	// Add control to off-axis if desired
	if (plane.sweep_axis == 4 || plane.sweep_axis == 6 ) {
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
	plane.nav_pitch_cd = plane.j_trim_pitch; // cdeg, trim pitch
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
		// Add noise to sweep. Lowpass at maximum frequency of sweep
		float jrandom_number;
		jrandom_number = ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
		plane.sweep_noise_u = 1.5*2.0*450.0*plane.sweep_amp*jrandom_number;
		plane.alpha_LP = expf(-plane.f_max_hz*2*3.14159*plane.sweep_time_step*0.001f);
		plane.sweep_noise_y = plane.alpha_LP*plane.sweep_noise_y + (1-plane.alpha_LP)*plane.sweep_noise_u;
		// Get sweep input
		plane.u_sweep = 4500.0*plane.sweep_amp*sinf(plane.theta_sweep) + plane.sweep_noise_y; // units - notational centi-degrees
		if (plane.t_in_mode < plane.t_fadein) {
			plane.u_sweep = plane.u_sweep*plane.t_in_mode/plane.t_fadein;
		} else if (plane.t_in_mode > plane.t_rec - plane.t_fadeout) {
			plane.u_sweep = plane.u_sweep*(plane.t_rec - plane.t_in_mode)/plane.t_fadeout;
		}
		// Call bang-bang controller
		plane.bang_bang_ctrl();
}

void Plane::auto_doublet(void) {
	if (plane.sweep_axis == 1) {
			// do roll doublet
			plane.training_manual_roll = true;
			plane.nav_roll_cd = 0;
			plane.training_manual_pitch = true;
			plane.nav_pitch_cd = 0;
			if (plane.t_in_mode < 500.0) {
				plane.u_sweep = 4500.0*plane.sweep_amp;
			} else if ((plane.t_in_mode >= 500.0) && (plane.t_in_mode < 1000.0)) {
				plane.u_sweep = -4500.0*plane.sweep_amp;
			} else {
				plane.u_sweep = 0.0;
			}
		} else if (plane.sweep_axis == 4) {
			// do roll doublet with pitch control - hold pitch for 3 seconds before
			plane.training_manual_roll = true;
			plane.training_manual_pitch = false;
			plane.nav_roll_cd = 0;
			plane.nav_pitch_cd = plane.j_trim_pitch; // cdeg, trim pitch
			if ((plane.t_in_mode > 3000.0) && (plane.t_in_mode < 3500.0)) {
				plane.u_sweep = 4500.0*plane.sweep_amp;
			} else if ((plane.t_in_mode >= 3500.0) && (plane.t_in_mode < 4000.0)) {
				plane.u_sweep = -4500.0*plane.sweep_amp;
			} else {
				plane.u_sweep = 0.0;
			}
		} else if (plane.sweep_axis == 2) {
			// do pitch doublet
			plane.training_manual_roll = true;
			plane.nav_roll_cd = 0;
			plane.training_manual_pitch = true;
			plane.nav_pitch_cd = 0;
			if (plane.t_in_mode < 403.0) {
				plane.u_sweep = 4500.0*plane.sweep_amp;
			} else if ((plane.t_in_mode >= 403.0) && (plane.t_in_mode < 806.0)) {
				plane.u_sweep = -4500.0*plane.sweep_amp;
			} else {
				plane.u_sweep = 0.0;
			}
		} else if (plane.sweep_axis == 5) {
			// do pitch doublet with roll control - hold for 3 seconds before
			plane.training_manual_roll = false;
			plane.nav_roll_cd = 0;
			plane.training_manual_pitch = true;
			plane.nav_pitch_cd = 0;
			if ((plane.t_in_mode > 3000.0) && (plane.t_in_mode < 3403.0)) {
				plane.u_sweep = 4500.0*plane.sweep_amp;
			} else if ((plane.t_in_mode >= 3403.0) && (plane.t_in_mode < 3806.0)) {
				plane.u_sweep = -4500.0*plane.sweep_amp;
			} else {
				plane.u_sweep = 0.0;
			}
		}
}

void Plane::bang_bang_ctrl(void) {
	float rate;
	float angle;
	float desired_angle;
	if (plane.sweep_axis == 2 || plane.sweep_axis == 5) {
		// During pitch sweeps, read pitch sensors
		rate = plane.ahrs.get_gyro().y; // rad/s
		angle = 0.01*plane.ahrs.pitch_sensor; // deg
		desired_angle = plane.g2.jbang_ptch_lim; //deg
	} else {
		// During roll or yaw sweeps, read roll sensors
		rate = plane.ahrs.get_gyro().x; // rad/s
		angle = 0.01*plane.ahrs.roll_sensor; // deg
		desired_angle = plane.g2.jbang_roll_lim; // deg
	}
	float amp = 0.01*plane.g2.jbang_amp;
	int8_t sign_angle = (angle > 0) ? 1 : ((angle < 0) ? -1 : 0); // return 1 if sign is positive, -1 if negative, and 0 if 0 
	int8_t sign_rate = (rate > 0) ? 1 : ((rate < 0) ? -1 : 0);
	// Initialize
	if (plane.t0_sweep == 0) {
		plane.shift = 0;
		plane.time_since_shift = 0;
		plane.shift_active = false;
		plane.t0_bangbang_shift = 0;
	}
	
	if (abs(angle) > desired_angle && sign_angle == sign_rate && sign_angle != 0) {
		// if measured attitude is above desired and sign of attitude and angular rate are the same
		plane.shift = -sign_angle*amp*4500.0;
		plane.shift_active = true;
		plane.t0_bangbang_shift = millis();
	}
	if (shift_active) {
		plane.time_since_shift = millis() - plane.t0_bangbang_shift; // ms
		if (plane.time_since_shift > 300) {
			plane.shift_active = false;
			plane.shift = 0;
		}
	}
	plane.u_sweep += plane.shift; 
	AP::logger().Write("BANG", "TimeUS,millis,t_shft,t0_shft,shft_act,shft,sign_a,sign_r", "QQQQffbb",
                                        AP_HAL::micros64(),
                                        millis(),
										plane.time_since_shift,
										plane.t0_bangbang_shift,
										double(plane.shift_active),
										plane.shift,
										sign_angle,
										sign_rate);
}