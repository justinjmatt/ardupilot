#include "mode.h"
#include "Plane.h"

void ModeFBWA::update()
{
	/////////////////// Modified 6/14/2021 - Justin Matt ////////////////////////////////
	// Broken loop sweep

	// Get parameters
	plane.sweep_axis  = plane.g2.jsysid_axis;
	plane.sweep_type  = plane.g2.jsysid_type;
	plane.sweep_amp   = 0.01f*plane.g2.jsysid_amp;
	plane.t_rec       = 1000.0*plane.g2.jsysid_t_rec;
	plane.f_min_hz    = plane.g2.jsysid_f_min_hz;
	plane.f_max_hz    = plane.g2.jsysid_f_max_hz;
	plane.t_fadein    = 1000.0*plane.g2.jsysid_t_fadein;
	
	if (plane.sweep_axis != 1 && 
		plane.sweep_axis != 2) {
		plane.do_sweep = false;
	} else if (plane.sweep_type != 2) {
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
	if (plane.t0_sweep_BL == 0) {
		// Set t0 to current time
		plane.t0_sweep_BL = millis();
		plane.t_last = millis();
		plane.theta_sweep = 0;
		plane.sweep_noise_y = 0;
	}
	
	plane.t_in_mode = millis() - plane.t0_sweep_BL;
	plane.t_current = millis();
	plane.sweep_time_step = plane.t_current - plane.t_last;
	plane.t_last = plane.t_current;

	// Main sweep code
	if (plane.t_in_mode < plane.t_rec && plane.do_sweep == true && plane.sweep_type == 2) {
		plane.sweep_active_BL = true;
		plane.BL_sweep();
	}
	else { // If sweep isn't active, behave normally. 
	    plane.sweep_active_BL = false;
		plane.u_sweep = 0; // End of modifications.
	}
		
	// set nav_roll and nav_pitch using sticks
	plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
	plane.update_load_factor();
	float pitch_input = plane.channel_pitch->norm_input();
	if (pitch_input > 0) {
		plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max_cd;
	} else {
		plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min_cd);
	}
	plane.adjust_nav_pitch_throttle();
	plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
	if (plane.fly_inverted()) {
		plane.nav_pitch_cd = -plane.nav_pitch_cd;
	}
	if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
		// FBWA failsafe glide
		plane.nav_roll_cd = 0;
		plane.nav_pitch_cd = 0;
		SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
	}
	RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
	if (chan != nullptr) {
		// check for the user enabling FBWA taildrag takeoff mode
		bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
		if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
			if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
				plane.auto_state.fbwa_tdrag_takeoff_mode = true;
				plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
			}
		}
	}
}

void Plane::BL_sweep(void) {
		if (plane.t_in_mode < plane.t_start) {
			plane.t_sweep = 0;
			plane.k_sweep = 0;
		} else {
			plane.t_sweep = plane.t_in_mode - plane.t_start;
			plane.k_sweep = 0.0187*(expf(4.0*plane.t_sweep/plane.t_sweep_end)-1.0);
		}
		plane.omega_rps = (plane.f_min_hz + plane.k_sweep*(plane.f_max_hz - plane.f_min_hz))*2*3.14159;
		plane.theta_sweep += plane.omega_rps*plane.sweep_time_step*0.001f; // rad
		// Get sweep input
		plane.u_sweep = 4500.0*plane.sweep_amp*sinf(plane.theta_sweep); // units - notational centi-degrees
		if (plane.t_in_mode < plane.t_fadein) {
			plane.u_sweep = plane.u_sweep*plane.t_in_mode/plane.t_fadein;
		} else if (plane.t_in_mode > plane.t_rec - plane.t_fadeout) {
			plane.u_sweep = plane.u_sweep*(plane.t_rec - plane.t_in_mode)/plane.t_fadeout;
		}
}
