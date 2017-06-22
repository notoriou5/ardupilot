#include "Plane.h"

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
bool Plane::start_command(const AP_Mission::Mission_Command& cmd)
{
    // default to non-VTOL loiter
    auto_state.vtol_loiter = false;

        // log when new commands start
    if (should_log(MASK_LOG_CMD)) {
        DataFlash.Log_Write_Mission_Cmd(mission, cmd);
    }

    // special handling for nav vs non-nav commands
    if (AP_Mission::is_nav_cmd(cmd)) {
        // set land_complete to false to stop us zeroing the throttle
        auto_state.sink_rate = 0;

        // set takeoff_complete to true so we don't add extra elevator
        // except in a takeoff
        auto_state.takeoff_complete = true;

        // start non-idle
        auto_state.idle_mode = false;
        
        nav_controller->set_data_is_stale();

        // reset loiter start time. New command is a new loiter
        loiter.start_time_ms = 0;

        AP_Mission::Mission_Command next_nav_cmd;
        const uint16_t next_index = mission.get_current_nav_index() + 1;
        auto_state.wp_is_land_approach = mission.get_next_nav_cmd(next_index, next_nav_cmd) && (next_nav_cmd.id == MAV_CMD_NAV_LAND);

        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Executing nav command ID #%i",cmd.id);
    } else {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Executing command ID #%i",cmd.id);
    }

    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        crash_state.is_crashed = false;
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // LAND to Waypoint
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              // Loiter N Times
        do_loiter_turns(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        set_mode(RTL, MODE_REASON_UNKNOWN);
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        do_continue_and_change_alt(cmd);
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        do_altitude_wait(cmd);
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        crash_state.is_crashed = false;
        return quadplane.do_vtol_takeoff(cmd);

    case MAV_CMD_NAV_VTOL_LAND:
        crash_state.is_crashed = false;
        return quadplane.do_vtol_land(cmd);
        
    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        do_within_distance(cmd);
        break;

    // Do commands

    case MAV_CMD_DO_CHANGE_SPEED:
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_SERVO:
        ServoRelayEvents.do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
        break;

    case MAV_CMD_DO_SET_RELAY:
        ServoRelayEvents.do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
        break;

    case MAV_CMD_DO_REPEAT_SERVO:
        ServoRelayEvents.do_repeat_servo(cmd.content.repeat_servo.channel, cmd.content.repeat_servo.pwm,
                                         cmd.content.repeat_servo.repeat_count, cmd.content.repeat_servo.cycle_time * 1000.0f);
        break;

    case MAV_CMD_DO_REPEAT_RELAY:
        ServoRelayEvents.do_repeat_relay(cmd.content.repeat_relay.num, cmd.content.repeat_relay.repeat_count,
                                         cmd.content.repeat_relay.cycle_time * 1000.0f);
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:
        if (cmd.p1 == 0 || cmd.p1 == 1) {
            auto_state.inverted_flight = (bool)cmd.p1;
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set inverted %u", cmd.p1);
        }
        break;

    case MAV_CMD_DO_LAND_START:
        break;

    case MAV_CMD_DO_FENCE_ENABLE:
#if GEOFENCE_ENABLED == ENABLED
        if (cmd.p1 != 2) {
            if (!geofence_set_enabled((bool) cmd.p1, AUTO_TOGGLED)) {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Unable to set fence. Enabled state to %u", cmd.p1);
            } else {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set fence enabled state to %u", cmd.p1);
            }
        } else { //commanding to only disable floor
            if (! geofence_set_floor_enabled(false)) {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Unable to disable fence floor");
            } else {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Fence floor disabled");
            }
        }    
#endif
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        autotune_enable(cmd.p1);
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        do_digicam_configure(cmd);
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        // do_digicam_control Send Digicam Control message with the camera library
        do_digicam_control(cmd);
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
        break;
#endif

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        do_parachute(cmd);
        break;
#endif

#if MOUNT == ENABLED
    // Sets the region of interest (ROI) for a sensor set or the
    // vehicle itself. This can then be used by the vehicles control
    // system to control the vehicle attitude and the attitude of various
    // devices such as cameras.
    //    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
    case MAV_CMD_DO_SET_ROI:
        if (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
            // switch off the camera tracking if enabled
            if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                camera_mount.set_mode_to_default();
            }
        } else {
            // set mount's target location
            camera_mount.set_roi_target(cmd.content.location);
        }
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        camera_mount.set_angle_targets(cmd.content.mount_control.roll, 
                                       cmd.content.mount_control.pitch, 
                                       cmd.content.mount_control.yaw);
        break;
#endif

    case MAV_CMD_DO_VTOL_TRANSITION:
        plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)cmd.content.do_vtol_transition.target_state);
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        plane.g2.ice_control.engine_control(cmd.content.do_engine_control.start_control,
                                            cmd.content.do_engine_control.cold_start,
                                            cmd.content.do_engine_control.height_delay_cm*0.01f);
        break;
    }

    return true;
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

bool Plane::verify_command(const AP_Mission::Mission_Command& cmd)        // Returns true if command complete
{
    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
        if (flight_stage == AP_Vehicle::FixedWing::FlightStage::FLIGHT_ABORT_LAND) {
            return landing.verify_abort_landing(prev_WP_loc, next_WP_loc, current_loc, auto_state.takeoff_altitude_rel_cm, throttle_suppressed);

        } else {
            // use rangefinder to correct if possible
            const float height = height_above_target() - rangefinder_correction();
            return landing.verify_land(prev_WP_loc, next_WP_loc, current_loc,
                height, auto_state.sink_rate, auto_state.wp_proportion, auto_state.last_flying_ms, arming.is_armed(), is_flying(), rangefinder_state.in_range);
        }

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlim();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns();

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        return verify_continue_and_change_alt();

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        return verify_altitude_wait(cmd);

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return quadplane.verify_vtol_takeoff(cmd);

    case MAV_CMD_NAV_VTOL_LAND:
        return quadplane.verify_vtol_land();

    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // assume parachute was released successfully
        return true;
#endif
        
    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_REPEAT_RELAY:
    case MAV_CMD_DO_INVERTED_FLIGHT:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
    case MAV_CMD_DO_CONTROL_VIDEO:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_VTOL_TRANSITION:
    case MAV_CMD_DO_ENGINE_CONTROL:
        return true;

    default:
        // error message
        gcs_send_text_fmt(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        return true;
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

void Plane::do_RTL(int32_t rtl_altitude)
{
    auto_state.next_wp_no_crosstrack = true;
    auto_state.no_crosstrack = true;
    prev_WP_loc = current_loc;
    next_WP_loc = rally.calc_best_rally_or_home_location(current_loc, rtl_altitude);
    setup_terrain_target_alt(next_WP_loc);
    set_target_altitude_location(next_WP_loc);

    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    setup_glide_slope();
    setup_turn_angle();

    if (should_log(MASK_LOG_MODE))
        DataFlash.Log_Write_Mode(control_mode);
}

void Plane::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    prev_WP_loc = current_loc;
    set_next_WP(cmd.content.location);
    // pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
    auto_state.takeoff_pitch_cd        = (int16_t)cmd.p1 * 100;
    if (auto_state.takeoff_pitch_cd <= 0) {
        // if the mission doesn't specify a pitch use 4 degrees
        auto_state.takeoff_pitch_cd = 400;
    }
    auto_state.takeoff_altitude_rel_cm = next_WP_loc.alt - home.alt;
    next_WP_loc.lat = home.lat + 10;
    next_WP_loc.lng = home.lng + 10;
    auto_state.takeoff_speed_time_ms = 0;
    auto_state.takeoff_complete = false;                            // set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
    auto_state.height_below_takeoff_to_level_off_cm = 0;
    // Flag also used to override "on the ground" throttle disable

    // zero locked course
    steer_state.locked_course_err = 0;
    steer_state.hold_course_cd = -1;
    auto_state.baro_takeoff_alt = barometer.get_altitude();
}

void Plane::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
}

void Plane::do_land(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);

    // configure abort altitude and pitch
    // if NAV_LAND has an abort altitude then use it, else use last takeoff, else use 50m
    if (cmd.p1 > 0) {
        auto_state.takeoff_altitude_rel_cm = (int16_t)cmd.p1 * 100;
    } else if (auto_state.takeoff_altitude_rel_cm <= 0) {
        auto_state.takeoff_altitude_rel_cm = 3000;
    }

    if (auto_state.takeoff_pitch_cd <= 0) {
        // If no takeoff command has ever been used, default to a conservative 10deg
        auto_state.takeoff_pitch_cd = 1000;
    }

    // zero rangefinder state, start to accumulate good samples now
    memset(&rangefinder_state, 0, sizeof(rangefinder_state));

    landing.do_land(cmd, relative_altitude);

#if GEOFENCE_ENABLED == ENABLED 
    if (g.fence_autoenable == 1) {
        if (! geofence_set_enabled(false, AUTO_TOGGLED)) {
            gcs_send_text(MAV_SEVERITY_NOTICE, "Disable fence failed (autodisable)");
        } else {
            gcs_send_text(MAV_SEVERITY_NOTICE, "Fence disabled (autodisable)");
        }
    } else if (g.fence_autoenable == 2) {
        if (! geofence_set_floor_enabled(false)) {
            gcs_send_text(MAV_SEVERITY_NOTICE, "Disable fence floor failed (autodisable)");
        } else {
            gcs_send_text(MAV_SEVERITY_NOTICE, "Fence floor disabled (auto disable)");
        }
    }
#endif
}

void Plane::loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.location.flags.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
}

void Plane::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    location_sanitize(current_loc, cmdloc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);
}

void Plane::do_loiter_turns(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    location_sanitize(current_loc, cmdloc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);

    loiter.total_cd = (uint32_t)(LOWBYTE(cmd.p1)) * 36000UL;
    condition_value = 1; // used to signify primary turns goal not yet met
}

void Plane::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    location_sanitize(current_loc, cmdloc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);

    // we set start_time_ms when we reach the waypoint
    loiter.time_max_ms = cmd.p1 * (uint32_t)1000;     // convert sec to ms
    condition_value = 1; // used to signify primary time goal not yet met
}

void Plane::do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd)
{
    // select heading method. Either mission, gps bearing projection or yaw based
    // If prev_WP_loc and next_WP_loc are different then an accurate wp based bearing can
    // be computed. However, if we had just changed modes before this, such as an aborted landing
    // via mode change, the prev and next wps are the same.
    float bearing;
    if (!locations_are_same(prev_WP_loc, next_WP_loc)) {
        // use waypoint based bearing, this is the usual case
        steer_state.hold_course_cd = -1;
    } else if (ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        // use gps ground course based bearing hold
        steer_state.hold_course_cd = -1;
        bearing = ahrs.get_gps().ground_course_cd() * 0.01f;
        location_update(next_WP_loc, bearing, 1000); // push it out 1km
    } else {
        // use yaw based bearing hold
        steer_state.hold_course_cd = wrap_360_cd(ahrs.yaw_sensor);
        bearing = ahrs.yaw_sensor * 0.01f;
        location_update(next_WP_loc, bearing, 1000); // push it out 1km
    }

    next_WP_loc.alt = cmd.content.location.alt + home.alt;
    condition_value = cmd.p1;
    reset_offset_altitude();
}

void Plane::do_altitude_wait(const AP_Mission::Mission_Command& cmd)
{
    // set all servos to trim until we reach altitude or descent speed
    auto_state.idle_mode = true;
}

void Plane::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd) 
{
    //set target alt  
    Location loc = cmd.content.location;
    location_sanitize(current_loc, loc);
    set_next_WP(loc);
    loiter_set_direction_wp(cmd);

    // init to 0, set to 1 when altitude is reached
    condition_value = 0;
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
bool Plane::verify_takeoff()
{
    if (ahrs.yaw_initialised() && steer_state.hold_course_cd == -1) {
        const float min_gps_speed = 5;
        if (auto_state.takeoff_speed_time_ms == 0 && 
            gps.status() >= AP_GPS::GPS_OK_FIX_3D && 
            gps.ground_speed() > min_gps_speed &&
            hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
            auto_state.takeoff_speed_time_ms = millis();
        }
        if (auto_state.takeoff_speed_time_ms != 0 &&
            millis() - auto_state.takeoff_speed_time_ms >= 2000) {
            // once we reach sufficient speed for good GPS course
            // estimation we save our current GPS ground course
            // corrected for summed yaw to set the take off
            // course. This keeps wings level until we are ready to
            // rotate, and also allows us to cope with arbitrary
            // compass errors for auto takeoff
            float takeoff_course = wrap_PI(radians(gps.ground_course_cd()*0.01f)) - steer_state.locked_course_err;
            takeoff_course = wrap_PI(takeoff_course);
            steer_state.hold_course_cd = wrap_360_cd(degrees(takeoff_course)*100);
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Holding course %ld at %.1fm/s (%.1f)",
                              steer_state.hold_course_cd,
                              (double)gps.ground_speed(),
                              (double)degrees(steer_state.locked_course_err));
        }
    }

    if (steer_state.hold_course_cd != -1) {
        // call navigation controller for heading hold
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    } else {
        nav_controller->update_level_flight();        
    }

    // see if we have reached takeoff altitude
    int32_t relative_alt_cm = adjusted_relative_altitude_cm();
    if (relative_alt_cm > auto_state.takeoff_altitude_rel_cm) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Takeoff complete at %.2fm",
                          (double)(relative_alt_cm*0.01f));
        steer_state.hold_course_cd = -1;
        auto_state.takeoff_complete = true;
        next_WP_loc = prev_WP_loc = current_loc;

        plane.complete_auto_takeoff();

        // don't cross-track on completion of takeoff, as otherwise we
        // can end up doing too sharp a turn
        auto_state.next_wp_no_crosstrack = true;
        return true;
    } else {
        return false;
    }
}

/*
  update navigation for normal mission waypoints. Return true when the
  waypoint is complete
 */
bool Plane::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    steer_state.hold_course_cd = -1;

    // depending on the pass by flag either go to waypoint in regular manner or
    // fly past it for set distance along the line of waypoints
    Location flex_next_WP_loc = next_WP_loc;

    uint8_t cmd_passby = HIGHBYTE(cmd.p1); // distance in meters to pass beyond the wp
    uint8_t cmd_acceptance_distance = LOWBYTE(cmd.p1); // radius in meters to accept reaching the wp

    if (cmd_passby > 0) {
        float dist = get_distance(prev_WP_loc, flex_next_WP_loc);

        if (!is_zero(dist)) {
            float factor = (dist + cmd_passby) / dist;

            flex_next_WP_loc.lat = flex_next_WP_loc.lat + (flex_next_WP_loc.lat - prev_WP_loc.lat) * (factor - 1.0f);
            flex_next_WP_loc.lng = flex_next_WP_loc.lng + (flex_next_WP_loc.lng - prev_WP_loc.lng) * (factor - 1.0f);
        }
    }

    if (auto_state.no_crosstrack) {
        nav_controller->update_waypoint(current_loc, flex_next_WP_loc);
    } else {
        nav_controller->update_waypoint(prev_WP_loc, flex_next_WP_loc);
    }

    // see if the user has specified a maximum distance to waypoint
    // If override with p3 - then this is not used as it will overfly badly
    if (g.waypoint_max_radius > 0 &&
        auto_state.wp_distance > (uint16_t)g.waypoint_max_radius) {
        if (location_passed_point(current_loc, prev_WP_loc, flex_next_WP_loc)) {
            // this is needed to ensure completion of the waypoint
            if (cmd_passby == 0) {
                prev_WP_loc = current_loc;
            }
        }
        return false;
    }

    float acceptance_distance_m = 0; // default to: if overflown - let it fly up to the point
    if (cmd_acceptance_distance > 0) {
        // allow user to override acceptance radius
        acceptance_distance_m = cmd_acceptance_distance;
    } else if (cmd_passby == 0) {
        acceptance_distance_m = nav_controller->turn_distance(g.waypoint_radius, auto_state.next_turn_angle);
    } else {

    }
    
    if (auto_state.wp_distance <= acceptance_distance_m) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, flex_next_WP_loc));
        return true;
	}

    // have we flown past the waypoint?
    if (location_passed_point(current_loc, prev_WP_loc, flex_next_WP_loc)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Passed waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, flex_next_WP_loc));
        return true;
    }

    return false;
}

bool Plane::verify_loiter_unlim()
{
    if (mission.get_current_nav_cmd().p1 <= 1 && abs(g.rtl_radius) > 1) {
        // if mission radius is 0,1, and rtl_radius is valid, use rtl_radius.
        loiter.direction = (g.rtl_radius < 0) ? -1 : 1;
        update_loiter(abs(g.rtl_radius));
    } else {
        // else use mission radius
        update_loiter(mission.get_current_nav_cmd().p1);
    }
    return false;
}

bool Plane::verify_loiter_time()
{
    bool result = false;
    // mission radius is always aparm.loiter_radius
    update_loiter(0);

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() && loiter.sum_cd > 1) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
        }
    } else if (condition_value != 0) {
        // primary goal, loiter time
        if ((millis() - loiter.start_time_ms) > loiter.time_max_ms) {
            // primary goal completed, initialize secondary heading goal
            condition_value = 0;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs_send_text(MAV_SEVERITY_INFO,"Loiter time complete");
        auto_state.vtol_loiter = false;
    }
    return result;
}

bool Plane::verify_loiter_turns()
{
    bool result = false;
    uint16_t radius = HIGHBYTE(mission.get_current_nav_cmd().p1);
    update_loiter(radius);

    // LOITER_TURNS makes no sense as VTOL
    auto_state.vtol_loiter = false;

    if (condition_value != 0) {
        // primary goal, loiter time
        if (loiter.sum_cd > loiter.total_cd && loiter.sum_cd > 1) {
            // primary goal completed, initialize secondary heading goal
            condition_value = 0;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs_send_text(MAV_SEVERITY_INFO,"Loiter orbits complete");
    }
    return result;
}

/*
  verify a LOITER_TO_ALT command. This involves checking we have
  reached both the desired altitude and desired heading. The desired
  altitude only needs to be reached once.
 */
bool Plane::verify_loiter_to_alt() 
{
    bool result = false;
    update_loiter(mission.get_current_nav_cmd().p1);

    // condition_value == 0 means alt has never been reached
    if (condition_value == 0) {
        // primary goal, loiter to alt
        if (labs(loiter.sum_cd) > 1 && (loiter.reached_target_alt || loiter.unable_to_acheive_target_alt)) {
            // primary goal completed, initialize secondary heading goal
            if (loiter.unable_to_acheive_target_alt) {
                gcs_send_text_fmt(MAV_SEVERITY_INFO,"Loiter to alt was stuck at %d", current_loc.alt/100);
            }

            condition_value = 1;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs_send_text(MAV_SEVERITY_INFO,"Loiter to alt complete");
    }
    return result;
}

bool Plane::verify_RTL()
{
    if (g.rtl_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    update_loiter(abs(g.rtl_radius));
	if (auto_state.wp_distance <= (uint32_t)MAX(g.waypoint_radius,0) || 
        reached_loiter_target()) {
			gcs_send_text(MAV_SEVERITY_INFO,"Reached RTL location");
			return true;
    } else {
        return false;
	}
}

bool Plane::verify_continue_and_change_alt()
{
    // is waypoint info not available and heading hold is?
    if (locations_are_same(prev_WP_loc, next_WP_loc) &&
        steer_state.hold_course_cd != -1) {
        //keep flying the same course with fixed steering heading computed at start if cmd
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    }
    else {
        // Is the next_WP less than 200 m away?
        if (get_distance(current_loc, next_WP_loc) < 200.0f) {
            //push another 300 m down the line
            int32_t next_wp_bearing_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);
            location_update(next_WP_loc, next_wp_bearing_cd * 0.01f, 300.0f);
        }

        //keep flying the same course
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }

    //climbing?
    if (condition_value == 1 && adjusted_altitude_cm() >= next_WP_loc.alt) {
        return true;
    }
    //descending?
    else if (condition_value == 2 &&
             adjusted_altitude_cm() <= next_WP_loc.alt) {
        return true;
    }    
    //don't care if we're climbing or descending
    else if (labs(adjusted_altitude_cm() - next_WP_loc.alt) <= 500) {
        return true;
    }

    return false;
}

/*
  see if we have reached altitude or descent speed
 */
bool Plane::verify_altitude_wait(const AP_Mission::Mission_Command &cmd)
{
    if (current_loc.alt > cmd.content.altitude_wait.altitude*100.0f) {
        gcs_send_text(MAV_SEVERITY_INFO,"Reached altitude");
        return true;
    }
    if (auto_state.sink_rate > cmd.content.altitude_wait.descent_rate) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached descent rate %.1f m/s", (double)auto_state.sink_rate);
        return true;        
    }

    // if requested, wiggle servos
    if (cmd.content.altitude_wait.wiggle_time != 0) {
        static uint32_t last_wiggle_ms;
        if (auto_state.idle_wiggle_stage == 0 &&
            AP_HAL::millis() - last_wiggle_ms > cmd.content.altitude_wait.wiggle_time*1000) {
            auto_state.idle_wiggle_stage = 1;
            last_wiggle_ms = AP_HAL::millis();
        }
        // idle_wiggle_stage is updated in set_servos_idle()
    }

    return false;
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void Plane::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value  = cmd.content.delay.seconds * 1000;    // convert seconds to milliseconds
}

void Plane::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool Plane::verify_wait_delay()
{
    if ((unsigned)(millis() - condition_start) > (unsigned)condition_value) {
        condition_value         = 0;
        return true;
    }
    return false;
}

bool Plane::verify_within_distance()
{
    if (auto_state.wp_distance < MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

void Plane::do_loiter_at_location()
{
    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    next_WP_loc = current_loc;
}


void Plane::do_loiter_ellipse()
{
    ellipse.center_loc = home;
    //ellipse.maxradius_cm = 15588.5;
    ellipse.maxradius_cm = 24000;
    //ellipse.minmaxratio = cosf(radians(60));
    ellipse.minmaxratio = cosf(radians(0));
    ellipse.azimuth_deg = 0; // azimuth of the larger principal axis from the north direction
    ellipse.orientation = 1;
    ellipse.height_cm = 2000;

    ellipse.center_loc.alt = ellipse.center_loc.alt + ellipse.height_cm;
    next_WP_loc = ellipse.center_loc;
}


void Plane::do_eight_plane()
{
    eight_in_R2.center_loc = home; // location of the center point on the ground
    eight_in_R2.d_c_cm = 20000; // horizontal distance from crossing point to turning point
    eight_in_R2.S1_radius_cm = 10000; // radius of the turning circles
    eight_in_R2.height_cm = 10000; // height of the pattern above the center point
    eight_in_R2.azimuth_deg = 0.0f;
    eight_in_R2.orientation = 1;     // orientation of the figure-eight pattern

    // the four segments are labeled by integers 0,1,2,3 in dependence of orientation
    // for orientation of the figure-eight pattern: +1: segment sequence is: geodesic_1 -> circle_1 -> geodesic_2 -> circle_2
    //                                              -1: segment sequence is: geodesic_1 -> circle_2 -> geodesic_2 -> circle_1

    // set current quadrant to a value that is not 0,1,2,3 such that it is altered in the initialization
    eight_in_R2.current_quadrant = 4;
    eight_in_R2.current_segment = 0; // sets the start segment when eight_sphere is initialized
                                     // and the entry segment if no segment switching occurs because the aircraft is located in the vicinity of the crossing point defined by _mindistxaplane

  // derived parameters
  // trigonometric functions of all angles
  // trigonometric functions of the angles determining the attitude (azimuthal position in the sky) of the figure-eight pattern
  eight_in_R2.cos_psi = cosf(radians(eight_in_R2.azimuth_deg));
  eight_in_R2.sin_psi = sinf(radians(eight_in_R2.azimuth_deg));
  // trigonometric functions of half of the crossing angle
  eight_in_R2.sin_chihalf = (eight_in_R2.S1_radius_cm / 100.0f) / (eight_in_R2.d_c_cm / 100.0f);
  eight_in_R2.cos_chihalf = sqrt(1 - sq(eight_in_R2.sin_chihalf));

  // crossing point of the figure-eight pattern: coordinate frame, rotation matrix, position vector
  // unit vector pointing from S2_loc to the crossing point of the figure-eight pattern
  eight_in_R2.erxv = Vector3f(0, 0, -1);
  // unit vector of the tangential plane at the crossing point in polar direction
  eight_in_R2.ethetaxv = Vector3f(eight_in_R2.cos_psi, eight_in_R2.sin_psi, 0);
  // unit vector of the tangential plane at the crossing point in azimuthal direction
  eight_in_R2.epsixv = Vector3f(-eight_in_R2.sin_psi, eight_in_R2.cos_psi, 0);
  // rotation matrix that yields the vectors of the figure_eight pattern with crossing point in the direction erxv
  // from those of the figure-eight pattern with crossing point at erv; // = (0,0,-1) and turning circle centers aligned with the east axis
  eight_in_R2.Rm = Matrix3f(eight_in_R2.ethetaxv, eight_in_R2.epsixv, -eight_in_R2.erxv);
  // vector pointing from S2_loc to the crossing point
  eight_in_R2.rxv = eight_in_R2.erxv * eight_in_R2.height_cm / 100.0f;

  // figure-eight pattern from four circle segments defined via four planes in canonical orientation
  // canonical orientation (at azimuth_deg = 0, elevation_deg = 90 in the NED coordinate system): crossing point is upwards (in the (0,0,-1)-direction) from S2_loc; turning circle centers are aligned along east axis
  // figure-8-pattern is given by the sequences:
  // for orientation = +1: geodesic_1 -> circle_1 -> geodesic_2 -> circle_2
  //                   -1: geodesic_1 -> circle_2 -> geodesic_2 -> circle_1
  // individual circle segment orientation = +1 corresponds to applying the right-hand rule to minus the normal vectors pointing to the center of the circle segment
  // vector of the first turning circle center
  // orientation of c1 = orientation
  eight_in_R2.rc1v = (eight_in_R2.Rm * Vector3f(0.0f, eight_in_R2.d_c_cm/100.0f, -eight_in_R2.height_cm/100.0f));
  // vector of the second turning circle center
  // orientation of c2 = - orientation
  eight_in_R2.rc2v = (eight_in_R2.Rm * Vector3f(0.0f, -eight_in_R2.d_c_cm/100.0f, -eight_in_R2.height_cm/100.0f));

  eight_in_R2.c1_loc = home;
  location_offset(eight_in_R2.c1_loc, eight_in_R2.rc1v.x, eight_in_R2.rc1v.y);
  eight_in_R2.c1_loc.alt = eight_in_R2.c1_loc.alt + eight_in_R2.height_cm/100.0f;
  eight_in_R2.c2_loc = home;
  location_offset(eight_in_R2.c2_loc, eight_in_R2.rc2v.x, eight_in_R2.rc2v.y);
  eight_in_R2.c2_loc.alt = eight_in_R2.c2_loc.alt + eight_in_R2.height_cm/100.0f;



  // vectors from the center of the plane to the eastern transition points
  // NE
  eight_in_R2.rtg1c1 = (eight_in_R2.Rm * (eight_in_R2.rc1v + Vector3f(eight_in_R2.S1_radius_cm/100.0f * eight_in_R2.cos_chihalf, -eight_in_R2.S1_radius_cm/100.0f * eight_in_R2.sin_chihalf, 0.0f)));
  // SE
  eight_in_R2.rtc1g2 = (eight_in_R2.Rm * (eight_in_R2.rc1v + Vector3f(-eight_in_R2.S1_radius_cm/100.0f * eight_in_R2.cos_chihalf, -eight_in_R2.S1_radius_cm/100.0f * eight_in_R2.sin_chihalf, 0.0f)));


  // locations of the transition points
  // NE
  eight_in_R2.g1c1_loc = eight_in_R2.center_loc;
  location_offset(eight_in_R2.g1c1_loc, eight_in_R2.rtg1c1.x, eight_in_R2.rtg1c1.y);
  eight_in_R2.g1c1_loc.alt = eight_in_R2.g1c1_loc.alt - eight_in_R2.height_cm/100.0f;
  // SE
  eight_in_R2.c1g2_loc = eight_in_R2.center_loc;
  location_offset(eight_in_R2.c1g2_loc, eight_in_R2.rtc1g2.x, eight_in_R2.rtc1g2.y);
  eight_in_R2.c1g2_loc.alt = eight_in_R2.c1g2_loc.alt - eight_in_R2.height_cm/100.0f;
  // SW
  eight_in_R2.c2g1_loc = eight_in_R2.center_loc;
  location_offset(eight_in_R2.c2g1_loc, -eight_in_R2.rtg1c1.x, -eight_in_R2.rtg1c1.y);
  eight_in_R2.g1c1_loc.alt = eight_in_R2.g1c1_loc.alt - eight_in_R2.height_cm/100.0f;
  // NW
  eight_in_R2.g2c2_loc = eight_in_R2.center_loc;
  location_offset(eight_in_R2.g2c2_loc, -eight_in_R2.rtc1g2.x, -eight_in_R2.rtc1g2.y);
  eight_in_R2.g2c2_loc.alt = eight_in_R2.g2c2_loc.alt - eight_in_R2.height_cm/100.0f;


  // unit tangent vectors of the two geodesic segments
  eight_in_R2.etg1v = eight_in_R2.Rm * Vector3f(eight_in_R2.sin_chihalf, eight_in_R2.cos_chihalf, 0.0f) * eight_in_R2.orientation;
  eight_in_R2.etg2v = eight_in_R2.Rm * Vector3f(eight_in_R2.sin_chihalf, -eight_in_R2.cos_chihalf, 0.0f) * eight_in_R2.orientation;

  // tangent vectors at the transgression points between the segments in the directions demanded by orientation
  // in NE quadrant:
  eight_in_R2.etg1c1v = eight_in_R2.etg1v;
  // in SE quadrant:
  eight_in_R2.etc1g2v = eight_in_R2.etg2v;
  // in NW quadrant
  eight_in_R2.etg2c2v = eight_in_R2.etg2v;
  // in SW quadrant
  eight_in_R2.etc2g1v = eight_in_R2.etg1v;

}

// code: Christoph Sieg:
void Plane::do_loiter_3d()
{
    S1_in_S2.S2_loc = home;
    S1_in_S2.S2_radius_cm = 12000;
    S1_in_S2.theta_rho_deg = 30.0f;
    S1_in_S2.S1_radius_cm = S1_in_S2.S2_radius_cm * sinf(radians(S1_in_S2.theta_rho_deg));
    S1_in_S2.azimuth_deg = 0.0f;
    S1_in_S2.elevation_deg = 90.0f;
    S1_in_S2.orientation = -1;

    float theta = 90.0f - S1_in_S2.elevation_deg;

    float cos_psi = cosf(radians(S1_in_S2.azimuth_deg));
    float sin_psi = sinf(radians(S1_in_S2.azimuth_deg));
    float cos_theta = cosf(radians(theta));
    float sin_theta = sinf(radians(theta));

    S1_in_S2.ercv = Vector3f(sin_theta * cos_psi, sin_theta * sin_psi, -cos_theta);
    //Vector3f rc = S1_in_S2.ercv * S1_in_S2.distance_cm;
    //S1_in_S2.S1_loc = S1_in_S2.S2_loc;
    //location_offset(S1_in_S2.S1_loc, rc.x/100.0f, rc.y/100.0f);
    //S1_in_S2.S1_loc.alt = S1_in_S2.S1_loc.alt - rc.z;
}

//void Plane::do_8_in_S2()
//{
//    8_in_S2.S2_loc = home;
//    8_in_S2.S2_radius_cm = 12000; // radius of the sphere in cm
//    8_in_S2.theta_c_deg = 30; // angle from the crossing point to the center of the turning circle in degrees
//    8_in_S2.theta_rho_deg = 20; // opening angle of the cone with tip at S2_loc and base given by the turning circle
//    8_in_S2.azimuth_deg = 0;
//    8_in_S2.elevation_deg = 45;
//    8_in_S2.orientation = 1;
//
//    float theta = 90 - S1_in_S2.elevation_deg;
//
//    float cos_psi = cosf(radians(S1_in_S2.azimuth_deg));
//    float sin_psi = sinf(radians(S1_in_S2.azimuth_deg));
//    float cos_theta = cosf(radians(theta));
//    float sin_theta = sinf(radians(theta));
//
//    // trigonometric functions of the opening angle
//    // ratio of distance of the turning circle plane and of the radius of the sphere
//    float cos_thetar = cosf(radians(8_in_S2.theta_rho_deg));
//    // ratio of the turning circle radius and of the radius of the sphere
//    float sin_thetar = sinf(radians(8_in_S2.theta_rho_deg));
//
//    // trigonometric functions of the polar angle of the center of the turning circle
//    float cos_thetac = cosf(radians(8_in_S2.theta_c_deg));
//    float sin_thetac = sinf(radians(8_in_S2.theta_c_deg));
//
//    // trigonometric functions of half of the crossing angle
//    float cos_chihalf = sqrt(1 - sq(sin_thetar/sin_thetac));
//    float sin_chihalf = sqrt(1 - sq(cos_chihalf));
//
//    // trigonometric functions of half of the sweeping angle of the geodesic segments
//    float cos_theta0 = cos_thetac/cos_thetar;
//    float sin_theta0 = sqrt(1 - sq(cos_theta0));
//
//    // canonical orientation of the figure-8-pattern at azimuth_deg =0, elevation_deg = 90 in the NED coordinate system:
//    // crossing point is upwards (in the (0,0,-1)-direction) from S2_loc;
//    // turning circle with orientation = +1 is in the eastern ((0, sin_thetac, -cos_thetac)-direction) from S2_loc;
//    // figure-8-pattern is given by the sequence geodesic_1 -> circle_1 -> geodesic_2 -> circle_2 for orientation = 1
//
//    // unit normal vector of the first turning circle plane (pointing to the center of the first turning circle)
//    8_in_S2.erc1v = Vector3f(0, sin_thetac, -cos_thetac);
//    8_in_S2.orientationc1 = 8_in_S2.orientation;
//    // unit normal vector of the second turning circle plane (pointing to the center of the second turning circle)
//    8 in S2.erc2v = Vector3f(0, -sin_thetac, 0, -cos_thetac);
//    8_in_S2.orientationc2 = -8_in_S2.orientation;
//    // unit normal vector of the first geodesic segment (from south-west to north-east)
//    8_in_S2.erg1v = Vector3f(-cos_chihalf, 0, sin_chihalf);
//    // orientation given by applying the right-hand-rule in the opposite direction of the unit normal vector
//    8_in_S2.orientationg1 = -8_in_S2.orientation;
//    // unit normal vector of the second geodesic segment (from south-east to north-west)
//    8_in_S2.erg2v = Vector3f(cos_chihalf, 0, sin_chihalf);
//    // orientation given by applying the right-hand-rule in the opposite direction of the unit normal vector
//    8_in_S2.orientationg2 = -8_in_S2.orientation;
//    //
//
//    // tangent vectors at the transgression points between the segments
//    const Vector3f etg1c1(cos_theta0 * sin_chihalf, cos_theta0 * cos_chihalf, sin_theta0);
//    const Vector3f etc1g2(cos_theta0 * sin_chihalf, -cos_theta0 * cos_chihalf, -sin_theta0);
//    const Vector3f etg2c2(cos_theta0 * sin_chihalf, -cos_theta0 * cos_chihalf, sin_theta0);
//    const Vector3f etc2g1(cos_theta0 * sin_chihalf, cos_theta0 * cos_chihalf, -sin_theta0);
//
//    // generic orientation with crossing point in the (sin_theta * cos_psi, sin_theta * sin_psi, - cos_theta)-direction is obtained by applying the rotation
//    //
//    // unit vector in radial direction
//    Vector3f erv(sin_theta * cos_psi, sin_theta * sin_psi, - cos_theta);
//    // unit vector in polar direction
//    Vector3f ethetav(cos_theta * cos_psi, cos_theta * sin_psi, sin_theta);
//    // unit vector in azimuthal direction
//    Vector3f epsiv(- sin_psi, cos_psi, 0);
//    // rotation matrix
//    Matrix3f Rm(ethetav, epsiv, -erv);
//
//
//    8_in_S2.ercv = Vector3f(sin_theta * cos_psi, sin_theta * sin_psi, - cos_theta);
//
//
//}


// initialization of an inclined circle S^1, whose center point is located in the direction parameterized by azimuth psi and inclination theta
// code: Thomas Gehrmann, modification: Christoph Sieg
//void Plane::do_loiter_3d()
//{
////    float LOCATION_SCALING_FACTOR_INV = 89.83204953368922;
//
//    intersection.psi_plane = radians(0);
//    intersection.theta_plane = radians(-0);
//
//    // do not vary omega and sigma, they have just to be defined because of the do_eight_sphere() function
//    eight_sphere.omega = radians(0);
//    eight_sphere.sigma = radians(0);
//
//    intersection.distance_cm = 15000;
//
//    intersection.sphere_radius_cm = 18000;
//
//    float cos_psi = cosf(intersection.psi_plane);
//    float cos_theta = cosf(intersection.theta_plane);
//    float sin_psi = sinf(intersection.psi_plane);
//    float sin_theta = sinf(intersection.theta_plane);
//
//    intersection.normal_vec.x = - sin_theta * cos_psi;
//    intersection.normal_vec.y = - sin_theta * sin_psi;
//    intersection.normal_vec.z = - cos_theta;
//
//    intersection.circle_radius = sqrtf(intersection.sphere_radius_cm/100.0f * intersection.sphere_radius_cm/100.0f - intersection.distance_cm/100.0f * intersection.distance_cm/100.0f);
//
//    intersection.eccent = sin_theta;
//
//    intersection.circle_center = home;
//    intersection.circle_center.lat += intersection.distance_cm/100.0f * intersection.normal_vec.x * LOCATION_SCALING_FACTOR_INV;
//    intersection.circle_center.lng += intersection.distance_cm/100.0f * intersection.normal_vec.y * LOCATION_SCALING_FACTOR_INV / longitude_scale(home);
//    intersection.circle_center.alt -= intersection.distance_cm * intersection.normal_vec.z;
//
//    intersection.rot_matrix_pe.a.x = cos_theta * cos_psi;
//    intersection.rot_matrix_pe.a.y = cos_theta * sin_psi;
//    intersection.rot_matrix_pe.a.z = -sin_theta;
//    intersection.rot_matrix_pe.b.x = -sin_psi;
//    intersection.rot_matrix_pe.b.y = cos_psi;
//    intersection.rot_matrix_pe.b.z = 0;
//    intersection.rot_matrix_pe.c.x = sin_theta * cos_psi;
//    intersection.rot_matrix_pe.c.y = sin_theta * sin_psi;
//    intersection.rot_matrix_pe.c.z = cos_theta;
//
//
//    loiter.direction = -1;
//
//}

// code: Christoph Sieg
void Plane::do_eight_sphere()
{
    eight_in_S2.S2_loc = home; // location of the center of the S2
    eight_in_S2.S2_radius_cm = 24000; // radius of the S2 in cm
    eight_in_S2.theta_c_deg = 30.0f; // half of the angle between the centers of the two turning circle segments, range: [0,90] degrees
    eight_in_S2.theta_r_deg = 20.705f; // opening angle of the cone with tip at S2_loc and base given by the turning circle, range: [0,90-theta_c_deg] degrees in order to guarantee that the sweeping angle between the two apices is less than 180 deg.
    eight_in_S2.azimuth_deg = 0.0f; // azimuth angle of the vector pointing from S2_loc to the crossing point of the figure-eight pattern, range: [0,360]
    eight_in_S2.elevation_deg = 90.0f; // inclination angle of the vector pointing from S2_loc to the crossing point of the figure-eight pattern, range [0,90]
    eight_in_S2.orientation = -1; // orientation of the figure-eight pattern: +1: downwards flight on geodesic, upwards flight on turning circle segments
                                 //                                          -1: upwards flight on geodesic, downwards flight on turning circle segments

    // the four segments are labeled by integers 0,1,2,3 in dependence of orientation
    // for orientation of the figure-eight pattern: +1: segment sequence is: geodesic_1 -> circle_1 -> geodesic_2 -> circle_2
    //                                              -1: segment sequence is: geodesic_1 -> circle_2 -> geodesic_2 -> circle_1
    // set current quadrant to a value that is not 0,1,2,3 such that it is altered in the initialization
     eight_in_S2.current_quadrant = 4;
     eight_in_S2.current_segment = 0; // sets the start segment when eight_sphere is initialized
                                     // and the entry segment if no segment switching occurs because the aircraft is located in the vicinity of the crossing point defined by _mindistxaplane


    // derived parameters
    // trigonometric functions of all angles
    // calculation of polar angle from elevation
    eight_in_S2.theta = 90.0f - eight_in_S2.elevation_deg;
    // trigonometric functions of the angles determining the attitude (azimuthal and polar position in the sky) of the figure-eight pattern
    eight_in_S2.cos_psi = cosf(radians(eight_in_S2.azimuth_deg));
    eight_in_S2.sin_psi = sinf(radians(eight_in_S2.azimuth_deg));
    eight_in_S2.cos_theta = cosf(radians(eight_in_S2.theta));
    eight_in_S2.sin_theta = sinf(radians(eight_in_S2.theta));
    // trigonometric functions of the opening angle
    // ratio of distance of the turning circle plane and of the radius of the sphere
    eight_in_S2.cos_theta_r = cosf(radians(eight_in_S2.theta_r_deg));
    // ratio of the turning circle radius and of the radius of the sphere
    eight_in_S2.sin_theta_r = sinf(radians(eight_in_S2.theta_r_deg));
    // trigonometric functions of the polar angle of the center of the turning circle
    eight_in_S2.cos_theta_c = cosf(radians(eight_in_S2.theta_c_deg));
    eight_in_S2.sin_theta_c = sinf(radians(eight_in_S2.theta_c_deg));
    // trigonometric functions of half of the crossing angle
    eight_in_S2.cos_chihalf = sqrt(1 - sq(eight_in_S2.sin_theta_r/eight_in_S2.sin_theta_c));
    eight_in_S2.sin_chihalf = sqrt(1 - sq(eight_in_S2.cos_chihalf));
    // trigonometric functions of half of the sweeping angle of the geodesic segments
    eight_in_S2.cos_theta_0 = eight_in_S2.cos_theta_c/eight_in_S2.cos_theta_r;
    eight_in_S2.sin_theta_0 = sqrt(1 - sq(eight_in_S2.cos_theta_0));


    // turning circle segments of the figure-eight pattern: circle radius, distance from origin
    // radius
    eight_in_S2.S1_radius_cm = eight_in_S2.S2_radius_cm * eight_in_S2.sin_theta_r;
    // distance of the centers from S2_loc;
    eight_in_S2.dist_cm = eight_in_S2.S2_radius_cm * eight_in_S2.cos_theta_r;

    // crossing point of the figure-eight pattern: coordinate frame, rotation matrix, position vector
    // unit vector pointing from S2_loc to the crossing point of the figure-eight pattern
    eight_in_S2.erxv = Vector3f(eight_in_S2.sin_theta * eight_in_S2.cos_psi, eight_in_S2.sin_theta * eight_in_S2.sin_psi, -eight_in_S2.cos_theta );
    // unit vector of the tangential plane at the crossing point in polar direction
    eight_in_S2.ethetaxv = Vector3f(eight_in_S2.cos_theta * eight_in_S2.cos_psi, eight_in_S2.cos_theta * eight_in_S2.sin_psi, eight_in_S2.sin_theta);
    // unit vector of the tangential plane at the crossing point in azimuthal direction
    eight_in_S2.epsixv = Vector3f(-eight_in_S2.sin_psi, eight_in_S2.cos_psi, 0);
    // rotation matrix that yields the vectors of the figure_eight pattern with crossing point in the direction erxv
    // from those of the figure-eight pattern with crossing point at erv = (0,0,-1) and turning circle centers aligned with the east axis
    eight_in_S2.Rm = Matrix3f(eight_in_S2.ethetaxv, eight_in_S2.epsixv, -eight_in_S2.erxv);
    // vector pointing from S2_loc to the crossing point
    eight_in_S2.rxv = eight_in_S2.erxv * eight_in_S2.dist_cm / 100.0f;

    // figure-eight pattern from four circle segments defined via four planes in canonical orientation
    // canonical orientation (at azimuth_deg = 0, elevation_deg = 90 in the NED coordinate system): crossing point is upwards (in the (0,0,-1)-direction) from S2_loc; turning circle centers are aligned along east axis
    // figure-8-pattern is given by the sequences:
    // for orientation = +1: geodesic_1 -> circle_1 -> geodesic_2 -> circle_2
    //                   -1: geodesic_1 -> circle_2 -> geodesic_2 -> circle_1
    // individual circle segment orientation = +1 corresponds to applying the right-hand rule to minus the normal vectors pointing to the center of the circle segment
    // unit normal vector of the first turning circle plane (pointing to the center of the first turning circle c1)
    // orientation of c1 = orientation
    eight_in_S2.erc1v = eight_in_S2.Rm * Vector3f(0.0f, eight_in_S2.sin_theta_c, -eight_in_S2.cos_theta_c);
    // unit normal vector of the second turning circle plane (pointing to the center of the second turning circle c2)
    // orientation of c2 = - orientation
    eight_in_S2.erc2v = eight_in_S2.Rm * Vector3f(0.0f, -eight_in_S2.sin_theta_c, -eight_in_S2.cos_theta_c);
    // unit normal vector of the first geodesic plane (geodesic g1: from south-west to north-east)
    // orientation of g1 = orientation
    eight_in_S2.erg1v = eight_in_S2.Rm * Vector3f(-eight_in_S2.cos_chihalf, eight_in_S2.sin_chihalf, 0.0f);
    // unit normal vector of the second geodesic segment (geodesic g2: from south-east to north-west)
    // orientation of g2 = orientation
    eight_in_S2.erg2v = eight_in_S2.Rm * Vector3f(eight_in_S2.cos_chihalf, eight_in_S2.sin_chihalf, 0.0f);

    // tangent vectors at the transgression points between the segments in the directions demanded by orientation
    // in NE quadrant:
    eight_in_S2.etg1c1v = eight_in_S2.Rm * Vector3f(eight_in_S2.cos_theta_0 * eight_in_S2.sin_chihalf, eight_in_S2.cos_theta_0 * eight_in_S2.cos_chihalf, eight_in_S2.sin_theta_0) * eight_in_S2.orientation;
    // in SE quadrant:
    eight_in_S2.etc1g2v = eight_in_S2.Rm * Vector3f(eight_in_S2.cos_theta_0 * eight_in_S2.sin_chihalf, -eight_in_S2.cos_theta_0 * eight_in_S2.cos_chihalf, -eight_in_S2.sin_theta_0) * eight_in_S2.orientation;
    // in NW quadrant
    eight_in_S2.etg2c2v = eight_in_S2.Rm * Vector3f(eight_in_S2.cos_theta_0 * eight_in_S2.sin_chihalf, -eight_in_S2.cos_theta_0 * eight_in_S2.cos_chihalf, eight_in_S2.sin_theta_0) * eight_in_S2.orientation;
    // in SW quadrant
    eight_in_S2.etc2g1v = eight_in_S2.Rm * Vector3f(eight_in_S2.cos_theta_0 * eight_in_S2.sin_chihalf, eight_in_S2.cos_theta_0 * eight_in_S2.cos_chihalf, -eight_in_S2.sin_theta_0) * eight_in_S2.orientation;

    // tangent vectors at the crossing point
    eight_in_S2.etg1xv  = eight_in_S2.Rm * Vector3f(eight_in_S2.sin_chihalf, eight_in_S2.cos_chihalf, 0.0f) * eight_in_S2.orientation;
    eight_in_S2.etg2xv = eight_in_S2.Rm * Vector3f(eight_in_S2.sin_chihalf, -eight_in_S2.cos_chihalf, 0.0f) * eight_in_S2.orientation;

}

// initialization of an inclined figure-eight pattern on a sphere S^2, whose crossing point is located in the direction parameterized by azimuth w and inclination sigma
// code: Thomas Gehrmann, slight modifications: Christoph Sieg
//void Plane::do_eight_sphere()
//{
////    float LOCATION_SCALING_FACTOR_INV = 89.83204953368922;
//
//    // define the shape of eight
//    eight_sphere.cross_angle = radians(30);
//    eight_sphere.arc_length_angle = radians(30);
//
//    // orientation of eight
//    eight_sphere.omega = radians(00);//radians(g.omega_wind);
//    eight_sphere.omega_old = g.omega_wind;
//    eight_sphere.sigma = radians(00);//radians(g.sigma_wind);
//
//    intersection.sphere_radius_cm = 24000;
//
//    float cos_cross = cosf(eight_sphere.cross_angle);
//    float cos_arc = cosf(eight_sphere.arc_length_angle);
//    float sin_cross = sinf(eight_sphere.cross_angle);
//    float sin_arc = sinf(eight_sphere.arc_length_angle);
//
//    float cos_omega = cosf(eight_sphere.omega);
//    float cos_sigma = cosf(eight_sphere.sigma);
//    float sin_omega = sinf(eight_sphere.omega);
//    float sin_sigma = sinf(eight_sphere.sigma);
//
//    eight_sphere.eta = atanf(sin_arc/(cos_arc*cos_cross));
//    float sin_eta = sinf(eight_sphere.eta);
//    float cos_eta = cosf(eight_sphere.eta);
//
// /*   eight_sphere.rot_8_matrix.a.x = cos_omega*cos_sigma;
//    eight_sphere.rot_8_matrix.a.y = -sin_omega*cos_sigma;
//    eight_sphere.rot_8_matrix.a.z = sin_sigma;
//    eight_sphere.rot_8_matrix.b.x = sin_omega;
//    eight_sphere.rot_8_matrix.b.y = cos_omega;
//    eight_sphere.rot_8_matrix.b.z = 0;
//    eight_sphere.rot_8_matrix.c.x = -cos_omega*sin_sigma;
//    eight_sphere.rot_8_matrix.c.y = sin_omega*sin_sigma;
//    eight_sphere.rot_8_matrix.c.z = cos_sigma;  */
//
//
//    intersection.distance_cm = intersection.sphere_radius_cm *(sin_eta*cos_cross*sin_arc + cos_eta*cos_arc);
//
//    intersection.circle_radius = sqrtf(intersection.sphere_radius_cm/100.0f * intersection.sphere_radius_cm/100.0f - intersection.distance_cm/100.0f * intersection.distance_cm/100.0f);
//
//    eight_sphere.secant = 2*intersection.sphere_radius_cm/100*sin_cross*sin_arc;
//    eight_sphere.sector_angle = acosf(eight_sphere.secant/(2*intersection.circle_radius));
//
//    eight_sphere.normal_vec.x = sin_omega*sin_eta - cos_omega*sin_sigma*cos_eta;        // product of rot_8_matrix times vector (0, -sin_eta, -cos_eta)^T
//    eight_sphere.normal_vec.y = -cos_omega*sin_eta - sin_omega*sin_sigma*cos_eta;       // normal_vec of left turning circle
//    eight_sphere.normal_vec.z = -cos_sigma*cos_eta;
//
//    eight_sphere.circle_center_left = home;
//    eight_sphere.circle_center_left.lat += intersection.distance_cm/100.0f * eight_sphere.normal_vec.x * LOCATION_SCALING_FACTOR_INV;
//    eight_sphere.circle_center_left.lng += intersection.distance_cm/100.0f * eight_sphere.normal_vec.y * LOCATION_SCALING_FACTOR_INV / longitude_scale(home);
//    eight_sphere.circle_center_left.alt -= intersection.distance_cm * eight_sphere.normal_vec.z;
//
//    eight_sphere.normal_vec.x = -sin_omega*sin_eta - cos_omega*sin_sigma*cos_eta;       // product of rot_8_matrix times vector (0, sin_eta, -cos_eta)^T
//    eight_sphere.normal_vec.y = cos_omega*sin_eta - sin_omega*sin_sigma*cos_eta;        // normal_vec of right turning circle
//    eight_sphere.normal_vec.z = -cos_sigma*cos_eta;
//
//    eight_sphere.circle_center_right = home;
//    eight_sphere.circle_center_right.lat += intersection.distance_cm/100.0f * eight_sphere.normal_vec.x * LOCATION_SCALING_FACTOR_INV;
//    eight_sphere.circle_center_right.lng += intersection.distance_cm/100.0f * eight_sphere.normal_vec.y * LOCATION_SCALING_FACTOR_INV / longitude_scale(home);
//    eight_sphere.circle_center_right.alt -= intersection.distance_cm * eight_sphere.normal_vec.z;
//
//    hal.console->print("right.lat: ");
//    hal.console->println( eight_sphere.circle_center_right.lat);
//    hal.console->print("right.lng: ");
//    hal.console->println( eight_sphere.circle_center_right.lng);
//    hal.console->print("left.lat: ");
//    hal.console->println( eight_sphere.circle_center_left.lat);
//    hal.console->print("left.lng: ");
//    hal.console->println( eight_sphere.circle_center_left.lng);
//
//    eight_sphere.rot_matrix_right.a.x = sin_omega*cos_eta - cos_omega*sin_sigma*sin_eta;
//    eight_sphere.rot_matrix_right.a.y = -cos_omega*cos_eta - sin_omega*sin_sigma*sin_eta;
//    eight_sphere.rot_matrix_right.a.z = -cos_sigma*sin_eta;
//    eight_sphere.rot_matrix_right.b.x = cos_omega*cos_sigma;
//    eight_sphere.rot_matrix_right.b.y = sin_omega*cos_sigma;
//    eight_sphere.rot_matrix_right.b.z = -sin_sigma;
//    eight_sphere.rot_matrix_right.c.x = sin_omega*sin_eta + cos_omega*sin_sigma*cos_eta;
//    eight_sphere.rot_matrix_right.c.y = -cos_omega*sin_eta + sin_omega*sin_sigma*cos_eta;
//    eight_sphere.rot_matrix_right.c.z = cos_sigma*cos_eta;
//
//    eight_sphere.rot_matrix_left.a.x = -sin_omega*cos_eta - cos_omega*sin_sigma*sin_eta;
//    eight_sphere.rot_matrix_left.a.y = cos_omega*cos_eta - sin_omega*sin_sigma*sin_eta;
//    eight_sphere.rot_matrix_left.a.z = -cos_sigma*sin_eta;
//    eight_sphere.rot_matrix_left.b.x = -cos_omega*cos_sigma;
//    eight_sphere.rot_matrix_left.b.y = -sin_omega*cos_sigma;
//    eight_sphere.rot_matrix_left.b.z = sin_sigma;
//    eight_sphere.rot_matrix_left.c.x = -sin_omega*sin_eta + cos_omega*sin_sigma*cos_eta;
//    eight_sphere.rot_matrix_left.c.y = cos_omega*sin_eta + sin_omega*sin_sigma*cos_eta;
//    eight_sphere.rot_matrix_left.c.z = cos_sigma*cos_eta;
//
//    eight_sphere.rot_matrix_cross1.a.x = -cos_omega*sin_sigma;
//    eight_sphere.rot_matrix_cross1.a.y = -sin_omega*sin_sigma;
//    eight_sphere.rot_matrix_cross1.a.z = -cos_sigma;
//    eight_sphere.rot_matrix_cross1.b.x = -cos_omega*cos_sigma*sin_cross - sin_omega*cos_cross;
//    eight_sphere.rot_matrix_cross1.b.y = -sin_omega*cos_sigma*sin_cross + cos_omega*cos_cross;
//    eight_sphere.rot_matrix_cross1.b.z = sin_sigma*sin_cross;
//    eight_sphere.rot_matrix_cross1.c.x = cos_omega*cos_sigma*cos_cross - sin_omega*sin_cross;
//    eight_sphere.rot_matrix_cross1.c.y = sin_omega*cos_sigma*cos_cross + cos_omega*sin_cross;
//    eight_sphere.rot_matrix_cross1.c.z = -sin_sigma*cos_cross;
//
//    eight_sphere.rot_matrix_cross2.a.x = -cos_omega*sin_sigma;
//    eight_sphere.rot_matrix_cross2.a.y = -sin_omega*sin_sigma;
//    eight_sphere.rot_matrix_cross2.a.z = -cos_sigma;
//    eight_sphere.rot_matrix_cross2.b.x = cos_omega*cos_sigma*sin_cross - sin_omega*cos_cross;
//    eight_sphere.rot_matrix_cross2.b.y = sin_omega*cos_sigma*sin_cross + cos_omega*cos_cross;
//    eight_sphere.rot_matrix_cross2.b.z = -sin_sigma*sin_cross;
//    eight_sphere.rot_matrix_cross2.c.x = cos_omega*cos_sigma*cos_cross + sin_omega*sin_cross;
//    eight_sphere.rot_matrix_cross2.c.y = sin_omega*cos_sigma*cos_cross - cos_omega*sin_cross;
//    eight_sphere.rot_matrix_cross2.c.z = -sin_sigma*cos_cross;
//
//    eight_sphere.segment = 1;
//
//}


void Plane::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.content.speed.speed_type)
    {
    case 0:             // Airspeed
        if (cmd.content.speed.target_ms > 0) {
            aparm.airspeed_cruise_cm.set(cmd.content.speed.target_ms * 100);
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set airspeed %u m/s", (unsigned)cmd.content.speed.target_ms);
        }
        break;
    case 1:             // Ground speed
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set groundspeed %u", (unsigned)cmd.content.speed.target_ms);
        aparm.min_gndspeed_cm.set(cmd.content.speed.target_ms * 100);
        break;
    }

    if (cmd.content.speed.throttle_pct > 0 && cmd.content.speed.throttle_pct <= 100) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set throttle %u", (unsigned)cmd.content.speed.throttle_pct);
        aparm.throttle_cruise.set(cmd.content.speed.throttle_pct);
    }
}

void Plane::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        init_home();
    } else {
        ahrs.set_home(cmd.content.location);
        home_is_set = HOME_SET_NOT_LOCKED;
        Log_Write_Home_And_Origin();
        GCS_MAVLINK::send_home_all(cmd.content.location);
    }
}

// do_digicam_configure Send Digicam Configure message with the camera library
void Plane::do_digicam_configure(const AP_Mission::Mission_Command& cmd)
{
#if CAMERA == ENABLED
    camera.configure(cmd.content.digicam_configure.shooting_mode,
                     cmd.content.digicam_configure.shutter_speed,
                     cmd.content.digicam_configure.aperture,
                     cmd.content.digicam_configure.ISO,
                     cmd.content.digicam_configure.exposure_type,
                     cmd.content.digicam_configure.cmd_id,
                     cmd.content.digicam_configure.engine_cutoff_time);
#endif
}

// do_digicam_control Send Digicam Control message with the camera library
void Plane::do_digicam_control(const AP_Mission::Mission_Command& cmd)
{
#if CAMERA == ENABLED
    if (camera.control(cmd.content.digicam_control.session,
                       cmd.content.digicam_control.zoom_pos,
                       cmd.content.digicam_control.zoom_step,
                       cmd.content.digicam_control.focus_lock,
                       cmd.content.digicam_control.shooting_cmd,
                       cmd.content.digicam_control.cmd_id)) {
        log_picture();
    }
#endif
}

// do_take_picture - take a picture with the camera library
void Plane::do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic(true);
    log_picture();
#endif
}

#if PARACHUTE == ENABLED
// do_parachute - configure or release parachute
void Plane::do_parachute(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.p1) {
        case PARACHUTE_DISABLE:
            parachute.enabled(false);
            break;
        case PARACHUTE_ENABLE:
            parachute.enabled(true);
            break;
        case PARACHUTE_RELEASE:
            parachute_release();
            break;
        default:
            // do nothing
            break;
    }
}
#endif

// log_picture - log picture taken and send feedback to GCS
void Plane::log_picture()
{
#if CAMERA == ENABLED
    if (!camera.using_feedback_pin()) {
        gcs_send_message(MSG_CAMERA_FEEDBACK);
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
        }
    } else {
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Trigger(ahrs, gps, current_loc);
        }      
    }
#endif
}

// start_command_callback - callback function called from ap-mission when it begins a new mission command
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Plane::start_command_callback(const AP_Mission::Mission_Command &cmd)
{
    if (control_mode == AUTO) {
        return start_command(cmd);
    }
    return true;
}

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Plane::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == AUTO) {
        bool cmd_complete = verify_command(cmd);

        // send message to GCS
        if (cmd_complete) {
            gcs_send_mission_item_reached_message(cmd.index);
        }

        return cmd_complete;
    }
    return false;
}

// exit_mission_callback - callback function called from ap-mission when the mission has completed
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
void Plane::exit_mission_callback()
{
    if (control_mode == AUTO) {
        set_mode(RTL, MODE_REASON_MISSION_END);
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Mission complete, changing mode to RTL");
    }
}

bool Plane::verify_loiter_heading(bool init)
{
    if (quadplane.in_vtol_auto()) {
        // skip heading verify if in VTOL auto
        return true;
    }

    //Get the lat/lon of next Nav waypoint after this one:
    AP_Mission::Mission_Command next_nav_cmd;
    if (! mission.get_next_nav_cmd(mission.get_current_nav_index() + 1,
                                   next_nav_cmd)) {
        //no next waypoint to shoot for -- go ahead and break out of loiter
        return true;
    }

    if (get_distance(next_WP_loc, next_nav_cmd.content.location) < abs(aparm.loiter_radius)) {
        /* Whenever next waypoint is within the loiter radius,
           maintaining loiter would prevent us from ever pointing toward the next waypoint.
           Hence break out of loiter immediately
         */
        return true;
    }

    // Bearing in degrees
    int32_t bearing_cd = get_bearing_cd(current_loc,next_nav_cmd.content.location);

    // get current heading.
    int32_t heading_cd = gps.ground_course_cd();

    int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    if (init) {
        loiter.total_cd =  wrap_360_cd(bearing_cd - heading_cd);
        loiter.sum_cd = 0;
    }

    /*
      Check to see if the the plane is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw. Allow turn count
      to stop it too to ensure we don't loop around forever in
      case high winds are forcing us beyond 200 deg/sec at this
      particular moment.
    */
    if (labs(heading_err_cd) <= 1000  ||
            loiter.sum_cd > loiter.total_cd) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp

        // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        if (next_WP_loc.flags.loiter_xtrack) {
            next_WP_loc = current_loc;
        }
        return true;
    }
    return false;
}
