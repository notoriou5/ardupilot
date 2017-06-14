#include "Plane.h"

// set the nav_controller pointer to the right controller
void Plane::set_nav_controller(void)
{
    switch ((AP_Navigation::ControllerType)g.nav_controller.get()) {

    default:
    case AP_Navigation::CONTROLLER_DEFAULT:
        // no break, fall through to L1 as default controller

    case AP_Navigation::CONTROLLER_L1:
        nav_controller = &L1_controller;
        break;
    }
}

/*
  reset the total loiter angle
 */
void Plane::loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
    loiter.reached_target_alt = false;
    loiter.unable_to_acheive_target_alt = false;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
void Plane::loiter_angle_update(void)
{
    static const int32_t lap_check_interval_cd = 3*36000;

    const int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;

    if (loiter.sum_cd == 0 && !reached_loiter_target()) {
        // we don't start summing until we are doing the real loiter
        loiter_delta_cd = 0;
    } else if (loiter.sum_cd == 0) {
        // use 1 cd for initial delta
        loiter_delta_cd = 1;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd = lap_check_interval_cd;
    } else {
        loiter_delta_cd = target_bearing_cd - loiter.old_target_bearing_cd;
    }

    loiter.old_target_bearing_cd = target_bearing_cd;
    loiter_delta_cd = wrap_180_cd(loiter_delta_cd);
    loiter.sum_cd += loiter_delta_cd * loiter.direction;

    if (labs(current_loc.alt - next_WP_loc.alt) < 500) {
        loiter.reached_target_alt = true;
        loiter.unable_to_acheive_target_alt = false;
        loiter.next_sum_lap_cd = loiter.sum_cd + lap_check_interval_cd;

    } else if (!loiter.reached_target_alt && labs(loiter.sum_cd) >= loiter.next_sum_lap_cd) {
        // check every few laps for scenario where up/downdrafts inhibit you from loitering up/down for too long
        loiter.unable_to_acheive_target_alt = labs(current_loc.alt - loiter.start_lap_alt_cm) < 500;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd += lap_check_interval_cd;
    }
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void Plane::navigate()
{
    // allow change of nav controller mid-flight
    set_nav_controller();

    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    auto_state.wp_distance = get_distance(current_loc, next_WP_loc);
    auto_state.wp_proportion = location_path_proportion(current_loc, 
                                                        prev_WP_loc, next_WP_loc);
    SpdHgt_Controller->set_path_proportion(auto_state.wp_proportion);

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    update_navigation();
}

void Plane::calc_airspeed_errors()
{
    float airspeed_measured_cm = airspeed.get_airspeed_cm();


    // FBW_B airspeed target
    if (control_mode == FLY_BY_WIRE_B || 
        control_mode == CRUISE) {
        target_airspeed_cm = ((int32_t)(aparm.airspeed_max -
                                        aparm.airspeed_min) *
                              channel_throttle->get_control_in()) +
                             ((int32_t)aparm.airspeed_min * 100);

    } else if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        // Landing airspeed target
        target_airspeed_cm = landing.get_target_airspeed_cm();

    } else {
        // Normal airspeed target
        target_airspeed_cm = aparm.airspeed_cruise_cm;
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (control_mode >= FLY_BY_WIRE_B && (aparm.min_gndspeed_cm > 0)) {
        int32_t min_gnd_target_airspeed = airspeed_measured_cm + groundspeed_undershoot;
        if (min_gnd_target_airspeed > target_airspeed_cm)
            target_airspeed_cm = min_gnd_target_airspeed;
    }

    // Bump up the target airspeed based on throttle nudging
    if (control_mode >= AUTO && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    // Apply airspeed limit
    if (target_airspeed_cm > (aparm.airspeed_max * 100))
        target_airspeed_cm = (aparm.airspeed_max * 100);

    // use the TECS view of the target airspeed for reporting, to take
    // account of the landing speed
    airspeed_error = SpdHgt_Controller->get_target_airspeed() - airspeed_measured_cm * 0.01f;
}

void Plane::calc_gndspeed_undershoot()
{
 	// Use the component of ground speed in the forward direction
	// This prevents flyaway if wind takes plane backwards
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
	    Vector2f gndVel = ahrs.groundspeed_vector();
		const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
		Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
		yawVect.normalize();
		float gndSpdFwd = yawVect * gndVel;
        groundspeed_undershoot = (aparm.min_gndspeed_cm > 0) ? (aparm.min_gndspeed_cm - gndSpdFwd*100) : 0;
    }
}

void Plane::update_loiter(uint16_t radius)
{
    if (radius <= 1) {
        // if radius is <=1 then use the general loiter radius. if it's small, use default
        radius = (abs(aparm.loiter_radius) <= 1) ? LOITER_RADIUS_DEFAULT : abs(aparm.loiter_radius);
        if (next_WP_loc.flags.loiter_ccw == 1) {
            loiter.direction = -1;
        } else {
            loiter.direction = (aparm.loiter_radius < 0) ? -1 : 1;
        }
    }

    if (loiter.start_time_ms != 0 &&
        quadplane.guided_mode_enabled()) {
        if (!auto_state.vtol_loiter) {
            auto_state.vtol_loiter = true;
            // reset loiter start time, so we don't consider the point
            // reached till we get much closer
            loiter.start_time_ms = 0;
            quadplane.guided_start();
        }
    } else if (loiter.start_time_ms == 0 &&
        control_mode == AUTO &&
        !auto_state.no_crosstrack &&
        get_distance(current_loc, next_WP_loc) > radius*3) {
        // if never reached loiter point and using crosstrack and somewhat far away from loiter point
        // navigate to it like in auto-mode for normal crosstrack behavior
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    } else {
        nav_controller->update_loiter(next_WP_loc, radius, loiter.direction);
    }

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() ||
            auto_state.wp_proportion > 1) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
            if (control_mode == GUIDED || control_mode == AVOID_ADSB) {
                // starting a loiter in GUIDED means we just reached the target point
                gcs_send_mission_item_reached_message(0);
            }
            if (quadplane.guided_mode_enabled()) {
                quadplane.guided_start();
            }
        }
    }
}

void Plane::update_loiter_ellipse()
{

    nav_controller->update_loiter_ellipse(home, ellipse.maxradius_cm, ellipse.minmaxratio, ellipse.azimuth_deg, ellipse.orientation);
}

void Plane::update_eight_plane()
{

    eight_in_R2.set_current_segment(eight_in_R2.aircraft_posR2center, eight_in_R2.aircraft_vel);
//
    hal.console->print("segment: ");
    hal.console->println(eight_in_R2.current_segment);


//
//    //nav_controller->update_loiter_3d(eight_in_S2.S2_loc, eight_in_S2.segments_ercv[eight_in_S2.current_segment], eight_in_S2.S2_radius_cm, eight_in_S2.segments_theta_r[eight_in_S2.current_segment], eight_in_S2.segments_orientation[eight_in_S2.current_segment], eight_in_S2.aircraft_posccenter, eight_in_S2.aircraft_vel, eight_in_S2.desired_loc);
    Vector3f current_ercv = eight_in_S2.current_ercv();
    int32_t current_theta_r = eight_in_S2.current_theta_r();
    int8_t current_orientation = eight_in_S2.current_orientation();

    switch(eight_in_R2.current_segment){
    case 0:
        nav_controller->update_loiter_ellipse(eight_in_R2.center_loc, const int32_t maxradius_cm, const float minmaxratio, const float psi, const int8_t orientation)
        update_eight_plane(eight_in_R2.center_loc, current_ercv, eight_in_S2.S2_radius_cm, current_theta_r, current_orientation, eight_in_S2.aircraft_posS2center, eight_in_S2.aircraft_vel, eight_in_S2.desired_loc);

        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    }

}



void Plane::update_loiter_3d()
{
    // code: Christoph Sieg
    //nav_controller->update_loiter_3d(S1_in_S2.S1_loc, S1_in_S2.S2_loc, S1_in_S2.S2_radius_cm, S1_in_S2.distance_cm, S1_in_S2.orientation, S1_in_S2.aircraft_posS2center, S1_in_S2.aircraft_vel, S1_in_S2.desired_loc);
//    hal.console->print("S2_loc: ");
//    hal.console->print(S1_in_S2.S2_loc.lat);
//    hal.console->print(", ");
//    hal.console->print(S1_in_S2.S2_loc.lng);
//    hal.console->print(", ");
//    hal.console->println(S1_in_S2.S2_loc.alt);
//    hal.console->print("ercv: ");
//    hal.console->print(S1_in_S2.ercv.x);
//    hal.console->print(", ");
//    hal.console->print(S1_in_S2.ercv.y);
//    hal.console->print(", ");
//    hal.console->println(S1_in_S2.ercv.z);
//    hal.console->print("S2radius: ");
//    hal.console->print(S1_in_S2.S2_radius_cm);
//    hal.console->print("theta_rho: ");
//    hal.console->print(S1_in_S2.theta_rho_deg);
//    hal.console->print("orientation: ");
//    hal.console->print(S1_in_S2.orientation);
    //int8_t array[2] = {0, 1};
    //hal.console->print("array: ");
    //hal.console->print(array[0]);
    nav_controller->update_loiter_3d(S1_in_S2.S2_loc, S1_in_S2.ercv, S1_in_S2.S2_radius_cm, S1_in_S2.theta_rho_deg, S1_in_S2.orientation, S1_in_S2.aircraft_posS2center, S1_in_S2.aircraft_vel, S1_in_S2.desired_loc);

    // code: Thomas Gehrmann
    // nav_controller->update_loiter_3d(home, intersection.circle_center, intersection.circle_radius, intersection.psi_plane, intersection.theta_plane, eight_sphere.omega, eight_sphere.sigma, intersection.distance_cm, loiter.direction, intersection.rot_matrix_pe, eight_sphere.segment, intersection.desired_loc);

//     hal.console->print(location_diff(home,intersection.circle_center).x);
//     hal.console->print(" ");
//     hal.console->println(location_diff(home,intersection.circle_center).x);
//     hal.console->print(intersection.circle_radius);
//     hal.console->print("psi: ");
//     hal.console->print(intersection.psi_plane);
//     hal.console->print("theta: ");
//     hal.console->print(intersection.theta_plane);
//     hal.console->print("omega: ");
//     hal.console->print(eight_sphere.omega);
//     hal.console->print("sigma: ");
//     hal.console->println(eight_sphere.sigma);
//     hal.console->print("distance: ");
//     hal.console->print(intersection.distance_cm);
//     hal.console->print("direction: ");
//     hal.console->print(loiter.direction);
//     hal.console->print("segment: ");
//     hal.console->println(eight_sphere.segment);
//     hal.console->print("desired_loc.alt: ");
//     hal.console->println(intersection.desired_loc.alt);

}

// code: Christoph Sieg
void Plane::update_eight_sphere() {

//    hal.console->print("aircraft_posS2center: ");
//    hal.console->print(eight_in_S2.aircraft_posS2center.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.aircraft_posS2center.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.aircraft_posS2center.z);
//
//    hal.console->print("rxaplanev: ");
//        hal.console->print(eight_in_S2.rxaplanev(eight_in_S2.aircraft_vel).x);
//        hal.console->print(", ");
//        hal.console->print(eight_in_S2.rxaplanev(eight_in_S2.aircraft_vel).y);
//        hal.console->print(", ");
//        hal.console->println(eight_in_S2.rxaplanev(eight_in_S2.aircraft_vel).z);
//
//    hal.console->print("aircraft_vel: ");
//    hal.console->print(eight_in_S2.aircraft_vel.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.aircraft_vel.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.aircraft_vel.z);

    hal.console->print("signs: ");
    hal.console->print(eight_in_S2.rxaplanev(eight_in_S2.aircraft_posS2center) * eight_in_S2.epsixv);
    hal.console->print(", ");
    hal.console->println(eight_in_S2.rxaplanev(eight_in_S2.aircraft_posS2center) * eight_in_S2.ethetaxv);

    hal.console->print("in_vicinity_of_crossing_point: ");
    hal.console->println(eight_in_S2.in_vicinity_of_crossing_point);

    hal.console->print("in_right_direction: ");
    hal.console->println(eight_in_S2.in_right_direction);

    hal.console->print("switch_to_next_segment: ");
    hal.console->println(eight_in_S2.switch_to_next_segment);

    eight_in_S2.set_current_segment(eight_in_S2.aircraft_posS2center, eight_in_S2.aircraft_vel);
//
    hal.console->print("segment: ");
    hal.console->println(eight_in_S2.current_segment);

//    hal.console->print("etg1c1v: ");
//    hal.console->print(eight_in_S2.etg1c1v.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.etg1c1v.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.etg1c1v.z);
//
//    hal.console->print("etc1g2v: ");
//    hal.console->print(eight_in_S2.etc1g2v.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.etc1g2v.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.etc1g2v.z);
//
//    hal.console->print("etg2c2v: ");
//    hal.console->print(eight_in_S2.etg2c2v.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.etg2c2v.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.etg2c2v.z);
//
//    hal.console->print("etc2g1v: ");
//    hal.console->print(eight_in_S2.etc2g1v.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.etc2g1v.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.etc2g1v.z);
//    hal.console->print("etg1c1v: ");
//
//
    hal.console->print("erc1v: ");
    hal.console->print(eight_in_S2.erc1v.x);
    hal.console->print(", ");
    hal.console->print(eight_in_S2.erc1v.y);
    hal.console->print(", ");
    hal.console->println(eight_in_S2.erc1v.z);

    hal.console->print("erc2v: ");
    hal.console->print(eight_in_S2.erc2v.x);
    hal.console->print(", ");
    hal.console->print(eight_in_S2.erc2v.y);
    hal.console->print(", ");
    hal.console->println(eight_in_S2.erc2v.z);

//    hal.console->print("erg1v: ");
//    hal.console->print(eight_in_S2.erg1v.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.erg1v.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.erg1v.z);
//
//    hal.console->print("erg2v: ");
//    hal.console->print(eight_in_S2.erg2v.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.erg2v.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.erg2v.z);
//
//
//    hal.console->print("trig of chi: ");
//    hal.console->print(eight_in_S2.cos_chihalf);
//    hal.console->println(eight_in_S2.sin_chihalf);


    //    // navigate along selected segment
//    hal.console->print("S2_loc: ");
//    hal.console->print(eight_in_S2.S2_loc.lat);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.S2_loc.lng);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.S2_loc.alt);
//
////    Vector3f array[3] = eight_in_S2.segments_ercv[0];
////    hal.console->print("current ercv: ");
////    hal.console->print(array[0].x);
//    hal.console->print(", ");
//    //hal.console->print(eight_in_S2.segments_ercv[eight_in_S2.current_segment].y);
//    hal.console->print(", ");
//    //hal.console->println(eight_in_S2.segments_ercv[eight_in_S2.current_segment].z);
//
//    hal.console->print("S2_radius_cm: ");
//    hal.console->println(eight_in_S2.S2_radius_cm);
//
//    hal.console->print("current theta_r: ");
//    //hal.console->print(eight_in_S2.segments_theta_r[eight_in_S2.current_segment]);
//
//    hal.console->print("current orientation: ");
//    //hal.console->print(eight_in_S2.segments_orientation[eight_in_S2.current_segment]);
//
//
//    //nav_controller->update_loiter_3d(eight_in_S2.S2_loc, eight_in_S2.segments_ercv[eight_in_S2.current_segment], eight_in_S2.S2_radius_cm, eight_in_S2.segments_theta_r[eight_in_S2.current_segment], eight_in_S2.segments_orientation[eight_in_S2.current_segment], eight_in_S2.aircraft_posccenter, eight_in_S2.aircraft_vel, eight_in_S2.desired_loc);
    Vector3f current_ercv = eight_in_S2.current_ercv();
    int32_t current_theta_r = eight_in_S2.current_theta_r();
    int8_t current_orientation = eight_in_S2.current_orientation();
//    hal.console->print("trigof chihalf: ");
//    hal.console->print(eight_in_S2.sin_theta_r);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.cos_theta_r);

//    hal.console->print("eight_in_S2.erg1v: ");
//    hal.console->print(eight_in_S2.erg1v.x);
//    hal.console->print(", ");
//    hal.console->print(eight_in_S2.erg1v.y);
//    hal.console->print(", ");
//    hal.console->println(eight_in_S2.erg1v.z);
//
    hal.console->print("eight_in_S2.current_ercv() :");
    hal.console->print(current_ercv.x);
    hal.console->print(", ");
    hal.console->print(current_ercv.y);
    hal.console->print(", ");
    hal.console->println(current_ercv.z);

//    hal.console->print("eight_in_S2.current_theta_r() :");
//    hal.console->println(current_theta_r);
//
//    hal.console->print("eight_in_S2.current_orientation() :");
//    hal.console->println(current_orientation);


nav_controller->update_loiter_3d(eight_in_S2.S2_loc, current_ercv, eight_in_S2.S2_radius_cm, current_theta_r, current_orientation, eight_in_S2.aircraft_posS2center, eight_in_S2.aircraft_vel, eight_in_S2.desired_loc);

}


// routine that switches between the four circle segments of which the figure-eight pattern consists
// code: Thomas Gehrmann, slight modifications: Christoph Sieg

//void Plane::update_eight_sphere()
//{
////  float arspd = ahrs.get_airspeed()->get_airspeed();
//    float delta_r = 0;
//    /*if(arspd > 30) {
//        delta_r = 0.5*(arspd - 30);
//    }*/
//// if the wind direction (azimuth omega) has changed, recalculate figure-eight pattern such that it is situated downwind
//    if(g.omega_wind != eight_sphere.omega_old) {
//        do_eight_sphere();
//    }
//
//    switch(eight_sphere.segment)
//    {
//    case 0: { //left turning circle
//        //hal.console->println("case 0");
//
//        nav_controller->update_loiter_3d(home, eight_sphere.circle_center_left, intersection.circle_radius, M_PI/2, eight_sphere.eta, eight_sphere.omega, eight_sphere.sigma, intersection.distance_cm, 1, eight_sphere.rot_matrix_left, eight_sphere.segment, intersection.desired_loc);
//             hal.console->print("desired_loc.alt: ");
//             hal.console->println(intersection.desired_loc.alt);
//
//        int32_t nav_bearing = nav_controller->nav_bearing_cd();
//        // the following code is reverting the effect of wrap_180_cd within nav_bearing_cd();
//        if(nav_bearing < 0) {nav_bearing = 36000 + nav_bearing;}
//        //hal.console->println("nav_bearing");
//        //hal.console->println(nav_bearing);
//        /*hal.console->println("radius");
//        hal.console->println(intersection.circle_radius);
//        hal.console->println("slope");
//        hal.console->println(eight_sphere.slope);
//        hal.console->println("distance");
//        hal.console->println(intersection.distance_cm);
//        hal.console->println("lat");
//        hal.console->println(eight_sphere.circle_center_left.lat);
//        hal.console->println("lng");
//        hal.console->println(eight_sphere.circle_center_left.lng);*/
//
//        // use 26850 instead of 27000 to overcome delay
//        // hal.console->println(nav_bearing);
//        if(nav_bearing >= 27000 + RadiansToCentiDegrees(eight_sphere.sector_angle)) {
//            eight_sphere.segment++;
//            eight_sphere.segment = eight_sphere.segment % 4;
//            hal.console->print("switching from segment 0 to ");
//            hal.console->println(eight_sphere.segment);
//
//        }
//        break;
//    }
//
//    case 1: {
//        //hal.console->println("case 1");
//        nav_controller->update_loiter_3d(home, home, intersection.sphere_radius_cm/100.0f+delta_r, eight_sphere.cross_angle, M_PI/2, eight_sphere.omega, eight_sphere.sigma, 0, 1, eight_sphere.rot_matrix_cross1, eight_sphere.segment, intersection.desired_loc);
//        hal.console->print("desired_loc.alt: ");
//        hal.console->println(intersection.desired_loc.alt);
//
//        int32_t nav_bearing = nav_controller->nav_bearing_cd();
//        //hal.console->println("nav_bearing");
//        //hal.console->println(nav_bearing);
//
//        if(nav_bearing >= RadiansToCentiDegrees(eight_sphere.arc_length_angle)) {
//            eight_sphere.segment++;
//            eight_sphere.segment = eight_sphere.segment % 4;
//            hal.console->print("switching from segment 1 to ");
//            hal.console->println(eight_sphere.segment);
//        }
//        break;
//    }
//
//    case 2: { // right turning circle
//        //hal.console->println("case 2");
//
//        nav_controller->update_loiter_3d(home, eight_sphere.circle_center_right, intersection.circle_radius, -M_PI/2, eight_sphere.eta, eight_sphere.omega, eight_sphere.sigma, intersection.distance_cm, -1, eight_sphere.rot_matrix_right, eight_sphere.segment, intersection.desired_loc);
//        hal.console->print("desired_loc.alt: ");
//        hal.console->println(intersection.desired_loc.alt);
//
//
//        int32_t nav_bearing = nav_controller->nav_bearing_cd();
//        // the following code is reverting the effect of wrap_180_cd within nav_bearing_cd();
//        if(nav_bearing < 0) {nav_bearing = 36000 + nav_bearing;}
//        //hal.console->println("nav_bearing");
//        //hal.console->println(nav_bearing);
//
//        // use 9150 instead of 9000 to overcome delay
//        // hal.console->println(nav_bearing);
//        if(nav_bearing <= 9000 - RadiansToCentiDegrees(eight_sphere.sector_angle) && nav_bearing > 0) {
//            eight_sphere.segment++;
//            eight_sphere.segment = eight_sphere.segment % 4;
//            hal.console->print("switching from segment 2 to ");
//            hal.console->println(eight_sphere.segment);
//        }
//        break;
//    }
//
//    case 3: {
//        //hal.console->println("case 3");
//        nav_controller->update_loiter_3d(home, home, intersection.sphere_radius_cm/100.0f+delta_r, -eight_sphere.cross_angle, M_PI/2, eight_sphere.omega, eight_sphere.sigma, 0, -1, eight_sphere.rot_matrix_cross2, eight_sphere.segment, intersection.desired_loc);
//        hal.console->print("desired_loc.alt: ");
//        hal.console->println(intersection.desired_loc.alt);
//
//
//        int32_t nav_bearing = nav_controller->nav_bearing_cd();
//        //hal.console->println("nav_bearing");
//        //hal.console->println(nav_bearing);
//
//        if(nav_bearing <= -RadiansToCentiDegrees(eight_sphere.arc_length_angle)) {
//            eight_sphere.segment++;
//            eight_sphere.segment = eight_sphere.segment % 4;
//            hal.console->print("switching from segment 3 to ");
//            hal.console->println(eight_sphere.segment);
//        }
//        break;
//    }
//
//    }
//
//}

/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
void Plane::update_cruise()
{
    if (!cruise_state.locked_heading &&
        channel_roll->get_control_in() == 0 &&
        rudder_input == 0 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        gps.ground_speed() >= 3 &&
        cruise_state.lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        cruise_state.lock_timer_ms = millis();
    }
    if (cruise_state.lock_timer_ms != 0 &&
        (millis() - cruise_state.lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        cruise_state.locked_heading = true;
        cruise_state.lock_timer_ms = 0;
        cruise_state.locked_heading_cd = gps.ground_course_cd();
        prev_WP_loc = current_loc;
    }
    if (cruise_state.locked_heading) {
        next_WP_loc = prev_WP_loc;
        // always look 1km ahead
        location_update(next_WP_loc,
                        cruise_state.locked_heading_cd*0.01f, 
                        get_distance(prev_WP_loc, current_loc) + 1000);
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }
}


/*
  handle speed and height control in FBWB or CRUISE mode. 
  In this mode the elevator is used to change target altitude. The
  throttle is used to change target airspeed or throttle
 */
void Plane::update_fbwb_speed_height(void)
{
    static float last_elevator_input;
    float elevator_input;
    elevator_input = channel_pitch->get_control_in() / 4500.0f;
    
    if (g.flybywire_elev_reverse) {
        elevator_input = -elevator_input;
    }
    
    change_target_altitude(g.flybywire_climb_rate * elevator_input * perf.delta_us_fast_loop * 0.0001f);
    
    if (is_zero(elevator_input) && !is_zero(last_elevator_input)) {
        // the user has just released the elevator, lock in
        // the current altitude
        set_target_altitude_current();
    }

    // check for FBWB altitude limit
    check_minimum_altitude();

    altitude_error_cm = calc_altitude_error_cm();
    
    last_elevator_input = elevator_input;
    
    calc_throttle();
    calc_nav_pitch();
}

/*
  calculate the turn angle for the next leg of the mission
 */
void Plane::setup_turn_angle(void)
{
    int32_t next_ground_course_cd = mission.get_next_ground_course_cd(-1);
    if (next_ground_course_cd == -1) {
        // the mission library can't determine a turn angle, assume 90 degrees
        auto_state.next_turn_angle = 90.0f;
    } else {
        // get the heading of the current leg
        int32_t ground_course_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);

        // work out the angle we need to turn through
        auto_state.next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
    }
}    

/*
  see if we have reached our loiter target
 */
bool Plane::reached_loiter_target(void)
{
    if (quadplane.in_vtol_auto()) {
        return auto_state.wp_distance < 3;
    }
    return nav_controller->reached_loiter_target();
}
    
