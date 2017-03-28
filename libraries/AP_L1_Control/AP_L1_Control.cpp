#include <AP_HAL/AP_HAL.h>
#include "AP_L1_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_L1_Control::var_info[] = {
    // @Param: PERIOD
    // @DisplayName: L1 control period
    // @Description: Period in seconds of L1 tracking loop. This parameter is the primary control for agressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default of 20 is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
    // @Units: seconds
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PERIOD",    0, AP_L1_Control, _L1_period, 20),

    // @Param: DAMPING
    // @DisplayName: L1 control damping ratio
    // @Description: Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
    // @Range: 0.6 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DAMPING",   1, AP_L1_Control, _L1_damping, 0.75f),

    // @Param: XTRACK_I
    // @DisplayName: L1 control crosstrack integrator gain
    // @Description: Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.
    // @Range: 0 0.1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XTRACK_I",   2, AP_L1_Control, _L1_xtrack_i_gain, 0.02),

    AP_GROUPEND
};

//Bank angle command based on angle between aircraft velocity vector and reference vector to path.
//S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
//Proceedings of the AIAA Guidance, Navigation and Control
//Conference, Aug 2004. AIAA-2004-4900.
//Modified to use PD control for circle tracking to enable loiter radius less than L1 length
//Modified to enable period and damping of guidance loop to be set explicitly
//Modified to provide explicit control over capture angle


/*
  Wrap AHRS yaw if in reverse - radians
 */
float AP_L1_Control::get_yaw()
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.yaw);
    }
    return _ahrs.yaw;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
float AP_L1_Control::get_yaw_sensor()
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t AP_L1_Control::nav_roll_cd(void) const
{
    float ret;
    ret = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * 0.101972f) * 100.0f); // 0.101972 = 1/9.81
    ret = constrain_float(ret, -9000, 9000);
    return ret;
}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_L1_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AP_L1_Control::nav_bearing_cd(void) const
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AP_L1_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t AP_L1_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float AP_L1_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    return MIN(wp_radius, _L1_dist);
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float AP_L1_Control::turn_distance(float wp_radius, float turn_angle) const
{
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

bool AP_L1_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */
void AP_L1_Control::_prevent_indecision(float &Nu)
{
    const float Nu_limit = 0.9f*M_PI;
    if (fabsf(Nu) > Nu_limit &&
        fabsf(_last_Nu) > Nu_limit &&
        labs(wrap_180_cd(_target_bearing_cd - get_yaw_sensor())) > 12000 &&
        Nu * _last_Nu < 0.0f) {
        // we are moving away from the target waypoint and pointing
        // away from the waypoint (not flying backwards). The sign
        // of Nu has also changed, which means we are
        // oscillating in our decision about which way to go
        Nu = _last_Nu;
    }
}

// update L1 control for waypoint navigation
void AP_L1_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP)
{

    struct Location _current_loc;
    float Nu;
    float xtrackVel;
    float ltrackVel;

    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1) {
        dt = 0.1;
    }
    _last_update_waypoint_us = now;

    // Calculate L1 gain required for specified damping
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    // Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update _target_bearing_cd
    _target_bearing_cd = get_bearing_cd(_current_loc, next_WP);

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/1/pipi
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

    // Calculate the NE position of WP B relative to WP A
    Vector2f AB = location_diff(prev_WP, next_WP);
    float AB_length = AB.length();

    // Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = location_diff(_current_loc, next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
    AB.normalize();

    // Calculate the NE position of the aircraft relative to WP A
    Vector2f A_air = location_diff(prev_WP, _current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = A_air % AB;

    //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference point
    //Otherwise do normal L1 guidance
    float WP_A_dist = A_air.length();
    float alongTrackDist = A_air * AB;
    if (WP_A_dist > _L1_dist && alongTrackDist/MAX(WP_A_dist, 1.0f) < -0.7071f)
    {
        //Calc Nu to fly To WP A
        Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft
        xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else if (alongTrackDist > AB_length + groundSpeed*3) {
        // we have passed point B by 3 seconds. Head towards B
        // Calc Nu to fly To WP B
        Vector2f B_air = location_diff(next_WP, _current_loc);
        Vector2f B_air_unit = (B_air).normalized(); // Unit vector from WP B to aircraft
        xtrackVel = _groundspeed_vector % (-B_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-B_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-B_air_unit.y , -B_air_unit.x); // bearing (radians) from AC to L1 point
    } else { //Calc Nu to fly along AB line

        //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
        xtrackVel = _groundspeed_vector % AB; // Velocity cross track
        ltrackVel = _groundspeed_vector * AB; // Velocity along track
        float Nu2 = atan2f(xtrackVel,ltrackVel);
        //Calculate Nu1 angle (Angle to L1 reference point)
        float sine_Nu1 = _crosstrack_error/MAX(_L1_dist, 0.1f);
        //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
        sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);
        float Nu1 = asinf(sine_Nu1);

        // compute integral error component to converge to a crosstrack of zero when traveling
        // straight but reset it when disabled or if it changes. That allows for much easier
        // tuning by having it re-converge each time it changes.
        if (_L1_xtrack_i_gain <= 0 || !is_equal(_L1_xtrack_i_gain.get(), _L1_xtrack_i_gain_prev)) {
            _L1_xtrack_i = 0;
            _L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
        } else if (fabsf(Nu1) < radians(5)) {
            _L1_xtrack_i += Nu1 * _L1_xtrack_i_gain * dt;

            // an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
            _L1_xtrack_i = constrain_float(_L1_xtrack_i, -0.1f, 0.1f);
        }

        // to converge to zero we must push Nu1 harder
        Nu1 += _L1_xtrack_i;

        Nu = Nu1 + Nu2;
        _nav_bearing = atan2f(AB.y, AB.x) + Nu1; // bearing (radians) from AC to L1 point
    }

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    //Limit Nu to +-pi
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    // Waypoint capture status is always false during waypoint following
    _WPcircle = false;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for loitering
void AP_L1_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
    struct Location _current_loc;

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    radius *= sq(_ahrs.get_EAS2TAS());

    // Calculate guidance gains used by PD loop (used during circle tracking)
    float omega = (6.2832f / _L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    //Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = MAX(_groundspeed_vector.length() , 1.0f);


    // update _target_bearing_cd
    _target_bearing_cd = get_bearing_cd(_current_loc, center_WP);


    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/pi
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

    //Calculate the NE position of the aircraft relative to WP A
    Vector2f A_air = location_diff(center_WP, _current_loc);

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_unit;
    if (A_air.length() > 0.1f) {
        A_air_unit = A_air.normalized();
    } else {
        if (_groundspeed_vector.length() < 0.1f) {
            A_air_unit = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            A_air_unit = _groundspeed_vector.normalized();
        }
    }

    //Calculate Nu to capture center_WP
    float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
    float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    //Calculate radial position and velocity errors
    float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
    float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle

    // keep crosstrack error for reporting
    _crosstrack_error = xtrackErrCirc;

    //Calculate PD control correction to circle waypoint_ahrs.roll
    float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

    //Calculate tangential velocity
    float velTangent = xtrackVelCap * float(loiter_direction);

    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
        latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
    }

    // Calculate centripetal acceleration demand
    float latAccDemCircCtr = velTangent * velTangent / MAX((0.5f * radius), (radius + xtrackErrCirc));

    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
    float latAccDemCirc = loiter_direction * (latAccDemCircPD + latAccDemCircCtr);

    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle
    if (xtrackErrCirc > 0.0f && loiter_direction * latAccDemCap < loiter_direction * latAccDemCirc) {
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else {
        _latAccDem = latAccDemCirc;
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians)from AC to L1 point
    }

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

void AP_L1_Control::update_loiter_ellipse(const struct Location &center_loc, const int32_t maxradius_cm, const float minmaxratio, const float psi, const int8_t orientation)
{
    struct Location _current_loc;

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    // maxradius_cm *= sq(_ahrs.get_EAS2TAS());

    // Calculate guidance gains used by PD loop (used during circle tracking)
    const float omega = (6.2832f / _L1_period);
    const float Kx = omega * omega;
    const float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    const float K_L1 = 4.0f * _L1_damping * _L1_damping;

    // get current position and velocity in NED frame
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }
        // position of aircraft relative to the center of the ellipse; vector components in meters
    const Vector3f posav(location_3d_diff_NED(center_loc, _current_loc));

    // lateral projection
    const Vector2f posalv(posav.x, posav.y);
    // update _target_bearing_cd
    _target_bearing_cd = get_bearing_cd(_current_loc, center_loc);

    // velocity of aircraft in NED coordinate system
    Vector3f velav;
    // only use if ahrs.have_inertial_nav() is true
    if (_ahrs.get_velocity_NED(velav)) {
    }
    else {Vector2f(velav.x, velav.y)=_ahrs.groundspeed_vector();
          /* if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
                  velav = gps.velocity().z;
              } else {
                  velav = -barometer.get_climb_rate();
              }; */
          velav.z = 0;
    }
    const Vector2f velalv(velav.x,velav.y);
    const float velal = MAX(velalv.length(), 1.0f);

    // unit vector pointing from the center to the  aircraft
    Vector2f erlv;
    erlv = posalv;
    if (erlv.length() > 0.1f) {
        erlv = posalv.normalized();
    } else {
        if (velalv.length() < 0.1f) {
            erlv = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            erlv = velalv.normalized();
        }
    }
    const float vela = velalv.length();

    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/pi
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * velal;



    //Calculate Nu to capture center_WP
    const float xtrackVelCap = erlv % velalv; // Velocity across line - perpendicular to radial inbound to WP
    const float ltrackVelCap = - (velalv * erlv); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    const float latAccDemCap = K_L1 * vela * vela / _L1_dist * sinf(Nu);

    // calculate desired position on ellipse with major and minor principal axes along unit vectors e1 and e2, respectively
    // for given position vector posalv(phia) = ra(cos(phia)e1 + cos(theta)sin(phia)e2) of the aircraft
    //
    //hal.console->println(maxradius_cm);
    const float cos_psi = cosf(psi);
    const float sin_psi = sinf(psi);
    // trigonometric functions of angle at which a circle has to be inclined in order to yield the ellipse
    // poselv(phi) = cos(phi)e1 + cos(theta)sin(phi)e2
    const float cos_theta = minmaxratio;
    const float sin_theta = sqrt(1.0f - minmaxratio);
    // unit vectors e1 and e2 into the direction of the major and minor principal axes, respectively
    const Vector2f e1(cos_psi,sin_psi);
    const Vector2f e2(-e1.y,e1.x);
    //hal.console->print(e1.x);
    //hal.console->print(" ");
    //hal.console->println(e1.y);
    // projections of the aircraft's position onto e1 and e2
    const float posal1 = posalv * e1;
    const float posal2 = posalv * e2;
    // determine parametrization (ra,phia) of the aircraft's position from lateral components
    // posal1 = ra cos(phia)
    // posal2 = ra cos(theta)sin(phia);
    // distance of the aircraft from the center of the ellipse in meter
    const float ra = sqrt(sq(posal1) + sq(1/cos_theta * posal2));
    // hal.console->println(ra);
    const float rho = ra - maxradius_cm/100.0f;
    // trigononometric functions of curve parameter phia at the aircraft's position
    const float cos_phia = posal1/ra;
    const float sin_phia = orientation * posal2/(ra * cos_theta);
    // first oder correction to curve parameter to approximate parameter at point of the ellipse closest to the aircraft's position
    const float dphi = - rho * sq(sin_theta) * sin_phia * cos_phia /(ra * (1 - sq(sin_theta * cos_phia)));
    const float cos_dphi = cosf(dphi);
    const float sin_dphi = sinf(dphi);
    // trigonometric functions of phi = phia + dphi, which is the first-order value of the curve parameter of the ellipse at the nearest point of the aircraft
    const float cos_phiapdphi = cos_phia * cos_dphi - sin_phia * sin_dphi;
    const float sin_phiapdphi = cos_phia * sin_dphi + sin_phia * cos_dphi;
    //hal.console->print(cos_phiapdphi);
    //hal.console->print(" ");
    //hal.console->println(sin_phiapdphi);
    // distance of the aircraft from the ellipse;
    const float dae = rho * cos_theta /sqrt(1 - sq(sin_theta * cos_phia));
    // position vector of point of the ellipse closest to the aircraft's position relative to center_loc
    const Vector2f poselv = Vector2f((e1 * cos_phiapdphi + e2 * cos_theta * sin_phiapdphi * orientation) * maxradius_cm/100.0f);
    // projections onto e1 and e2
    const float posel1 = poselv * e1;
    const float posel2 = poselv * e2;
    //hal.console->print(posel1);
    //hal.console->print(" ");
    //hal.console->println(posel2);
    const Vector2f telv = Vector2f(-e1 * sin_phiapdphi + e2 * cos_theta * cos_phiapdphi * orientation);
    const float telvnorm = telv.length();
    // unit tangent vector at point of the ellipse closest to the aircraft's position relative to center_loc
    const Vector2f etelv = telv / telvnorm;
    // unit outer normal vector at point of the ellipse closest to the aircraft's position relative to center_loc
    const Vector2f enelv(etelv.y * orientation, -etelv.x * orientation);
    // curvature at point of the ellipse closest to the aircraft's position relative to center_loc
    const float kappa = cos_theta /(ra * powf(telvnorm,3));
    //hal.console->print(cos_theta);
    //hal.console->print(" ");
    //hal.console->println(norm);

    //Calculate radial position and velocity errors
    const float xtrackVelCirc = enelv * velalv; // normal outbound velocity
    const float xtrackErrCirc = dae; // Radial distance from the loiter circle
    //hal.console->print("should be zero: ");
    //hal.console->println((posalv-poselv) * enelv - dae);
    const float ltrackVelCirc = etelv * velalv; // tangential velocity in the tangential direction (depends on orientation)

    // keep crosstrack error for reporting
    _crosstrack_error = xtrackErrCirc;

    //Calculate PD control correction to circle waypoint_ahrs.roll
    float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

    //Calculate tangential velocity
    float velTangent = ltrackVelCirc; // * float(orientation);

    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
        latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
    }

    // Calculate centripetal acceleration demand
    float latAccDemCircCtr = velTangent * velTangent * kappa;
    //hal.console->print(velTangent);
    //hal.console->print("kappa: ");
    //hal.console->println(1/kappa);

    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
    float latAccDemCirc = orientation * (latAccDemCircPD + latAccDemCircCtr);
    hal.console->print(latAccDemCircPD);
    hal.console->print(" ");
    hal.console->println(latAccDemCircCtr);

    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle
    if (xtrackErrCirc > maxradius_cm * (1-minmaxratio)/100.0f && orientation * latAccDemCap < orientation * latAccDemCirc) {
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = atan2f(-posalv.y , -posalv.x); // bearing (radians) from AC to L1 point
        hal.console->println("capture");
    } else {
        _latAccDem = latAccDemCirc;
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
        _nav_bearing = atan2f(-posalv.y , -posalv.x); // bearing (radians)from AC to L1 point
        hal.console->println("loiter_ellipse");
    }

    _data_is_stale = false; // status are correctly updated with current waypoint data


    // current location of the aircraft
}


void AP_L1_Control::update_loiter_3d(const struct Location &S2center, const struct Location &S1center, int32_t S1radius, int32_t D, int8_t orientation, int32_t &height)
{

    // current location of the aircraft
    struct Location _current_loc;

//    float theta_c = gamma_c - M_PI_2; // determine polar angle from inclination
//    float cos_theta_c = cosf(theta_c);
//    float sin_theta_c = sinf(theta_c);
//    float cos_psi_c = cosf(psi_c);
//    float sin_psi_c = sinf(psi_c);

    // unit vector pointing from anchor to center of the circle
    //Vector3f erc(sin_theta_c * cos_psi_c, sin_theta_c * sin_psi_c, -cos_theta_c);

    // circle radius in meter
    //float S1radius = S2radius/100  * sinf(theta_r);
    // distance from anchor to the center of the circle in meter
    //float D = S2radius/100 * cosf(theta_r);
    // position vector from center of the sphere to the center of the circle; components in meter
    // Vector3f posccv = erc * D / 100;
    // location of center of circle
    //struct Location _center_loc;
    //_center_loc = anchor;
    //location_offset(_center_loc, posccv.x, posccv.y);
    //_center_loc.alt = _center_loc.alt - 100 * posccv.z; // altitude in cm

    // Calculate guidance gains used by PD loop (used during circle tracking)
     float omega = (6.2832f / _L1_period);
     float Kx = omega * omega;
     float Kv = 2.0f * _L1_damping * omega;
     // Calculate L1 gain required for specified damping (used during waypoint capture)
     float K_L1 = 4.0f * _L1_damping * _L1_damping;

     Vector3f posccv(location_3d_diff_NED(S2center, S1center));
     Vector3f erc = posccv.normalized();
    // get current position and velocity in NED frame
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }
    // aircraft's position vector (direction of ideal (straight) tether) from anchor
    Vector3f posav(location_3d_diff_NED(S2center, _current_loc));
    // lateral projection
    Vector2f posalv(posav.x, posav.y);
    // update _target_bearing_cd
    _target_bearing_cd = get_bearing_cd(_current_loc, S1center);

    // track velocity in NED frame
    Vector3f velav;
    // only use if ahrs.have_inertial_nav() is true
    if (_ahrs.get_velocity_NED(velav)) {
    }
    else {Vector2f(velav.x, velav.y) =_ahrs.groundspeed_vector();
          /* if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
                  velav = gps.velocity().z;
              } else {
                  velav = -barometer.get_climb_rate();
              }; */
          velav.z = 0;
    }
    Vector2f velalv(velav.x,velav.y);
    float vela = MAX(velalv.length(), 1.0f);
     // unit tangent vector at point on circle that is closest to _current_loc
     Vector3f etv = (posav % posccv) * orientation;
     etv = etv.normalized();
     // lateral projection of the unit tangent vector
     Vector2f etlv(etv.x, etv.y);
     etlv = etlv.normalized();

     // outer unit normal (radial) vector at point on circle that is closest to _current_loc
     Vector3f env = (erc % etv) * orientation;
     env = env.normalized(); // renormalize in order to compensate for numerical inaccuracies
     // position vector of the point on circle that is closest to _current_loc
     Vector3f poscirclev = posccv + env * S1radius / 100.0f;
     Vector2f poscirclelv(poscirclev.x,poscirclev.y);
     // lateral renormalized projection of the unit normal vector; this is the radial vector of the point of the ellipse (the lateral projection of the circle) but NOT its normal vector;
     Vector2f erlv(env.x, env.y);
     if (erlv.length() > 0.1f) {
         erlv = erlv.normalized();
     } else {
         if (velalv.length() < 0.1f) {
             erlv = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
         } else {
             erlv = velalv.normalized();
         }
     }
     // outer unit normal vector of the point of the ellipse (the lateral projection of the circle);
     Vector2f enlv(etv.y, -etv.x);
     enlv = enlv.normalized() * orientation;

     // deviations of _current_loc from desired point
     // 1st approach: difference of the lengths of the direct lateral projections
     float xtrackErrCirc = posalv.length() - poscirclelv.length();
     // 2nd approach: projection of difference onto circle plane in direction env, then lateral projection of result
//     Vector3f posdeviationv = posav - poscirclev;
//     Vector2f posdeviationlv(posdeviationv.x,posdeviationv.y);
//     float posdeviationproj = env * posdeviationv;
//     float envproj = sqrt(env.x * env.x + env.y * env.y);
//     float xtrackErrCirc = posdeviationproj * envproj;
     // keep crosstrack error for reporting
     _crosstrack_error = xtrackErrCirc;


     // velocity component of velav in the direction of etv is desired
     // deviations is velav minus the component in the direction of etv
     // Vector3f veldeviationv = velav - etv * (velav * etv);     // lateral projection of velocity
     // lateral projection of velocity deviation
     // Vector2f veldeviationlv(veldeviationv.x,veldeviationv.y);


     Vector2f veldeviationlv(velav.x,velav.y);
     // lateral velocity component in the direction of the outer normal vector
     float xtrackVelCirc = veldeviationlv * enlv;

     // 0th approach: direct projection
     // velocity component of the aircraft tangential to the circle
     //float xtrackVelCap = velav * etv;
     // velocity component of the aircraft normal to the circle
     //float ltrackVelCap = - velav * env;
          // 1st approach: direct lateral projection of velocity vector
     // velocity component of the aircraft tangential to the circle
     float xtrackVelCap = velalv * etlv;
     // velocity component of the aircraft radial inbound
     float ltrackVelCap = - velalv * erlv;

     // 2nd approach: projection of velocity onto circle plane
     //Vector3f velapv = velav - erc * (velav * erc);
     // velocity component of the aircraft tangential to the circle
     //float xtrackVelCap = velapv * etv;
     // velocity component of the aircraft radial inbound
     //float ltrackVelCap = - velapv * env;


     //float ltrackVelCap = - env * velav;
     //ltrackVelCap = ltrackVelCap * sqrt(env.x * env.x + env.y * env.y);

      //xtrackVelCap = xtrackVelCap * sqrt(etv.x * etv.x + etv.y * etv.y);


//     float xtrackVelCap = erlv % velalv; // lateral velocity component perpendicular to lateral projection of position vector from circle center to aircraft's position
//     float ltrackVelCap = - erlv * velalv; // lateral velocity component towards the circle center
     float Nu = atan2f(xtrackVelCap,ltrackVelCap);
     _prevent_indecision(Nu);
     _last_Nu = Nu;
     Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2
     //Calculate lateral acceleration demand to capture center_WP (use L1 guidance law)
     float latAccDemCap = K_L1 * vela * vela / _L1_dist * sinf(Nu);


     // calculate lateral acceleration for following the ellipse (the lateral projection of the circle)
     // factor that contains the lateral curvature kappa_l and
     // lateral projection of the velocity component of the aircraft tangential to the circle;
     // can be obtained from components of the lateral unit normal vector
     // azimuth and polar angle can be obtained from erc = (sin_theta_c * cos_psi_c, sin_theta_c * sin_psi_c, -cos_theta_c);
     Vector2f e1(erc.x,erc.y);
     if (e1 == Vector2f(0,0)) {
         e1 = Vector2f(1,0);
     } else {
             e1 = e1.normalized();
     }
     Vector2f e2(-e1.y,e1.x);
     float cos_theta_c = - erc.z;
     float e1proj = e1 * enlv;
     float e2proj = e2 * enlv;
     float costhetac_over_sqrtfac = sqrt(cos_theta_c * cos_theta_c * e1proj * e1proj + e2proj * e2proj);
     // calculate lateral acceleration
     float latAccDemCircCtr = xtrackVelCap * xtrackVelCap * 100/S1radius * costhetac_over_sqrtfac;
//     Vector2f gspv =_ahrs.groundspeed_vector();
//     float tanvel = gspv * etlv;
//     hal.console->print("xtr: ");
//     hal.console->println(xtrackVelCap);
//     hal.console->print("gsp: ");
//     hal.console->println(tanvel);

     // calculate PD control correction to lateral acceleration
     // flight path outside desired circle -> positive correction
     // velocity pointing outwards -> positive correction
     float latAccDemCircPD = xtrackErrCirc * Kx + xtrackVelCirc * Kv;

     //Calculate tangential velocity
      float velTangent = xtrackVelCap * float(orientation);

      //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
      if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
          latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
      }


      float tetherErr =0;
     // sign of lateral acceleration corresponds to sign of roll angle
     // roll angle and hence acceleration is negative if orientation is positive
     float latAccDemCirc = orientation * (latAccDemCircPD + latAccDemCircCtr + tetherErr);


     // Perform switchover between 'capture' and 'circle' modes at the
     // point where the commands cross over to achieve a seamless transfer
     // Only fly 'capture' mode if outside the circle
     if (xtrackErrCirc > 0.0f && orientation * latAccDemCap < orientation * latAccDemCirc) {
         // capture mode
         _latAccDem = latAccDemCap;
         _WPcircle = false;
         _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
         _nav_bearing = 0;//atan2f(-erlv.y , -erlv.x); // bearing (radians) from AC to L1 point
         // height of center point of circle
         height = - 100 * posccv.z;
     } else {
         // loiter
         _latAccDem = latAccDemCirc;
         _WPcircle = true;
         _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
         _nav_bearing = atan2f(-erlv.y , -erlv.x); // bearing (radians)from AC to L1 point
         // height of desired point of circle
         height = - 100 * poscirclev.z;

         // hal.console->print("height demanded: ");
         // hal.console->println(height);
     }

}

void AP_L1_Control::update_loiter_3d(const struct Location &anchor, const struct Location &center_WP, float radius, float psi, float theta, float w, float sigma, int32_t dist, int8_t loiter_direction, Matrix3f M_pe, int32_t segment, struct Location &desired_loc)
{
    struct Location _current_loc;

    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float cos_psi = cosf(psi);
    float sin_psi = sinf(psi);
    float cos_w = cosf(w);
    float sin_w = sinf(w);
    float cos_sigma = cosf(sigma);
    float sin_sigma = sinf(sigma);

    // Calculate guidance gains used by PD loop (used during circle tracking)
    float omega = (6.2832f / _L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    // Get current position and velocity
    _ahrs.get_position(_current_loc);

    //position vector or rather tether
    Vector3f _position_vec_ef = location_3d_diff_NED(anchor, _current_loc);

    // track velocity in earth frame
    Vector3f _track_vel_ef;
    _ahrs.get_velocity_NED(_track_vel_ef);

    Vector2f _groundspeed_vector;
    _groundspeed_vector.x = _track_vel_ef.x;
    _groundspeed_vector.y = _track_vel_ef.y;
    float groundSpeed = MAX(_groundspeed_vector.length() , 1.0f);

    Vector3f _track_vel_pf = M_pe * _track_vel_ef; // track velocity in plane frame
    Vector2f _track_vel; // projection of track velocity in xy plane of plane frame
    _track_vel.x = _track_vel_pf.x;
    _track_vel.y = _track_vel_pf.y;

    // Calculate the NED position of the aircraft relative to WP A
    Vector3f A_air_ef = location_3d_diff_NED(center_WP, _current_loc);
    Vector3f A_air_pf = M_pe * A_air_ef; // position of the aircraft in the plane frame

    Vector2f A_air; // projection of A_air_pf in xy plane of plane frame
    A_air.x = A_air_pf.x;
    A_air.y = A_air_pf.y;

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_unit;
    if (A_air.length() > 0.1f) {
        A_air_unit = A_air.normalized();
    } else {
        if (_track_vel.length() < 0.1f) {
            A_air_unit = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            A_air_unit = _track_vel.normalized();
        }
    }

    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/pi
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

    /*//Calculate the NE position of the aircraft relative to WP A
    Vector2f A_air_ef_2d = location_diff(center_WP, _current_loc);

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_ef_2d_unit;
    if (A_air.length() > 0.1f) {
        A_air_ef_2d_unit = A_air_ef_2d.normalized();
    } else {
        if (_groundspeed_vector.length() < 0.1f) {
            A_air_ef_2d_unit = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            A_air_ef_2d_unit = _groundspeed_vector.normalized();
        }
    }

    // update _target_bearing_cd

    _target_bearing_cd = get_bearing_cd(center_WP, _current_loc) - 100 * degrees(orientation);

    float cos_bearing = cosf(radians(_target_bearing_cd/100.0f));
    float sin_bearing = sinf(radians(_target_bearing_cd/100.0f));

    float bearing = radians(_target_bearing_cd/100.0f);
    float beta = atanf(cos_eta*cos_eta*tanf(bearing - M_PI/2));

    //Calculate Nu to capture center_WP
    float xtrackVelCap = (A_air_ef_2d_unit % _groundspeed_vector)* cosf(beta); // Velocity across line - perpendicular to radial inbound to WP
    float ltrackVelCap = - (_groundspeed_vector * A_air_ef_2d_unit)* cosf(beta); // Velocity along line - radial to ellipse
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    //hal.console->println("beta");
    //hal.console->println(beta);
     */

    //hal.console->println("A Air x");
    //hal.console->println(A_air_unit.x);
    //hal.console->println("A Air y");
    //hal.console->println(A_air_unit.y);

    Vector3f Height_ef;         // calculate height of position on circle
    Height_ef.x = radius * A_air_unit.x;
    Height_ef.y = radius * A_air_unit.y;
    Height_ef.z = -dist/100.0f;
    Height_ef = M_pe.transposed() * Height_ef;

    //Calculate Nu to capture center_WP
    float xtrackVelCap = A_air_unit % _track_vel; // Velocity across line - perpendicular to radial inbound to WP
    float ltrackVelCap = -(_track_vel * A_air_unit); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    /*hal.console->println("groundspeed");
    hal.console->println(groundSpeed);
    hal.console->println("ltrackVelCap");
    hal.console->println(ltrackVelCap);
    hal.console->println("xtrackVelCap");
    hal.console->println(xtrackVelCap);*/

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    //hal.console->println("Nu");
    //hal.console->println(Nu);

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //Calculate errors for controlling
    //Calculate radial position and velocity errors and tether tension error
    //float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
    //float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle
    //float xtrackErrCirc = A_air_ef_2d.length() - radius*cos_eta/sqrtf(1-sin_eta*sin_eta*sin_bearing*sin_bearing);    // distance from circle projection (ellipse)
    //float v_unit_ef_x = loiter_direction*(-cos_eta*cos_orient*A_air_unit.y - sin_orient*A_air_unit.x); //M_ep * tangentialvector
    //float v_unit_ef_y = loiter_direction*(-cos_eta*sin_orient*A_air_unit.y + cos_orient*A_air_unit.x);
    float v_unit_ef_x = loiter_direction*(- A_air_unit.y*(cos_w*cos_sigma*cos_theta*cos_psi - sin_w*cos_theta*sin_psi - cos_w*sin_sigma*sin_theta)
                                          + A_air_unit.x*(-cos_w*cos_sigma*sin_psi - sin_w*cos_psi));   //M_ep * tangent vector
    float v_unit_ef_y = loiter_direction*(- A_air_unit.y*(sin_w*cos_sigma*cos_theta*cos_psi + cos_w*cos_theta*sin_psi - sin_w*sin_sigma*sin_theta)
                                          + A_air_unit.x*(-sin_w*cos_sigma*sin_psi + cos_w*cos_psi));
    float chi = atan2f(v_unit_ef_y, v_unit_ef_x);
    float sin_chi = sinf(chi);
    float cos_chi = cosf(chi);

    // outer normal vector in the lateral plane
    Vector2f v_n_lat(v_unit_ef_y, -v_unit_ef_x);
    // position vector of the aircraft relative to the center of the circle in the lateral plane
    Vector2f v_air_lat(A_air_ef.x, A_air_ef.y);


    // desired position vector on the circle relative to the center of the circle
    Vector3f radius_vec_pf;
    radius_vec_pf.x = radius * A_air_unit.x;
    radius_vec_pf.y = radius * A_air_unit.y;
    radius_vec_pf.z = 0;
    //float delta_height = dist/100.0f * cos_sigma*cos_theta - A_air_ef.z + Height_ef.z;  // cos_sigma*cos_theta = z component of normal_vec
    //Vector3f A_air_diff_pf = A_air_pf - radius_vec_pf;

    // desired position vector on the circle relative to the center of the circle
    Vector3f v_ellipse_ef = M_pe.transposed()*radius_vec_pf;
    // desired lateral position vector on the lateral projection of the circle (the ellipse) relative to the center of the ellipse
    Vector2f v_ellipse_lat(v_ellipse_ef.x, v_ellipse_ef.y);
    // vector of the lateral deviation from current and desired position
    Vector2f A_diff_lat = v_air_lat - v_ellipse_lat;

    // sign of the deviation: +1 if aircraft is outside the circle, -1 if aircraft is inside the circle
    float sgn = v_n_lat * A_diff_lat;
    sgn = sgn/fabsf(sgn);



    float xtrackVelCirc = - sin_chi*_track_vel_ef.x + cos_chi*_track_vel_ef.y; // y Komponente von _track_vel_ef im bahnfesten Koordinatensystem
    //float xtrackVelCirc = -ltrackVelCap * cos_eta; // projection to xy plane of radial outbound velocity - reuse previous radial inbound velocity
//    int8_t sign;
//    //if((-sin_chi*cos_eta*cos_orient + cos_chi*cos_eta*sin_orient)*A_air_diff_pf.x +(sin_chi*sin_orient+cos_chi*cos_orient)*A_air_diff_pf.y +(-sin_chi*sin_eta*cos_orient+cos_chi*sin_eta*sin_orient)*A_air_diff_pf.z < 0) sign = -1;  //y componet of a_air_diff_kf
//    if (-sin_chi*(A_air_diff_pf.x * (cos_w*cos_sigma*cos_theta*cos_psi - sin_w*cos_theta*sin_psi - cos_w*sin_sigma*sin_theta)
//              + A_air_diff_pf.y * (-cos_w*cos_sigma*sin_psi - sin_w*cos_psi)
//              + A_air_diff_pf.z * (cos_w*cos_sigma*sin_theta*cos_psi - sin_w*sin_theta*sin_psi + cos_w*sin_sigma*cos_theta))
//
//      +cos_chi*(A_air_diff_pf.x * (sin_w*cos_sigma*cos_theta*cos_psi + cos_w*cos_theta*sin_psi - sin_w*sin_sigma*sin_theta)
//              + A_air_diff_pf.y * (-sin_w*cos_sigma*sin_psi + cos_w*cos_psi)
//              + A_air_diff_pf.z * (sin_w*cos_sigma*sin_theta*cos_psi + cos_w*sin_theta*sin_psi + sin_w*sin_sigma*cos_theta))
//              < 0) //y Komponente of A_air_diff im bahnfesten KS
//    {
//      sign = -1;
//    } else sign = 1;
//    //float xtrackErrCirc2 = sign * sqrtf(A_air_diff_pf.length()*A_air_diff_pf.length() - delta_height*delta_height); // distance from circle in xy plane
    float xtrackErrCirc = -sgn * A_diff_lat.length();


    float spring_const;
    float tether_length_demand;
    if (_position_vec_ef.length() > 390) {
        spring_const = 2;
    } else {
        spring_const = 0;
    }
    // float airspeed = _ahrs.get_airspeed()->get_airspeed();
    tether_length_demand = 400;// + 0.2*(airspeed - 30);


    Vector3f tether_tension = _position_vec_ef.normalized()*(spring_const*(tether_length_demand - 390)*4/7.785);
    float tetherErr = - sin_chi*tether_tension.x + cos_chi*tether_tension.y;

    //end of error calculation
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    /*hal.console->print("tether_length_demand: ");
    hal.console->println(tether_length_demand);
    hal.console->print("xtrackVelCirc: ");
    hal.console->println(-loiter_direction*xtrackVelCirc);
    hal.console->print("A_air_diff_pf.length() - delta height: ");
    hal.console->println(A_air_diff_pf.length() - delta_height);
    hal.console->print("delta_height angle stuff: ");
    hal.console->println(sin_sigma*sin_theta*sin_psi + cos_sigma*cos_theta);*/
    //hal.console->println(radius*cos_eta/sqrtf(1-sin_eta*sin_eta*sin_bearing*sin_bearing));

    // keep crosstrack error for reporting
   _crosstrack_error = xtrackErrCirc;


    //Calculate PD control correction to circle waypoint_ahrs.roll
    float latAccDemCircPD = -(xtrackErrCirc * Kx + xtrackVelCirc * Kv);

    //Calculate tangential velocity
    float velTangent = xtrackVelCap * float(loiter_direction);

    //hal.console->print("veltang: ");
    //hal.console->println(velTangent);
    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    //if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
    //    latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
    //}

    // Calculate centripetal acceleration demand
    //float latAccDemCircCtr = velTangent * velTangent / MAX((0.5f * radius), (radius + xtrackErrCirc));
    //if(cos_eta < 0.05) cos_eta = 0.05;
    //float powerbasis = (cos_bearing*cos_bearing + cos_eta*cos_eta*cos_eta*cos_eta*sin_bearing*sin_bearing)/(1 - sin_eta*sin_eta*sin_bearing*sin_bearing);
    //float latAccDemCircCtr = velTangent * velTangent / MAX(0.5 * radius/cos_eta*sqrtf(pow(powerbasis,3)), ( radius/cos_eta*sqrtf(pow(powerbasis,3)) + xtrackErrCirc));
    //float latAccDemCircCtr = velTangent * velTangent / MAX(0.5 * radius, radius + xtrackErrCirc);
    //float latAccDemCircCtr = velTangent * velTangent / MAX(0.5 * radius, radius) * cos_clear / sqrtf(A_air_unit.y*A_air_unit.y*cos_eta*cos_eta + A_air_unit.x*A_air_unit.x); // y Komponente von a im bahnfesten Koordinatensystem
    float latAccDemCircCtr;
    if (segment % 2 == 0) {
        latAccDemCircCtr = velTangent * velTangent / MAX(0.5 * radius, radius)
            * cos_sigma*cos_theta / sqrtf(A_air_unit.y*A_air_unit.y*cos_theta*cos_theta + (-A_air_unit.y*sin_sigma*sin_theta + A_air_unit.x*cos_sigma)*(-A_air_unit.y*sin_sigma*sin_theta + A_air_unit.x*cos_sigma));
    }
    else {
        latAccDemCircCtr = velTangent * velTangent / MAX(0.5 * radius, radius)
                    * (-sin_sigma)*cos_psi / sqrtf(A_air_unit.x*A_air_unit.x*cos_psi*cos_psi + (A_air_unit.x*cos_sigma*sin_psi - A_air_unit.y*sin_sigma)*(A_air_unit.x*cos_sigma*sin_psi - A_air_unit.y*sin_sigma));
    }

    // correction for latAccDemCircCtr because of tether tension
/*    float latAccDemTether;
    if(_position_vec_ef.length() > 450) {
        latAccDemTether = 0.4/6*3*(_position_vec_ef.length()-450)*(A_air_ef*_position_vec_ef)/(A_air_ef.length()*_position_vec_ef.length());
    } else {
        latAccDemTether = 0;
    }
    hal.console->print("product air and pos: ");
    hal.console->println(A_air_ef*_position_vec_ef/(A_air_ef.length()*_position_vec_ef.length()));
*/
    //hal.console->print("a_lat: ");
    //hal.console->println(loiter_direction*latAccDemCircCtr);
    //hal.console->println("aProj");
    //hal.console->println(cos_eta / sqrtf(A_air_unit.y*A_air_unit.y*cos_eta*cos_eta + A_air_unit.x*A_air_unit.x));
    //hal.console->println("radius");
    //hal.console->println(radius);


    //hal.console->println("Kruemmungsradius");
    //hal.console->println(radius/cos_eta*sqrtf(pow(powerbasis,3)));

    //_nav_bearing = wrap_2PI(atan2f(A_air_unit.y , A_air_unit.x));


    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
    //float latAccDemCirc = loiter_direction * (latAccDemCircPD + latAccDemCircCtr) * sinf(acosf(-cosf(_nav_bearing)*sin_eta)); // projection to xy plane
    float latAccDemCirc = (latAccDemCircPD + loiter_direction * latAccDemCircCtr + tetherErr); // +tetherErr instead of -tetherErr because tethertension is showing from anchor to current position
//    float latAccDemCirc = (latAccDemCircPD + loiter_direction * latAccDemCircCtr);

    hal.console->print("latAccDemCirc: ");
    hal.console->println(loiter_direction*latAccDemCirc);
    hal.console->print("latAccDemCap: ");
    hal.console->println(latAccDemCap);

    // hal.console->println(sinf(acosf(-cosf(_nav_bearing)*sinf(M_PI/9))));
    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle
    if (xtrackErrCirc > 0.0f && loiter_direction * latAccDemCap < loiter_direction * latAccDemCirc && false) {
        desired_loc = center_WP;
        // desired_loc.alt = dist * cos_sigma*cos_theta;
        //height = dist * cos_sigma*cos_theta; //center_WP.alt - home.alt ;
        //desired_loc = center_WP;
        //hal.console->println("height cap");
        //hal.console->println(height);
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = 0;
        hal.console->println("capture");
        //hal.console->println("nav_bearing");
        //hal.console->println(_nav_bearing);
        //hal.console->println("latAccDemCap");
        //hal.console->println(_latAccDem);
    } else {
//        height = -100*(Height_ef.z);
        // determine the location of the desired point on the circle
        desired_loc = center_WP;
        location_offset(desired_loc, v_ellipse_ef.x, v_ellipse_ef.y);
        //desired_loc.alt = dist * cos_sigma*cos_theta - 100.0f * v_ellipse_ef.z;
        desired_loc.alt = center_WP.alt - 100.0f * v_ellipse_ef.z;
        //hal.console->print("height demanded: ");
        //hal.console->println(height);
        _latAccDem = latAccDemCirc; //-9.81/_position_vec_ef.z*sqrtf(_position_vec_ef.length_squared()-_position_vec_ef.z*_position_vec_ef.z);
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
        _nav_bearing = wrap_2PI(atan2f(A_air_unit.y , A_air_unit.x)); // bearing (radians)from AC to Minimum point of circle
        hal.console->println("loiter_3d");
        //hal.console->print("pitch: ");
        //hal.console->println(_ahrs.pitch_sensor/100.0f);
        //hal.console->println(nav_bearing_cd());
        //hal.console->println("latAccDemCirc");
        //hal.console->println(_latAccDem);
    }
    // determine the location of the desired point on the circle
//   fstream f;
//    uint32_t now = AP_HAL::micros();
//    f.open("GPS3D.txt", ios::out | ios::app);
//    f << now << " " << Height_ef.x << " " << Height_ef.y  << " "
//      << _position_vec_ef.x << " "<< _position_vec_ef.y << " " <<  -_position_vec_ef.z << " "<< A_air_ef.z  << " " << -Height_ef.z << " "
//    << _nav_bearing << " " << _ahrs.roll_sensor << " " << _ahrs.pitch_sensor << " " << _ahrs.yaw_sensor << " " << _latAccDem << " "
//    << xtrackVelCap << " " << ltrackVelCap << " " << A_air_diff_pf.x << " " << A_air_diff_pf.y << " " << A_air_diff_pf.z << " " << xtrackErrCirc << " "
//    << xtrackVelCirc << " " << _track_vel_ef.x << " " << _track_vel_ef.y << " " << _track_vel_ef.z << " " << segment << " " << _roll_dem << " ";
//    f.close();
}

// update L1 control for heading hold navigation
void AP_L1_Control::update_heading_hold(int32_t navigation_heading_cd)
{
    // Calculate normalised frequency for tracking loop
    const float omegaA = 4.4428f/_L1_period; // sqrt(2)*pi/period
    // Calculate additional damping gain

    int32_t Nu_cd;
    float Nu;

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = wrap_180_cd(navigation_heading_cd);
    _nav_bearing = radians(navigation_heading_cd * 0.01f);

    Nu_cd = _target_bearing_cd - wrap_180_cd(_ahrs.yaw_sensor);
    Nu_cd = wrap_180_cd(Nu_cd);
    Nu = radians(Nu_cd * 0.01f);

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();

    // Calculate time varying control parameters
    _L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
    float VomegaA = groundSpeed * omegaA;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _crosstrack_error = 0;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    // Limit Nu to +-pi
    Nu = constrain_float(Nu, -M_PI_2, M_PI_2);
    _latAccDem = 2.0f*sinf(Nu)*VomegaA;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for level flight on current heading
void AP_L1_Control::update_level_flight(void)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = _ahrs.yaw_sensor;
    _nav_bearing = _ahrs.yaw;
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
