/*
   Lead developer: Andrew Tridgell & Tom Pittenger

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See http://dev.ardupilot.com for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Menu/AP_Menu.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_GPS/AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro/AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass/AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC/AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_InertialSensor/AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RangeFinder/AP_RangeFinder.h>     // Range finder library
#include <Filter/Filter.h>                     // Filter library
#include <AP_Buffer/AP_Buffer.h>      // APM FIFO Buffer
#include <AP_Relay/AP_Relay.h>       // APM relay
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Stats/AP_Stats.h>     // statistics library
#include <AP_Beacon/AP_Beacon.h>

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <APM_Control/APM_Control.h>
#include <APM_Control/AP_AutoTune.h>
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_SerialManager/AP_SerialManager.h>   // Serial manager library
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination/AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <DataFlash/DataFlash.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler

#include <AP_Navigation/AP_Navigation.h>
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Mission/AP_Mission.h>     // Mission command library

#include <AP_Soaring/AP_Soaring.h>
#include <AP_Notify/AP_Notify.h>      // Notify library
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library

#include <AP_Arming/AP_Arming.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>

#include <AP_Rally/AP_Rally.h>

#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library
#include <AP_RSSI/AP_RSSI.h>                   // RSSI Library
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Button/AP_Button.h>
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Landing/AP_Landing.h>

#include "GCS_Mavlink.h"
#include "GCS_Plane.h"
#include "quadplane.h"
#include "tuning.h"

// Configuration
#include "config.h"

// Local modules
#include "defines.h"

#include "Parameters.h"
#include "avoidance_adsb.h"
#include "AP_Arming.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif



/*
  a plane specific AP_AdvancedFailsafe class
 */
class AP_AdvancedFailsafe_Plane : public AP_AdvancedFailsafe
{
public:
    AP_AdvancedFailsafe_Plane(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps, const RCMapper &_rcmap);

    // called to set all outputs to termination state
    void terminate_vehicle(void);
    
protected:
    // setup failsafe values for if FMU firmware stops running
    void setup_IO_failsafe(void);

    // return the AFS mapped control mode
    enum control_mode afs_mode(void);
};

/*
  main APM:Plane class
 */
class Plane : public AP_HAL::HAL::Callbacks {
public:
    friend class GCS_MAVLINK_Plane;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Plane;
    friend class QuadPlane;
    friend class AP_Tuning_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    friend class AP_Avoidance_Plane;
    friend class GCS_Plane;

    Plane(void);

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    // key aircraft parameters passed to multiple libraries
    AP_Vehicle::FixedWing aparm;
    AP_HAL::BetterStream* cliSerial;

    // Global parameters are all contained within the 'g' and 'g2' classes.
    Parameters g;
    ParametersG2 g2;

    // main loop scheduler
    AP_Scheduler scheduler;
 
    // mapping between input channels
    RCMapper rcmap;

    // board specific config
    AP_BoardConfig BoardConfig;

    // primary input channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_rudder;

    // notification object for LEDs, buzzers etc (parameter set to false disables external leds)
    AP_Notify notify;

    DataFlash_Class DataFlash;

    // has a log download started?
    bool in_log_download;

    // scaled roll limit based on pitch
    int32_t roll_limit_cd;
    int32_t pitch_limit_min_cd;

    // Sensors
    AP_GPS gps;

    // flight modes convenience array
    AP_Int8 *flight_modes = &g.flight_mode1;

    AP_Baro barometer;
    Compass compass;

    AP_InertialSensor ins;

    RangeFinder rangefinder {serial_manager, ROTATION_PITCH_270};

    AP_Vehicle::FixedWing::Rangefinder_State rangefinder_state;

    AP_RPM rpm_sensor;
    
// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
    NavEKF2 EKF2{&ahrs, barometer, rangefinder};
    NavEKF3 EKF3{&ahrs, barometer, rangefinder};
    AP_AHRS_NavEKF ahrs {ins, barometer, gps, rangefinder, EKF2, EKF3};
#else
    AP_AHRS_DCM ahrs {ins, barometer, gps};
#endif

    AP_L1_Control L1_controller {ahrs};
    AP_TECS TECS_controller {ahrs, aparm, landing, g2.soaring_controller};

    // Attitude to servo controllers
    AP_RollController  rollController {ahrs, aparm, DataFlash};
    AP_PitchController pitchController {ahrs, aparm, DataFlash};
    AP_YawController   yawController {ahrs, aparm};
    AP_SteerController steerController {ahrs};

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // Training mode
    bool training_manual_roll;  // user has manual roll control
    bool training_manual_pitch; // user has manual pitch control

    /*
      keep steering and rudder control separated until we update servos,
      to allow for a separate wheel servo from rudder servo
    */
    struct {
        bool ground_steering; // are we doing ground steering?
        int16_t steering; // value for nose/tail wheel
        int16_t rudder;   // value for rudder
    } steering_control;

    // should throttle be pass-thru in guided?
    bool guided_throttle_passthru;

    // are we doing calibration? This is used to allow heartbeat to
    // external failsafe boards during baro and airspeed calibration
    bool in_calibration;


    // GCS selection
    AP_SerialManager serial_manager;
    GCS_Plane _gcs; // avoid using this; use gcs()
    GCS_Plane &gcs() { return _gcs; }

    // selected navigation controller
    AP_Navigation *nav_controller = &L1_controller;

    // selected navigation controller
    AP_SpdHgtControl *SpdHgt_Controller = &TECS_controller;

    // Relay
    AP_Relay relay;

    // handle servo and relay events
    AP_ServoRelayEvents ServoRelayEvents {relay};

    // Camera
#if CAMERA == ENABLED
    AP_Camera camera {&relay};
#endif

#if OPTFLOW == ENABLED
    // Optical flow sensor
    OpticalFlow optflow{ahrs};
#endif

    // Rally Ponints
    AP_Rally rally {ahrs};
    
    // RSSI 
    AP_RSSI rssi;      

    // remember if USB is connected, so we can adjust baud rate
    bool usb_connected;

    // This is the state of the flight control system
    // There are multiple states defined such as MANUAL, FBW-A, AUTO
    enum FlightMode control_mode = INITIALISING;
    mode_reason_t control_mode_reason = MODE_REASON_UNKNOWN;
    enum FlightMode previous_mode = INITIALISING;
    mode_reason_t previous_mode_reason = MODE_REASON_UNKNOWN;

    // Used to maintain the state of the previous control switch position
    // This is set to 254 when we need to re-read the switch
    uint8_t oldSwitchPosition = 254;

    // This is used to enable the inverted flight feature
    bool inverted_flight;

    // This is used to enable the PX4IO override for testing
    bool px4io_override_enabled;

    struct {
        // These are trim values used for elevon control
        // For elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are
        // equivalent aileron and elevator, not left and right elevon
        uint16_t trim1;
        uint16_t trim2;
        // These are used in the calculation of elevon1_trim and elevon2_trim
        uint16_t ch1_temp;
        uint16_t ch2_temp;
    } elevon { 1500, 1500, 1500, 1500 };

    // Failsafe
    struct {
        // Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
        // RC receiver should be set up to output a low throttle value when signal is lost
        uint8_t ch3_failsafe:1;

        // has the saved mode for failsafe been set?
        uint8_t saved_mode_set:1;

        // flag to hold whether battery low voltage threshold has been breached
        uint8_t low_battery:1;

        // true if an adsb related failsafe has occurred
        uint8_t adsb:1;

        // saved flight mode
        enum FlightMode saved_mode;

        // A tracking variable for type of failsafe active
        // Used for failsafe based on loss of RC signal or GCS signal
        int16_t state;

        // number of low ch3 values
        uint8_t ch3_counter;

        // the time when the last HEARTBEAT message arrived from a GCS
        uint32_t last_heartbeat_ms;
        
        // A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
        uint32_t ch3_timer_ms;
        
        uint32_t last_valid_rc_ms;

        //keeps track of the last valid rc as it relates to the AFS system
        //Does not count rc inputs as valid if the standard failsafe is on
        uint32_t AFS_last_valid_rc_ms;
    } failsafe;

    // A counter used to count down valid gps fixes to allow the gps estimate to settle
    // before recording our home position (and executing a ground start if we booted with an air start)
    uint8_t ground_start_count = 5;

    // true if we have a position estimate from AHRS
    bool have_position;

    // Airspeed
    // The calculated airspeed to use in FBW-B.  Also used in higher modes for insuring min ground speed is met.
    // Also used for flap deployment criteria.  Centimeters per second.
    int32_t target_airspeed_cm;

    // The difference between current and desired airspeed.  Used in the pitch controller.  Meters per second.
    float airspeed_error;

    // An amount that the airspeed should be increased in auto modes based on the user positioning the
    // throttle stick in the top half of the range.  Centimeters per second.
    int16_t airspeed_nudge_cm;

    // Similar to airspeed_nudge, but used when no airspeed sensor.
    // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
    int16_t throttle_nudge;

    // receiver RSSI
    uint8_t receiver_rssi;

    // Ground speed
    // The amount current ground speed is below min ground speed.  Centimeters per second
    int32_t groundspeed_undershoot;

    // Difference between current altitude and desired altitude.  Centimeters
    int32_t altitude_error_cm;

    // Battery Sensors
    AP_BattMonitor battery;

#if FRSKY_TELEM_ENABLED == ENABLED
    // FrSky telemetry support
    AP_Frsky_Telem frsky_telemetry {ahrs, battery, rangefinder};
#endif

    // Variables for extended status MAVLink messages
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;
 
    // Airspeed Sensors
    AP_Airspeed airspeed;

    // ACRO controller state
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
    } acro_state;

    // CRUISE controller state
    struct CruiseState {
        bool locked_heading;
        int32_t locked_heading_cd;
        uint32_t lock_timer_ms;
    } cruise_state;

    struct {
        uint32_t last_tkoff_arm_time;
        uint32_t last_check_ms;
        uint32_t last_report_ms;
        bool launchTimerStarted;
    } takeoff_state;
    
    // ground steering controller state
    struct {
        // Direction held during phases of takeoff and landing centidegrees
        // A value of -1 indicates the course has not been set/is not in use
        // this is a 0..36000 value, or -1 for disabled
        int32_t hold_course_cd;

        // locked_course and locked_course_cd are used in stabilize mode 
        // when ground steering is active, and for steering in auto-takeoff
        bool locked_course;
        float locked_course_err;
    } steer_state { -1, false, 0 };

    // flight mode specific
    struct {
        // Flag for using gps ground course instead of INS yaw.  Set false when takeoff command in process.
        bool takeoff_complete:1;

        // are we headed to the land approach waypoint? Works for any nav type
        bool wp_is_land_approach:1;

        // should we fly inverted?
        bool inverted_flight:1;

        // should we disable cross-tracking for the next waypoint?
        bool next_wp_no_crosstrack:1;

        // should we use cross-tracking for this waypoint?
        bool no_crosstrack:1;

        // in FBWA taildragger takeoff mode
        bool fbwa_tdrag_takeoff_mode:1;

        // have we checked for an auto-land?
        bool checked_for_autoland:1;

        // Altitude threshold to complete a takeoff command in autonomous modes.  Centimeters
        // are we in idle mode? used for balloon launch to stop servo
        // movement until altitude is reached
        bool idle_mode:1;

        // used to 'wiggle' servos in idle mode to prevent them freezing
        // at high altitudes
        uint8_t idle_wiggle_stage;

        // Altitude threshold to complete a takeoff command in autonomous
        // modes.  Centimeters above home
        int32_t takeoff_altitude_rel_cm;

        // Minimum pitch to hold during takeoff command execution.  Hundredths of a degree
        int16_t takeoff_pitch_cd;

        // Begin leveling out the enforced takeoff pitch angle min at this height to reduce/eliminate overshoot
        int32_t height_below_takeoff_to_level_off_cm;

        // the highest airspeed we have reached since entering AUTO. Used
        // to control ground takeoff
        float highest_airspeed;
        
        // initial pitch. Used to detect if nose is rising in a tail dragger
        int16_t initial_pitch_cd;
        
        // turn angle for next leg of mission
        float next_turn_angle {90};

        // filtered sink rate for landing
        float sink_rate;

        // time when we first pass min GPS speed on takeoff
        uint32_t takeoff_speed_time_ms;
        
        // distance to next waypoint
        float wp_distance;
        
        // proportion to next waypoint
        float wp_proportion;
        
        // last time is_flying() returned true in milliseconds
        uint32_t last_flying_ms;

        // time stamp of when we start flying while in auto mode in milliseconds
        uint32_t started_flying_in_auto_ms;

        // barometric altitude at start of takeoff
        float baro_takeoff_alt;

        // are we in VTOL mode in AUTO?
        bool vtol_mode:1;

        // are we doing loiter mode as a VTOL?
        bool vtol_loiter:1;
    } auto_state;

    struct {
        // roll pitch yaw commanded from external controller in centidegrees
        Vector3l forced_rpy_cd;
        // last time we heard from the external controller
        Vector3l last_forced_rpy_ms;

        // throttle  commanded from external controller in percent
        float forced_throttle;
        uint32_t last_forced_throttle_ms;
} guided_state;

    struct {
        // on hard landings, only check once after directly a landing so you
        // don't trigger a crash when picking up the aircraft
        bool checkedHardLanding:1;

        // crash detection. True when we are crashed
        bool is_crashed:1;

        // impact detection flag. Expires after a few seconds via impact_timer_ms
        bool impact_detected:1;

        // debounce timer
        uint32_t debounce_timer_ms;

        // delay time for debounce to count to
        uint32_t debounce_time_total_ms;

        // length of time impact_detected has been true. Times out after a few seconds. Used to clip isFlyingProbability
        uint32_t impact_timer_ms;
    } crash_state;

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool auto_throttle_mode:1;

    // true if we are in an auto-navigation mode, which controls whether control input is ignored
    // with STICK_MIXING=0
    bool auto_navigation_mode:1;
    
    // this controls throttle suppression in auto modes
    bool throttle_suppressed;
	
    // reduce throttle to eliminate battery over-current
    int8_t  throttle_watt_limit_max;
    int8_t  throttle_watt_limit_min; // for reverse thrust
    uint32_t throttle_watt_limit_timer_ms;

    AP_Vehicle::FixedWing::FlightStage flight_stage = AP_Vehicle::FixedWing::FLIGHT_NORMAL;

    // probability of aircraft is currently in flight. range from 0 to
    // 1 where 1 is 100% sure we're in flight
    float isFlyingProbability;

    // previous value of is_flying()
    bool previous_is_flying;

    // time since started flying in any mode in milliseconds
    uint32_t started_flying_ms;

    // Navigation control variables
    // The instantaneous desired bank angle.  Hundredths of a degree
    int32_t nav_roll_cd;

    // The instantaneous desired pitch angle.  Hundredths of a degree
    int32_t nav_pitch_cd;

    // we separate out rudder input to allow for RUDDER_ONLY=1
    int16_t rudder_input;

    // the aerodymamic load factor. This is calculated from the demanded
    // roll before the roll is clipped, using 1/sqrt(cos(nav_roll))
    float aerodynamic_load_factor = 1.0f;

    // a smoothed airspeed estimate, used for limiting roll angle
    float smoothed_airspeed;

    // Mission library
    AP_Mission mission {ahrs, 
            FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::exit_mission_callback, void)};


#if PARACHUTE == ENABLED
    AP_Parachute parachute {relay};
#endif

    // terrain handling
#if AP_TERRAIN_AVAILABLE
    AP_Terrain terrain {ahrs, mission, rally};
#endif

    AP_Landing landing {mission,ahrs,SpdHgt_Controller,nav_controller,aparm,
        FUNCTOR_BIND_MEMBER(&Plane::set_target_altitude_proportion, void, const Location&, float),
        FUNCTOR_BIND_MEMBER(&Plane::constrain_target_altitude_location, void, const Location&, const Location&),
        FUNCTOR_BIND_MEMBER(&Plane::adjusted_altitude_cm, int32_t),
        FUNCTOR_BIND_MEMBER(&Plane::adjusted_relative_altitude_cm, int32_t),
        FUNCTOR_BIND_MEMBER(&Plane::disarm_if_autoland_complete, void),
        FUNCTOR_BIND_MEMBER(&Plane::update_flight_stage, void)};

    AP_ADSB adsb {ahrs};

    // avoidance of adsb enabled vehicles (normally manned vheicles)
    AP_Avoidance_Plane avoidance_adsb {ahrs, adsb};

    // Outback Challenge Failsafe Support
    AP_AdvancedFailsafe_Plane afs {mission, barometer, gps, rcmap};

    /*
      meta data to support counting the number of circles in a loiter
    */
    struct {
        // previous target bearing, used to update sum_cd
        int32_t old_target_bearing_cd;

        // Total desired rotation in a loiter.  Used for Loiter Turns commands. 
        int32_t total_cd;

        // total angle completed in the loiter so far
        int32_t sum_cd;

        // Direction for loiter. 1 for clockwise, -1 for counter-clockwise
        int8_t direction;

        // when loitering and an altitude is involved, this flag is true when it has been reached at least once
        bool reached_target_alt;

        // check for scenarios where updrafts can keep you from loitering down indefinitely.
        bool unable_to_acheive_target_alt;

        // start time of the loiter.  Milliseconds.
        uint32_t start_time_ms;

        // altitude at start of loiter loop lap. Used to detect delta alt of each lap.
        // only valid when sum_cd > 36000
        int32_t start_lap_alt_cm;
        int32_t next_sum_lap_cd;

        // The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
        uint32_t time_max_ms;
    } loiter;

    struct Ellipse {
            struct Location center_loc;
            int32_t maxradius_cm;
            float minmaxratio;
            float azimuth_deg;
            int8_t orientation;
            int32_t height_cm;

            // variables to store the current and desired location and velocity
            struct Location aircraft_loc;
            Vector3f aircraft_vel;
            struct Location desired_loc;
    } ellipse;

    struct eight_on_plane {
        struct Location center_loc; // location of the center point on the ground
        int32_t d_c_cm; // horizontal distance from crossing point to turning point
        int32_t S1_radius_cm; // radius of the turning circles
        int32_t height_cm; // height of the pattern above the center point
        float azimuth_deg;
        int8_t orientation;     // orientation of the figure-eight pattern

        // variables to store the current and desired location and velocity
        struct Location aircraft_loc;
        Vector3f aircraft_vel;
        struct Location desired_loc;

      // the four segments are labeled by integers 0,1,2,3 in dependence of orientation
      // for orientation of the figure-eight pattern: +1: segment sequence is: geodesic_1 -> circle_1 -> geodesic_2 -> circle_2
      //                                              -1: segment sequence is: geodesic_1 -> circle_2 -> geodesic_2 -> circle_1

      // set current quadrant to a value that is not 0,1,2,3 such that it is altered in the initialization
        int8_t current_quadrant; // = 4
        int8_t current_segment; // sets the start segment when eight_sphere is initialized
                              // and the entry segment if no segment switching occurs because the aircraft is located in the vicinity of the crossing point defined by _mindistxaplane
      Vector3f current_cv;
      Vector3f current_tv;
      // derived parameters
      // trigonometric functions of all angles
      // trigonometric functions of the angles determining the attitude (azimuthal position in the sky) of the figure-eight pattern
      float cos_psi; // = cosf(radians(azimuth_deg));
      float sin_psi; // = sinf(radians(azimuth_deg));
      // trigonometric functions of half of the crossing angle
      float sin_chihalf; // = eight_in_R2.S1_radius_cm / eight_in_R2.d_c_cm;
      float cos_chihalf; // = sqrt(1 - sq(sin_chihalf));

      // crossing point of the figure-eight pattern: coordinate frame, rotation matrix, position vector
      // unit vector pointing from S2_loc to the crossing point of the figure-eight pattern
      Vector3f erxv; // = Vector3f(0, 0, -1);
      // unit vector of the tangential plane at the crossing point in polar direction
      Vector3f ethetaxv; // = Vector3f(cos_psi, sin_psi, 0);
      // unit vector of the tangential plane at the crossing point in azimuthal direction
      Vector3f epsixv; // = Vector3f(-sin_psi, cos_psi, 0);
      // rotation matrix that yields the vectors of the figure_eight pattern with crossing point in the direction erxv
      // from those of the figure-eight pattern with crossing point at erv; // = (0,0,-1) and turning circle centers aligned with the east axis
      Matrix3f Rm; // = Matrix3f(ethetaxv, epsixv, -erxv);
      // vector pointing from S2_loc to the crossing point
      Vector3f rxv; // = erxv * height_cm / 100.0f;

      // figure-eight pattern from four circle segments defined via four planes in canonical orientation
      // canonical orientation (at azimuth_deg = 0, elevation_deg = 90 in the NED coordinate system): crossing point is upwards (in the (0,0,-1)-direction) from S2_loc; turning circle centers are aligned along east axis
      // figure-8-pattern is given by the sequences:
      // for orientation = +1: geodesic_1 -> circle_1 -> geodesic_2 -> circle_2
      //                   -1: geodesic_1 -> circle_2 -> geodesic_2 -> circle_1
      // individual circle segment orientation = +1 corresponds to applying the right-hand rule to minus the normal vectors pointing to the center of the circle segment
      // vector of the first turning circle center
      // orientation of c1 = orientation
      Vector3f rc1v; // = (Rm * Vector3f(0.0f, d_c_cm, -h_cm));
      // vector of the second turning circle center
      // orientation of c2 = - orientation
      Vector3f rc2v; // = (Rm * Vector3f(0.0f, -d_c_cm, -h_cm));
      // location of the turning circle centers
      struct Location c1_loc;
      struct Location c2_loc;


      // vectors from the center of the plane to the eastern transition points
      // NE
      Vector3f rtg1c1; // = (Rm * (rc1v + Vector3f(S1_radius_cm/100.0f * cos_chihalf, -S1_radius_cm/100.0f * sin_chihalf, 0.0f)));
      // SE
      Vector3f rtc1g2; // = (Rm * (rc1v + Vector3f(-S1_radius_cm/100.0f * cos_chihalf, -S1_radius_cm/100.0f * sin_chihalf, 0.0f)));


      // locations of the transition points
      // NE
      struct Location g1c1_loc;
      // SE
      struct Location c1g2_loc;
      // SW
      struct Location c2g1_loc;
      // NW
      struct Location g2c2_loc;

      // unit tangent vectors of the two geodesic segments
      Vector3f etg1v; // = Rm * Vector3f(sin_chihalf, cos_chihalf, 0.0f) * orientation;
      Vector3f etg2v; // = Rm * Vector3f(sin_chihalf, -cos_chihalf, 0.0f) * orientation;

      // tangent vectors at the transgression points between the segments in the directions demanded by orientation
      // in NE quadrant:
      Vector3f etg1c1v; // = etg1v;
      // in SE quadrant:
      Vector3f etc1g2v; // = etg2v;
      // in NW quadrant
      Vector3f etg2c2v; // = etg2v;
      // in SW quadrant
      Vector3f etc2g1v; // = etg1v;

      // array of the quadrant numbers (0:NE, 1:SE, 2:SW, 3:NW) as passed by the aircraft flying the figure-eight pattern with orientation = +1
      int8_t quadrants[4] = {0, 1, 3, 2};
      // yields the quadrant counter for the figure-eight-pattern (0:NE, 1:SE, 2:NW, 3:SW) in dependence of the quadrant number
      // for given quadrant, the counter is obtained as quadrant_count[quadrant]
       int8_t quadrant_count[4] = {0, 1, 3, 2};
       // array of turning circle center vectors labeled by the quadrant number
       Vector3f centervectors[4]; // = {erc1v * dist_cm / 100.0f, erc1v * dist_cm / 100.0f, erc2v * dist_cm / 100.0f, erc2v * dist_cm / 100.0f};
        // array of unit tangent vectors at the transgression points of the segments labeled by the quadrant number
        Vector3f tangentvectors[4]; // = //{etg1c1v, etc1g2v, etc2g1v, etg2c2v};
       // array of directions: +1:outbound -1:inbound labeled by the quadrant number
       int8_t norientation = -orientation;
       int8_t directions[4] = {orientation, norientation, norientation, orientation};
       // array of numbers of the first segment in each quadrant for orientation = +1 and second segment in each quadrant for orientation = -1 labeled by the quadrant number
       int8_t firstsegments[4] = {0, 1, 3, 2};
       // array of numbers of the second segment in each quadrant for orientation = +1 and first segment in each quadrant for orientation = -1 labeled by the quadrant number
       int8_t secondsegments[4] = {1, 2, 0, 3};




      // segments are labeled by integers: initial:0, second:1, third:2, fourth:3
      // orientation = 1: sequence is g1 -> c1 -> g2 -> c2
      //                  orientations are 1, 1, 1, -1
      // orientation = -1: sequence is g1 -> c2 -> g2 -> c1
      //                   orientations are -1, 1, -1, -1

      // orientation of the current segment in dependence of the orientation of the figure-eight pattern
      int8_t current_orientation() {
          int8_t _orient=1;
          switch(current_segment * orientation){
              case 0: _orient = orientation; break;
              // orientation = 1
              case 1: _orient = orientation; break;
              case 2: _orient = orientation; break;
              case 3: _orient = -orientation; break;
              // orientation = -1;
              case -1: _orient = -orientation; break;
              case -2: _orient = orientation; break;
              case -3: _orient = orientation; break;
          };
          return _orient;
      }



      // vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
      Vector3f rxaplanev(Vector3f rav) {
          // vector from crossing point to the aircraft
          Vector3f _rxav;
          // projection of _rxav onto plane defining the circle segment
          _rxav = rav - erxv * (erxv * rav);
          // alternative construction via cross product
          // _rxav = -(_rav % erxv) % erxv;
          return _rxav;
      }

      // quadrants are labeled by integers: NE:0, SE:1, SW:2, NW:3 for the figure-eight pattern with crossing point in the direction (0,0,-1) and east-west attitude;
      // returns the index of the quadrant in which the aircraft is located
      int8_t quadrant(const Vector3f rav) {
          Vector3f _rxaplanev = rxaplanev(rav);
          int8_t _current_quadrant;
          // north or south
          if (_rxaplanev * ethetaxv >= 0) {_current_quadrant = -2;} else {_current_quadrant = -1;}
          // east or west
          if (_rxaplanev * epsixv >= 0) {_current_quadrant = _current_quadrant + 2;} else {_current_quadrant = -_current_quadrant + 1;};
          // current_quadrant is set to integer 0,1,2,3, where: NE:0, SE:1, SW:2, NW:3
          return _current_quadrant;
       }

      bool entered_next_quadrant;
      bool close_to_crossing_point;
      bool moving_matches_orientation;
      bool switch_to_2nd_segment_in_quadrant;


      // set the current segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
      void set_current_segment(const struct Location aloc, const Vector3f vav) {
          Vector3f rav = location_3d_diff_NED(center_loc, aloc);
          // position vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
          Vector3f _rxaplanev = rxaplanev(rav);
          // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
          float _mindistxaplane = 0.25f * d_c_cm / 100.0f; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
          //
          close_to_crossing_point = bool(_rxaplanev.length() <= _mindistxaplane);
          // set internal variable _current quadrant in dependence of the location of the aircraft
          int8_t _current_quadrant = quadrant(rav);
          // set internal variable _current segment
          int8_t _current_segment = current_segment;
          int8_t _next_quadrant = quadrants[(quadrant_count[_current_quadrant] + orientation) % 4];


          if (current_quadrant != _current_quadrant){
              // aircraft is not too close to the crossing point and the quadrant has changed
              // determine if aircraft entered the next quadrant
              entered_next_quadrant = bool(_current_quadrant == _next_quadrant);
              if (entered_next_quadrant) {
                  // aircraft has entered correct next quadrant
                  // switch to next quadrant
                  current_quadrant = _current_quadrant;
                  // reset variable
                  entered_next_quadrant = false;
              }
          }

          // center vector associated with the current quadrant
          Vector3f _current_cv = centervectors[current_quadrant];
          current_cv = _current_cv;
          // tangent vector at the transgression point between two segments associated with the current quadrant
          Vector3f _current_tv = tangentvectors[current_quadrant];
          current_tv = _current_tv;
          // position vector from center of the current turning circle to the aircraft
          // direction of flight in the current quadrant: +1:outbound, -1:inbound
          int8_t _current_direction = directions[current_quadrant];

          Vector3f _rcav = rav - _current_cv;
          //rcav = _rcav;
          // true if the current segment is the first in the quadrant: transgression point of that quadrant will be passed
          switch_to_2nd_segment_in_quadrant  = bool(_rcav * _current_tv >= 0);
          // true if the velocity vector of the aircraft is outbound / inbound  in the quadrants (0,3) / (1,2) for orientation = +1 and vice versa for orientation = -1
          moving_matches_orientation = bool(vav * _current_cv * _current_direction > 0);
          //projection = _rcav * _current_tv;


          //          if(close_to_crossing_point) {
          //              // aircraft is in the vicinity of the crossing point
          //              // select geodesic segment with orientation best aligned with the velocity vector of the aircraft
          //              if(vav * (etg1xv - etg2xv) >= 0){
          //                  // select segment corresponding to g1
          //                  _current_segment = 0;
          //              } else {
          //                  // select segment corresponding to g2
          //                  _current_segment = 2;
          //            }
          //          } else {

          // switch to the second segment in the quadrant if required
          if (close_to_crossing_point){
              // aircraft is in the vicinity  to crossing point
              // select geodesic segment with orientation best aligned with the velocity vector of the aircraft
//              if(vav * (etg1v - etg2v) >= 0){
//                  // select segment corresponding to g1
//                  _current_segment = 0;
//              } else {
//                  // select segment corresponding to g2
//                  _current_segment = 2;
//              }
          } else {
              // aircraft is not in the vicinity of the crossing point
              // switch from first to second segment in the quadrant
              _current_segment = firstsegments[current_quadrant];
              if (switch_to_2nd_segment_in_quadrant){
                  _current_segment = secondsegments[current_quadrant];
                  //current_segment = _current_segment;
              }
          }

          current_quadrant = _current_quadrant;
          current_segment = _current_segment;
      }







//      bool entered_new_quadrant;
//      bool close_to_crossing_point;
//      bool moving_matches_orientation;
//      bool passed_transgression_point;


//// set the current segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
//      void set_current_segment(const struct Location aloc, const Vector3f vav) {
//          Vector3f rav = location_3d_diff_NED(center_loc, aloc);
//          // position vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
//          Vector3f _rxaplanev = rxaplanev(rav);
//      // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
//         float _mindistxaplane = 0.25f * d_c_cm / 100.0f; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
//         //
//         close_to_crossing_point = bool(_rxaplanev.length() <= _mindistxaplane);
//      // set internal variable _current segment
//      int8_t _current_segment = current_segment;
//      // set internal variable _current quadrant in dependence of the location of the aircraft
//      int8_t _current_quadrant = quadrant(rav);
//      entered_new_quadrant = bool(_current_quadrant != current_quadrant);
//      current_quadrant = _current_quadrant;
//      if(close_to_crossing_point){
//        // the aircraft is in the vicinity of the crossing point; no consecutive switching of segments s allowed
//        // select the geodesic segment along which the aircraft can fly in accord with the segment's orientation by minimal course corrections
//        // vector etg1v - etg2v poin
//        if(vav * (etg1v - etg2v) >= 0){
//          // select segment corresponding to g1
//          _current_segment = 0;
//        } else {
//          // select segment corresponding to g2
//          _current_segment = 2;
//        }
//      } else {
//          // segments are labeled by integers: g1:0, c1:1, g2:2, c2:3 for the figure-eight pattern
//          // array of center vectors of the segments
//          Vector3f segments_cv[4] = {rc1v, rc1v, rc2v, rc2v};
//          // array of unit tangent vectors at the transgression points of the segments
//          Vector3f segments_tv[4] = {etg1c1v, etc1g2v, etg2c2v, etc2g1v};
//          // set current quadrant in dependence of the location of the aircraft
//          _current_segment = current_segment;
//          // select the center of the turning circle that is used to determine switching from the current to the subsequent segment
//          Vector3f _current_cv = segments_cv[_current_segment];
//          // selecting the tangent vector at the endpoint of the current segment
//          Vector3f _current_tv = segments_tv[_current_segment];
//          current_cv = _current_cv;
//          current_tv = _current_tv;
//          // position vector from center of the current turning circle to the aircraft
//          Vector3f _rcav = rav - _current_cv;
//          passed_transgression_point = bool(_rcav * _current_tv >= 0);
//          // if since last segment switching the aircraft has entered a new quadrant and has passed the transgression point in that quadrant, segment switching has to occur
//          if (entered_new_quadrant && passed_transgression_point){
//              // if vav * _current_tv >= 0: the aircraft moves along the figure_eight pattern as prescribed by orientation
//              //                       < 0: the aircraft moves along the figure_eight pattern in the opposite direction against the prescribed by orientation
//              // => temporally next segment is segment = segment + sign(vav * _current_tv)
//              moving_matches_orientation = 1;//bool(vav * _current_tv >= 0);
//              int8_t step;
//              if(moving_matches_orientation){step = 1;} else {step = -1;}
//              _current_segment = _current_segment + step;
//              _current_segment = _current_segment % 4;
//              current_segment = _current_segment;
//          }
//      }
//      }
//
    } eight_in_R2;



    struct circ_on_sphere {
        struct Location S2_loc;
        //struct Location S1_loc;
        Vector3f ercv;
        int32_t S2_radius_cm;
        int32_t S1_radius_cm;
        int32_t distance_cm; // distance of center of the circle from the center of the sphere
        float theta_rho_deg; // half of the opening angle of the cone with base S1
        float azimuth_deg;
        float elevation_deg;
        int8_t orientation;
        // variables to store the current and desired location and velocity
        //        int32_t height_cm;
        struct Location aircraft_loc;
        Vector3f aircraft_vel;
        struct Location desired_loc;
    } S1_in_S2;


    // eight_on_sphere consists of two geodesic segments and two turning (small) circle (S^1) segments on the sphere (S^2)
    // each S^1 is defined as intersection of a plane with S^2
    // definition of S^2 (hemisphere): center at Location S2_loc,  radius S2_radius_cm
    // definition of a plane: unit normal Vector3f erc, angle theta_r_deg (yields distance of the plane from the origin dist_cm = S2_radius_cm * cos(theta_r_deg))
    struct eight_on_sphere {

        // parameters to be defined by the user
        struct Location S2_loc; // location of the center of the S2
        int32_t S2_radius_cm;   // radius of the S2 in cm
        float theta_c_deg;      // half of the angle between the centers of the two turning circle segments, range: [0,90] degrees
        float theta_r_deg;      // half of the opening angle of the cone with tip at S2_loc and base given by the turning circle, range: [0,90-theta_c_deg] degrees in order to guarantee that the sweeping angle between the two apices is less than 180 deg
        float azimuth_deg;      // azimuth angle of the vector pointing from S2_loc to the crossing point of the figure-eight pattern, range: [0,360]
        float elevation_deg;    // elevation angle of the vector pointing from S2_loc to the crossing point of the figure-eight pattern, range [0,90]
        int8_t orientation;     // orientation of the figure-eight pattern: +1: downwards flight on geodesic, upwards flight on turning circle segments
                              //                                          -1: upwards flight on geodesic, downwards flight on turning circle segments
                              //                                          for 0 < elevation_deg < 90


        // variables to store the current and desired location and velocity
        struct Location aircraft_loc;
        Vector3f aircraft_vel;
        struct Location desired_loc;

      // the four segments are labeled by integers 0,1,2,3 in dependence of orientation
      // for orientation of the figure-eight pattern: +1: segment sequence is: geodesic_1 -> circle_1 -> geodesic_2 -> circle_2
      //                                              -1: segment sequence is: geodesic_1 -> circle_2 -> geodesic_2 -> circle_1
      // set current quadrant to a value that is not 0,1,2,3 such that it is altered in the initialization
         int8_t current_quadrant; // = 4
         int8_t current_segment; // sets the start segment when eight_sphere is initialized
                                // and the entry segment if no segment switching occurs because the aircraft is located in the vicinity of the crossing point defined by _mindistxaplane
        Vector3f current_cv;
        Vector3f current_tv;

      // derived parameters
      // trigonometric functions of all angles
      // calculation of polar angle from elevation
      float theta; // = 90 - elevation_deg;
      // trigonometric functions of the angles determining the attitude (azimuthal and polar position in the sky) of the figure-eight pattern
      float cos_psi; // = cosf(radians(azimuth_deg));
      float sin_psi; // = sinf(radians(azimuth_deg));
      float cos_theta; // = cosf(radians(theta));
      float sin_theta; // = sinf(radians(theta));
      // trigonometric functions of the opening angle
      // ratio of distance of the turning circle plane and of the radius of the sphere
      float cos_theta_r; // = cosf(radians(theta_r_deg));
      // ratio of the turning circle radius and of the radius of the sphere
      float sin_theta_r; // = sinf(radians(theta_r_deg));
      // trigonometric functions of the polar angle of the center of the turning circle
      float cos_theta_c; // = cosf(radians(theta_c_deg));
      float sin_theta_c; // = sinf(radians(theta_c_deg));
      // trigonometric functions of half of the crossing angle
      float cos_chihalf; // = sqrt(1 - sq(sin_theta_r/sin_theta_c));
      float sin_chihalf; // = sqrt(1 - sq(cos_chihalf));
      // trigonometric functions of half of the sweeping angle of the geodesic segments
      float cos_theta_0; // = cos_theta_c/cos_theta_r;
      float sin_theta_0; // = sqrt(1 - sq(cos_theta_0));

      // turning circle segments of the figure-eight pattern: circle radius, distance from origin
      // radius
      int32_t S1_radius_cm; // = S2_radius_cm * sin_theta_r;
      // distance of the centers from S2_loc;
      int32_t dist_cm; // = S2_radius_cm * sin_theta_r;

      // crossing point of the figure-eight pattern: coordinate frame, rotation matrix, position vector
      // unit vector pointing from S2_loc to the crossing point of the figure-eight pattern
      Vector3f erxv; // = Vector3f(sin_theta * cos_psi, sin_theta * sin_psi, -cos_theta );
      // unit vector of the tangential plane at the crossing point in polar direction
      Vector3f ethetaxv; // = Vector3f(cos_theta * cos_psi, cos_theta * sin_psi, sin_theta);
      // unit vector of the tangential plane at the crossing point in azimuthal direction
      Vector3f epsixv; // = Vector3f(-sin_psi, cos_psi, 0);
      // rotation matrix that yields the vectors of the figure_eight pattern with crossing point in the direction erxv
      // from those of the figure-eight pattern with crossing point at erv; // = (0,0,-1) and turning circle centers aligned with the east axis
      Matrix3f Rm; // = Matrix3f(ethetaxv, epsixv, -erxv);
      // vector pointing from S2_loc to the crossing point
      Vector3f rxv; // = erxv * dist_cm / 100.0f;

      // figure-eight pattern from four circle segments defined via four planes in canonical orientation
      // canonical orientation (at azimuth_deg = 0, elevation_deg = 90 in the NED coordinate system): crossing point is upwards (in the (0,0,-1)-direction) from S2_loc; turning circle centers are aligned along east axis
      // figure-8-pattern is given by the sequences:
      // for orientation = +1: geodesic_1 -> circle_1 -> geodesic_2 -> circle_2
      //                   -1: geodesic_1 -> circle_2 -> geodesic_2 -> circle_1
      // individual circle segment orientation = +1 corresponds to applying the right-hand rule to minus the normal vectors pointing to the center of the circle segment
      // unit normal vector of the first turning circle plane (pointing to the center of the first turning circle c1)
      // orientation of c1 = orientation
      Vector3f erc1v; // = Rm * Vector3f(0.0f, sin_theta_c, -cos_theta_c);
      // unit normal vector of the second turning circle plane (pointing to the center of the second turning circle c2)
      // orientation of c2 = - orientation
      Vector3f erc2v; // = Rm * Vector3f(0.0f, -sin_theta_c, -cos_theta_c);
      // unit normal vector of the first geodesic plane (geodesic g1: from south-west to north-east)
      // orientation of g1 = orientation
      Vector3f erg1v; // = Rm * Vector3f(-cos_chihalf, sin_chihalf, 0.0f);
      // unit normal vector of the second geodesic segment (geodesic g2: from south-east to north-west)
      // orientation of g2 = orientation
      Vector3f erg2v; // = Rm * Vector3f(cos_chihalf, sin_chihalf, 0.0f);

      // tangent vectors at the transgression points between the segments in the directions demanded by orientation
      // in NE quadrant:
      Vector3f etg1c1v; // = Rm * Vector3f(cos_theta_0 * sin_chihalf, cos_theta_0 * cos_chihalf, sin_theta_0) * orientation;
      // in SE quadrant:
      Vector3f etc1g2v; // = Rm * Vector3f(cos_theta_0 * sin_chihalf, -cos_theta_0 * cos_chihalf, -sin_theta_0) * orientation;
      // in NW quadrant
      Vector3f etg2c2v; // = Rm * Vector3f(cos_theta_0 * sin_chihalf, -cos_theta_0 * cos_chihalf, sin_theta_0) * orientation;
      // in SW quadrant
      Vector3f etc2g1v; // = Rm * Vector3f(cos_theta_0 * sin_chihalf, cos_theta_0 * cos_chihalf, -sin_theta_0) * orientation;

      // tangent vectors at the crossing point
      Vector3f etg1xv; // = Rm * Vector3f(sin_chihalf, cos_chihalf, 0.0f) * orientation;
      Vector3f etg2xv; // = Rm * Vector3f(sin_chihalf, -cos_chihalf, 0.0f) * orientation;

      // segments are labeled by integers: initial:0, second:1, third:2, fourth:3
      // orientation = 1: sequence is g1 -> c1 -> g2 -> c2
      //                  orientations are 1, 1, 1, -1
      // orientation = -1: sequence is g1 -> c2 -> g2 -> c1
      //                   orientations are -1, 1, -1, -1


      // array of the quadrant numbers (0:NE, 1:SE, 2:SW, 3:NW) as passed by the aircraft flying the figure-eight pattern with orientation = +1
      int8_t quadrants[4] = {0, 1, 3, 2};
      // yields the quadrant counter for the figure-eight-pattern (0:NE, 1:SE, 2:NW, 3:SW) in dependence of the quadrant number
      // for given quadrant, the counter is obtained as quadrant_count[quadrant]
       int8_t quadrant_count[4] = {0, 1, 3, 2};
       // array of turning circle center vectors labeled by the quadrant number
       Vector3f centervectors[4]; // = {erc1v * dist_cm / 100.0f, erc1v * dist_cm / 100.0f, erc2v * dist_cm / 100.0f, erc2v * dist_cm / 100.0f};
        // array of unit tangent vectors at the transgression points of the segments labeled by the quadrant number
        Vector3f tangentvectors[4]; // = //{etg1c1v, etc1g2v, etc2g1v, etg2c2v};
        // array of directions: +1:outbound -1:inbound labeled by the quadrant       // array of unit tangent vectors at the transgression points of the segments labeled by the quadrant counter (not the quadrant number)
       // array of directions: +1:outbound -1:inbound labeled by the quadrant number
       int8_t norientation = -orientation;
       int8_t directions[4] = {orientation, norientation, norientation, orientation};
       // array of numbers of the first segment in each quadrant for orientation = +1 and second segment in each quadrant for orientation = -1 labeled by the quadrant number
       int8_t firstsegments[4] = {0, 1, 3, 2};
       // array of numbers of the second segment in each quadrant for orientation = +1 and first segment in each quadrant for orientation = -1 labeled by the quadrant number
       int8_t secondsegments[4] = {1, 2, 0, 3};



      // unit vector to the center of the current segment in dependence of the orientation of the figure-eight pattern
//       Vector3f ercvs[4] = {erg1v, erc1v, erg2v, erc2v};
       Vector3f current_ercv() {
          Vector3f _vec = Vector3f(0,0,-1);
          switch(current_segment){
              case 0: _vec = erg1v; break;
              // orientation = 1
              case 1: _vec = erc1v; break;
              case 2: _vec = erg2v; break;
              case 3: _vec = erc2v; break;
//              // orientation = -1;
//              case -1: _vec = erc2v; break;
//              case -2: _vec = erg2v; break;
//              case -3: _vec = erc1v; break;
          };
          return _vec;
      }

      // orientation of the current segment in dependence of the orientation of the figure-eight pattern
//      int8_t orientations[4] = {orientation, orientation, orientation, norientation};
      int8_t current_orientation() {
          int8_t _orient=1;
          switch(current_segment){
              case 0: _orient = orientation; break;
              // orientation = 1
              case 1: _orient = orientation; break;
              case 2: _orient = orientation; break;
              case 3: _orient = -orientation; break;
//              // orientation = -1;
//              case -1: _orient = -orientation; break;
//              case -2: _orient = orientation; break;
//              case -3: _orient = orientation; break;
          };
          return _orient;
      }

      // angle theta_r of the current segment in dependence of the orientation of the figure-eight pattern
      // planes defining geodesics contain the center and hence have theta_r_deg = 90
//      float thetars[4] = {90.0f, theta_r_deg, 90.0f, theta_r_deg};
            float current_theta_r() {
          float _theta_r=30.0f;
          switch(current_segment){
              case 0: _theta_r = 90.0f; break;
              // orientation = 1
              case 1: _theta_r = theta_r_deg; break;
              case 2: _theta_r = 90.0f; break;
              case 3: _theta_r = theta_r_deg; break;
//              // orientation = -1;
//              case -1: _theta_r = theta_r_deg; break;
//              case -2: _theta_r = 90.0f; break;
//              case -3: _theta_r = theta_r_deg; break;
          };
          return _theta_r;
      }



      // vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
      Vector3f rxaplanev(Vector3f rav) {
          // vector from crossing point to the aircraft
          Vector3f _rxav;
          // projection of _rxav onto plane defining the circle segment
          _rxav = rav - erxv * (erxv * rav);
          // alternative construction via cross product
          // _rxav = -(_rav % erxv) % erxv;
          return _rxav;
      }



      // quadrants are labeled by integers: NE:0, SE:1, SW:2, NW:3 for the figure-eight pattern with crossing point in the direction (0,0,-1) and east-west attitude;
      // returns the index of the quadrant in which the aircraft is located
      int8_t quadrant(const Vector3f rav) {
          Vector3f _rxaplanev = rxaplanev(rav);
          int8_t _current_quadrant;
          // north or south
          if (_rxaplanev * ethetaxv >= 0) {_current_quadrant = -2;} else {_current_quadrant = -1;}
          // east or west
          if (_rxaplanev * epsixv >= 0) {_current_quadrant = _current_quadrant + 2;} else {_current_quadrant = -_current_quadrant + 1;};
          // current_quadrant is set to integer 0,1,2,3, where: NE:0, SE:1, SW:2, NW:3
          return _current_quadrant;
       }

           // bool entered_new_quadrant;
            bool close_to_crossing_point;
            bool in_initial_quadrant; // = true; // has to be set to true in order to enable segment switching in the initial quadrant
            bool entered_next_quadrant;
            bool moving_matches_orientation;
            bool switch_to_2nd_segment_in_quadrant;
            //Vector3f current_cv;
            Vector3f rcav;
            float projection;

      // set the current segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
      void set_current_segment(const struct Location aloc, const Vector3f vav) {
          Vector3f rav = location_3d_diff_NED(S2_loc, aloc);
          // position vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
          Vector3f _rxaplanev = rxaplanev(rav);
          // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
          float _mindistxaplane = 0.25f * S2_radius_cm / 100.0f * sin_theta_0; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
          //
          close_to_crossing_point = bool(_rxaplanev.length() <= _mindistxaplane);
          // set internal variable _current quadrant in dependence of the location of the aircraft
          int8_t _current_quadrant = quadrant(rav);
          // set internal variable _current segment
          int8_t _current_segment = current_segment;
          int8_t _next_quadrant = quadrants[(quadrant_count[_current_quadrant] + orientation) % 4];


          if (current_quadrant != _current_quadrant){
              // aircraft is not too close to the crossing point and the quadrant has changed
              // determine if aircraft entered the next quadrant
              entered_next_quadrant = bool(_current_quadrant == _next_quadrant);
              if (entered_next_quadrant) {
                  // aircraft has entered correct next quadrant
                  // switch to next quadrant
                  current_quadrant = _current_quadrant;
                  // reset variable
                  entered_next_quadrant = false;
              }
          }

          // center vector associated with the current quadrant
          Vector3f _current_cv = centervectors[current_quadrant];
          current_cv = _current_cv;
          // tangent vector at the transgression point between two segments associated with the current quadrant
          Vector3f _current_tv = tangentvectors[current_quadrant];
          current_tv = _current_tv;
          // position vector from center of the current turning circle to the aircraft
          // direction of flight in the current quadrant: +1:outbound, -1:inbound
          int8_t _current_direction = directions[current_quadrant];

          Vector3f _rcav = rav - _current_cv;
          rcav = _rcav;
          Vector2f _rcavl(_rcav.x,_rcav.y);
          Vector2f _current_tvl(_current_tv.x,_current_tv.y);
          projection = _rcavl * _current_tvl;
          // true if the current segment is the first in the quadrant: transgression point of that quadrant will be passed
          switch_to_2nd_segment_in_quadrant  = bool(projection >=0);//bool(_rcav * _current_tv >= 0);
          // true if the velocity vector of the aircraft is outbound / inbound  in the quadrants (0,3) / (1,2) for orientation = +1 and vice versa for orientation = -1
          moving_matches_orientation = bool(vav * _current_cv * _current_direction > 0);




          //          if(close_to_crossing_point) {
          //              // aircraft is in the vicinity of the crossing point
          //              // select geodesic segment with orientation best aligned with the velocity vector of the aircraft
          //              if(vav * (etg1xv - etg2xv) >= 0){
          //                  // select segment corresponding to g1
          //                  _current_segment = 0;
          //              } else {
          //                  // select segment corresponding to g2
          //                  _current_segment = 2;
          //            }
          //          } else {

          // switch to the second segment in the quadrant if required
          if (close_to_crossing_point){
              // aircraft is in the vicinity  to crossing point
              // select geodesic segment with orientation best aligned with the velocity vector of the aircraft
//              if(vav * (etg1v - etg2v) >= 0){
//                  // select segment corresponding to g1
//                  _current_segment = 0;
//              } else {
//                  // select segment corresponding to g2
//                  _current_segment = 2;
//              }
          } else {
              // aircraft is not in the vicinity of the crossing point
              // switch from first to second segment in the quadrant
              _current_segment = firstsegments[current_quadrant];
              if (switch_to_2nd_segment_in_quadrant){
                  _current_segment = secondsegments[current_quadrant];
                  //current_segment = _current_segment;
              }
          }

          current_quadrant = _current_quadrant;
          current_segment = _current_segment;
      }



//      // bool entered_new_quadrant;
//      bool close_to_crossing_point;
//      bool entered_next_quadrant;
//      bool moving_matches_orientation;
//      bool switch_to_2nd_segment_in_quadrant;
//      //Vector3f current_cv;
//      Vector3f rcav;
//
//      float projection;
//
//      // set current_segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
//      // current_segment labels the segment the aircraft is guided along: 0:g1, 1:c1, 2:g2, 3:c2
//      // current_quadrant in which the aircraft is located is determined from the aircraft's location
//      // current_quadrant labels the quadrants: 0:NE, 1:SE, 2:SW, 3:NW
//      // current_segmentpair labels the pair of segments; it corresponds to the first number of the pair in the sequence for orientation = +1: 0:(0,1), 1:(1,2), 2:(2,3), 3:(3,0)
//      void set_current_segment(const struct Location aloc, const Vector3f vav) {
//
//          Vector3f rav = location_3d_diff_NED(S2_loc, aloc);
//          // position vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
//          Vector3f _rxaplanev = rxaplanev(rav);
//          // determine current quadrant in which the aircraft is located
//          int8_t _current_quadrant = quadrant(rav);
//          int8_t _current_segment = current_segment;
//          // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
//          float _mindistxaplane = 0.25f * S2_radius_cm / 100.0f * sin_theta_0; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
//          // set to true if aircraft is within the distance _mindistaxplane from the crossing point of the figure-eight pattern
//          close_to_crossing_point = bool(_rxaplanev.length() <= _mindistxaplane);
//          // determine next quadrant from stored current quadrant
//          int8_t next_quadrant = quadrants[(quadrant_count[current_quadrant] + orientation) % 4];
//          // check if aircraft has moved into the quadrant which is the next one when leaving the stored current_quadrant
//          entered_next_quadrant = bool(_current_quadrant == next_quadrant);
//          // center vector associated with the current quadrant
//
//          if (entered_next_quadrant){
//              current_quadrant = _current_quadrant;
//          }
//
//          Vector3f _current_cv = centervectors[current_quadrant];
//          current_cv = _current_cv;
//          // tangent vector at the transgression point between two segments associated with the current quadrant
//           Vector3f _current_tv = tangentvectors[current_quadrant];
//           current_tv = _current_tv;
//          // position vector from center of the current turning circle to the aircraft
//           // first segments in each quadrant for orientation = +1 = second segments in each quadrant for orientation = -1
//            // direction of flight in the current quadrant: +1:outbound, -1:inbound
//            int8_t _current_direction = directions[current_quadrant];
//
//           Vector3f _rcav = rav - _current_cv;
//           rcav = _rcav;
//          // true if the current segment is the first in the quadrant: transgression point of that quadrant will be passed
//          switch_to_2nd_segment_in_quadrant  = bool(_rcav * _current_tv >= 0);
//          // true if the velocity vector of the aircraft is outbound / inbound  in the quadrants (0,3) / (1,2) for orientation = +1 and vice versa for orientation = -1
//          moving_matches_orientation = bool(vav * _current_cv * _current_direction > 0);
//          projection = _rcav * _current_tv;
//
//                // switch to the second segment in the quadrant if required
//          if (close_to_crossing_point){
//          } else {
//              _current_segment = firstsegments[_current_quadrant];
//              if (switch_to_2nd_segment_in_quadrant){
//                  _current_segment = secondsegments[_current_quadrant];
//                  //current_segment = _current_segment;
//              }
//          }
//
//          current_quadrant = _current_quadrant;
//          current_segment = _current_segment;
//      }

// 24.06.17
      //      // bool entered_new_quadrant;
//      bool close_to_crossing_point;
//      bool entered_next_quadrant;
//      bool moving_matches_orientation;
//      bool current_segment_is_first;
//
//
//      // set current_segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
//      // current_segment labels the segment the aircraft is guided along: 0:g1, 1:c1, 2:g2, 3:c2
//      // current_quadrant in which the aircraft is located is determined from the aircraft's location
//      // current_quadrant labels the quadrants: 0:NE, 1:SE, 2:SW, 3:NW
//      // current_segmentpair labels the pair of segments; it corresponds to the first number of the pair in the sequence for orientation = +1: 0:(0,1), 1:(1,2), 2:(2,3), 3:(3,0)
//      void set_current_segment(const struct Location aloc, const Vector3f vav) {
//
//          int8_t _current_segment = current_segment;
//          Vector3f rav = location_3d_diff_NED(S2_loc, aloc);
//          // position vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
//          Vector3f _rxaplanev = rxaplanev(rav);
//          // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
//          float _mindistxaplane = 0.125f * S2_radius_cm/ 100.0f * sin_theta_0; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
//          // set to true if aircraft is within the distance _mindistaxplane from the crossing point of the figure-eight pattern
//          close_to_crossing_point = bool(_rxaplanev.length() <= _mindistxaplane);
//          // determine current quadrant in which the aircraft is located
//          int8_t _current_quadrant = quadrant(rav);
//          // sequence of quadrants: next quadrant is obtained as quadrants[current_quadrant + orientation]
//          int8_t quadrants[4] = {0, 1, 3, 2};
//          // next quadrant
//          int8_t next_quadrant = quadrants[(quadrant_counter(current_quadrant) + orientation) % 4];
//          // check if aircraft has moved into a new quadrant
//          entered_next_quadrant = bool(_current_quadrant == next_quadrant);
//          // aircraft is not in the vicinity of the crossing point
//          // array of center vectors of the segments labeled by the quadrant
//          //Vector3f segments_cv[4] = {erg1v, erc1v, erg2v, erc2v};
//          Vector3f centervectors[4] = {erc1v * dist_cm, erc1v * dist_cm, erc2v * dist_cm, erc2v * dist_cm};
//          // array of unit tangent vectors at the transgression points of the segments labeled by the quadrant
//          Vector3f tangentvectors[4] = {etg1c1v, etc1g2v, etc2g1v, etg2c2v};
//          // array of directions: +1:outbound -1:inbound labeled by the quadrant
//          int8_t norientation = -orientation;
//          int8_t directions[4] = {orientation, norientation, norientation, orientation};
//          // direction of flight in the current quadrant: +1:outbound, -1:inbound
//          int8_t _current_direction = directions[_current_quadrant];
//          // center of the turning circle associated with the current quadrant that is used to determine switching from the current to the subsequent segment
//          Vector3f _current_cv = centervectors[_current_quadrant];
//          // true if the velocity vector of the aircraft is outbound / inbound  in the quadrants (0,3) / (1,2) for orientation = +1 and vice versa for orientation = -1
//          moving_matches_orientation = 1;//bool(vav * _current_cv * _current_direction > 0);
//
//
//          //          if (entered_new_quadrant && (orientation >=0 && (current_quadrant == 1 || current_quadrant ==2)) || (orientation < 0 && (current_quadrant == 0 || current_quadrant ==3))) {
//          //          // if aircraft entered a new quadrant, ensure that 1 -> 3 and 2 -> 0 / 3 -> 1 and 0 -> 2 are the only possible changes for orientations +1 / -1
//          //              if (abs(current_quadrant - _current_quadrant) == 2){
//          //                  // switch quadrants diagonally as demanded when passing the crossing point
//          //                  current_quadrant = _current_quadrant;
//          //              } else {
//          //                  // preserve current quadrant
//          //                  _current_quadrant = current_quadrant;
//          //              }
//          //           }
//          if(entered_next_quadrant) {
//              // switch to the next quadrant in which the aircraft is now located
//              current_quadrant = _current_quadrant;
//          } else {
//              // preserve current quadrant
//              _current_quadrant = current_quadrant;
//          }
//
//
//
//          // tangent vector at the transgression point between two segments associated with the current quadrant
//          Vector3f _current_tv = tangentvectors[_current_quadrant];
//
//          // first segments in each quadrant for orientation = +1 = second segments in each quadrant for orientation = -1
//          int8_t firstsegments[4] = {0, 1, 3, 2};
//          // second segments in each quadrant for orientation = +1 = first segments in each quadrant for orientation = -1
//          int8_t secondsegments[4] = {1, 2, 0, 3};
//
//          // position vector from center of the current turning circle to the aircraft
//          Vector3f _rcav = rav - _current_cv;
//          // true if the current segment is the first in the quadrant: transgression point of that quadrant will be passed
//          current_segment_is_first  = bool(_rcav * _current_tv < 0);
//
//
//
//
//
//          if(close_to_crossing_point) {
//              // aircraft is in the vicinity of the crossing point
//              // select geodesic segment with orientation best aligned with the velocity vector of the aircraft
//              if(vav * (etg1xv - etg2xv) >= 0){
//                  // select segment corresponding to g1
//                  _current_segment = 0;
//              } else {
//                  // select segment corresponding to g2
//                  _current_segment = 2;
//            }
//          } else {
//
//              // check if velocity of the aircraft is in accord with the orientation
//              if(moving_matches_orientation){
//                  if(1){//entered_new_quadrant){
//                      if(current_segment_is_first){
//                          // guarantee that _current_segment is set to first segment of the current quadrant
//                          if(orientation >= 0){_current_segment = firstsegments[_current_quadrant];} else {_current_segment = secondsegments[_current_quadrant];};
//                      } else {
//                          // switch to second segment of the current quadrant
//                          if(orientation >= 0){_current_segment = secondsegments[_current_quadrant];} else {_current_segment = firstsegments[_current_quadrant];};
//                      }
//                  }
//              } //else {
//                  // moving does not match orientation
//                  // loiter around center of turning circle
////                  if (_current_quadrant == 0 || _current_quadrant == 1){
////                      _current_segment = 1;
////                  } else {
////                      _current_segment = 3;
////                  }
//
//              //}
//          }
//          current_quadrant = _current_quadrant;
//          current_segment = _current_segment;
//      }
//            bool entered_new_quadrant;
//            bool close_to_crossing_point;
//            bool moving_matches_orientation;
//            bool passed_transgression_point;
////  23.06.17:
//      // set current_segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
//      // current_segment labels the segment the aircraft is guided along: 0:g1, 1:c1, 2:g2, 3:c2
//      // current_quadrant in which the aircraft is located is determined from the aircraft's location
//      // current_quadrant labels the quadrants: 0:NE, 1:SE, 2:SW, 3:NW
//      // current_segmentpair labels the pair of segments; it corresponds to the first number of the pair in the sequence for orientation = +1: 0:(0,1), 1:(1,2), 2:(2,3), 3:(3,0)
//      void set_current_segment(const struct Location aloc, const Vector3f vav) {
//          Vector3f rav = location_3d_diff_NED(S2_loc, aloc);
//          // position vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
//          Vector3f _rxaplanev = rxaplanev(rav);
//          // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
//          float _mindistxaplane = 0.125f * S2_radius_cm/ 100.0f * sin_theta_0; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
//          // set to true if aircraft is within the distance _mindistaxplane from the crossing point of the figure-eight pattern
//          close_to_crossing_point = bool(_rxaplanev.length() <= _mindistxaplane);
//            // set internal variable _current segment
//            int8_t _current_segmentpair = current_segment;
//            // set internal variable _current quadrant in dependence of the location of the aircraft
//            int8_t _current_quadrant = quadrant(rav);
//            //entered_new_quadrant = bool(_current_quadrant != current_quadrant);
//            current_quadrant = _current_quadrant;
//
//            // array of segment pairs indexed by the quadrant number: 0:(0,1), 1:(1,2), 2:(3,0), 3:(2,3)
//            int8_t segmentpairs[4] = {0, 1, 3, 2};
//            // identify segment pair located in the current quadrant
//            // sequences with periodicity four (one figure-eight period) of segment pairs starting with 0 are:
//            // orientation = +1: (0,1,2,3,0,...) can be realized as _current_segmentpair = (_current_segmentpair + 1) % 4 when passing a transgression point
//            // orientation = -1: (0,3,2,1,0,...) can be realized as _current_segmentpair = (4 - _current_segmentpair) % 4 when passing a trangsression point
//            _current_segmentpair = segmentpairs[_current_quadrant];
//            // array of center vectors of the segments labeled by the segment pair
//            //Vector3f segments_cv[4] = {erg1v, erc1v, erg2v, erc2v};
//            Vector3f centervectors[4] = {erc1v * dist_cm, erc1v * dist_cm, erc2v * dist_cm, erc2v * dist_cm};
//            // array of unit tangent vectors at the transgression points of the segments labeled by the segment pair
//            Vector3f tangentvectors[4] = {etg1c1v, etc1g2v, etg2c2v, etc2g1v};
//            // set current quadrant in dependence of the location of the aircraft
//            //_current_segment = current_segment;
//            // select the center of the turning circle that is used to determine switching from the current to the subsequent segment
//            Vector3f _current_cv = centervectors[_current_segmentpair];
//            // selecting the tangent vector at the endpoint of the current segment
//            Vector3f _current_tv = tangentvectors[_current_segmentpair];
//            current_cv = _current_cv;
//            current_tv = _current_tv;
//            // position vector from center of the current turning circle to the aircraft
//            Vector3f _rcav = rav - _current_cv;
//            passed_transgression_point = bool(_rcav * _current_tv >= 0);
//            if(close_to_crossing_point){
//              // the aircraft is in the vicinity of the crossing point; no consecutive switching of segments s allowed
//              // select the geodesic segment along which the aircraft can fly in accord with the segment's orientation by minimal course corrections
//              // vector etg1xv - etg2xv points into the right half of the eight seen from above towards the (rotated) north direction
//                if(vav * (etg1xv - etg2xv) >= 0){
//                // select segment corresponding to g1
//                _current_segmentpair = 0;
//              } else {
//                // select segment corresponding to g2
//                _current_segmentpair = 2;
//              }
//            } else {
//                // aircraft is not close to the crossing point of the ellipse
//                // segment switching occurs when the transgression point associated with _current_quadrant is passed
//                // it must only occur between the two segments (segment pair) that lie within this quadrant
//
//                // if since last segment switching the aircraft has entered a new quadrant and has passed the transgression point in that quadrant, segment switching has to occur
//                if (passed_transgression_point){
//                    // if vav * _current_tv >= 0: the aircraft moves along the figure_eight pattern as prescribed by orientation
//                    //                       < 0: the aircraft moves along the figure_eight pattern in the opposite direction against the prescribed by orientation
//                    // => temporally next segment is segment = segment + sign(vav * _current_tv)
//                    moving_matches_orientation = 1;//bool(vav * _current_tv >= 0);
//                    int8_t step;
//                    if(moving_matches_orientation){step = 1;} else {step = -1;}
//                    if(orientation >=0 ){
//                        _current_segmentpair = _current_segmentpair + step;
//                        _current_segmentpair = _current_segmentpair % 4;
//                    } else {
//                       _current_segmentpair = _current_segmentpair + step;
//                       _current_segmentpair = (4 - _current_segmentpair) % 4;
//                    }
//
//                    //current_segment = _current_segmentpair;
//                }
//            }
//            current_segment = _current_segmentpair;
//      }


// 21.06.17
//      bool entered_new_quadrant;
//      bool close_to_crossing_point;
//      bool moving_matches_orientation;
//      bool passed_transgression_point;
//
//
//      // set the current segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
//            void set_current_segment(const struct Location aloc, const Vector3f vav) {
//                Vector3f rav = location_3d_diff_NED(S2_loc, aloc);
//                // position vector from the center of the S2 to the aircraft projected onto the tangential plane at the crossing point
//                Vector3f _rxaplanev = rxaplanev(rav);
//            // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
//               float _mindistxaplane = 0.125f * S2_radius_cm/ 100.0f * sin_theta_0; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
//               //
//               close_to_crossing_point = bool(_rxaplanev.length() <= _mindistxaplane);
//            // set internal variable _current segment
//            int8_t _current_segment = current_segment;
//            // set internal variable _current quadrant in dependence of the location of the aircraft
//            int8_t _current_quadrant = quadrant(rav);
//            entered_new_quadrant = bool(_current_quadrant != current_quadrant);
//            current_quadrant = _current_quadrant;
//            if(close_to_crossing_point){
//              // the aircraft is in the vicinity of the crossing point; no consecutive switching of segments s allowed
//              // select the geodesic segment along which the aircraft can fly in accord with the segment's orientation by minimal course corrections
//              // vector etg1v - etg2v poin
//              if(vav * (etg1xv - etg2xv) >= 0){
//                // select segment corresponding to g1
//                _current_segment = 0;
//              } else {
//                // select segment corresponding to g2
//                _current_segment = 2;
//              }
//            } else {
//                // segments are labeled by integers: g1:0, c1:1, g2:2, c2:3 for the figure-eight pattern
//                // array of center vectors of the segments
//                //Vector3f segments_cv[4] = {erg1v, erc1v, erg2v, erc2v};
//                Vector3f segments_cv[4] = {erc1v * dist_cm, erc1v * dist_cm, erc2v * dist_cm, erc2v * dist_cm};
//                // array of unit tangent vectors at the transgression points of the segments
//                Vector3f segments_tv[4] = {etg1c1v, etc1g2v, etg2c2v, etc2g1v};
//                // set current quadrant in dependence of the location of the aircraft
//                _current_segment = current_segment;
//                // select the center of the turning circle that is used to determine switching from the current to the subsequent segment
//                Vector3f _current_cv = segments_cv[_current_segment];
//                // selecting the tangent vector at the endpoint of the current segment
//                Vector3f _current_tv = segments_tv[_current_segment];
//                current_cv = _current_cv;
//                current_tv = _current_tv;
//                // position vector from center of the current turning circle to the aircraft
//                Vector3f _rcav = rav - _current_cv;
//                passed_transgression_point = bool(_rcav * _current_tv >= 0);
//                // if since last segment switching the aircraft has entered a new quadrant and has passed the transgression point in that quadrant, segment switching has to occur
//                if (entered_new_quadrant && passed_transgression_point){
//                    // if vav * _current_tv >= 0: the aircraft moves along the figure_eight pattern as prescribed by orientation
//                    //                       < 0: the aircraft moves along the figure_eight pattern in the opposite direction against the prescribed by orientation
//                    // => temporally next segment is segment = segment + sign(vav * _current_tv)
//                    moving_matches_orientation = 1;//bool(vav * _current_tv >= 0);
//                    int8_t step;
//                    if(moving_matches_orientation){step = 1;} else {step = -1;}
//                    //if(orientation >=0 ){
//                        _current_segment = _current_segment + step;
//                        _current_segment = _current_segment % 4;
//                    //} else {
//
//                    //   _current_segment = _current_segment + step;
//                    //   _current_segment = (4-_current_segment) % 4;
//                    //}
//
//                    current_segment = _current_segment;
//                }
//            }
//            }
//

//      bool in_vicinity_of_crossing_point;
//      bool in_right_direction;
//      bool switch_to_next_segment;
//
//
//// set the current segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
//      void set_current_segment(const Vector3f rav, const Vector3f vav) {
//          // position vector from the center of the S2 to the aircraft projected onto the tangential plane
//          Vector3f _rxaplanev = rxaplanev(rav);
//      // minimum distance of aircraft from crossing point in the plane at which consecutive segment switching is allowed
//         float _mindistxaplane = 0.5f * S2_radius_cm * sin_theta_0; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
//
//      // bool variable
//      in_vicinity_of_crossing_point = bool(_rxaplanev.length() < _mindistxaplane);
//      // projections of the aircraft's velocity vector onto the tangential vectors of the geodesic segments a the crossing point
//      float vag1 = vav * etg1xv;
//      float vag2 = vav * etg2xv;
//      int8_t _current_segment = current_segment;
//      if(in_vicinity_of_crossing_point){
//        // the aircraft is in the vicinity of the crossing point; no consecutive switching of segments
//        // select the geodesic segment with tangent vector maximal which has the maximal projection onto the aircraft's velocity vector
//        if(vag1 > vag2){
//          // select segment corresponding to g1
//          _current_segment = 0;
//        } else {
//          // select segment corresponding to g2
//          _current_segment = 2;
//        }
//      } else {
//        // the aircraft is not in the vicinity of the crossing point; consecutive switching of segments required at the transgression points
//        // quadrants are labeled by integers: NE:0, SE:1, SW:2, NW:3 for the figure-eight pattern with crossing point in the direction (0,0,-1) and east-west attitude;
//        // turning circle centers for each quadrant
//        Vector3f quadrants_cv[4] = {erc1v * dist_cm, erc1v * dist_cm, erc2v * dist_cm, erc2v * dist_cm};
//        // unit tangent vectors at the transgression point of each quadrant
//        Vector3f quadrants_tv[4] = {etg1c1v, etc1g2v, etg2c2v, etc2g1v};
//        // set current quadrant in dependence of the location of the aircraft
//        int8_t _current_quadrant = quadrant(rav);
//        // selecting the center vector of the turning circle and tangent vector at transgression point for the current quadrant
//        Vector3f _current_cv = quadrants_cv[_current_quadrant];
//        Vector3f _current_tv = quadrants_tv[_current_quadrant];
//        // position vector from center of the current turning circle to the aircraft
//        Vector3f _rcav = rav - _current_cv;
//        if (_rcav * _current_tv >= 0){
//          // if _rcav * _current_tv >= 0, the switching to the next segment as determined by the orientation of the figure_eight pattern has to occur
//          // if vav * _current_tv >= 0: the aircraft moves along the figure_eight pattern as prescribed by orientation
//          //                       < 0: the aircraft moves along the figure_eight pattern in the opposite direction against the prescribed by orientation
//          // => temporally next segment is segment = segment + sign(vav * _current_tv)
//          in_right_direction = bool(vav * _current_tv >= 0);
//          switch_to_next_segment = bool(_rcav * _current_tv>= 0);
//          int8_t step;
//          if(in_right_direction){step = 1;} else {step = -1;}
//          _current_segment = _current_segment + step;
//        }
//      }
//      current_segment = _current_segment % 4;
//      }
//


//      bool sufficient_dist_to_crossing_point;
//      bool in_right_direction;
//      bool switch_to_next_segment;
//
//      // set the current segment in dependence of the position relative to the center of the sphere and velocity vector of the aircraft
//      void set_current_segment(const Vector3f rav, const Vector3f vav) {
//          // quadrants are labeled by integers: NE:0, SE:1, SW:2, NW:3 for the figure-eight pattern with crossing point in the direction (0,0,-1) and east-west attitude;
//          // turning circle centers for each quadrant
//          Vector3f quadrants_cv[4] = {erc1v * dist_cm, erc1v * dist_cm, erc2v * dist_cm, erc2v * dist_cm};
//          // unit tangent vectors at the transgression point of each quadrant
//          Vector3f quadrants_tv[4] = {etg1c1v, etc1g2v, etg2c2v, etc2g1v};
//          // set current quadrant in dependence of the location of the aircraft
//          int8_t _current_quadrant = quadrant(rav);
//          // selecting the center vector of the turning circle and tangent vector at transgression point for the current quadrant
//          Vector3f _current_cv = quadrants_cv[_current_quadrant];
//          Vector3f _current_tv = quadrants_tv[_current_quadrant];
//          // position vector from center of the current turning circle to the aircraft
//          Vector3f _rcav = rav - _current_cv;
//          // position vector from the center of the S2 to the aircraft projected onto the tangential plane
//          Vector3f _rxaplanev = rxaplanev(rav);
//          // minimum distance of aircraft from crossing point in the plane at which segment switching is allowed
//          float _mindistxaplane = 0.5f * S2_radius_cm * sin_theta_0; // set to half of length of each of the four geodesic arms projected onto the tangential plane at the crossing point
//          // decide if  aircraft moves (roughly) in the direction foreseen by the orientation of the current segment (expressed via _current_tv)
//          // and has moved sufficiently (more than _mindistxaplane) away from crossing point be safely allow for segment switching
//          int8_t _current_segment = current_segment;
//          sufficient_dist_to_crossing_point = bool(_rxaplanev.length() >= _mindistxaplane);
//          in_right_direction = bool(vav * _current_tv >= 0);
//          switch_to_next_segment = bool(_rcav * _current_tv>= 0);
//          if (vav * _current_tv >= 0) {
//              // aircraft moves (roughly) in the right direction
//              // switch to the next segment if
//              // 1) aircraft is sufficiently away from the crossing point (such that the correct quadrant is selected)
//              // 2) aircraft has passes the relevant transgression point
//              // switch to next segment if the we pass the relevant transgression point
//              // determined by a sign change of the projection of _rav on _current_tv
//              if (_rxaplanev.length() >= _mindistxaplane && _rcav * _current_tv >= 0) {
//                  // switch to next segment
//                  _current_segment++;
//                  // if the orientation = -1, the order of the segments 0,1,2,3 has to be mapped to 0,3,2,1
//                  if (orientation == -1) { current_segment = 4 - current_segment;}
//                  _current_segment = _current_segment % 4;
//              }
//          } else {
//              // aircraft moves into the wrong direction because
//              // 1) of the initial conditions at initialization time (quadrant determined by rav and flight direction determined by vav are incompatible with chosen orientation)
//              // 2) in close vicinity to crossing point the quadrant and hence geodesic segment has (accidentally) swapped
//              // -> switch to geodesic segment that is not the one of the quadrant the aircraft is located right now
//              if (_current_quadrant == 1 or _current_quadrant == 3) {
//                  // geodesic segment is g1, switch to g2: this is component 2 of the vectors segments_ercv and segments_theta_r
//                  _current_segment = 2;
//              } else {
//                  // geodesic segment is g2, switch to g1: this is component 0 of the vectors segments_ercv and segments_theta_r
//                  _current_segment = 0;
//              }
//          }
//          current_segment =_current_segment;
//      }

    }   eight_in_S2;



// code: Thomas Gehrmann
    struct {
        // rotation around e_z and e_y
        float psi_plane;
        float theta_plane;

        int32_t distance_cm;         // distance between plane and origin in cm

        Vector3f normal_vec;    // normal vector, perpendicular to plane spanned by rotated e_y and e_x
        struct Location circle_center;

        int32_t sphere_radius_cm;    // radius of sphere in cm
        float circle_radius;    // radius of cross section between sphere and plane in m

        Matrix3f rot_matrix_pe;    // describes an earth frame vector in the new plane frame

//        int32_t height;           // z position on circle
        Location desired_loc;        // location of desired point on circle

        float eccent;              // eccentricity of circle projection (ellipse)

    } intersection;

// code: Thomas Gehrmann
    struct {

        float omega;              // rotation of eight around e_z in ef, omega equals winddirection
        float omega_old;            // check if omega has changed during flight
        float sigma;                // rotation of eight around e_y'

        float cross_angle;       // half angle of crossing path
        float arc_length_angle;

        float eta;            // angle of intersection circle

        float secant;           // distance between two ending points

        float sector_angle;     // 2*sector_angle + pi = sector of circle

        Vector3f normal_vec;    // Unitvector to circle center
        //Matrix3f rot_8_matrix;  // rotates (active) the whole eight around omega and eta

        Matrix3f rot_matrix_right; // describes an earth frame vector in the plane frame of right turning circles
        Matrix3f rot_matrix_left;
        Matrix3f rot_matrix_cross1;
        Matrix3f rot_matrix_cross2;

        struct Location circle_center_left;
        struct Location circle_center_right;

        int32_t segment = 0;         // defines on which segment of the eight the plane is

    } eight_sphere;



    // Conditional command
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    int32_t condition_value;

    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    uint32_t condition_start;
    // A value used in condition commands.  For example the rate at which to change altitude.
    int16_t condition_rate;

    // 3D Location vectors
    // Location structure defined in AP_Common
    const struct Location &home = ahrs.get_home();

    // Flag for if we have g_gps lock and have set the home location in AHRS
    enum HomeState home_is_set = HOME_UNSET;

    // The location of the previous waypoint.  Used for track following and altitude ramp calculations
    Location prev_WP_loc {};

    // The plane's current location
    struct Location current_loc {};

    // The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
    Location next_WP_loc {};

    // The location of the active waypoint in Guided mode.
    struct Location guided_WP_loc {};

    // Altitude control
    struct {
        // target altitude above sea level in cm. Used for barometric
        // altitude navigation
        int32_t amsl_cm;

        // Altitude difference between previous and current waypoint in
        // centimeters. Used for glide slope handling
        int32_t offset_cm;

#if AP_TERRAIN_AVAILABLE
        // are we trying to follow terrain?
        bool terrain_following;

        // target altitude above terrain in cm, valid if terrain_following
        // is set
        int32_t terrain_alt_cm;

        // lookahead value for height error reporting
        float lookahead;
#endif
    } target_altitude {};

    float relative_altitude = 0.0f;

    // INS variables
    // The main loop execution time.  Seconds
    // This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
    float G_Dt = 0.02f;

    struct {
        // Performance monitoring
        // Timer used to accrue data and trigger recording of the performanc monitoring log message
        uint32_t start_ms;

        // The maximum and minimum main loop execution time recorded in the current performance monitoring interval
        uint32_t G_Dt_max;
        uint32_t G_Dt_min;

        // System Timers
        // Time in microseconds of start of main control loop
        uint32_t fast_loopTimer_us;

        // Number of milliseconds used in last main loop cycle
        uint32_t delta_us_fast_loop;

        // Counter of main loop executions.  Used for performance monitoring and failsafe processing
        uint16_t mainLoop_count;

        // number of long loops
        uint16_t num_long;

        // accumulated lost log messages
        uint32_t last_log_dropped;
    } perf;

    struct {
        uint32_t last_trim_check;
        uint32_t last_trim_save;
    } auto_trim;

    // last time home was updated while disarmed
    uint32_t last_home_update_ms;
    
    // Camera/Antenna mount tracking and stabilisation stuff
#if MOUNT == ENABLED
    // current_loc uses the baro/gps soloution for altitude rather than gps only.
    AP_Mount camera_mount {ahrs, current_loc};
#endif

    // Arming/Disarming mangement class
    AP_Arming_Plane arming {ahrs, barometer, compass, battery};

    AP_Param param_loader {var_info};

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];

    // use this to prevent recursion during sensor init
    bool in_mavlink_delay = false;

    // true if we are out of time in our event timeslice
    bool gcs_out_of_time = false;

    // time that rudder arming has been running
    uint32_t rudder_arm_timer;

    // support for quadcopter-plane
    QuadPlane quadplane{ahrs};

    // support for transmitter tuning
    AP_Tuning_Plane tuning;

    static const struct LogStructure log_structure[];
    
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // the crc of the last created PX4Mixer
    int32_t last_mixer_crc = -1;
#endif // CONFIG_HAL_BOARD
    
    void adjust_nav_pitch_throttle(void);
    void update_load_factor(void);
    void send_heartbeat(mavlink_channel_t chan);
    void send_attitude(mavlink_channel_t chan);
    void send_fence_status(mavlink_channel_t chan);
    void update_sensor_status_flags(void);
    void send_extended_status1(mavlink_channel_t chan);
    void send_location(mavlink_channel_t chan);
    void send_nav_controller_output(mavlink_channel_t chan);
    void send_position_target_global_int(mavlink_channel_t chan);
    void send_servo_out(mavlink_channel_t chan);
    void send_vfr_hud(mavlink_channel_t chan);
    void send_simstate(mavlink_channel_t chan);
    void send_hwstatus(mavlink_channel_t chan);
    void send_wind(mavlink_channel_t chan);
    void send_pid_tuning(mavlink_channel_t chan);
    void send_rpm(mavlink_channel_t chan);
    void send_rangefinder(mavlink_channel_t chan);
    void send_current_waypoint(mavlink_channel_t chan);
    bool telemetry_delayed(mavlink_channel_t chan);
    void gcs_send_message(enum ap_message id);
    void gcs_send_mission_item_reached_message(uint16_t mission_index);
    void gcs_data_stream_send(void);
    void gcs_update(void);
    void gcs_send_text(MAV_SEVERITY severity, const char *str);
    void gcs_send_airspeed_calibration(const Vector3f &vg);
    void gcs_retry_deferred(void);

    void do_erase_logs(void);
    void Log_Write_Fast(void);
    void Log_Write_Attitude(void);
    void Log_Write_Performance();
    void Log_Write_Startup(uint8_t type);
    void Log_Write_Control_Tuning();
    void Log_Write_Nav_Tuning();
    void Log_Write_Status();
    void Log_Write_Sonar();
    void Log_Write_Optflow();
    void Log_Write_Current();
    void Log_Arm_Disarm();
    void Log_Write_GPS(uint8_t instance);
    void Log_Write_IMU();
    void Log_Write_RC(void);
    void Log_Write_Baro(void);
    void Log_Write_Airspeed(void);
    void Log_Write_Home_And_Origin();
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Read(uint16_t log_num, int16_t start_page, int16_t end_page);
    void start_logging();

    void load_parameters(void);
    void adjust_altitude_target();
    void setup_glide_slope(void);
    int32_t get_RTL_altitude();
    float relative_ground_altitude(bool use_rangefinder_if_available);
    void set_target_altitude_current(void);
    void set_target_altitude_current_adjusted(void);
    void set_target_altitude_location(const Location &loc);
    int32_t relative_target_altitude_cm(void);
    void change_target_altitude(int32_t change_cm);
    void set_target_altitude_proportion(const Location &loc, float proportion);
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2);
    int32_t calc_altitude_error_cm(void);
    void check_minimum_altitude(void);
    void reset_offset_altitude(void);
    void set_offset_altitude_location(const Location &loc);
    bool above_location_current(const Location &loc);
    void setup_terrain_target_alt(Location &loc);
    int32_t adjusted_altitude_cm(void);
    int32_t adjusted_relative_altitude_cm(void);
    float mission_alt_offset(void);
    float height_above_target(void);
    float lookahead_adjustment(void);
    float rangefinder_correction(void);
    void rangefinder_height_update(void);
    void set_next_WP(const struct Location &loc);
    void set_guided_WP(void);
    void init_home();
    void update_home();
    void do_RTL(int32_t alt);
    bool verify_takeoff();
    bool verify_loiter_unlim();
    bool verify_loiter_time();
    bool verify_loiter_turns();
    bool verify_loiter_to_alt();
    bool verify_RTL();
    bool verify_continue_and_change_alt();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_altitude_wait(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(const AP_Mission::Mission_Command &cmd);
    void do_loiter_at_location();
    void do_loiter_ellipse();
    void do_eight_plane();
    void do_loiter_3d();
    void do_eight_sphere();
    void do_take_picture();
    bool verify_loiter_heading(bool init);
    void log_picture();
    void exit_mission_callback();
    void mavlink_delay(uint32_t ms);
    void read_control_switch();
    uint8_t readSwitch(void);
    void reset_control_switch();
    void autotune_start(void);
    void autotune_restore(void);
    void autotune_enable(bool enable);
    bool fly_inverted(void);
    void failsafe_short_on_event(enum failsafe_state fstype, mode_reason_t reason);
    void failsafe_long_on_event(enum failsafe_state fstype, mode_reason_t reason);
    void failsafe_short_off_event(mode_reason_t reason);
    void low_battery_event(void);
    void update_events(void);
    uint8_t max_fencepoints(void);
    Vector2l get_fence_point_with_index(unsigned i);
    void set_fence_point_with_index(Vector2l &point, unsigned i);
    void geofence_load(void);
    bool geofence_present(void);
    void geofence_update_pwm_enabled_state();
    bool geofence_set_enabled(bool enable, GeofenceEnableReason r);
    bool geofence_enabled(void);
    bool geofence_set_floor_enabled(bool floor_enable);
    bool geofence_check_minalt(void);
    bool geofence_check_maxalt(void);
    void geofence_check(bool altitude_check_only);
    bool geofence_stickmixing(void);
    void geofence_send_status(mavlink_channel_t chan);
    bool geofence_breached(void);
    void disarm_if_autoland_complete();
    float tecs_hgt_afe(void);
    void set_nav_controller(void);
    void loiter_angle_reset(void);
    void loiter_angle_update(void);
    void navigate();
    void calc_airspeed_errors();
    void calc_gndspeed_undershoot();
    void update_loiter(uint16_t radius);
    void update_loiter_ellipse();
    void update_eight_plane();
    void update_loiter_3d();
    void update_eight_sphere();
    void update_cruise();
    void update_fbwb_speed_height(void);
    void setup_turn_angle(void);
    bool reached_loiter_target(void);
    bool print_buffer(char *&buf, uint16_t &buf_size, const char *fmt, ...);
    uint16_t create_mixer(char *buf, uint16_t buf_size, const char *filename);
    bool setup_failsafe_mixing(void);
    void set_control_channels(void);
    void init_rc_in();
    void init_rc_out_main();
    void init_rc_out_aux();
    void rudder_arm_disarm_check();
    void read_radio();
    void control_failsafe(uint16_t pwm);
    void trim_control_surfaces();
    void trim_radio();
    bool rc_failsafe_active(void);
    void init_barometer(bool full_calibration);
    void init_rangefinder(void);
    void read_rangefinder(void);
    void read_airspeed(void);
    void zero_airspeed(bool in_startup);
    void read_battery(void);
    void read_receiver_rssi(void);
    void rpm_update(void);
    void button_update(void);
    void stats_update();
    void ice_update(void);
    void report_radio();
    void report_ins();
    void report_compass();
    void print_radio_values();
    void print_done();
    void print_blanks(int16_t num);
    void print_divider(void);
    void zero_eeprom(void);
    void print_enabled(bool b);
    void print_accel_offsets_and_scaling(void);
    void print_gyro_offsets(void);
    void init_ardupilot();
    void startup_ground(void);
    enum FlightMode get_previous_mode();
    void set_mode(enum FlightMode mode, mode_reason_t reason);
    bool mavlink_set_mode(uint8_t mode);
    void exit_mode(enum FlightMode mode);
    void check_long_failsafe();
    void check_short_failsafe();
    void startup_INS_ground(void);
    void update_notify();
    void resetPerfData(void);
    void check_usb_mux(void);
    void print_comma(void);
    bool should_log(uint32_t mask);
    int8_t throttle_percentage(void);
    void change_arm_state(void);
    bool disarm_motors(void);
    bool arm_motors(AP_Arming::ArmingMethod method);
    bool auto_takeoff_check(void);
    void takeoff_calc_roll(void);
    void takeoff_calc_pitch(void);
    int8_t takeoff_tail_hold(void);
    int16_t get_takeoff_pitch_min_cd(void);
    void complete_auto_takeoff(void);
    void print_hit_enter();
    void ahrs_update();
    void update_speed_height(void);
    void update_GPS_50Hz(void);
    void update_GPS_10Hz(void);
    void update_compass(void);
    void update_alt(void);
    void afs_fs_check(void);
    void compass_accumulate(void);
    void compass_cal_update();
    void barometer_accumulate(void);
    void update_optical_flow(void);
    void one_second_loop(void);
    void airspeed_ratio_update(void);
    void update_mount(void);
    void update_trigger(void);    
    void log_perf_info(void);
    void compass_save(void);
    void update_logging1(void);
    void update_logging2(void);
    void terrain_update(void);
    void avoidance_adsb_update(void);
    void update_flight_mode(void);
    void stabilize();
    void set_servos_idle(void);
    void set_servos();
    void set_servos_manual_passthrough(void);
    void set_servos_controlled(void);
    void set_servos_old_elevons(void);
    void set_servos_flaps(void);
    void servo_output_mixers(void);
    void servos_output(void);
    void servos_auto_trim(void);
    void servos_twin_engine_mix();
    void throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle);
    bool allow_reverse_thrust(void);
    void update_aux();
    void update_is_flying_5Hz(void);
    void crash_detection_update(void);
    bool in_preLaunch_flight_stage(void);
    void gcs_send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...);
    void handle_auto_mode(void);
    void calc_throttle();
    void calc_nav_roll();
    void calc_nav_pitch();
    void update_flight_stage();
    void update_navigation();
    void set_flight_stage(AP_Vehicle::FixedWing::FlightStage fs);
    bool is_flying(void);
    float get_speed_scaler(void);
    bool stick_mixing_enabled(void);
    void stabilize_roll(float speed_scaler);
    void stabilize_pitch(float speed_scaler);
    static void stick_mix_channel(RC_Channel *channel, int16_t &servo_out);
    void stabilize_stick_mixing_direct();
    void stabilize_stick_mixing_fbw();
    void stabilize_yaw(float speed_scaler);
    void stabilize_training(float speed_scaler);
    void stabilize_acro(float speed_scaler);
    void calc_nav_yaw_coordinated(float speed_scaler);
    void calc_nav_yaw_course(void);
    void calc_nav_yaw_ground(void);
    void throttle_slew_limit(void);
    bool suppress_throttle(void);
    void channel_output_mixer_pwm(uint8_t mixing_type, uint16_t & chan1, uint16_t & chan2)const;
    void channel_output_mixer(uint8_t mixing_type, SRV_Channel::Aux_servo_function_t servo1, SRV_Channel::Aux_servo_function_t servo2);
    void flaperon_update(int8_t flap_percent);
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_loiter_turns(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_altitude_wait(const AP_Mission::Mission_Command& cmd);
    void do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_digicam_configure(const AP_Mission::Mission_Command& cmd);
    void do_digicam_control(const AP_Mission::Mission_Command& cmd);
    bool start_command_callback(const AP_Mission::Mission_Command &cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);
    void notify_flight_mode(enum FlightMode mode);
    void run_cli(AP_HAL::UARTDriver *port);
    void log_init();
    void init_capabilities(void);
    void dataflash_periodic(void);
    void parachute_check();
#if PARACHUTE == ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
    void parachute_release();
    bool parachute_manual_release();
#endif
    void accel_cal_update(void);
    void update_soft_armed();
    void update_soaring();

    // support for AP_Avoidance custom flight mode, AVOID_ADSB
    bool avoid_adsb_init(bool ignore_checks);
    void avoid_adsb_run();

public:
    void mavlink_delay_cb();
    void failsafe_check(void);
    bool print_log_menu(void);
    int8_t dump_log(uint8_t argc, const Menu::arg *argv);
    int8_t erase_logs(uint8_t argc, const Menu::arg *argv);
    int8_t select_logs(uint8_t argc, const Menu::arg *argv);
    int8_t process_logs(uint8_t argc, const Menu::arg *argv);
    int8_t setup_mode(uint8_t argc, const Menu::arg *argv);
    int8_t setup_factory(uint8_t argc, const Menu::arg *argv);
    int8_t setup_erase(uint8_t argc, const Menu::arg *argv);
    int8_t test_mode(uint8_t argc, const Menu::arg *argv);
    int8_t reboot_board(uint8_t argc, const Menu::arg *argv);
    int8_t main_menu_help(uint8_t argc, const Menu::arg *argv);
    int8_t test_radio_pwm(uint8_t argc, const Menu::arg *argv);
    int8_t test_radio(uint8_t argc, const Menu::arg *argv);
    int8_t test_failsafe(uint8_t argc, const Menu::arg *argv);
    int8_t test_relay(uint8_t argc, const Menu::arg *argv);
    int8_t test_wp(uint8_t argc, const Menu::arg *argv);
    void test_wp_print(const AP_Mission::Mission_Command& cmd);
    int8_t test_xbee(uint8_t argc, const Menu::arg *argv);
    int8_t test_modeswitch(uint8_t argc, const Menu::arg *argv);
    int8_t test_logging(uint8_t argc, const Menu::arg *argv);
    int8_t test_gps(uint8_t argc, const Menu::arg *argv);
    int8_t test_ins(uint8_t argc, const Menu::arg *argv);
    int8_t test_mag(uint8_t argc, const Menu::arg *argv);
    int8_t test_airspeed(uint8_t argc, const Menu::arg *argv);
    int8_t test_pressure(uint8_t argc, const Menu::arg *argv);
    int8_t test_shell(uint8_t argc, const Menu::arg *argv);
};

#define MENU_FUNC(func) FUNCTOR_BIND(&plane, &Plane::func, int8_t, uint8_t, const Menu::arg *)

extern const AP_HAL::HAL& hal;
extern Plane plane;

using AP_HAL::millis;
using AP_HAL::micros;
