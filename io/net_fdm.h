// net_fdm.hxx -- defines a common net I/O interface to the flight
//                dynamics model
//
// Written by Curtis Olson - http://www.flightgear.org/~curt
// Started September 2001.
//
// This file is in the Public Domain, and comes with no warranty.
//
// $Id$

#ifndef _NET_FDM_HXX
#define _NET_FDM_HXX

#include <math.h>

const unsigned int FG_NET_FDM_VERSION = 24;

enum {
	FG_MAX_ENGINES = 4,
	FG_MAX_WHEELS = 3,
	FG_MAX_TANKS = 4
};


// Define a structure containing the top level flight dynamics model
// parameters

#pragma pack(4)
struct FDMDATA{

    unsigned int version;		// increment when data values change
    unsigned int padding;		// padding

    // Positions
    double longitude;		// geodetic (radians)
    double latitude;		// geodetic (radians)
    double altitude;		// above sea level (meters)
    float agl;			// above ground level (meters)
    float phi;			// roll (radians)
    float theta;		// pitch (radians)
    float psi;			// yaw or true heading (radians)
    float alpha;                // angle of attack (radians)
    float beta;                 // side slip angle (radians)

    // Velocities
    float phidot;		// roll rate (radians/sec)
    float thetadot;		// pitch rate (radians/sec)
    float psidot;		// yaw rate (radians/sec)
    float vcas;		        // calibrated airspeed
    float climb_rate;		// feet per second
    float v_north;              // north velocity in local/body frame, fps
    float v_east;               // east velocity in local/body frame, fps
    float v_down;               // down/vertical velocity in local/body frame, fps
    float v_wind_body_north;    // north velocity in local/body frame
                                // relative to local airmass, fps
    float v_wind_body_east;     // east velocity in local/body frame
                                // relative to local airmass, fps
    float v_wind_body_down;     // down/vertical velocity in local/body
                                // frame relative to local airmass, fps

    // Accelerations
    float A_X_pilot;		// X accel in body frame ft/sec^2
    float A_Y_pilot;		// Y accel in body frame ft/sec^2
    float A_Z_pilot;		// Z accel in body frame ft/sec^2

    // Stall
    float stall_warning;        // 0.0 - 1.0 indicating the amount of stall
    float slip_deg;		// slip ball deflection

    // Pressure
    
    // Engine status
    unsigned int  num_engines;	     // Number of valid engines
    unsigned int  eng_state[FG_MAX_ENGINES];// Engine state (off, cranking, running)
    float rpm[FG_MAX_ENGINES];	     // Engine RPM rev/min
    float fuel_flow[FG_MAX_ENGINES]; // Fuel flow gallons/hr
    float fuel_px[FG_MAX_ENGINES];   // Fuel pressure psi
    float egt[FG_MAX_ENGINES];	     // Exhuast gas temp deg F
    float cht[FG_MAX_ENGINES];	     // Cylinder head temp deg F
    float mp_osi[FG_MAX_ENGINES];    // Manifold pressure
    float tit[FG_MAX_ENGINES];	     // Turbine Inlet Temperature
    float oil_temp[FG_MAX_ENGINES];  // Oil temp deg F
    float oil_px[FG_MAX_ENGINES];    // Oil pressure psi

    // Consumables
    unsigned int  num_tanks;		// Max number of fuel tanks
    float fuel_quantity[FG_MAX_TANKS];

    // Gear status
    unsigned int  num_wheels;
    unsigned int  wow[FG_MAX_WHEELS];
    float gear_pos[FG_MAX_WHEELS];
    float gear_steer[FG_MAX_WHEELS];
    float gear_compression[FG_MAX_WHEELS];

    // Environment
    unsigned int  cur_time;           // current unix time
                                 // FIXME: make this uint64_t before 2038
    unsigned int  warp;                // offset in seconds to unix time
    float visibility;            // visibility in meters (for env. effects)

    // Control surface positions (normalized values)
    float elevator;
    float elevator_trim_tab;
    float left_flap;
    float right_flap;
    float left_aileron;
    float right_aileron;
    float rudder;
    float nose_wheel;
    float speedbrake;
    float spoilers;
};
#pragma pack()

union PACKSEND{
	FDMDATA fdm;
	char buff[408];
};

#define MAX_MODEL_NAME_LEN      96
#define MAX_CALLSIGN_LEN 8

#pragma pack(1)
struct T_MsgHdr {
    unsigned int  Magic;                  // Magic Value
    unsigned int  Version;                // Protocoll version
    unsigned int  MsgId;                  // Message identifier 
    unsigned int  MsgLen;                 // absolute length of message
    unsigned int  ReplyAddress;           // (player's receiver address
    unsigned int  ReplyPort;              // player's receiver port
    char Callsign[MAX_CALLSIGN_LEN];    // Callsign used by the player
};

// Position message
#pragma pack(1)
struct T_PositionMsg {
    char Model[MAX_MODEL_NAME_LEN];    // Name of the aircraft model

    // Time when this packet was generated
    unsigned long long time;
    unsigned long long lag;

    // position wrt the earth centered frame
    unsigned long long position[3];
    // orientation wrt the earth centered frame, stored in the angle axis
    // representation where the angle is coded into the axis length
    unsigned int orientation[3];

    // linear velocity wrt the earth centered frame measured in
    // the earth centered frame
    unsigned int linearVel[3];
    // angular velocity wrt the earth centered frame measured in
    // the earth centered frame
    unsigned int angularVel[3];

    // linear acceleration wrt the earth centered frame measured in
    // the earth centered frame
    unsigned int linearAccel[3];
    // angular acceleration wrt the earth centered frame measured in
    // the earth centered frame
    unsigned int angularAccel[3];
    // Padding. The alignment is 8 bytes on x86_64 because there are
    // 8-byte types in the message, so the size should be explicitly
    // rounded out to a multiple of 8. Of course, it's a bad idea to
    // put a C struct directly on the wire, but that's a fight for
    // another day...
    unsigned int pad;
};

union T_pro{
	float f;
	unsigned int x;
	unsigned char byte[4];
};

#pragma pack(1)
struct T_MsgPro{
    unsigned int  index;                  // Magic Value
    T_pro  value;                // Protocoll version
};

#pragma pack(1)
struct T_MsgTail {
	T_MsgPro p100;
	T_MsgPro p101;
	T_MsgPro p102;
	T_MsgPro p103;
	T_MsgPro p104;
	T_MsgPro p105;
	T_MsgPro p106;
	//T_MsgPro p107;
	T_MsgPro p108;
	//T_MsgPro p109;
	//T_MsgPro p110;
	//T_MsgPro p111;
	T_MsgPro p112;

	T_MsgPro p200;
	T_MsgPro p201;
	T_MsgPro p210;
	T_MsgPro p211;
	T_MsgPro p220;
	T_MsgPro p221;
	T_MsgPro p230;
	T_MsgPro p231;
	T_MsgPro p240;
	T_MsgPro p241;
	T_MsgPro p302;
	T_MsgPro p312;
	T_MsgPro p322;
	T_MsgPro p330;
	T_MsgPro p331;
	T_MsgPro p332;
	T_MsgPro p340;
	T_MsgPro p341;
	T_MsgPro p342;
	T_MsgPro p350;
	T_MsgPro p351;
	T_MsgPro p352;
	T_MsgPro p360;
	T_MsgPro p361;
	T_MsgPro p362;
	T_MsgPro p370;
	T_MsgPro p371;
	T_MsgPro p372;
	T_MsgPro p380;
	T_MsgPro p381;
	T_MsgPro p382;
	T_MsgPro p390;
	T_MsgPro p391;
	T_MsgPro p392;
	T_MsgPro p800;
  T_MsgPro p801;
  T_MsgPro p810;
  T_MsgPro p811;
  T_MsgPro p812;
  T_MsgPro p813;
  T_MsgPro p820;
  T_MsgPro p821;
  T_MsgPro p822;
  T_MsgPro p823;
  T_MsgPro p830;
  T_MsgPro p831;

  T_MsgPro p1001;
  T_MsgPro p1002;
  T_MsgPro p1003;
  T_MsgPro p1004;
  T_MsgPro p1005;
  T_MsgPro p1006;
  T_MsgPro p1100;

  unsigned int  p1101Index;  
  unsigned char p1101[17*4];//string

  T_MsgPro p1200;//string,but is zero
  T_MsgPro p1201;

  T_MsgPro p1400;

  T_MsgPro p1500;
  T_MsgPro p1501;
  T_MsgPro p1502;
  T_MsgPro p1503;

  unsigned int  p10001Index; 
  unsigned char p10001[13*4];//string
  unsigned int  p10002Index; 
  unsigned char p10002[9*4];//string
T_MsgPro p10100;
  T_MsgPro p10200;
   T_MsgPro p10201;
    T_MsgPro p10202;
	 T_MsgPro p10203;
	  T_MsgPro p10204;
	   T_MsgPro p10205;
	    T_MsgPro p10206;
		 T_MsgPro p10207;
		  T_MsgPro p10208;
		   T_MsgPro p10209;
		    T_MsgPro p10210;
			 T_MsgPro p10211;
			 T_MsgPro p10212;
			 T_MsgPro p10213;
			 T_MsgPro p10214;
			 T_MsgPro p10215;
			 T_MsgPro p10216;
			 T_MsgPro p10217;

	 T_MsgPro p10303;
	 T_MsgPro p10304;
	 T_MsgPro p10305;
	 T_MsgPro p10306;
	 T_MsgPro p10307;
	 T_MsgPro p10308;
	 T_MsgPro p10309;
	 T_MsgPro p10310;
	 T_MsgPro p10311;
	 T_MsgPro p10312;
	 T_MsgPro p10313;
};

#pragma pack(1)
struct T_MsgStruct{
	T_MsgHdr header;
	T_PositionMsg pos;
	T_MsgTail tail;
};

union T_Msg{
	T_MsgStruct msg;
	unsigned char buff[1192];
};
#endif // _NET_FDM_HXX
