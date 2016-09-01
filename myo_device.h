//
// Thalmic Myo Armband Plugin for Autodesk Stingray 
//
// Author: Hans Kellner
//

#pragma once

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

#include <plugin_foundation/types.h>
#include <plugin_foundation/plugin_api.h>

#include <array>
#include <vector>
#include <string>

namespace myo_device {
	using namespace stingray_plugin_foundation;

	enum VibrateTypes {
		VIBRATE_SHORT	= 0,
		VIBRATE_MEDIUM	= 1,
		VIBRATE_LONG	= 2
	};

	enum ArmTypes {
		ARM_LEFT	= 0,
		ARM_RIGHT	= 1,
		ARM_UNKNOWN	= 2
	};
	
	enum DirectionTypes {
		DIRECTION_TOWARD_WRIST	= 0,
		DIRECTION_TOWARD_ELBOW	= 1,
		DIRECTION_UNKNOWN   	= 2
	};
	
	enum WarmupStateTypes {
		WARMUP_STATE_UNKNOWN    = 0,
		WARMUP_STATE_COLD	    = 1,
		WARMUP_STATE_WARM	    = 2
	};
	
	enum PoseTypes {
        POSE_REST			= 0,
        POSE_FIST			= 1,
        POSE_WAVEIN			= 2,
        POSE_WAVEOUT		= 3,
        POSE_FINGERSSPREAD	= 4,
        POSE_DOUBLETAP		= 5,
        POSE_UNKNOWN		= 6
    };
	
	struct Information {
		unsigned int	arm;
		unsigned int	batteryLevel;
        unsigned int    rssi;
		bool			connected;
		unsigned int	connectVersion[4];
		unsigned int	direction;
		bool			locked;
		bool			synced;
		unsigned int	warmupState;
	};

	struct NodeLink {
		int			node;
		World*		world;
	};

	// Initializes/shutdown the lib. This must be called prior to creating
	// a device and calling any other Myo functions.
	bool            initialize_lib();
	void            shutdown_lib();

	// Get Myo sdk/runtime version
	const char*     sdk_version();

	// Initialize/shutdown Myo.
	bool            initialize();
	bool            isInitialized();
	void            shutdown();

    // Attempts to connect to a Myo device.  Returns true if connected; otherwise false.
    // May be called more than once.  For example, for multiple Myos.
    // @arg unsigned int timeout_ms Timeout in milliseconds to wait for a device
    bool            connect(unsigned int timeout_ms);

    // Poll for data from connected devices for the specified amount of time (milliseconds)
	void            run(unsigned int length);
	
    // Returns the number of paired Myo devices
    int             countMyos();

    // Returns the set of Ids of paired Myo devices
    void            idsMyos(std::vector<unsigned int>& ids);

	// Lock/Unlock
	bool            lock(unsigned int id);
	bool            unlock(unsigned int id, bool timed);
	
	// Vibrate
	bool            vibrate(unsigned int id, unsigned int length);
	
	// Last pose (gesture).  See PoseTypes for values.
	int             pose(unsigned int id);
	
	// Last pose (gesture) as a string.
	bool            poseToString(unsigned int id, std::string& str);
	
	// Enable/Disable EMG data
    // REVIEW: Quirks with Myo SDK as to when this can be called.
    bool            enableEMGData(unsigned int id, bool flag);

	// Get the EMG data
	bool            emgData(unsigned int id, std::array<int8_t, 8>& emg);
	
	// Get orientation
	bool            orientation(unsigned int id, Matrix4x4 &pose, bool world);
	
	// Set the world orientation that's applied to the Myo device orientation when returned
    // by orientation() with the param 'world' == true
	void            set_world_orientation(const Matrix4x4 &world);
	
	// Get the orientation as roll, pitch, and yaw
	bool            rollPitchYaw(unsigned int id, float& roll, float& pitch, float& yaw);
	
	// Get the gyroscope values
	bool            gyroscope(unsigned int id, Vector3 &v);
	
	// Get the accelerometer values
	bool            accelerometer(unsigned int id, Vector3 &v);
	
    // Request that information should be acquired from device.  This should be called before the call
    // to info().  Otherwise, values such as battery level and rssi may not be correct.
    bool            requestInfo(unsigned id);

	// Get information about the device.  Should be preceded by a call to requestInfo().
	bool            info(unsigned int id, Information& info);
	
	// Register node linked to device
	void            register_node_link(const NodeLink &link);
	
	// Set the current orientation as the origin
	void            zeroOrientation();
	
} // namespace myo_device
