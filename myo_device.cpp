//
// Thalmic Myo Armband Plugin for Autodesk Stingray 
//
// Author: Hans Kellner
//

#include "myo_device.h"
#include "plugin_interface.h"

#include <plugin_foundation/assert.h>
#include <plugin_foundation/vector3.h>
#include <plugin_foundation/quaternion.h>
#include <plugin_foundation/array.h>

#include <array>
#include <unordered_map>
//#include <sstream>

using namespace stingray_plugin_foundation;

namespace myo_device {
	using namespace myo_plugin;

    // Data associated with each Myo
    struct MyoData
    {
        // Local Id of Myo
        unsigned int            id;

        // These values are set by onArmSync() and onArmUnsync().
		bool                    isSynced;
		myo::Arm                whichArm;
		myo::XDirection         direction;
		myo::WarmupState        warmupState;

		bool                    isConnected;

        unsigned int            version[4];
		
        unsigned int            rssi;
		unsigned int            batteryLevel;
		
		// This is set by onUnlocked() and onLocked().
		bool                    isUnlocked;

		// These values are set by onOrientationData() and onPose().
		float                   roll, pitch, yaw;
		int                     roll_w, pitch_w, yaw_w;
		myo::Pose               currentPose;
		
		myo::Quaternion<float>  currentOrientation;
		myo::Vector3<float>     currentAccelerometer;
		myo::Vector3<float>     currentGyroscope;
		
		// The values of this array is set by onEmgData() above.
		std::array<int8_t, 8>   emgSamples;

        MyoData()
   			: id(0)
            , isSynced(false)
			, isUnlocked(false)
			, isConnected(false)
			, roll(0.0f)
			, pitch(0.0f)
			, yaw(0.0f)
			, roll_w(0)
			, pitch_w(0)
			, yaw_w(0)
			, currentPose()
			, emgSamples()
            , rssi(0)
			, batteryLevel(0)
        {}
    };

    typedef std::unordered_map<myo::Myo*, MyoData*> MyoDataMap; 

	// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
	// provides several virtual functions for handling different kinds of events. If you do not override an event, the
	// default behaviour is to do nothing.
	class DataCollector : public myo::DeviceListener
    {
	public:
        // We store each Myo pointer that we pair with in this list, so that we can keep track of the order we've seen
        // each Myo and give it a unique short identifier (see onPair() and identifyMyo()).
        MyoDataMap      _myos;
    
        // Keeps a counter of Myo Ids
        unsigned int    _idCounter;

	public:
        
        DataCollector() : _idCounter(0)
		{
		}

        ~DataCollector()
        {
            // Free memory used by MyoData
            for (MyoDataMap::const_iterator it = _myos.begin(); it != _myos.end(); ++it)
            {
                if (it->second)
                {
                    MAKE_DELETE(plugin_allocator(), it->second);
                }
            }
        }
        
        // This is a utility function implemented for this sample that maps a myo::Myo* to a unique ID starting at 1.
        // It does so by looking for the Myo pointer in knownMyos, which onPair() adds each Myo into as it is paired.
        MyoData* identifyMyo(myo::Myo* myo)
        {
            MyoData* pMyoData = nullptr;

            if (nullptr != myo)
            {
                // Look for this instance
                MyoDataMap::iterator it = _myos.find(myo);
                if (it == _myos.end())
                {
                    // Does not exist in map so add it and data struct
                    pMyoData = MAKE_NEW(plugin_allocator(), MyoData);
                    _idCounter++; pMyoData->id = _idCounter;
                    _myos.insert(MyoDataMap::value_type(myo, pMyoData));
                }
                else
                {
                    // Exists so get the data struct
                    pMyoData = it->second;
                }
            }

            return pMyoData;
        }

        // Remove the Myo and associated data from the set of known devices
        void unidentifyMyo(myo::Myo* myo)
        {
            if (nullptr != myo)
            {
                // Look for this instance
                MyoDataMap::const_iterator it = _myos.find(myo);
                if (it != _myos.end())
                {
                    // Found it.  Remove data struct and erase entry.
                    MAKE_DELETE(plugin_allocator(), it->second);
                    _myos.erase(it);
                }
            }
        }

        int countMyos() const
        {
            return (int)_myos.size();
        }

        myo::Myo* findMyo(unsigned int id)
        {
            if (0 == id)    // default to first device?
            {
                if (_myos.size() > 0)
                    return _myos.begin()->first;
            }
            else
            {
                // Look for Myo instance with id
                for (MyoDataMap::const_iterator it = _myos.begin(); it != _myos.end(); ++it)
                {
                    if (it->second->id == id)
                        return it->first;
                }
            }

            return nullptr;
        }

        MyoData* findMyoData(unsigned int id)
        {
            if (0 == id)    // default to first device?
            {
                if (_myos.size() > 0)
                    return _myos.begin()->second;
            }
            else
            {
                // Look for Myo instance with id
                for (MyoDataMap::const_iterator it = _myos.begin(); it != _myos.end(); ++it)
                {
                    if (it->second->id == id)
                        return it->second;
                }
            }

            return nullptr;
        }

        /////////////////////////////////////////////////////////////////////
        // Myo Listener Callbacks

		/// Called when a Myo has been paired.  Will only be called once for each device.
		void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
		{
            // Will add a new entry for this Myo and return data struct
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onPair : Failed to identify and register device");
                return;
            }

            // Now that we've added it to our list, get our short ID for it and print it out.
            plugin_log(INFO_LOG, "Myo : Paired device : ID = %d",pMyoData->id);
		}

		// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
		void onUnpair(myo::Myo* myo, uint64_t timestamp)
		{
            // Remove entry for this Myo
            unidentifyMyo(myo);

            plugin_log(INFO_LOG, "Myo : Unpaired");
		}
		
		/// Called when a paired Myo has been connected.
		void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onConnect : Failed to identify device");
                return;
            }

			pMyoData->isConnected = true;
			
			pMyoData->version[0] = firmwareVersion.firmwareVersionMajor;
			pMyoData->version[1] = firmwareVersion.firmwareVersionMinor;
			pMyoData->version[2] = firmwareVersion.firmwareVersionPatch;
			pMyoData->version[3] = firmwareVersion.firmwareVersionHardwareRev;

            myo->setStreamEmg( myo::Myo::streamEmgEnabled );

            plugin_log(INFO_LOG, "Myo : Connected : ID = %d", pMyoData->id);
		}

		/// Called when a paired Myo has been disconnected.
		void onDisconnect(myo::Myo* myo, uint64_t timestamp)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onDisconnect : Failed to identify device");
                return;
            }

			pMyoData->isConnected = false;			

            plugin_log(INFO_LOG, "Myo : Disconnected : ID = %d",pMyoData->id);
		}	
		
		// onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
		// arm. This lets Myo know which arm it's on and which way it's facing.
		void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
					   myo::WarmupState warmupState)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onArmSync : Failed to identify device");
                return;
            }

			pMyoData->isSynced = true;
			pMyoData->whichArm = arm;
            pMyoData->direction = xDirection;
            pMyoData->warmupState = warmupState;

            plugin_log(INFO_LOG, "Myo : Arm Synced : ID = %d",pMyoData->id);
		}

		// onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
		// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
		// when Myo is moved around on the arm.
		void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onArmUnsync : Failed to identify device");
                return;
            }

			pMyoData->isSynced = false;

            plugin_log(INFO_LOG, "Myo : Arm Un-synced : ID = %d",pMyoData->id);
		}

		// onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
		void onUnlock(myo::Myo* myo, uint64_t timestamp)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onUnlock : Failed to identify device");
                return;
            }

			pMyoData->isUnlocked = true;

            plugin_log(INFO_LOG, "Myo : Unlocked : ID = %d",pMyoData->id);
		}

		// onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
		void onLock(myo::Myo* myo, uint64_t timestamp)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onLock : Failed to identify device");
                return;
            }

			pMyoData->isUnlocked = false;

            plugin_log(INFO_LOG, "Myo : Locked : ID = %d",pMyoData->id);
		}

		// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
		void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onEmgData : Failed to identify device");
                return;
            }

			for (int i = 0; i < 8; i++) {
				pMyoData->emgSamples[i] = emg[i];
			}
		}
	
		// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
		// as a unit quaternion.
		void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
		{
			using std::atan2;
			using std::asin;
			using std::sqrt;
			using std::max;
			using std::min;
			
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onOrientationData : Failed to identify device");
                return;
            }

			pMyoData->currentOrientation = quat;

			// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
			pMyoData->roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
						   1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
			pMyoData->pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
			pMyoData->yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
						1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

			// Convert the floating point angles in radians to a scale from 0 to 18.
			pMyoData->roll_w = static_cast<int>((pMyoData->roll + (float)M_PI)/(M_PI * 2.0f) * 18);
			pMyoData->pitch_w = static_cast<int>((pMyoData->pitch + (float)M_PI/2.0f)/M_PI * 18);
			pMyoData->yaw_w = static_cast<int>((pMyoData->yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
		}

		/// Called when a paired Myo has provided new accelerometer data in units of g.
		void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onAccelerometerData : Failed to identify device");
                return;
            }

			pMyoData->currentAccelerometer = accel;
		}

		/// Called when a paired Myo has provided new gyroscope data in units of deg/s.
		void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onGyroscopeData : Failed to identify device");
                return;
            }

			pMyoData->currentGyroscope = gyro;
		}

		// onPose() is called whenever the Myo detects that the person wearing it has changed
		// their pose, for example, making a fist, or not making a fist any more.
		void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onPose : Failed to identify device");
                return;
            }

			pMyoData->currentPose = pose;

			if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
				// Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
				// Myo becoming locked.
				myo->unlock(myo::Myo::unlockHold);

				// Notify the Myo that the pose has resulted in an action, in this case changing
				// the text on the screen. The Myo will vibrate.
				myo->notifyUserAction();
			} else {
				// Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
				// are being performed, but lock after inactivity.
				myo->unlock(myo::Myo::unlockTimed);
			}
		}

		/// Called when a paired Myo receives an RSSI update.
		void onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onRssi : Failed to identify device");
                return;
            }

			pMyoData->rssi = rssi;
		}

		/// Called when a paired Myo receives an battery level update.
		void onBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp, uint8_t level)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onBatteryLevelReceived : Failed to identify device");
                return;
            }

			pMyoData->batteryLevel = level;
		}

		/// Called when the warmup period for a Myo has completed.
		void onWarmupCompleted(myo::Myo* myo, uint64_t timestamp, myo::WarmupResult warmupResult)
		{
            MyoData* pMyoData = identifyMyo(myo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Myo : onWarmupCompleted : Failed to identify device");
                return;
            }

			pMyoData->warmupState = myo::WarmupState::warmupStateWarm;
		}
	};

    /////////////////////////////////////////////////////////////////////////
    // utilities

	__forceinline Vector3 convert(const myo::Vector3<float> &v)
	{
		return vector3(v.x(), v.y(), v.z());
	}

	__forceinline Quaternion convert(myo::Quaternion<float> &q)
	{
		//return quaternion(q.x, -q.z, q.y, q.w);
		return quaternion(q.x(), q.y(), q.z(), q.w());
	}

    /////////////////////////////////////////////////////////////////////////
    // Myo library

	bool 			_lib_initialized = false;
	bool     		_enabled = false;

    myo::Hub*		_myoHub = nullptr;
	DataCollector*	_dataCollector = nullptr;
	
	typedef Array<NodeLink> NodeLinkArray;
	NodeLinkArray*	_node_links = nullptr;
	Matrix4x4       _worldOrientation;
	Quaternion 		_orientationOffset;
	
	Information		_info;
	
	bool initialize_lib( )
	{
		_lib_initialized = true;
		return _lib_initialized;
	}

	void shutdown_lib( )
	{
		_lib_initialized = false;
	}

	const char* sdk_version( )
	{
		return "0.9.0"; // TODO
	}

    /////////////////////////////////////////////////////////////////////////
    // Myo init/shutdown

	bool initialize()
	{
		if (_myoHub)
        {
			plugin_log(INFO_LOG, "Myo plugin is already initialized");
			return true;
		}

		// Create a Hub with our application identifier.
		_myoHub = MAKE_NEW(plugin_allocator(), myo::Hub, std::string("com.autodesk.stingray.myo"));

        // Initialize some other values.
		_node_links = MAKE_NEW(plugin_allocator(), NodeLinkArray, plugin_allocator());
		_worldOrientation = matrix4x4_identity();
				
		_orientationOffset.x = 0;
		_orientationOffset.y = 0;
		_orientationOffset.z = 0;
		_orientationOffset.w = 1;
	
        // Attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this
		// will return that Myo immediately.
		myo::Myo* pMyo = _myoHub->waitForMyo(10000);

		// It's possible waitForMyo() failed to find a Myo so test for that.
		if (pMyo) {
            // REVIEW: Myo SDK quirk - Calling this after adding listener doesn't seem to change streaming state.
            // Calling here so it's enabled by default.
            pMyo->setStreamEmg( myo::Myo::streamEmgEnabled );
        }

		// Create data collector
		_dataCollector = MAKE_NEW(plugin_allocator(), DataCollector);
				
		// Hub::addListener() takes the address of any object whose class inherits
		// from DeviceListener, and will cause Hub::run() to send events to all 
		// registered device listeners.
		_myoHub->addListener(_dataCollector);

        // Add the Myo device found above
        if (pMyo)
        {
            MyoData* pMyoData = _dataCollector->identifyMyo(pMyo);
            if (pMyoData != nullptr)
                plugin_log(INFO_LOG, "Myo : Paired default device : ID = %d",pMyoData->id);
        }

		return true;
	}

	bool isInitialized()
	{
        return (_myoHub != nullptr);
    }

    bool connect(unsigned int timeout_ms)
    {
		if (!_myoHub || !_dataCollector)
        {
			plugin_log(WARNING_LOG, "Myo plugin not initialized");
			return false;
		}

        // Attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this
		// will return that Myo immediately.
		myo::Myo* pMyo = _myoHub->waitForMyo(timeout_ms);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!pMyo) {
			plugin_log(ERROR_LOG,"Unable to find and connect to a Myo device!");
		}
        else {
            pMyo->setStreamEmg( myo::Myo::streamEmgEnabled );

            // Will add a new entry for this Myo and return data struct
            MyoData* pMyoData = _dataCollector->identifyMyo(pMyo);
            if (nullptr == pMyoData)
            {
                plugin_log(WARNING_LOG, "Failed to identify and register Myo.");
            }
            else
            {
                // Now that we've added it to our list, get our short ID for it and print it out.
                plugin_log(INFO_LOG, "Register Myo with Id = %d", pMyoData->id);
            }
        }

        return pMyo != nullptr;
    }

	void shutdown()
	{
		if (!_myoHub)
			return;

		plugin_interface().render_api->wait_for_fence(
			plugin_interface().render_api->create_fence()
			);

		// Disconnect listener
		if (_dataCollector)
        {
			_myoHub->removeListener(_dataCollector);
			MAKE_DELETE(plugin_allocator(), _dataCollector);
			_dataCollector = nullptr;
		}
		
        // Remove hub
		MAKE_DELETE(plugin_allocator(), _myoHub);
		_myoHub = nullptr;

		MAKE_DELETE(plugin_allocator(), _node_links);
		_node_links = nullptr;
	}
	
    // Acquire some data from the Myo devices
	void run(unsigned int length)
	{
		if (!_myoHub)
			return;
			
		if (length < 1 || length > 5000)	// sanity check
			length = 1;
				
		// Run the Myo event loop for a set number of milliseconds.
        _myoHub->run(length);
	}

    // Returns the number of Myo devices that are connected
    int countMyos()
    {
        if (_dataCollector)
            return _dataCollector->countMyos();
        else
            return 0;
    }

    void idsMyos(std::vector<unsigned int>& ids)
    {
		if (!_myoHub || !_dataCollector)
			return;

        for (MyoDataMap::const_iterator it = _dataCollector->_myos.begin(); it != _dataCollector->_myos.end(); ++it)
        {
            ids.push_back(it->second->id);
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // Myo device calls

	bool lock(unsigned int id)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        myo::Myo* pMyo = _dataCollector->findMyo(id);
		if (!pMyo)
			return false;

        pMyo->lock();
        return true;
	}
	
	bool unlock(unsigned int id, bool timed)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        myo::Myo* pMyo = _dataCollector->findMyo(id);
		if (!pMyo)
			return false;

        pMyo->unlock(timed ? myo::Myo::UnlockType::unlockTimed : myo::Myo::UnlockType::unlockHold);
        return true;
	}
	
	bool vibrate(unsigned int id, unsigned int length)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        myo::Myo* pMyo = _dataCollector->findMyo(id);
		if (!pMyo)
			return false;

        pMyo->vibrate((myo::Myo::VibrationType)length);
        return true;
	}
	
	int pose(unsigned int id)
	{
		if (!_myoHub || !_dataCollector)
			return POSE_UNKNOWN;

        MyoData* pMyoData = _dataCollector->findMyoData(id);
		if (pMyoData)
        {
		    switch (pMyoData->currentPose.type())
		    {
			    case myo::Pose::Type::rest:
				    return POSE_REST;
			    case myo::Pose::Type::fist:
				    return POSE_FIST;
			    case myo::Pose::Type::waveIn:
				    return POSE_WAVEIN;
			    case myo::Pose::Type::waveOut:
				    return POSE_WAVEOUT;
			    case myo::Pose::Type::fingersSpread:
				    return POSE_FINGERSSPREAD;
			    case myo::Pose::Type::doubleTap:
				    return POSE_DOUBLETAP;
		    }
        }

        return POSE_UNKNOWN;
	}
	
	// Pose::toString() provides the human-readable name of a pose. 
	bool poseToString(unsigned int id, std::string& str)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        MyoData* pMyoData = _dataCollector->findMyoData(id);
		if (!pMyoData)
            return false;

    	str = pMyoData->currentPose.toString();
        return true;
	}

    bool enableEMGData(unsigned int id, bool flag)
    {
		if (!_myoHub || !_dataCollector)
			return false;

        // Find the Myo device with the specified id
        myo::Myo* pMyo = _dataCollector->findMyo(id);
		if (!pMyo)
            return false;

        // Enable EMG streaming on the found Myo.
		pMyo->setStreamEmg(flag ? myo::Myo::streamEmgEnabled : myo::Myo::streamEmgDisabled);
        return true;
    }
	
	bool emgData(unsigned int id, std::array<int8_t, 8>& emg)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        MyoData* pMyoData = _dataCollector->findMyoData(id);
		if (!pMyoData)
            return false;

    	emg = pMyoData->emgSamples;
        return true;
	}
	
	bool orientation(unsigned int id, Matrix4x4 &mtx, bool world)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        MyoData* pMyoData = _dataCollector->findMyoData(id);
		if (!pMyoData)
            return false;

		// Get the Myo orientation and convert to SR matrix
		Quaternion q = convert(pMyoData->currentOrientation);
		Vector3 pos;
		mtx = matrix4x4(q, pos);
			
		if (world) {
			mtx *= _worldOrientation;
		}

        return true;
	}

	void set_world_orientation(const Matrix4x4 &world)
	{
		_worldOrientation = world;
	}

	bool rollPitchYaw(unsigned int id, float& roll, float& pitch, float& yaw)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        MyoData* pMyoData = _dataCollector->findMyoData(id);
		if (!pMyoData)
            return false;

		roll = pMyoData->roll;
		pitch = pMyoData->pitch;
		yaw = pMyoData->yaw;
        return true;
	}

	bool gyroscope(unsigned int id, Vector3 &v)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        MyoData* pMyoData = _dataCollector->findMyoData(id);
		if (!pMyoData)
			return false;

		v = convert(pMyoData->currentGyroscope);
        return true;
	}
	
	bool accelerometer(unsigned int id, Vector3 &v)
	{
		v = vector3(0,0,0);

		if (!_myoHub || !_dataCollector)
			return false;

        MyoData* pMyoData = _dataCollector->findMyoData(id);
		if (!pMyoData)
            return false;

		v = convert(pMyoData->currentAccelerometer);
        return true;
	}

    bool requestInfo(unsigned int id)
    {
		if (!_myoHub || !_dataCollector)
			return false;

        myo::Myo* pMyo = _dataCollector->findMyo(id);
		if (!pMyo)
            return false;

        pMyo->requestRssi();
        pMyo->requestBatteryLevel();
        return true;
    }

	bool info(unsigned int id, Information& info)
	{
		if (!_myoHub || !_dataCollector)
			return false;

        MyoData* pMyoData = _dataCollector->findMyoData(id);
		if (!pMyoData)
            return false;

		// Collect the information
		info.arm = (pMyoData->whichArm == myo::armRight) ? ARM_RIGHT : ARM_LEFT;
		info.batteryLevel = pMyoData->batteryLevel;
        info.rssi = pMyoData->rssi;
		info.connected = pMyoData->isConnected;
		
		info.connectVersion[0] = pMyoData->version[0]; // firmwareVersionMajor;
		info.connectVersion[1] = pMyoData->version[1]; // firmwareVersionMinor;
		info.connectVersion[2] = pMyoData->version[2]; // firmwareVersionPatch;
		info.connectVersion[3] = pMyoData->version[3]; // firmwareVersionHardwareRev;
	
		info.direction = (pMyoData->direction == myo::xDirectionTowardWrist) ? DIRECTION_TOWARD_WRIST : DIRECTION_TOWARD_ELBOW;
		info.locked = !pMyoData->isUnlocked;
		info.synced = pMyoData->isSynced;
		info.warmupState = (pMyoData->warmupState == myo::warmupStateWarm) ? WARMUP_STATE_WARM : WARMUP_STATE_COLD;

		return true;
	}

    /////////////////////////////////////////////////////////////////////////
    // TODOs

	void register_node_link(const NodeLink &link)
	{
		if (!_myoHub)
			return;

		_node_links->push_back(link);
	}
	
	void zeroOrientation()
	{
		if (!_myoHub || !_dataCollector)
			return;

		// TODO
	}

} // namespace myo_device
