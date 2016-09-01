//
// Thalmic Myo Armband Plugin for Autodesk Stingray 
//
// Author: Hans Kellner
//

#include "if_myo.h"
#include "myo_device.h"
#include "plugin_interface.h"

#include <plugin_foundation/assert.h>

#include <memory.h>

namespace script_interface_myo {
	using namespace myo_plugin;
	using namespace myo_device;
	using namespace stingray_plugin_foundation;

/////////////////////////////////////////////////////////////////////////////
#pragma region HELPER LUA FUNCTIONS

	__forceinline void push(lua_State *L, bool b)
	{
		plugin_interface().lua_api->pushboolean(L, b);
	}

	__forceinline void push(lua_State *L, int n)
	{
		plugin_interface().lua_api->pushinteger(L, n);
	}

	__forceinline void push(lua_State *L, unsigned n)
	{
		plugin_interface().lua_api->pushinteger(L, n);
	}

	__forceinline void push(lua_State *L, float n)
	{
		plugin_interface().lua_api->pushnumber(L, n);
	}

	__forceinline void push(lua_State *L, const char *s)
	{
		plugin_interface().lua_api->pushstring(L, s);
	}

	__forceinline void push(lua_State *L, const Matrix4x4 &m)
	{
		plugin_interface().lua_api->pushmatrix4x4(L, (float*)(&m));
	}

	__forceinline void push(lua_State * L, const Vector3 &v)
	{
		plugin_interface().lua_api->pushvector3(L, (float*)&v.x);
	}

	__forceinline int get_unsigned(lua_State* L, int i)
	{
		return (unsigned)plugin_interface().lua_api->tointeger(L, i);
	}

	__forceinline bool get_bool(lua_State *L, int i)
	{
		return plugin_interface().lua_api->toboolean(L, i) != 0;
	}

	__forceinline float get_float(lua_State *L, int i)
	{
		return (float)plugin_interface().lua_api->tonumber(L, i);
	}

	__forceinline const char* get_cstr(lua_State *L, int i)
	{
		return plugin_interface().lua_api->tolstring(L, i, nullptr);
	}

	__forceinline int get_num_parameters(lua_State *L)
	{
		return plugin_interface().lua_api->gettop(L);
	}

	__forceinline Matrix4x4 get_matrix4x4(lua_State *L, int i)
	{
		Matrix4x4 m;
		memcpy(&m, plugin_interface().lua_api->getmatrix4x4(L, i), sizeof(Matrix4x4));
		return m;
	}

	template <typename T>
	inline T * get_userdata(lua_State * L, int i)
	{
		void *ud = plugin_interface().lua_api->touserdata(L, i);
		void *p = *(void **)ud;
		return (T *)p;
	}

	template<typename T>
	__forceinline void set_field(lua_State *L, const char *key, T value)
	{
		push(L, value);
		plugin_interface().lua_api->setfield(L, -2, key);
	}

	template<typename T>
	__forceinline void rawseti(lua_State *L, int idx, T value)
	{
		push(L, value);
		plugin_interface().lua_api->rawseti(L, -2, idx);
	}

	__forceinline void push_new_table(lua_State* L)
	{
		plugin_interface().lua_api->createtable(L, 0, 0);
	}

#pragma endregion

/////////////////////////////////////////////////////////////////////////////
#pragma region DEFINES

	#define SCRIPT_MYO_VIBRATE_TYPES(func) \
		func(VIBRATE_SHORT) \
		func(VIBRATE_MEDIUM) \
		func(VIBRATE_LONG)

	#define SCRIPT_MYO_ARM_TYPES(func) \
		func(ARM_LEFT) \
		func(ARM_RIGHT) \
		func(ARM_UNKNOWN)

	#define SCRIPT_MYO_DIRECTION_TYPES(func) \
		func(DIRECTION_TOWARD_WRIST) \
		func(DIRECTION_TOWARD_ELBOW) \
		func(DIRECTION_UNKNOWN)

	#define SCRIPT_MYO_WARMUP_STATE_TYPES(func) \
		func(WARMUP_STATE_UNKNOWN) \
		func(WARMUP_STATE_COLD) \
		func(WARMUP_STATE_WARM)

	#define SCRIPT_MYO_POSE_TYPES(func) \
		func(POSE_REST) \
		func(POSE_FIST) \
		func(POSE_WAVEIN) \
		func(POSE_WAVEOUT) \
		func(POSE_FINGERSSPREAD) \
		func(POSE_DOUBLETAP) \
		func(POSE_UNKNOWN)

#pragma endregion

/////////////////////////////////////////////////////////////////////////////
#pragma region SCRIPT LOAD FUNCTIONS

	/* @adoc lua
	@obj stingray.Myo : table
	@grp plugins
	@des This object provides access to the Myo interface.
	*/
	const char* const lua_namespace = "Myo";

	/////////////////////////////////////////////////////////////////////////////
    // Core Functions

	void load_myo_core_func(LuaApi &api)
	{
		/* @adoc lua
		   @sig stingray.Myo.initialize() : boolean
		   @ret boolean  Returns true if Myo was initialized properly, otherwise returns false.
		   @des Initializes Myo system.
		*/
		api.add_module_function(lua_namespace, "initialize", [](lua_State *L)
		{
			push(L, myo_device::initialize());
			return 1;
		});

		/* @adoc lua
		   @sig stingray.Myo.id_initialized() : boolean
		   @ret boolean  Returns true if Myo library has been initialized, otherwise returns false.
		   @des Returns true if Myo library has been intitialized.
		*/
		api.add_module_function(lua_namespace, "is_initialized", [](lua_State *L)
		{
			push(L, myo_device::isInitialized());
			return 1;
		});

		/* @adoc lua
		   @sig stingray.Myo.shutdown() : nil
		   @des Shuts down Myo system.
		*/
		api.add_module_function(lua_namespace, "shutdown", [](lua_State *L)
		{
			myo_device::shutdown();
			return 0;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.connect(timeout:integer?) : boolean
		   @arg timeout  Optional; Timeout in milliseconds to wait for a device. Default = 10000.
           @ret boolean Returns true if a Myo device is found and connected; otherwise false.
		   @des Attempts to connect to a Myo device.
		*/
		api.add_module_function(lua_namespace, "connect", [](lua_State *L)
		{
			unsigned int timeout = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 10000;
			push(L, myo_device::connect(timeout));
			return 1;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.run(length:integer?) : nil
		   @arg length  Optional; Run the Myo event loop for a specified time (milliseconds) to acquire data. Default length = 1.  Max length = 5000.
		   @des Run the Myo event loop for a specified time to acquire data.
		*/
		api.add_module_function(lua_namespace, "run", [](lua_State *L)
		{
			unsigned int length = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 1;
			if (length < 1 || length > 5000)	// sanity check
				length = 1;
			myo_device::run(length);
			return 0;
		});

		/* @adoc lua
		   @sig stingray.Myo.count_myos() : integer
		   @ret integer Count of paired Myos
		   @des Returns the number of paired Myo devices.
		*/
		api.add_module_function(lua_namespace, "count_myos", [](lua_State *L)
		{
			int count = myo_device::countMyos();
			push(L, count);
			return 1;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.get_myo_ids() : table
		   @ret table Table filled with Ids of paired Myo devices
		   @des Returns a table of the Ids of paired Myo devices.
		*/
		api.add_module_function(lua_namespace, "get_myo_ids", [](lua_State *L)
		{
			std::vector<unsigned int> ids;
			myo_device::idsMyos(ids);
		
			push_new_table(L);

			for (int i = 0; i < ids.size(); ++i)
			{
				rawseti(L, i+1, (int)ids[i]);
			}
			
			return 1;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.lock(id:integer?) : nil
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @des Locks the Myo device.
		*/
		api.add_module_function(lua_namespace, "lock", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;
			myo_device::lock(id);
			return 0;
		});

		/* @adoc lua
		   @sig stingray.Myo.unlock(id:integer?, timed:bool?) : nil
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @arg timed    Optional; If true then unlock the Myo for ~2 seconds, then lock it.
		   @des Unlocks the Myo device.
		*/
		api.add_module_function(lua_namespace, "unlock", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;
			bool argTimed = get_unsigned(L, 2) ? true : false;
			bool timed = (get_num_parameters(L) > 0) ? argTimed : false;
			myo_device::unlock(id, timed);
			return 0;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.vibrate(id:integer?, length:integer?) : nil
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @arg length    Optional; Vibrate the Myo for specified length (Myo.VIBRATE_SHORT | Myo.VIBRATE_MEDIUM | Myo.VIBRATE_LONG). Default to Myo.VIBRATE_MEDIUM.
		   @des Vibrates the Myo device.
		*/
		api.add_module_function(lua_namespace, "vibrate", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;
			unsigned int length = (get_num_parameters(L) > 1) ? get_unsigned(L, 2) : VibrateTypes::VIBRATE_MEDIUM;
			if (length < VibrateTypes::VIBRATE_SHORT || length > VibrateTypes::VIBRATE_LONG)
				length = VibrateTypes::VIBRATE_MEDIUM;
			myo_device::vibrate(id, length);
			return 0;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.pose(id:integer?) : int
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @ret int Pose (gesture)
		   @des Returns the last pose (gesture).
		*/
		api.add_module_function(lua_namespace, "pose", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;
			int pose = myo_device::pose(id);
			push(L, pose);
			return 1;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.pose_as_str(id:integer?) : string
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @ret string Pose (gesture) as a string
		   @des Returns the last pose (gesture) as a string.
		*/
		api.add_module_function(lua_namespace, "pose_as_str", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;
			std::string str;
			myo_device::poseToString(id, str);
			push(L, str.c_str());
			return 1;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.enable_emg_data(id:integer?) : nil
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @des Enables EMG data streaming of the Myo device.
		*/
		api.add_module_function(lua_namespace, "enable_emg_data", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;
			myo_device::enableEMGData(id, true);
			return 0;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.disable_emg_data(id:integer?) : nil
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @des Disables EMG data streaming of the Myo device.
		*/
		api.add_module_function(lua_namespace, "disable_emg_data", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;
			myo_device::enableEMGData(id, false);
			return 0;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.emg_data(id:integer?) : table
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @ret table Table filled with EMG data
		   @des Returns a table of the EMG data.
		*/
		api.add_module_function(lua_namespace, "emg_data", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;

			std::array<int8_t, 8> emg;
			myo_device::emgData(id, emg);
		
			push_new_table(L);
			for (int i = 0; i < 8; ++i)
			{
				rawseti(L, i+1, (int)emg[i]);
			}
			
			return 1;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.link_node(unit:Unit, node:integer, world:World) : nil
		   @arg unit Unit to link to device
		   @arg node Scene graph node that is linked within the unit
		   @arg world World containing the unit
		   @des Links a Unit node to be automatically updated by the Myo device.
		*/
		api.add_module_function(lua_namespace, "link_node", [](lua_State *L)
		{
			Unit *u = plugin_interface().lua_api->getunit(L, 1);
			SceneGraph *sg = plugin_interface().unit_api->scene_graph(u);

			myo_device::NodeLink link;
			link.node  = plugin_interface().lua_api->getindex_0_or_1_based(L,2);
			link.world = get_userdata<World>(L, 3);

			myo_device::register_node_link(link);
			return 0;
		});
	}

	/////////////////////////////////////////////////////////////////////////////
    // Movement Functions

	void load_myo_movement_func(LuaApi &api)
	{		
		/* @adoc lua
		   @sig stingray.Myo.set_world_orientation(world:Matrix4x4) : nil
		   @arg world World orientation
		   @des Sets world orientation
		*/
		api.add_module_function(lua_namespace, "set_world_orientation", [](lua_State *L)
		{
			myo_device::set_world_orientation(get_matrix4x4(L, 1));
			return 0;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.local_orientation(id:integer?) :  Matrix4x4
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @ret Matrix4x4 Local orientation
		   @des Returns the local orientation of the Myo device.
		*/
		api.add_module_function(lua_namespace, "local_orientation", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;

			Matrix4x4 mtx;
			myo_device::orientation(id, mtx, false);
			push(L, mtx);
			return 1;
		});

		/* @adoc lua
		   @sig stingray.Myo.world_orientation(id:integer?) :  Matrix4x4
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @ret Matrix4x4 World orientation
		   @des Returns the world orientation of the Myo device.
		*/
		api.add_module_function(lua_namespace, "world_orientation", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;

			Matrix4x4 mtx;
			myo_device::orientation(id, mtx, true);
			push(L, mtx);
			return 1;
		});

		/* @adoc lua
		   @sig stingray.Myo.zero_orientation() : nil
		   @des Sets the current Myo orientation as the origin.
		*/
		api.add_module_function(lua_namespace, "zero_orientation", [](lua_State* L)
		{
			myo_device::zeroOrientation();
			return 0;
		});
		
		/* @adoc lua
		   @sig stingray.Myo.gyroscope(id:integer?) :  Vector3
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @ret Vector3 Gyroscope values
		   @des Returns the gyroscope values of the Myo device.
		*/
		api.add_module_function(lua_namespace, "gyroscope", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;

			Vector3 v;
			myo_device::gyroscope(id, v);
			push(L, v);
			return 1;
		});

		/* @adoc lua
		   @sig stingray.Myo.accelerometer(id:integer?) :  Vector3
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @ret Vector3 Accelerometer values
		   @des Returns the accelerometer values of the Myo device.
		*/
		api.add_module_function(lua_namespace, "accelerometer", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;

			Vector3 v;
			myo_device::accelerometer(id, v);
			push(L, v);
			return 1;
		});

	}

	/////////////////////////////////////////////////////////////////////////////
    // Information Functions

	void load_myo_info_func(LuaApi &api)
	{

		/* @adoc lua
		   @sig stingray.Myo.request_information(id:integer?) : nil
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @des Requests that information should be queried from the Myo device.  This call can be followed with Myo.information() to get the values.
		*/
		api.add_module_function(lua_namespace, "request_information", [](lua_State* L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;
			myo_device::requestInfo(id);
			return 0;
		});

		/* @adoc lua
		   @sig stingray.Myo.information(id:integer?) : table
           @arg id    Optional; Id of Myo device or none to use first device found.
		   @ret table  Table populated with Myo information.
		   @des Returns a table containing information about the Myo device.  This call must be proceeded with a call to Myo.request_information().
		*/
		api.add_module_function(lua_namespace, "information", [](lua_State *L)
		{
			unsigned int id = (get_num_parameters(L) > 0) ? get_unsigned(L, 1) : 0;

			Information info;
            myo_device::info(id, info);

			push_new_table(L);
			set_field(L, "arm", 				(int)info.arm );
			set_field(L, "battery_level", 		(int)info.batteryLevel );
            set_field(L, "rssi",                (int)info.rssi);
			set_field(L, "connected", 			info.connected );
			
			push_new_table(L);
			rawseti(L, 1, info.connectVersion[0]);
			rawseti(L, 2, info.connectVersion[1]);
			rawseti(L, 3, info.connectVersion[2]);
			rawseti(L, 4, info.connectVersion[3]);
			plugin_interface().lua_api->setfield(L, -2,  "connect_version");
			
			set_field(L, "direction", 			(int)info.direction );
			set_field(L, "locked", 				info.locked );
			set_field(L, "synced",				info.synced );
			set_field(L, "warmup_state",		(int)info.warmupState );
			return 1;
		});
	
	}

	/////////////////////////////////////////////////////////////////////////////
	void load_myo_enums(LuaApi &api)
	{
		#define DECLARE_NUM_FUNC(num) api.set_module_number(lua_namespace, #num, num);
		SCRIPT_MYO_VIBRATE_TYPES(DECLARE_NUM_FUNC)
		SCRIPT_MYO_ARM_TYPES(DECLARE_NUM_FUNC)
		SCRIPT_MYO_DIRECTION_TYPES(DECLARE_NUM_FUNC)
		SCRIPT_MYO_WARMUP_STATE_TYPES(DECLARE_NUM_FUNC)
        SCRIPT_MYO_POSE_TYPES(DECLARE_NUM_FUNC)
		#undef DECLARE_NUM_FUNC
	}

#pragma endregion

/////////////////////////////////////////////////////////////////////////////
// Load and Unload the API

	void load( )
	{
		LuaApi &api = *plugin_interface().lua_api;

		load_myo_core_func(api);
		load_myo_movement_func(api);
		load_myo_info_func(api);
		load_myo_enums(api);
	}

	void unload( ) {}
}
