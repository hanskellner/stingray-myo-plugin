//
// Thalmic Myo Armband Plugin for Autodesk Stingray 
//
// Author: Hans Kellner
//

#include "myo_plugin.h"
#include "plugin_interface.h"
#include "if_myo.h"
#include "myo_device.h"

#include <plugin_foundation/hash_function.h>

#if defined( STATIC_LINKING )
	void* get_myo_plugin_api(unsigned api)
#else
	__declspec(dllexport) void* get_plugin_api(unsigned api)
#endif
{
	switch (api) {
		case PLUGIN_API_ID: {
			static struct PluginApi api = { 0 };

			api.plugins_loaded     = &myo_plugin::plugins_loaded;
			api.setup_resources    = &myo_plugin::setup_resources;
			api.shutdown_resources = &myo_plugin::shutdown_resources;
			api.setup_game         = &myo_plugin::setup_game;
			api.shutdown_game      = &myo_plugin::shutdown_game;
			api.get_id             = &myo_plugin::get_id;
			api.get_name           = &myo_plugin::get_name;

			return &api;
		}

		default:
			return 0;
	}
}

namespace myo_plugin {
	bool _lib_initialized = false;

	void plugins_loaded(GetApiFunction get_engine_api)
	{
		if (get_engine_api(APPLICATION_API_ID)) {
			_lib_initialized = myo_device::initialize_lib();
		}
	}

	void setup_resources(GetApiFunction get_engine_api)
	{
		initialize_plugin_interface(get_engine_api);
	}

	void shutdown_resources( )
	{
		shutdown_plugin_interface();
		myo_device::shutdown_lib();
	}

	void setup_game(GetApiFunction get_engine_api)
	{
		initialize_plugin_interface(get_engine_api);
		script_interface_myo::load();
	}

	void shutdown_game( )
	{
		script_interface_myo::unload();
		shutdown_plugin_interface();
	}

	unsigned get_id( )
	{
		return hash32(plugin_name());
	}

	const char* get_name( )
	{
		return plugin_name();
	}

}
