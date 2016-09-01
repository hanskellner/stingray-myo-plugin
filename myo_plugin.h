//
// Thalmic Myo Armband Plugin for Autodesk Stingray 
//
// Author: Hans Kellner
//

#pragma once

#include <plugin_foundation/plugin_api.h>

namespace myo_plugin {
	void plugins_loaded(GetApiFunction get_engine_api);

	void setup_resources(GetApiFunction get_engine_api);
	void shutdown_resources();

	void setup_game(GetApiFunction get_engine_api);
	void shutdown_game();

	unsigned get_id();
	const char *get_name();

}

// external access
extern "C" {
#if defined(STATIC_LINKING)
	void *get_myo_plugin_api(unsigned api);
#else
	__declspec(dllexport) void *get_plugin_api(unsigned api);
#endif

} // extern "C"

