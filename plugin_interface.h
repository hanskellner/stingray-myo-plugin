//
// Thalmic Myo Armband Plugin for Autodesk Stingray 
//
// Author: Hans Kellner
//

#pragma once

#include <plugin_foundation/plugin_api.h>
#include <plugin_foundation/allocator.h>

namespace myo_plugin {
	using namespace stingray_plugin_foundation;

	struct PluginInterface
	{
		PluginInterface( ) { clear(); }

		void clear() {
			lua_api                = nullptr;
			allocator_api          = nullptr;
			logging_api            = nullptr;
			render_api             = nullptr;
			unit_api               = nullptr;
			scene_graph_api        = nullptr;
			render_scene_graph_api = nullptr;
		}

		struct LuaApi              *lua_api;
		struct AllocatorApi        *allocator_api;
		struct LoggingApi          *logging_api;
		struct RenderInterfaceApi  *render_api;
		struct UnitApi             *unit_api;
		struct SceneGraphApi       *scene_graph_api;
		struct RenderSceneGraphApi *render_scene_graph_api;
	};

	void initialize_plugin_interface(GetApiFunction get_engine_api);
	void shutdown_plugin_interface();

	PluginInterface &plugin_interface( );
	Allocator       &plugin_allocator( );
	const char      *plugin_name( );

	enum PluginLogType { INFO_LOG, WARNING_LOG, ERROR_LOG };
	void plugin_log(PluginLogType type, const char *fmt, ...);
}
