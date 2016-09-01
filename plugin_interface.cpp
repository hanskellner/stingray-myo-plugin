//
// Thalmic Myo Armband Plugin for Autodesk Stingray 
//
// Author: Hans Kellner
//

#include "plugin_interface.h"

#include <plugin_foundation/assert.h>

#include <stdio.h>

namespace myo_plugin
{
	PluginInterface  _interface;
	AllocatorObject *_allocator_object = nullptr;
	ApiAllocator     _allocator(nullptr, nullptr);

	const char* const _plugin_name = "myo_plugin";
	char              _log[256];

	void initialize_plugin_interface(GetApiFunction get_engine_api)
	{
		_interface.lua_api                = (LuaApi*             )get_engine_api(LUA_API_ID               );
		_interface.allocator_api          = (AllocatorApi*       )get_engine_api(ALLOCATOR_API_ID         );
		_interface.logging_api            = (LoggingApi*         )get_engine_api(LOGGING_API_ID           );
		_interface.render_api             = (RenderInterfaceApi* )get_engine_api(RENDER_INTERFACE_API_ID  );
		_interface.unit_api               = (UnitApi*            )get_engine_api(UNIT_API_ID              );
		_interface.scene_graph_api        = (SceneGraphApi*      )get_engine_api(SCENE_GRAPH_API_ID       );
		_interface.render_scene_graph_api = (RenderSceneGraphApi*)get_engine_api(RENDER_SCENE_GRAPH_API_ID);

		if (!_allocator_object) {
			XENSURE(!_allocator.api());
			_allocator_object = _interface.allocator_api->make_plugin_allocator(_plugin_name);
			_allocator = ApiAllocator(_interface.allocator_api, _allocator_object);
		}
	}

	void shutdown_plugin_interface( )
	{
		if (_allocator_object) {
			XENSURE(_allocator.api());
			_allocator = ApiAllocator(nullptr, nullptr);
			_interface.allocator_api->destroy_plugin_allocator(_allocator_object);
			_allocator_object = nullptr;
		}
		_interface.clear();
	}

	PluginInterface& plugin_interface( )
	{
		return _interface;
	}

	Allocator& plugin_allocator( )
	{
		return _allocator;
	}

	const char* plugin_name( ) {
		return _plugin_name;
	}

	void plugin_log(PluginLogType type, const char *fmt, ...)
	{
		if (!_interface.logging_api)
			return;

		static const char *system = "MyoPlugin";

		va_list ap;
		va_start(ap, fmt);
		vsnprintf(_log, sizeof(_log), fmt, ap);
		va_end(ap);

		switch(type) {
			case INFO_LOG:
				_interface.logging_api->info(system, _log);
				break;
			case WARNING_LOG:
				_interface.logging_api->warning(system, _log);
				break;
			case ERROR_LOG:
				_interface.logging_api->error(system, _log);
				break;
			default:
				break;
		}
	}
}
