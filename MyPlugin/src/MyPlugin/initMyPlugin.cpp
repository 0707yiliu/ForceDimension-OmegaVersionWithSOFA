#include <MyPlugin/config.h>
#include <string>
#include <sofa/helper/system/FileRepository.h>
#include <stdio.h>
#include <sofa/core/ObjectFactory.h>
using sofa::core::ObjectFactory;
namespace sofa
{
namespace component
{

extern "C" {
                SOFA_MYPLUGIN_API void initExternalModule();
                SOFA_MYPLUGIN_API const char* getModuleName();
                SOFA_MYPLUGIN_API const char* getModuleVersion();
                SOFA_MYPLUGIN_API const char* getModuleLicense();
                SOFA_MYPLUGIN_API const char* getModuleDescription();
                SOFA_MYPLUGIN_API const char* getModuleComponentList();
}
    void initExternalModule()
    {
        // Here is the place to write initialisation code, that will be executed
        // before any component is created.
        static bool first = true;
        if (first) {
            first = false;

           // sofa::helper::system::DataRepository.addLastPath(std::string(PLUGIN_DATA_DIR_));
           // sofa::helper::system::DataRepository.addLastPath(std::string(PLUGIN_DATA_DIR_) + "/data");
        }
    }

    const char* getModuleName()
    {
        return sofa_tostring(SOFA_TARGET);
    }

    const char* getModuleVersion()
    {
        return sofa_tostring(MYPLUGIN_VERSION);
    }

    const char* getModuleLicense()
    {
        return "LGPL";
    }

    const char* getModuleDescription()
    {
        return "Omega7 Device Driver Testing";
    }

    const char* getModuleComponentList()
    {
        // Comma-separated list of the components in this plugin, empty for now
        //return "Omega7Driver";
	static std::string classes = ObjectFactory::getInstance()->listClassesFromTarget(sofa_tostring(SOFA_TARGET));
	return classes.c_str();
    }
}
}
