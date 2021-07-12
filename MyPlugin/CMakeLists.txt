cmake_minimum_required(VERSION 3.12)
project(MyPlugin VERSION 1.0 LANGUAGES CXX)

#look for all packages on which your project depends
find_package(SofaFramework REQUIRED)
find_package(SofaBase REQUIRED)
find_library(DHDLIB libdhd.so.3 lib)
find_library(DRDLIB libdrd.so.3 lib)
find_library(USB1LIB libusb-1.0.so)
sofa_find_package(SofaGui)
sofa_find_package(Qt5 COMPONENTS Core) # Dependency to Qt5Core (needed for qt5_wrap_cpp
sofa_find_package(SofaHaptics REQUIRED)
sofa_find_package(SofaOpenglVisual REQUIRED)
sofa_find_package(SofaUserInteraction REQUIRED)
sofa_find_package(SofaLoader REQUIRED)

set(MYPLUGIN_SRC_DIR src/MyPlugin)
set(HEADER_FILES
	#MyHeaderFile1.h
	#MyHeaderFile2.h
	#MyHeaderFile3.h
   ${MYPLUGIN_SRC_DIR}/config.h.in
   ${MYPLUGIN_SRC_DIR}/OmegaDriver.h
   ${MYPLUGIN_SRC_DIR}/dhdc.h
   ${MYPLUGIN_SRC_DIR}/drdc.h
   ${MYPLUGIN_SRC_DIR}/TestBehaviorModel.h
)
set(HEADER_FILES_TO_MOC
    )
set(SOURCE_FILES
	#MySourceFile1.cpp
	#MySourceFile2.cpp
	#MySourceFile3.cpp
   ${MYPLUGIN_SRC_DIR}/initMyPlugin.cpp
   ${MYPLUGIN_SRC_DIR}/TestBehaviorModel.cpp
   ${MYPLUGIN_SRC_DIR}/OmegaDriver.cpp
)
set(README_FILES
    MyPlugin.md
    )

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES_TO_MOC} ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
target_link_libraries(${PROJECT_NAME} 
	SofaCore 
	SofaBaseVisual 
	SofaHelper 
	SofaUserInteraction 
	SofaHaptics 
	SofaLoader 
	SofaOpenglVisual 
	${USB1LIB} 
	#/home/liu/sofa/src/applications/plugins/MyPlugin/lib/libdhd.so.3
	#/home/liu/sofa/src/applications/plugins/MyPlugin/lib/libdrd.so.3
	#${DHD}
	#${DRD}
  	/home/liu/omega_ws/src/haptic_ros_driver/lib/libdhd.so.3
  	/home/liu/omega_ws/src/haptic_ros_driver/lib/libdrd.so.3
	) 
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
