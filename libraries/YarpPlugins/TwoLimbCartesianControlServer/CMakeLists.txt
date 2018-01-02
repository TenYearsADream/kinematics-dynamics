# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(TwoLimbCartesianControlServer
                    CATEGORY device
                    TYPE roboticslab::TwoLimbCartesianControlServer
                    INCLUDE TwoLimbCartesianControlServer.hpp
                    DEFAULT ON)

if(NOT SKIP_TwoLimbCartesianControlServer)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(TwoLimbCartesianControlServer TwoLimbCartesianControlServer.hpp
                                              TwoLimbCartesianControlServer.cpp
                                              DeviceDriverImpl.cpp)

add_dependencies(TwoLimbCartesianControlServer COLOR_DEBUG)

target_link_libraries(TwoLimbCartesianControlServer YARP::YARP_OS
                                                    YARP::YARP_dev)

yarp_install(TARGETS TwoLimbCartesianControlServer
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES TwoLimbCartesianControlServer.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()