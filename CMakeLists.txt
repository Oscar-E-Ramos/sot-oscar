# Copyright 2013, Oscar E. Ramos Ponce, Nicolas Mansard, LAAS-CNRS
#
# This file is part of sot-oscar.
# sot-oscar is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-oscar is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-oscar. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)

INCLUDE(cmake/fcl.cmake)
INCLUDE(cmake/FindCGAL.cmake)
#INCLUDE(cmake/geometric-tools.cmake)


SET(PROJECT_NAME sot-oscar)
SET(PROJECT_DESCRIPTION "extensions to control by inverse dynamics.")
SET(PROJECT_URL "http://github.com/laas/sot-oscar")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.8.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("soth >= 0.0.1")
ADD_REQUIRED_DEPENDENCY("sot-dyninv >= 1.0.0")


# List plug-ins that will be compiled.
SET(libs
  
  solver-motion-reduced
  fcl-box-mesh-collision
  foot-collision-grid

  velocity-correction
  velocity-control
  )
SET(fcl-box-mesh-collision_plugins_dependencies fcl)

LIST(APPEND LOGGING_WATCHED_TARGETS ${libs})

SET(headers

  solver-motion-reduced.h
  fcl-box-mesh-collision.h
  foot-collision-grid.h

  velocity-correction.h
  velocity-control.h
)

# list(APPEND task-dyn-limits_plugins_dependencies task-dyn-pd)
# list(APPEND task-dyn-inequality_plugins_dependencies task-dyn-pd)
# list(APPEND task-dyn-joint-limits_plugins_dependencies task-dyn-pd)
# list(APPEND pseudo-robot-dynamic_plugins_dependencies dynamic-integrator)
# list(APPEND contact-selecter_plugins_dependencies solver-dyn-reduced)

# foreach(lib task-weight task-inequality task-dyn-pd task-joint-limits)
# 	IF(WIN32)
# 		list(APPEND ${lib}_ext_plugins_dependencies
# 			${DYNAMIC_GRAPH_PLUGINDIR}/task${CMAKE_LINK_LIBRARY_SUFFIX})
# 	ELSE(WIN32)
# 		list(APPEND ${lib}_ext_plugins_dependencies
# 			${DYNAMIC_GRAPH_PLUGINDIR}/task${CMAKE_SHARED_LIBRARY_SUFFIX})
# 	ENDIF(WIN32)
# endforeach(lib)

# foreach(lib solver-op-space solver-dyn-reduced solver-dyn-red2)
# 	list(APPEND ${lib}_plugins_dependencies task-dyn-pd)

# 	IF(WIN32)
# 		list(APPEND ${lib}_ext_plugins_dependencies
# 			${DYNAMIC_GRAPH_PLUGINDIR}/feature-point6d${CMAKE_LINK_LIBRARY_SUFFIX})
# 	ELSE(WIN32)
# 		list(APPEND ${lib}_ext_plugins_dependencies
# 			${DYNAMIC_GRAPH_PLUGINDIR}/feature-point6d${CMAKE_SHARED_LIBRARY_SUFFIX})
# 	ENDIF(WIN32)
# endforeach(lib)

# Add subdirectories.
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unitTesting)

# Configure some of the python files
# CONFIGURE_FILE(
#    "${CMAKE_CURRENT_SOURCE_DIR}/python/robot_specific.py.cmake"
#    "${CMAKE_CURRENT_BINARY_DIR}/python/robot_specific.py"
# )

#INCLUDE(cmake/python.cmake)
#INSTALL(FILES
#  "${CMAKE_CURRENT_BINARY_DIR}/python/robotSpecific.py"
#  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/dyninv
#)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
