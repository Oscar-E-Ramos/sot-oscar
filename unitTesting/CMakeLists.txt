# Copyright 2011, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
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

#ADD_DEFINITIONS(-DDEBUG=2)

SET(tests
  testCollisionBoxMesh
  testCGALConvexHull
  #testCollisionBoxMeshConvexHull
  )

SET(testCollisionBoxMesh_plugins_dependencies fcl)
SET(testCGALConvexHull_plugins_dependencies ${CGAL_LIBRARIES})
# SET(testCollisionBoxMeshConvexHull_plugins_dependencies 
#   fcl
#   ${CGAL_LIBRARIES}
#   )


FOREACH(test ${tests})
  SET(EXECUTABLE_NAME "${test}_exe")
  ADD_EXECUTABLE(${EXECUTABLE_NAME}
    ${test}.cpp)

  IF(${test}_plugins_dependencies)
    ADD_DEPENDENCIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
  ENDIF(${test}_plugins_dependencies)

  # TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
  #   fcl
  #  )

  # PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} sot-core)
  # IF(${test}_plugins_dependencies)
  #   ADD_DEPENDENCIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
  #   TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
  # ENDIF(${test}_plugins_dependencies)
  # TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} "-lsoth")

ENDFOREACH(test)

INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/plane2m.obj
  # ${CMAKE_CURRENT_BINARY_DIR}/../python/robot_specific.py
  DESTINATION ${CMAKE_BINARY_DIR}/unitTesting
)

ADD_EXECUTABLE(testCollisionBoxMeshConvexHull testCollisionBoxMeshConvexHull.cpp)
TARGET_LINK_LIBRARIES(testCollisionBoxMeshConvexHull fcl ${CGAL_LIBRARIES} )



