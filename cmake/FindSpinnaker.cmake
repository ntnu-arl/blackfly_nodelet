# Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 

unset(Spinnaker_FOUND)
unset(Spinnaker_INCLUDE_DIRS)
unset(Spinnaker_LIBRARIES)

find_path(Spinnaker_INCLUDE_DIRS NAMES
  Spinnaker.h
  PATHS
  /usr/include/spinnaker/
  /usr/local/include/spinnaker/
  /opt/spinnaker/include/
)

find_library(Spinnaker_LIBRARIES NAMES Spinnaker
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/spinnaker/lib
)

if (Spinnaker_INCLUDE_DIRS AND Spinnaker_LIBRARIES)
  set(Spinnaker_FOUND 1)
endif (Spinnaker_INCLUDE_DIRS AND Spinnaker_LIBRARIES)
