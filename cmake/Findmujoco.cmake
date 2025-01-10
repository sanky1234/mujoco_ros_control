# Find the MuJoCo header file
find_path(mujoco_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    PATHS /opt/mujoco/include
    NO_DEFAULT_PATH
)

# Find the MuJoCo library
find_library(mujoco_LIBRARIES
    NAMES mujoco
    PATHS /opt/mujoco/lib
    NO_DEFAULT_PATH
)

# Find the GLEW library (if needed)
find_library(libglew_LIBRARIES
    NAMES libGLEW.so
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu/
)

# Check if all required components are found
if (mujoco_LIBRARIES AND mujoco_INCLUDE_DIR AND libglew_LIBRARIES)
    message("MuJoCo library: ${mujoco_LIBRARIES}")
    message("GLEW library: ${libglew_LIBRARIES}")
    message("MuJoCo include directory: ${mujoco_INCLUDE_DIR}")
    set(FOUND_mujoco TRUE)
else()
    if (mujoco_FIND_REQUIRED)
        message("GLEW library: ${libglew_LIBRARIES}")

        message(FATAL_ERROR "Could not find MuJoCo!")
    endif()
    set(FOUND_mujoco FALSE)
endif()

mark_as_advanced(mujoco_INCLUDE_DIR mujoco_LIBRARIES FOUND_mujoco libglew_LIBRARIES)
