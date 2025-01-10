# Find the GLFW3 header file
find_path(glfw3_INCLUDE_DIR
    NAMES GLFW/glfw3.h
    PATHS /usr/include
)

# Find the GLFW3 library
find_library(glfw3_LIBRARIES
    NAMES glfw
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
)

# Check if GLFW3 is found
if (glfw3_LIBRARIES AND glfw3_INCLUDE_DIR)
    message("GLFW3 library: ${glfw3_LIBRARIES}")
    message("GLFW3 include directory: ${glfw3_INCLUDE_DIR}")
    set(FOUND_glfw3 TRUE)
else()
    if (glfw3_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find GLFW3!")
    endif()
    set(FOUND_glfw3 FALSE)
endif()

mark_as_advanced(glfw3_INCLUDE_DIR glfw3_LIBRARIES FOUND_glfw3)
