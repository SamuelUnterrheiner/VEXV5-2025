cmake_minimum_required(VERSION 3.16)
project(skills_auton)

# Find PROS package (PROS must be installed)
find_package(pros REQUIRED)

# Ensure LemLib directory exists and add it
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lemlib/CMakeLists.txt")
  message(FATAL_ERROR "lemlib not found in ${CMAKE_CURRENT_SOURCE_DIR}/lemlib")
endif()
add_subdirectory(lemlib)

# Make sure compiler can see headers (your project include + LemLib)
include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/lemlib/include"
)

# Collect all .cpp files in src (handles subfolders)
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

if(PROJECT_SOURCES)
  add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
else()
  message(FATAL_ERROR "No source files found in src/ - please add your .cpp files")
endif()

# Link LemLib
target_link_libraries(${PROJECT_NAME} PRIVATE lemlib)
