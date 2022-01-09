# -----------------------------------------------
# CMakeLists for Wosler Robot Driver. 01-08-2022
# -----------------------------------------------

cmake_minimum_required(VERSION 3.5)

project(kortexApiCppExamples VERSION 2.3.0 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


option(USE_CONAN "Use the Conan package manager to automatically fetch the Kortex API" ON)


# Activate C++ 11
set (CMAKE_CXX_STANDARD 11)


macro(configure_msvc_runtime)

    # Default to statically-linked runtime.
    if("${MSVC_RUNTIME}" STREQUAL "")
      set(MSVC_RUNTIME "static")
    endif()

    # Set compiler options.
    set(variables
      CMAKE_C_FLAGS_DEBUG
      CMAKE_C_FLAGS_MINSIZEREL
      CMAKE_C_FLAGS_RELEASE
      CMAKE_C_FLAGS_RELWITHDEBINFO
      CMAKE_CXX_FLAGS_DEBUG
      CMAKE_CXX_FLAGS_MINSIZEREL
      CMAKE_CXX_FLAGS_RELEASE
      CMAKE_CXX_FLAGS_RELWITHDEBINFO
    )
    
    if(${MSVC_RUNTIME} STREQUAL "static")
      message(STATUS
        "MSVC -> forcing use of statically-linked runtime."
      )
      foreach(variable ${variables})
        if(${variable} MATCHES "/MD")
          string(REGEX REPLACE "/MD" "/MT" ${variable} "${${variable}}")
        endif()
      endforeach()
    
    else()
      message(STATUS
        "MSVC -> forcing use of dynamically-linked runtime."
      )
      foreach(variable ${variables})
        if(${variable} MATCHES "/MT")
          string(REGEX REPLACE "/MT" "/MD" ${variable} "${${variable}}")
        endif()
      endforeach()

    endif()

endmacro()


include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts/)

if(MSVC)
  configure_msvc_runtime()
else()
  add_compile_options(-Wall)
  add_compile_options(-Wno-reorder)

endif()

if(UNIX)
  add_definitions(-D_OS_UNIX)
elseif(WIN32)
  add_definitions(-D_OS_WINDOWS -DNOMINMAX)
  if(MSVC)
     add_compile_options(/bigobj)
  endif()
endif()

if(USE_CONAN)

  include(${PROJECT_SOURCE_DIR}/thirdParty/conan.cmake)

  conan_check(REQUIRED)
  conan_add_remote(
    NAME kinova_public 
    URL https://artifactory.kinovaapps.com/artifactory/api/conan/conan-public)

  if(UNIX)
    conan_cmake_run(REQUIRES kortex_api_cpp/2.3.0-r.34@kortex/stable
                    SETTINGS kortex_api_cpp:compiler=gcc
                    SETTINGS kortex_api_cpp:compiler.version=5
                    SETTINGS compiler.libcxx=libstdc++11
                    PROFILE_AUTO build_type
                    BASIC_SETUP
                    UPDATE)
  elseif(WIN32)
    if(MSVC)

      _get_msvc_ide_version(_VISUAL_VERSION)

      # TODO Conan artifacts should not require to specify a target to download
      if (_VISUAL_VERSION EQUAL 14)
        set(kortex_api_cpp_target "msvc-2015")
      elseif(_VISUAL_VERSION EQUAL 15)
        set(kortex_api_cpp_target "msvc-2017")
      elseif(_VISUAL_VERSION EQUAL 16)
        set(kortex_api_cpp_target "msvc-2019")
      endif()

      conan_cmake_run(REQUIRES kortex_api_cpp/2.3.0-r.34@kortex/stable
                      PROFILE_AUTO build_type
                      BASIC_SETUP
                      UPDATE)
      
    else()
      conan_cmake_run(REQUIRES kortex_api_cpp/2.3.0-r.34@kortex/stable
                      SETTINGS kortex_api_cpp:compiler=gcc
                      SETTINGS kortex_api_cpp:compiler.version=5
                      SETTINGS compiler.libcxx=libstdc++11
                      PROFILE_AUTO build_type
                      BASIC_SETUP
                      UPDATE)
    endif()
  endif()

  link_libraries(${CONAN_LIBS})

else() # Not using Conan

  # Setup Kortex Api Path
  if(NOT KORTEX_SUB_DIR)
    set(KORTEX_SUB_DIR "")
  else()
    set(KORTEX_SUB_DIR "${KORTEX_SUB_DIR}/")
  endif()

  set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/kortex_api/${KORTEX_SUB_DIR}")

  if(CMAKE_BUILD_TYPE EQUAL "Debug")
    set(KORTEX_LIB_SUBDIR "debug")
  else()
    set(KORTEX_LIB_SUBDIR "release")
  endif()

  # Download the API
  if(UNIX)
    execute_process(COMMAND ./download_kortex_api.sh ${KORTEX_SUB_DIR}
      WORKING_DIRECTORY ../scripts
      RESULT_VARIABLE DOWNLOAD_API_RESULT
      OUTPUT_VARIABLE DOWNLOAD_API_OUTPUT)
    if(NOT DOWNLOAD_API_RESULT EQUAL 0)
      message("Kortex API was not downloaded prior to running CMake.")
      message(FATAL_ERROR ${DOWNLOAD_API_OUTPUT})
    endif()
    link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/libKortexApiCpp.a)
  elseif(WIN32)
    execute_process(COMMAND ./download_kortex_api.bat ${KORTEX_SUB_DIR}
      WORKING_DIRECTORY ../scripts
      RESULT_VARIABLE DOWNLOAD_API_RESULT
      OUTPUT_VARIABLE DOWNLOAD_API_OUTPUT)
    if(NOT DOWNLOAD_API_RESULT EQUAL 0)
      message("Kortex API was not downloaded prior to running CMake.")
      message(FATAL_ERROR ${DOWNLOAD_API_OUTPUT})
    endif()
    link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/KortexApiCpp.lib)
  endif()

  # Add Include Directories
  message("Including Directories")
  include_directories(${KORTEX_DIR}include)
  include_directories(${KORTEX_DIR}include/client)
  include_directories(${KORTEX_DIR}include/common)
  include_directories(${KORTEX_DIR}include/messages)
  include_directories(${KORTEX_DIR}include/client_stubs)
  include_directories(${PROJECT_SOURCE_DIR}/Classes/include)

endif()

# link other libs
if(UNIX)
  link_libraries(pthread)
elseif(WIN32)
  link_libraries(winMM ws2_32)
else()
  MESSAGE(FATAL_ERROR "Unknown os! Not supported yet")
endif()


# Create executable for each example
# Look for examples under folders // only looks for files with the format ###Words/###MoreWords.cpp
file(GLOB EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR} "[0-9]*-*/[0-9]*.cpp")
foreach ( SRC_FILE ${EXE_LIST} )

  string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
  string(REPLACE "/" "_" TARGET_EXE_NAME ${TARGET_EXE_NAME})
  
  message("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")

  add_executable(${TARGET_EXE_NAME} ${SRC_FILE} utilities.cpp)
  
endforeach()


file(GLOB EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR} "Integration/*.cpp")
foreach ( SRC_FILE ${EXE_LIST} )

   string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
   string(REPLACE "/" "_" TARGET_EXE_NAME ${TARGET_EXE_NAME})
  
   message("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")


   add_executable(${TARGET_EXE_NAME} ${SRC_FILE} utilities.cpp Classes/src/KortexRobot.cpp Classes/src/serialib.cpp)
 endforeach()
 


# Add executable called "helloDemo" that is built from the source files
# "demo.cxx" and "demo_b.cxx". The extensions are automatically found.
#####add_executable (helloDemo demo.cxx demo_b.cxx)

# Link the executable to the Hello library. Since the Hello library has
# public include directories we will use those link directories when building
# helloDemo
#####target_link_libraries (helloDemo LINK_PUBLIC Hello)
