cmake_minimum_required(VERSION 3.0.0)
project(simplugsketest VERSION 0.1.0)

# include(CTest)
# enable_testing()

#Check C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(NOT SUPPORTS_CXX11)
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

set(coppelia_sim_include_lib_dir1 "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(coppelia_sim_include_lib_dir2 "${CMAKE_CURRENT_SOURCE_DIR}/include/stack")
set(EIGEN_INCLUDE_DIR "D:/eigen-3.3.9")
set(ADVANTECH_HEADER_DIR "D:/ADVANTECH_BACKUP/Public/X64")
set(ADV_GENERAL_HEADER_DIR "D:/ADVANTECH_BACKUP/Examples_1203/General")
set(SCIPLOT_INCLUDE_DIR "D:/CPLOT/sciplot/include")
# set(coppelia_sim_dll_dir "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu")

#add_library(cplsim_dll STATIC ${coppelia_sim_dll_dir}/coppeliaSim.dll)
#set_target_properties(cplsim_dll PROPERTIES LINKER_LANGUAGE C)
file(GLOB my_cplsimlib_source1 "${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp")
file(GLOB my_cplsimlib_source2 "${CMAKE_CURRENT_SOURCE_DIR}/common/stack/*.cpp")
# add_library(my_cplsimlib STATIC ${my_cplsimlib_source})
# target_include_directories(my_cplsimlib PUBLIC
# ${coppelia_sim_include_lib_dir1}
# ${coppelia_sim_include_lib_dir2}
# )

add_library(joint_6dof_lib joint_6dof.cxx)
target_link_libraries(joint_6dof_lib fastcdr fastrtps)

add_executable(joint_6dof joint_6dofPubSubTypes.cxx joint_6dofPublisher.cxx joint_6dofSubscriber.cxx joint_6dofPubSubMain.cxx EfortKine.cpp)
target_include_directories(joint_6dof PUBLIC ${EIGEN_INCLUDE_DIR})
target_link_libraries(joint_6dof fastcdr fastrtps
        joint_6dof_lib )


add_executable(ctl_test2 ctl_test2.cpp joint_6dofPubSubTypes.cxx joint_6dofPublisher.cxx EfortKine.cpp)
target_include_directories(ctl_test2 PUBLIC
${ADVANTECH_HEADER_DIR}
${ADV_GENERAL_HEADER_DIR}
${EIGEN_INCLUDE_DIR}
)


target_link_libraries(ctl_test2
${ADVANTECH_HEADER_DIR}/ADVMOT.lib
fastcdr 
fastrtps
joint_6dof_lib)


# add_library(simExtmytest2 SHARED simExtPluginUpdateArms.cpp ${my_cplsimlib_source1} ${my_cplsimlib_source2} joint_6dofSubscriber.cxx joint_6dofPubSubTypes.cxx)
# target_include_directories(simExtmytest2 PUBLIC
# ${coppelia_sim_include_lib_dir1}
# ${coppelia_sim_include_lib_dir2}
# )

# target_link_libraries(simExtmytest2 joint_6dof_lib)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
