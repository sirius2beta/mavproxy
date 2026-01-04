cmake_minimum_required(VERSION 3.16)

project(mavlinkProxy LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core SerialPort Network)

add_executable(mavlinkProxy
  main.cpp
)
target_link_libraries(mavlinkProxy Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::SerialPort Qt${QT_VERSION_MAJOR}::Network)

include(GNUInstallDirs)
include_directories(c_library_v2/common)
install(TARGETS mavlinkProxy
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
