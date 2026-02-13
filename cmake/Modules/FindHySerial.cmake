find_package(HySerial QUIET)

if (NOT HySerial_FOUND AND NOT HySerial::HySerial)
    FetchContent_Declare(
            HySerial
            GIT_REPOSITORY https://github.com/RoboMaster-DLMU-CONE/HySerial.git
            GIT_TAG main
    )
    FetchContent_MakeAvailable(HySerial)
endif ()