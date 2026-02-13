find_package(rpl QUIET)
if (NOT rpl_FOUND AND NOT rpl::rpl)
    FetchContent_Declare(
            rpl
            GIT_REPOSITORY https://github.com/RoboMaster-DLMU-CONE/rpl.git
            GIT_TAG main
    )
    FetchContent_MakeAvailable(rpl)
endif ()