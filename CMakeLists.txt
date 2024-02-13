

find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
target_include_directories(main PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})