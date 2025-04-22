if(BUILD_TEST)
  foreach(CPP_FILE ${CPP_FILES})
    # Get the filename without extension
    get_filename_component(filename_no_extension ${CPP_FILE} NAME_WE)

    get_filename_component(file_directory "${CPP_FILE}" DIRECTORY)
    file(RELATIVE_PATH relative ${CMAKE_SOURCE_DIR}/src ${file_directory})

    # add_executable(${filename_no_extension}_test test/${relative}/${filename_no_extension}_test.cpp src/${relative}/${filename_no_extension}.cpp)
    add_executable(${filename_no_extension}_test test/${relative}/${filename_no_extension}_test.cpp ${CPP_FILES})
    target_link_libraries(${filename_no_extension}_test ${common_lib} gtest gtest_main)
    # Register test
    add_test(NAME ${filename_no_extension}Test COMMAND ${filename_no_extension}_test)
  endforeach()

endif()