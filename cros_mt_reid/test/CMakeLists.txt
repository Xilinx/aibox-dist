add_executable(test_merge test_merge.cpp ../../src/cross_track.cpp)
target_link_libraries(test_merge ${PROJECT_NAME} pthread vitis_ai_library-refinedet vitis_ai_library-classification vitis_ai_library-dpu_task)

add_executable(test_accuracy test_merge_accuracy.cpp)
target_link_libraries(test_accuracy ${PROJECT_NAME} pthread vitis_ai_library-refinedet vitis_ai_library-classification)
#install(
#  TARGETS test_merge
#  DESTINATION ${INSTALL_BIN_DIR}
#  )


#add_executable(test_videos test_videos_reidtracker.cpp)
#target_link_libraries(test_videos ${PROJECT_NAME} pthread vitis_ai_library-refinedet)
