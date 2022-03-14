add_executable(run_kitti run_kitti.cc)

target_link_libraries(
    run_kitti
    ${PROJECT_NAME}_app
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_module
    ${PROJECT_NAME}_tool
    ${THIRD_PARTY_LIBS}
    )
