add_executable(run_kitti run_kitti.cc)

target_link_libraries(
    run_kitti
    ${PROJECT_NAME}_app
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_module
    ${PROJECT_NAME}_tool
    ${THIRD_PARTY_LIBS}
    )
add_executable(run_m300 run_m300.cc)

target_link_libraries(
    run_m300
    ${PROJECT_NAME}_app
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_module
    ${PROJECT_NAME}_tool
    ${THIRD_PARTY_LIBS}
    )

add_executable(test_m300_dataset test_m300_dataset.cc)

target_link_libraries(
    test_m300_dataset
    ${PROJECT_NAME}_app
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_module
    ${PROJECT_NAME}_tool
    ${THIRD_PARTY_LIBS}
    )
