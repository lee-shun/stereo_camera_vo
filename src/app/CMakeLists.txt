add_library(
  ${PROJECT_NAME}_app
  SHARED
  visual_odometry.cc
)

target_link_libraries(
  ${PROJECT_NAME}_app
  ${PROJECT_NAME}_module
  ${THIRD_PARTY_LIBS}
)
