add_library(
  ${PROJECT_NAME}_common
  SHARED
  frame.cc
  feature.cc
  map_point.cc
  map.cc
  camera.cc
)

target_link_libraries(
  ${PROJECT_NAME}_common
  ${THIRD_PARTY_LIBS}
)
