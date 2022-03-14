add_library(
  ${PROJECT_NAME}_module
  frontend.cc
  backend.cc
)

target_link_libraries(
  ${PROJECT_NAME}_module
  ${PROJECT_NAME}_common
  ${PROJECT_NAME}_tool
  ${THIRD_PARTY_LIBS}
)
