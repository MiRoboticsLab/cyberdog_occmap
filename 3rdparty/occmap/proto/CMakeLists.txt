find_package(Protobuf REQUIRED)

file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
message(STATUS ${PROTO_FILES})

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
message("-- PROTO_SRCS = ${PROTO_SRCS}")
message("-- PROTO_HDRS = ${PROTO_HDRS}")

add_library(proto STATIC ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(proto ${Protobuf_LIBRARIES})
message("-- Found protobuf libraries: " ${Protobuf_LIBRARIES})
message("-- Found protobuf include: " ${PROTOBUF_INCLUDE_DIRS})
target_include_directories(proto
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC ${PROTOBUF_INCLUDE_DIRS}
        PUBLIC "/usr/local/include")

install(TARGETS
        proto
        LIBRARY DESTINATION lib)