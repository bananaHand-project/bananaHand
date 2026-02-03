cmake_minimum_required(VERSION 3.8)
project(banana_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add message files to this list as you create them.
set(msg_files
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
# )

ament_package()
