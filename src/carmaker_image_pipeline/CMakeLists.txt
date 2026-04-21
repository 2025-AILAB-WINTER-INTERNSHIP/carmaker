cmake_minimum_required(VERSION 3.0.2)
project(carmaker_image_pipeline)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/make_mask.py
  scripts/apply_mask.py
  scripts/extract_bag_images.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
