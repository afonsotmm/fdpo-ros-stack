cmake_minimum_required(VERSION 3.0.2)
project(conf)

find_package(catkin REQUIRED)
catkin_package()

# Instala (ou torna visíveis) os ficheiros do conf para $(find conf)
install(DIRECTORY launch system script
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL
        PATTERN ".git*" EXCLUDE
        PATTERN "*.swp" EXCLUDE)
