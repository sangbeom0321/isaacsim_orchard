cmake_minimum_required(VERSION 3.8)
project(isaacsim_orchard)

# 기본 컴파일 옵션 (경고를 에러로 안 봄)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# -----------------------------
# 실행 스크립트 설치 (setup.py 없이)
# -----------------------------
# scripts/start_sim.py 에 실행 권한(+x) 필요
install(PROGRAMS
  scripts/start_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# 설정/런치 파일 설치 (있을 때만)
# -----------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
cmake_minimum_required(VERSION 3.8)
project(isaacsim_orchard)

# 기본 컴파일 옵션 (경고를 에러로 안 봄)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# -----------------------------
# 실행 스크립트 설치 (setup.py 없이)
# -----------------------------
# scripts/start_sim.py 에 실행 권한(+x) 필요
install(PROGRAMS
  scripts/start_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# 설정/런치 파일 설치 (있을 때만)
# -----------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()