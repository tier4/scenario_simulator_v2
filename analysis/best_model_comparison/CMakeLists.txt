cmake_minimum_required(VERSION 3.14)
project(best_model_comparison)

find_package(ament_cmake REQUIRED)

install(FILES
  _params_utils.py
  compare_logs.py
  make_lite.py
  analyze_real_curve2.py
  analyze_trajectory.py
  compare_curve2.py
  compare_dp_vs_final.py
  analyze_curve2_per_step.py
  diagnose_curve2.py
  diagnose_departure.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
