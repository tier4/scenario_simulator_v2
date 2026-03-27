group "default" {
  targets = ["humble"]
}

target "humble" {
  inherits = ["base"]
  name = "humble"
  tags = ["ghcr.io/tier4/scenario_simulator_v2:humble"]
  args = {"ROS_DISTRO" : "humble"}
  group = ["humble"]
  matrix = {}
  dockerfile = "Dockerfile"
}
