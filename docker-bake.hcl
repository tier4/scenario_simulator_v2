group "default" {
  targets = ["humble"]
}

target "base" {
  target = "build-stage"
  dockerfile = "Dockerfile"
  platforms = ["linux/amd64", "linux/arm64/v8"]
}

target "humble" {
  inherits = ["base"]
  name = replace("humble-${cuda_version}-${cuda_distro}", ".", "_")
  tags = ["ghcr.io/tier4/scenario_simulator_v2:humble"]
  args = {"ROS_DISTRO" : "humble"}
  group = ["humble"]
}

