group "default" {
  targets = ["humble"]
}

target "base" {
  target = "build-stage"
  dockerfile = "Dockerfile"
  platforms = ["linux/amd64"]
}

target "base_arm64" {
  dockerfile = "Dockerfile.arm64"
  platforms = ["linux/arm64/v8"]
}

target "humble" {
  inherits = [base]
  name = "humble-${base}"
  tags = ["ghcr.io/tier4/scenario_simulator_v2:humble"]
  args = {"ROS_DISTRO" : "humble"}
  group = ["humble"]
  matrix = {
    base = ["base", "base_arm64"]
  }
}
