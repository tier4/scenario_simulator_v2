group "default" {
  targets = [
    "full_development",
    "full_runtime",
    "full_desktop",
    "traffic_simulator_development",
    "traffic_simulator_runtime",
    "traffic_simulator_desktop"
  ]
}

target "base" {
  args = {"ROS_DISTRO" : "humble"}
  platforms = [
    "linux/amd64",
    "linux/arm64"
  ]
}

# Each stage
target "development" {
  inherits = ["base"]
  target = "development"
}

target "runtime" {
  inherits = ["base"]
  target = "runtime"
}

target "desktop" {
  inherits = ["base"]
  target = "desktop"
}

# Each product
target "full" {
  dockerfile = "Dockerfile"
}

target "traffic_simulator" {
  dockerfile = "Dockerfile.traffic_simulator"
}

# Each output
target "full_development" {
  inherits = ["development", "full"]
  tags = ["ghcr.io/tier4/scenario_simulator_v2:humble-devel"]
}

target "full_runtime" {
  inherits = ["runtime", "full"]
  tags = ["ghcr.io/tier4/scenario_simulator_v2:humble"]
}

target "full_desktop" {
  inherits = ["desktop", "full"]
  tags = ["ghcr.io/tier4/scenario_simulator_v2:humble-desktop"]
}

target "traffic_simulator_development" {
  inherits = ["development", "traffic_simulator"]
  tags = ["ghcr.io/tier4/scenario_simulator_v2:traffic_simulator_humble-devel"]
}

target "traffic_simulator_runtime" {
  inherits = ["runtime", "traffic_simulator"]
  tags = ["ghcr.io/tier4/scenario_simulator_v2:traffic_simulator_humble"]
}

target "traffic_simulator_desktop" {
  inherits = ["desktop", "traffic_simulator"]
  tags = ["ghcr.io/tier4/scenario_simulator_v2:traffic_simulator_humble-desktop"]
}
