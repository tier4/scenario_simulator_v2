group "default" {
  targets = ["build"]
}

target "build" {
  target = "build-stage"
  tags = ["embree_vendor"]
  args = {
    "ROS_DISTRO" : "humble"
  }
  platforms = ["linux/amd64", "linux/arm64/v8"]
}
