### Requirements

This package depends on these packages
- beautiful soap
- xmltodict
- argparse
- xmlplain

xmlplain package is now in PR for rosdep before merging to master use below instead
before that you need python3-pip
```
python3-pip
pip3 install -r requirements.txt
```

### How To Build
when building use these commands below
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --cmake-clean-cache --cmake-clean-first --packages-select scenario_test_utility --symlink-install
```
