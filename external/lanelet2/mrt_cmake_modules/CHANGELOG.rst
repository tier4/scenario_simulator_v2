^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrt_cmake_modules
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2020-09-30)
------------------
* Fix finding boost python on versions with old cmake but new boost
* Contributors: Fabian Poggenhans

1.0.7 (2020-09-30)
------------------
* Fix versioning of sofiles
* Ensure unittests use the right gtest include dir
* Contributors: Fabian Poggenhans

1.0.6 (2020-09-30)
------------------
* Fix boost python building for python3
* Contributors: Fabian Poggenhans

1.0.5 (2020-09-29)
------------------
* Fix build for ROS2, gtest should no longer be installed in ROS2 mode
* Improve python nosetest info
* Update boost-python depend message
* Fix python module setup
* Packages can now have both a python module and a python api
* Add qtbase5-dev key
* Contributors: Fabian Poggenhans, Kevin Rösch, Maximilian Naumann

1.0.4 (2020-08-12)
------------------
* Deleted deprecated configuration files
* Fix cuda host compiler used for cuda 11
* Fix __init__.py template for python3
* Fix target handling for ros2
* Fix build failures on ROS1
* Fix the conan support
* Add a dependency on ros_environment to ensure ROS_VERSION is set
* Default to building shared libraries
* Add QtScript to the list of qt components
* Change license to BSD
* Remove traces of GPL-licensed libgps
* Remove unnecessary includes of cuda files
* Update tensorflow c findscript to set new tensorflow include paths
* Add cuda support for node and nodelet.
* Remove usage of ast package for evaulating package.xml conditions
* Fix crash if eval_coverage.py runs with python3
* Ensure that coverage is also generated for cpp code called from plain rostests
* Contributors: Fabian Poggenhans, Ilia Baltashov, Sven Richter

1.0.3 (2020-05-25)
------------------
* Replace deprecated platform.distro call with distro module
* Raise required CMake version to 3.0.2 to suppress warning with Noetic
* Remove boost signals component that is no longer part of boost
* Fixed c++14 test path include.
* Fix installation of python api files
* Update README.md
* Reformat with new version of cmake-format
* Add lcov as dependency again
* Fix FindBoostPython.cmake for cmake below 3.11 and python3
* Fix multiple include of MrtPCL
* Contributors: Christian-Eike Framing, Fabian Poggenhans, Johannes Beck, Johannes Janosovits, Moritz Cremer

1.0.2 (2020-03-24)
------------------
* Fix PCL findscript, disable precompiling
* added jsoncpp
* Make sure packages search for mrt_cmake_modules in their package config
* Fix resolution of packages in underlaying workspaces
* Mention rosdoc.yaml in package.xml
* Contributors: Fabian Poggenhans, Johannes Beck, Johannes Janosovits

1.0.1 (2020-03-11)
------------------
* Update maintainer
* Update generate_dependency_file to search CMAKE_PREFIX_PATH for packages instead of ROS_PACKAGE_PATH
* Update package xml to contain ROS urls and use format 3 to specify python version specific deps
* Add a rosdoc file so that ros can build the cmake api
* Contributors: Fabian Poggenhans

1.0.0 (2020-02-24)
------------------
* Initial release for ROS
* Contributors: Andre-Marcel Hellmund, Claudio Bandera, Fabian Poggenhans, Johannes Beck, Johannes Graeter, Niels Ole Salscheider, Piotr Orzechowski
