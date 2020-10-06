^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrt_cmake_modules
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
