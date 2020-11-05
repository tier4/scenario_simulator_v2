# MRT CMake Modules (Massively Reduced Time writing CMake Modules(*))

Maintainer status: maintained
- Maintainer: Johannes Beck <johannes.beck@kit.edu>, Fabian Poggenhans <fabian.poggenhans@kit.edu>
- Author: Johannes Beck <johannes.beck@kit.edu>, Claudio Bandera claudio.bandera@kit.edu, Fabian Poggenhans <fabian.poggenhans@kit.edu>
- License: BSD, some files MIT
- Bug / feature tracker: https://gitlab.mrt.uni-karlsruhe.de/MRT/mrt_cmake_modules/issues
- Source: git https://gitlab.mrt.uni-karlsruhe.de/MRT/mrt_cmake_modules.git (branch: master)

Imagine you whould never have to write a **CMakeLists.txt** file again. Never forget to install everything, no need to update it whenever you add a file, not time lost for figuring out how to call and use `find_package` for your dependencies, etc.

Well, this is exactly what `mrt_cmake_modules` do for you! The only thing you have to do is:
- Keep your files in a [fixed structure](#package-structure),
- keep library and executable code in separate packages and
- make sure the package.xml actually contains all the packages you depend on.

If you don't want to agree to this, there are [ways to get this done as well](#cmake-api). This will require you to modify our template file a bit.

On the other hand, you get a lot of things for free:
- Resolution of your [dependencies in CMake](#how-dependency-resolution-works-in-detail)
- Automatic generation of Nodes/Nodelets
- Supports C++ and Python(2/3)
- Python bindings for pybind11/boost-python
- Automated unittest detection and execution (including ROS tests)
- Automated [code coverage generation](#generating-code-coverage)
- Support for [sanitizers](#sanitizing-your-code)
- Support for [running clang-tidy](#using-clang-tidy)
- Automated install of your scripts/launchfiles/executables/libraries... to the correct location
- experimental support for **ROS2** and Conan builds

*) Actually MRT stands for *Institut für Mess- und Regelungstechnik*, the institute that develops this package.

## Building
`mrt_cmake_modules` is kept as leightweight as possible. It just contains a few CMake (and python) scripts. Its only dependency is `catkin` and `lcov` (for code coverage).

## Getting started

Interested? In order to get the CMake template file, you have to run a small script: `rosrun mrt_cmake_modules generate_cmakelists.py <package_name> [--ros] [--exe]`.
This will create a CMakeLists.txt file ready to be used in your project. We distinguish four different types of packages (this the `--ros` and `--exe` flags):
- **Library package**: They have no dependency to ros (except catkin). Their goal is to either build a `lib<package>.so`, contain only headers, contain a python module, contain python bindings. Or a mixture of these. And unittests, of course.
- **Ros library package (--ros)**: Similar to the above, but can also contain message, action or configuration files
- **Executable package (--exe)**: Provides either a number of executables, scripts or python modules for these scripts. And unittests of course.
- **Node/Nodelet package (--ros --exe)**: Provides a number of nodes or nodelets, python nodes and launchfiles. And rostests of course.

## Package structure

The best way to understand how packages have to be layed out is to have a look at the tree of an example package, and what it implies on the build.

### Libraries

Here is the structure of a package called `example_package`. It works out of the box with a *CMakeLists.txt* created with `generate_cmakelists.py example_package --ros`:
```bash
.
├── CMakeLists.txt                    # The generated CMakeLists
├── include                           # The headers of this package. Available in the whole package
│   └── example_package               # It is a ROS convention to keep them within include/<package_name>
│       ├── a_header.hpp
│       └── internal                  # Headers here are only to be used within cpp files of this package
│           └── internal_header.hpp
├── msg
│   └── a_message.msg                 # Messages files that will be automatically generated (only available for --ros)
├── package.xml                       # You should know that. Should contain at least pybind11-dev for the bindings
├── python_api                        # Folder for python bindings. Every file here becoms a module
│   ├── python_bindings.cpp           # Will be available as "import example_package.python_bindings"
│   └── more_python_bindings.cpp      # Refer to the pybind11 doc for the content of this file
├── README.md                         # The readme
├── src                               # Every cpp file in this filder will become part of libexample_package.so
│   ├── examplefile.cpp
│   ├── onemorefile.cpp
│   └── example_package               # Python modules have to go to src/<package_name>
│       ├── pythonmodule.py           # Will be available as "import example_package.pythonmodule"
│       └── __init__.py
└── test                              # Contains the unittests. Will be executed when running the test or run_tests target
    ├── test_example_package.cpp      # Every file here will be a separate unittest executable
    └── test_pyapi.py                 # Every py file that matches the testMatch regular expression will be executed
                                      # using nosetest. Executables are ignored. See https://nose.readthedocs.io/
```

Note that everything in this structure is optional and can be left away if you don't need it (except for the CMakeLists.txt of course).

### Executables, Nodes and Nodelets

Here is the structure of a package called `example_package_ros_tool`.
It works out of the box with a *CMakeLists.txt* created with `generate_cmakelists.py example_package_ros_tool --exe --ros`:

```bash
.
├── CMakeLists.txt
├── cfg
│   └── ConfigFile.cfg                    # Files to be passed to dynamic_reconfigure
├── launch                                # Contains launch files provided by this package
│   ├── some_node.launch
│   ├── some_nodelet.launch
│   ├── some_python_node.launch
│   └── params                            # Contains parameter files that will be installed
│       ├── some_parameters.yaml
│       └── some_python_parameters.yaml
├── nodelet_plugins.xml                   # Should reference the nodelet library at lib/lib<package_name>-<nodename>-nodelet
├── package.xml                           # The manifest. It should reference the nodelet_plugins.xml
├── README.md
├── scripts                               # Executable scripts that will be installed. These can be python nodes as well
│   ├── bias_python_node.py
│   └── bias_script.sh
├── src                                   # Every folder in here will be a node/nodelet or both
│   ├── nodename                          # nodename is the name of the node/nodelet
│   │   ├── somefile.cpp                  # Files compiled into the node and the nodelet
│   │   ├── somefile.hpp
│   │   ├── some_node.cpp                 # files ending with _node will end up being compiled into the node executable
│   │   └── a_nodelet.cpp                 # files ending whth _nodelet will end up as part of the nodelet library
│   └── example_package_ros_tool          # python modules have to go to src/<package_name>.
│       ├── node_module_python.py         # These files can provide the node implementation for the scripts
│       └── __init__.py
└── test                                  # The unittests
    ├── test_node.cpp                     # Every pair of .cpp and .test files represents one rostest
    ├── test_node.test
    ├── test.cpp                          # A cpp file without a matching .test a normal unittest and launched without ROS
    ├── node_python_test.py               # Same for .py and .test files with the same name. The .py file must be executable!
    └── node_python_test.test
```

For a normal, non-ros executable the structure is similar, but simpler. Every folder in the src file will become the name of an executable, and all cpp files within it will be part of the executable.

## Generating code coverage
By default, your project is compiled without code coverage information. This can be changed by setting the cmake parameter `MRT_ENABLE_COVERAGE` (e.g. by configuring cmake with `-DMRT_ENABLE_COVERAGE=true`).
Setting this requires a recompilation of the project.
If this is set, running the `run_tests` target will run the unittests and afterwards automatically generate an html coverage report.
The location is printed in the command line. If you set `MRT_ENABLE_COVERAGE=2`, it will also be opened in firefox afterwards.

## Sanitizing your code
The Sanitizers [Asan](https://clang.llvm.org/docs/AddressSanitizer.html), [Tsan](https://clang.llvm.org/docs/ThreadSanitizer.html) and [UBsan](https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html) are great ways of detecting bugs at runtime.
Sadly, you cannot enable them all as once, because Tsan cannot be used together with Asan and UBsan.
You can enable them similarly to the code coverage by setting the `MRT_SANITIZER` cmake variable. It has two possible values:
- checks (enables Asan and UBsan)
- check_race (enables tsan)

If one of the sanitzers discovers an error at runtime, it will be printed to cerr.

## Using clang-tidy
Clang-tidy checks your code statically at compile time for patterns of unsave or wrong code. Using this feature requires clang-tidy to be installed.
You might also want to provide a .clang-tidy file within your project that enables the checks of your choice. Refere to clang-tidys documentation for details.
You can enable clang-tidy by setting the CMake variable `MRT_CLANG_TIDY`.

If you set it to "check", clang-tidy will be run at compile time alongside the normal compiler and print all issues as compiler errors.
If you set it to "fix" the recommended fixes will be applied directly to your code.
**Careful**: Not all fixes ensure that the code compiles.
Also the "fix" mode of clang-tidy is not thread safe, meaning that if you compile with multiple threads, multiple clang-tidy instances might try to fix the same file at the same time.

## How dependency resolution works in detail
The *mrt_cmake_modules* parse your manifest file (i.e. `package.xml`) in order to figure out your dependencies. The following lines in the `CMakeLists.txt` are responsible for this:
```cmake
find_package(mrt_cmake_modules REQUIRED)                          # finds this module
include(UseMrtStdCompilerFlags)                                   # sets some generally useful compiler/warning flags
include(GatherDeps)                                               # collects your dependencies and writes them into DEPENDEND_PACKAGES
find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})  # resolves your dependencies by calling find_package appropriately.
```
AutoDeps then figures out, which includes, libraries and targets are needed in order to build this package.
The result is written into `MRT_INCLUDE_DIRS`, `MRT_LIBRARIES` and `MRT_LIBRARY_DIRS`. These variables should be passed to the cmake targets.

The magic that happens under the hood can be understood by looking at the [cmake.yaml](yaml/cmake.yaml) within this project. It defines, for which dependency in the package.xml which findscript has to be searched and what variables
it will set by the script if successful. These will then be appended to the `MRT_*` variables.

## CMake API
This package contains a lot of useful cmake functions that are automatically available in all packages using the _mrt_cmake_modules_ as dependency, e.g. `mrt_install()`, `mrt_add_node_and_nodelet()`, `mrt_add_library()`, etc.
The automatically generated file already makes use of them, but you can also use them in your custom CMakeLists.
See [here](http://htmlpreview.github.io/?https://github.com/KIT-MRT/mrt_cmake_modules/blob/master/doc/generated_cmake_api.html) for a full documentation.

## Findscripts
This package also contains a collection of [find-scripts](cmake/Modules) for thirdparty dependencies that are usually not shipped with a findscript. These are required by __AutoDeps__.

## Going further
After you have saved time writing cmake boilerplate, it is now time to also save time writing ROS boilerplate. You might want to look into the [rosinterface_handler](https://github.com/KIT-MRT/rosinterface_handler) as well. It is already intergrated in the CMakeLists template.
All you have to do is write a `.rosif` file, and this project will automatically handle reading parameters from the parameter server, creating subscribers and publishers and handling reconfigure callbacks for you!
