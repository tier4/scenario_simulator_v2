// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__VISIBILITY_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__VISIBILITY_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OPENSCENARIO_INTERPRETER_EXPORT __attribute__((dllexport))
#define OPENSCENARIO_INTERPRETER_IMPORT __attribute__((dllimport))
#else
#define OPENSCENARIO_INTERPRETER_EXPORT __declspec(dllexport)
#define OPENSCENARIO_INTERPRETER_IMPORT __declspec(dllimport)
#endif

#ifdef OPENSCENARIO_INTERPRETER_BUILDING_DLL
#define OPENSCENARIO_INTERPRETER_PUBLIC OPENSCENARIO_INTERPRETER_EXPORT
#else
#define OPENSCENARIO_INTERPRETER_PUBLIC OPENSCENARIO_INTERPRETER_IMPORT
#endif

#define OPENSCENARIO_INTERPRETER_PUBLIC_TYPE OPENSCENARIO_INTERPRETER_PUBLIC
#define OPENSCENARIO_INTERPRETER_LOCAL
#else
#define OPENSCENARIO_INTERPRETER_EXPORT __attribute__((visibility("default")))
#define OPENSCENARIO_INTERPRETER_IMPORT

#if __GNUC__ >= 4
#define OPENSCENARIO_INTERPRETER_PUBLIC __attribute__((visibility("default")))
#define OPENSCENARIO_INTERPRETER_LOCAL __attribute__((visibility("hidden")))
#else
#define OPENSCENARIO_INTERPRETER_PUBLIC
#define OPENSCENARIO_INTERPRETER_LOCAL
#endif

#define OPENSCENARIO_INTERPRETER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__VISIBILITY_HPP_
