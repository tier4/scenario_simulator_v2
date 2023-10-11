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

#ifndef CONCEALER__VISIBILITY_HPP_
#define CONCEALER__VISIBILITY_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CONCEALER_EXPORT __attribute__((dllexport))
#define CONCEALER_IMPORT __attribute__((dllimport))
#else
#define CONCEALER_EXPORT __declspec(dllexport)
#define CONCEALER_IMPORT __declspec(dllimport)
#endif

#ifdef CONCEALER_BUILDING_DLL
#define CONCEALER_PUBLIC CONCEALER_EXPORT
#else
#define CONCEALER_PUBLIC CONCEALER_IMPORT
#endif

#define CONCEALER_PUBLIC_TYPE CONCEALER_PUBLIC
#define CONCEALER_LOCAL
#else
#define CONCEALER_EXPORT __attribute__((visibility("default")))
#define CONCEALER_IMPORT

#if __GNUC__ >= 4
#define CONCEALER_PUBLIC __attribute__((visibility("default")))
#define CONCEALER_LOCAL __attribute__((visibility("hidden")))
#else
#define CONCEALER_PUBLIC
#define CONCEALER_LOCAL
#endif

#define CONCEALER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CONCEALER__VISIBILITY_HPP_
