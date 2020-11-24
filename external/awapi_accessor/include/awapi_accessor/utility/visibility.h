// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef AWAPI_ACCESSOR__UTILITY__VISIBILITY_H_
#define AWAPI_ACCESSOR__UTILITY__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
#  ifdef __GNUC__
#    define AWAPI_ACCESSOR_EXPORT __attribute__ ((dllexport))
#    define AWAPI_ACCESSOR_IMPORT __attribute__ ((dllimport))
#  else
#    define AWAPI_ACCESSOR_EXPORT __declspec(dllexport)
#    define AWAPI_ACCESSOR_IMPORT __declspec(dllimport)
#  endif

#  ifdef AWAPI_ACCESSOR_BUILDING_DLL
#    define AWAPI_ACCESSOR_PUBLIC AWAPI_ACCESSOR_EXPORT
#  else
#    define AWAPI_ACCESSOR_PUBLIC AWAPI_ACCESSOR_IMPORT
#  endif

#  define AWAPI_ACCESSOR_PUBLIC_TYPE AWAPI_ACCESSOR_PUBLIC
#  define AWAPI_ACCESSOR_LOCAL
#else
#  define AWAPI_ACCESSOR_EXPORT __attribute__ ((visibility("default")))
#  define AWAPI_ACCESSOR_IMPORT

#  if __GNUC__ >= 4
#    define AWAPI_ACCESSOR_PUBLIC __attribute__ ((visibility("default")))
#    define AWAPI_ACCESSOR_LOCAL __attribute__ ((visibility("hidden")))
#  else
#    define AWAPI_ACCESSOR_PUBLIC
#    define AWAPI_ACCESSOR_LOCAL
#  endif

#  define AWAPI_ACCESSOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // AWAPI_ACCESSOR__UTILITY__VISIBILITY_H_
