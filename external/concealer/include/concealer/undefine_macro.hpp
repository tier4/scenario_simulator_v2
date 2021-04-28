// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef CONCEALER__UNDEFINE_MACRO_HPP_
#define CONCEALER__UNDEFINE_MACRO_HPP_

#ifdef DEFINE_SUBSCRIPTION
#undef DEFINE_SUBSCRIPTION
#endif

#ifdef DEFINE_PUBLISHER
#undef DEFINE_PUBLISHER
#endif

#ifdef INIT_SUBSCRIPTION
#undef INIT_SUBSCRIPTION
#endif

#ifdef INIT_PUBLICATION
#undef INIT_PUBLICATION
#endif

#undef CONCEALER__DEFINE_MACRO_HPP_

#endif  // CONCEALER__UNDEFINE_MACRO_HPP_
