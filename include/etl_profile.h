/**
 * @file    etl_profile.h
 * @author  Jacopo Labardi (labodj)
 * @brief   Defines the default ETL compile-time profile used by lsh-bridge.
 * @details This file provides the default ETL policy used by `lsh-bridge` on
 *          the supported Arduino/PlatformIO targets. ETL auto-detects the
 *          active compiler and language support through
 *          `etl/profiles/auto.h`.
 *
 *          Advanced consumers that need a different ETL setup may define
 *          `LSH_ETL_PROFILE_OVERRIDE_HEADER` to a quoted header name through
 *          their build flags. That header is included last, so it may `#undef`
 *          and redefine any of the defaults below.
 *
 * Copyright 2026 Jacopo Labardi
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ETL_PROFILE_H
#define ETL_PROFILE_H

// Default ETL safety/debug policy for lsh-bridge.
#define ETL_VERBOSE_ERRORS
#define ETL_CHECK_PUSH_POP

// Let ETL detect the active compiler, language level and platform traits.
#include "etl/profiles/auto.h"

// Optional project-local override hook for non-default toolchains or policies.
#ifdef LSH_ETL_PROFILE_OVERRIDE_HEADER
#include LSH_ETL_PROFILE_OVERRIDE_HEADER
#endif

#endif  // ETL_PROFILE_H
