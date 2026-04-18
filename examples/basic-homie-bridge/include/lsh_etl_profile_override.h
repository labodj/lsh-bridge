/**
 * @file    lsh_etl_profile_override.h
 * @author  Jacopo Labardi (labodj)
 * @brief   Example project-local ETL override hook for the basic bridge example.
 * @details This header mirrors the historical `lsh-bridge` ETL policy and
 *          shows how a consumer project can keep ETL policy decisions local
 *          without editing the library copy of `etl_profile.h`.
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

#pragma once

#ifndef ETL_VERBOSE_ERRORS
#define ETL_VERBOSE_ERRORS
#endif

#ifndef ETL_CHECK_PUSH_POP
#define ETL_CHECK_PUSH_POP
#endif
