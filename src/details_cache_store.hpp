/**
 * @file    details_cache_store.hpp
 * @author  Jacopo Labardi (labodj)
 * @brief   Declares the small NVS-backed cache used to persist controller topology details across bridge reboots.
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

#ifndef LSH_BRIDGE_DETAILS_CACHE_STORE_HPP
#define LSH_BRIDGE_DETAILS_CACHE_STORE_HPP

#include "virtual_device.hpp"

namespace DetailsCacheStore
{
[[nodiscard]] auto load(DeviceDetailsSnapshot &outDetails) -> bool;

[[nodiscard]] auto save(const DeviceDetailsSnapshot &details) -> bool;

[[nodiscard]] auto clear() -> bool;
}  // namespace DetailsCacheStore

#endif  // LSH_BRIDGE_DETAILS_CACHE_STORE_HPP
