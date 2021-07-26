# -*- python -*-

# Copyright 2020 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("//tools/workspace:github_archive.bzl", "github_archive")

def bazel_toolchain_repository():
    github_archive(
        name = "com_grail_bazel_toolchain",
        repo = "mjbots/bazel-toolchain",
        commit = "9b0bae6a79392920698b9778e334ce2354b425a5",
        sha256 = "36420a9fc39fb9462f06c0da386bb987a191cc0e388909fe4873e2a3c758c2be",
    )
