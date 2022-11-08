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
        repo = "grailbio/bazel-toolchain",
        commit = "056aeaa01900f5050a9fed9b11e2d365a684831a",
        sha256 = "d319a1155a3a5079954868d814d316d77b18cd89b8cb3554ad06d64c12b8bef1",
    )
