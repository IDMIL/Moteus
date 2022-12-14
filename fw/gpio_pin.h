// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#pragma once

namespace moteus {

class GpioPin {
 public:
  virtual ~GpioPin() {}

  virtual void Set(bool) = 0;
  virtual bool Read() const = 0;

  enum PinMode {
    kInput = 0,
    kGeneralOutput = 1,
    kAlternateFunction = 2,
    kAnalog = 3,
  };
  virtual void SetMode(PinMode) = 0;
};

}
