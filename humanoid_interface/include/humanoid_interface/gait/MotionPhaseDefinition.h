// /******************************************************************************
// Copyright (c) 2021, Farbod Farshidian. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//  * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

//  * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

//  * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ******************************************************************************/

// #pragma once

// #include <iostream>
// #include <map>
// #include <string>
// #include <vector>

// #include <boost/property_tree/info_parser.hpp>
// #include <boost/property_tree/ptree.hpp>

// #include <ocs2_core/misc/LoadData.h>

// #include "humanoid_interface/common/Types.h"

// namespace ocs2 {
// namespace humanoid {

// enum ModeNumber {
//   FLY = 0,
//   LCONTACT = 1,
//   RCONTACT = 2,
//   STANCE = 3,
// };

// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
//   contact_flag_t stanceLegs;

//   switch (modeNumber) {
//     case 0:
//       stanceLegs = contact_flag_t{false, false, false, false};
//       break;  // 0:  0-leg-stance
//     case 1:
//         stanceLegs = contact_flag_t{true, false, true, false};
//         break;  // 1:  1-leg-stance
//     case 2:
//         stanceLegs = contact_flag_t{false, true, false, true};
//         break;  // 2:  1-leg-stance
//     case 3:
//         stanceLegs = contact_flag_t{true, true, true, true};
//         break;  // 3:  2-leg-stance
//   }

//   return stanceLegs;
// }

// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// inline size_t stanceLeg2ModeNumber(const contact_flag_t& stanceLegs) {
//   return static_cast<size_t>(stanceLegs[0]) + 2 * static_cast<size_t>(stanceLegs[3]);
// }

// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// inline std::string modeNumber2String(const size_t& modeNumber) {
//   // build the map from mode number to name
//   std::map<size_t, std::string> modeToName;
//   modeToName[FLY] = "FLY";
//   modeToName[LCONTACT] = "LCONTACT";
//   modeToName[RCONTACT] = "RCONTACT";
//   modeToName[STANCE] = "STANCE";

//   return modeToName[modeNumber];
// }

// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// inline size_t string2ModeNumber(const std::string& modeString) {
//   // build the map from name to mode number
//   std::map<std::string, size_t> nameToMode;
//   nameToMode["FLY"] = FLY;
//   nameToMode["LCONTACT"] = LCONTACT;
//   nameToMode["RCONTACT"] = RCONTACT;
//   nameToMode["STANCE"] = STANCE;

//   return nameToMode[modeString];
// }

// }  // namespace humanoid
// }  // end of namespace ocs2
#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "humanoid_interface/common/Types.h"

namespace ocs2 {
namespace humanoid {

// 1. 修正枚举值：匹配 4 个接触点的二进制位掩码
// 位0: 左脚尖 (l_toe)
// 位1: 右脚尖 (r_toe)
// 位2: 左脚跟 (l_heel)
// 位3: 右脚跟 (r_heel)
enum ModeNumber {
  FLY = 0,        // 0000 -> 空中
  LCONTACT = 5,   // 0101 -> 左脚尖(1) + 左脚跟(4) = 5
  RCONTACT = 10,  // 1010 -> 右脚尖(2) + 右脚跟(8) = 10
  STANCE = 15,    // 1111 -> 全部触地 = 15
};

/******************************************************************************************************/
/* 将数字 Mode 解码为 4 个接触点的布尔数组 */
/******************************************************************************************************/
inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs;
  // 直接使用位与运算解析，绝对不会出错
  stanceLegs[0] = (modeNumber & 1) != 0; // l_toe
  stanceLegs[1] = (modeNumber & 2) != 0; // r_toe
  stanceLegs[2] = (modeNumber & 4) != 0; // l_heel
  stanceLegs[3] = (modeNumber & 8) != 0; // r_heel
  
  return stanceLegs;
}

/******************************************************************************************************/
/* 将 4 个接触点的布尔数组编码为数字 Mode */
/******************************************************************************************************/
inline size_t stanceLeg2ModeNumber(const contact_flag_t& stanceLegs) {
  // 使用位移(Shift)组装掩码
  return (static_cast<size_t>(stanceLegs[0]) << 0) |
         (static_cast<size_t>(stanceLegs[1]) << 1) |
         (static_cast<size_t>(stanceLegs[2]) << 2) |
         (static_cast<size_t>(stanceLegs[3]) << 3);
}

/******************************************************************************************************/
/* 数字转字符串 (用于日志打印等) */
/******************************************************************************************************/
inline std::string modeNumber2String(const size_t& modeNumber) {
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[LCONTACT] = "LCONTACT";
  modeToName[RCONTACT] = "RCONTACT";
  modeToName[STANCE] = "STANCE";

  if (modeToName.find(modeNumber) != modeToName.end()) {
    return modeToName[modeNumber];
  } else {
    // 允许打印出混合模式(比如转换瞬间只有一只脚跟离地)
    return "MIXED_" + std::to_string(modeNumber); 
  }
}

/******************************************************************************************************/
/* 字符串转数字 (用于解析 .info 文件) */
/******************************************************************************************************/
inline size_t string2ModeNumber(const std::string& modeString) {
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["LCONTACT"] = LCONTACT;
  nameToMode["RCONTACT"] = RCONTACT;
  nameToMode["STANCE"] = STANCE;

  return nameToMode[modeString];
}

}  // namespace humanoid
}  // end of namespace ocs2