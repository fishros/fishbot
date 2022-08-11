/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#pragma once

#include <ydlidar_protocol.h>
#include <map>

namespace ydlidar {

enum YDLIDAR_MODLES {
  YDLIDAR_F4      = 1,/**< F4雷达型号代号. */
  YDLIDAR_T1      = 2,/**< T1雷达型号代号. */
  YDLIDAR_F2      = 3,/**< F2雷达型号代号. */
  YDLIDAR_S4      = 4,/**< S4雷达型号代号. */
  YDLIDAR_G4      = 5,/**< G4雷达型号代号. */
  YDLIDAR_X4      = 6,/**< X4雷达型号代号. */
  YDLIDAR_G4PRO   = 7,/**< G4PRO雷达型号代号. */
  YDLIDAR_F4PRO   = 8,/**< F4PRO雷达型号代号. */
  YDLIDAR_R2      = 9,/**< R2雷达型号代号. */
  YDLIDAR_G10     = 10,/**< G10雷达型号代号. */
  YDLIDAR_S4B     = 11,/**< S4B雷达型号代号. */
  YDLIDAR_S2      = 12,/**< S2雷达型号代号. */
  YDLIDAR_G6      = 13,/**< G6雷达型号代号. */
  YDLIDAR_G2A     = 14,/**< G2A雷达型号代号. */
  YDLIDAR_G2B     = 15,/**< G2雷达型号代号. */
  YDLIDAR_G2C     = 16,/**< G2C雷达型号代号. */
  YDLIDAR_G4B     = 17,/**< G4B雷达型号代号. */
  YDLIDAR_G4C     = 18,/**< G4C雷达型号代号. */
  YDLIDAR_G1      = 19,/**< G1雷达型号代号. */

  YDLIDAR_TG15    = 100,/**< TG15雷达型号代号. */
  YDLIDAR_TG30    = 101,/**< T30雷达型号代号. */
  YDLIDAR_TG50    = 102,/**< TG50雷达型号代号. */
  YDLIDAR_Tail,
};

enum YDLIDAR_RATE {
  YDLIDAR_RATE_4K = 0,
  YDLIDAR_RATE_8K = 1,
  YDLIDAR_RATE_9K = 2,
  YDLIDAR_RATE_10K = 3,
};

/*!
 * @brief lidarModelToString
 * @param model
 * @return
 */
inline std::string lidarModelToString(int model) {
  std::string name = "unkown";

  switch (model) {
    case YDLIDAR_F4:
      name = "F4";
      break;

    case YDLIDAR_T1:
      name = "T1";

      break;

    case YDLIDAR_F2:
      name = "F2";

      break;

    case YDLIDAR_S4:
      name = "S4";

      break;

    case YDLIDAR_G4:
      name = "G4";

      break;

    case YDLIDAR_X4:
      name = "X4";

      break;

    case YDLIDAR_G4PRO:
      name = "G4PRO";

      break;

    case YDLIDAR_F4PRO:
      name = "F4PRO";

      break;

    case YDLIDAR_R2:
      name = "R2";

      break;

    case YDLIDAR_G10:
      name = "G10";

      break;

    case YDLIDAR_S4B:
      name = "S4B";

      break;

    case YDLIDAR_S2:
      name = "S2";

      break;

    case YDLIDAR_G6:
      name = "G6";

      break;

    case YDLIDAR_G2A:
      name = "G2A";

      break;

    case YDLIDAR_G2B:
      name = "G2B";

      break;

    case YDLIDAR_G2C:
      name = "G2C";

      break;

    case YDLIDAR_G4B:
      name = "G4B";

      break;

    case YDLIDAR_G4C:
      name = "G4C";
      break;

    case YDLIDAR_G1:
      name = "G1";

      break;

    case YDLIDAR_TG15:
      name = "TG15";

      break;

    case YDLIDAR_TG30:
      name = "TG30";

      break;

    case YDLIDAR_TG50:
      name = "TG50";
      break;

    default:
      name = "unkown";
      break;
  }

  return name;
}

/*!
 * @brief lidarModelDefaultSampleRate
 * @param model
 * @return
 */
inline int lidarModelDefaultSampleRate(int model) {
  int sample_rate = 4;

  switch (model) {
    case YDLIDAR_F4:
      break;

    case YDLIDAR_T1:
      break;

    case YDLIDAR_F2:
      break;

    case YDLIDAR_S4:
      break;

    case YDLIDAR_G4:
      sample_rate = 9;
      break;

    case YDLIDAR_X4:
      sample_rate = 5;
      break;

    case YDLIDAR_G4PRO:
      sample_rate = 9;
      break;

    case YDLIDAR_F4PRO:
      sample_rate = 4;
      break;

    case YDLIDAR_R2:
      sample_rate = 5;
      break;

    case YDLIDAR_G10:
      sample_rate = 10;
      break;

    case YDLIDAR_S4B:
      sample_rate = 4;
      break;

    case YDLIDAR_S2:
      sample_rate = 3;
      break;

    case YDLIDAR_G6:
      sample_rate = 18;
      break;

    case YDLIDAR_G2A:
      sample_rate = 5;
      break;

    case YDLIDAR_G2B:
      sample_rate = 5;
      break;

    case YDLIDAR_G2C:
      sample_rate = 4;
      break;

    case YDLIDAR_G4B:
      break;

    case YDLIDAR_G4C:
      break;

    case YDLIDAR_G1:
      sample_rate = 9;
      break;

    case YDLIDAR_TG15:
      sample_rate = 20;
      break;

    case YDLIDAR_TG30:
      sample_rate = 20;
      break;

    case YDLIDAR_TG50:
      sample_rate = 20;
      break;

    default:
      break;
  }

  return sample_rate ;
}

/*!
 * @brief isOctaveLidar
 * @param model
 * @return
 */
inline bool isOctaveLidar(int model) {
  bool ret = false;

  if (model == YDLIDAR_G6 ||
      model == YDLIDAR_TG15 ||
      model == YDLIDAR_TG30 ||
      model == YDLIDAR_TG50) {
    ret = true;
  }

  return ret;
}

/*!
 * @brief hasSampleRate
 * @param model
 * @return
 */
inline bool hasSampleRate(int model) {
  bool ret = false;

  if (model == YDLIDAR_G4 ||
      model == YDLIDAR_G4PRO ||
      model == YDLIDAR_F4PRO ||
      model == YDLIDAR_G6 ||
      model == YDLIDAR_TG15 ||
      model == YDLIDAR_TG50 ||
      model == YDLIDAR_TG30) {
    ret = true;
  }

  return ret;
}
/*!
 * @brief hasZeroAngle
 * @param model
 * @return
 */

inline bool hasZeroAngle(int model) {
  bool ret = false;

  if (model == YDLIDAR_R2 ||
      model == YDLIDAR_G2A ||
      model == YDLIDAR_G2B ||
      model == YDLIDAR_G2C ||
      model == YDLIDAR_G1) {
    ret = true;
  }

  return ret;
}

/*!
 * @brief hasScanFrequencyCtrl
 * @param model
 * @return
 */
inline bool hasScanFrequencyCtrl(int model) {
  bool ret = true;

  if (model == YDLIDAR_S4 ||
      model == YDLIDAR_S4B ||
      model == YDLIDAR_S2 ||
      model == YDLIDAR_X4) {
    ret = false;
  }

  return ret;
}

/*!
 * @brief isSupportLidar
 * @param model
 * @return
 */
inline bool isSupportLidar(int model) {
  bool ret = true;

  if (model < YDLIDAR_F4 || (model > YDLIDAR_G1 &&
                             model < YDLIDAR_TG15) ||
      model > YDLIDAR_TG50) {
    ret = false;

  }

  return ret;
}

/*!
 * @brief hasIntensity
 * @param model
 * @return
 */
inline bool hasIntensity(int model) {
  bool ret = false;

  if (model == YDLIDAR_G2B ||
      model == YDLIDAR_G4B ||
      model == YDLIDAR_S4B) {
    ret = true;
  }

  return ret;
}

/*!
 * @brief isSupportMotorCtrl
 * @param model
 * @return
 */
inline bool isSupportMotorCtrl(int model) {
  bool ret = false;

  if (model == YDLIDAR_X4 ||
      model == YDLIDAR_S2 ||
      model == YDLIDAR_S4 ||
      model == YDLIDAR_S4B) {
    ret = true;

  }

  return true;
}

/*!
 * @brief isSupportScanFrequency
 * @param model
 * @param frequency
 * @return
 */
inline bool isSupportScanFrequency(int model, double frequency) {
  bool ret = false;

  if (model >= YDLIDAR_TG15) {
    if (3 <= frequency && frequency <= 15.7) {
      ret = true;
    }
  } else {
    if (5 <= frequency && frequency <= 15.7) {
      ret = true;
    }
  }

  return ret;
}

inline bool isTOFLidar(int type) {
  bool ret = false;

  if (type == TYPE_TOF) {
    ret = true;
  }

  return ret;
}

inline bool isOldVersionTOFLidar(int model, int Major, int Minor) {
  bool ret = false;

  if (model == YDLIDAR_TG15 ||
      model == YDLIDAR_TG50 ||
      model == YDLIDAR_TG30)  {
    if (Major <= 1 && Minor <= 2) {
      ret = true;
    }

  }

  return ret;
}

inline bool isValidSampleRate(std::map<int, int>  smap) {
  if (smap.size() < 1) {
    return false;
  }

  if (smap.size() == 1) {
    if (smap.begin()->second > 1) {
      return true;
    }

    return false;
  }

  return false;
}

inline int ConvertUserToLidarSmaple(int model, int m_SampleRate,
                                    int defaultRate) {
  int _samp_rate = 9;

  switch (m_SampleRate) {
    case 10:
      _samp_rate = YDLIDAR_RATE_4K;
      break;

    case 16:
      _samp_rate = YDLIDAR_RATE_8K;
      break;

    case 18:
      _samp_rate = YDLIDAR_RATE_9K;
      break;

    case 20:
      _samp_rate = YDLIDAR_RATE_10K;
      break;

    default:
      _samp_rate = defaultRate;
      break;
  }

  if (!isOctaveLidar(model)) {
    _samp_rate = 2;

    switch (m_SampleRate) {
      case 4:
        _samp_rate = YDLIDAR_RATE_4K;
        break;

      case 8:
        _samp_rate = YDLIDAR_RATE_8K;
        break;

      case 9:
        _samp_rate = YDLIDAR_RATE_9K;
        break;

      default:
        break;
    }

    if (model == YDLIDAR_F4PRO) {
      _samp_rate = 0;

      switch (m_SampleRate) {
        case 4:
          _samp_rate = YDLIDAR_RATE_4K;
          break;

        case 6:
          _samp_rate = YDLIDAR_RATE_8K;
          break;

        default:
          break;
      }

    }
  }

  return _samp_rate;
}


inline int ConvertLidarToUserSmaple(int model, int rate) {
  int _samp_rate = 9;

  switch (rate) {
    case YDLIDAR_RATE_4K:
      _samp_rate = 10;

      if (!isOctaveLidar(model)) {
        _samp_rate = 4;
      }

      break;

    case YDLIDAR_RATE_8K:
      _samp_rate = 16;

      if (!isOctaveLidar(model)) {
        _samp_rate = 8;

        if (model == YDLIDAR_F4PRO) {
          _samp_rate = 6;
        }
      }

      break;

    case YDLIDAR_RATE_9K:
      _samp_rate = 18;

      if (!isOctaveLidar(model)) {
        _samp_rate = 9;
      }

      break;

    case YDLIDAR_RATE_10K:
      _samp_rate = 20;

      if (!isOctaveLidar(model)) {
        _samp_rate = 10;
      }

      break;

    default:
      _samp_rate = 9;

      if (!isOctaveLidar(model)) {
        _samp_rate = 18;
      }

      break;
  }

  return _samp_rate;
}


inline bool isValidValue(uint8_t value) {
  if (value & 0x80) {
    return false;
  }

  return true;
}

inline bool isVersionValid(const LaserDebug &info) {
  bool ret = false;

  if (isValidValue(info.W3F4CusMajor_W4F0CusMinor) &&
      isValidValue(info.W4F3Model_W3F0DebugInfTranVer) &&
      isValidValue(info.W3F4HardwareVer_W4F0FirewareMajor) &&
      isValidValue(info.W3F4BoradHardVer_W4F0Moth)) {
    ret = true;
  }

  return ret;
}

inline bool isSerialNumbValid(const LaserDebug &info) {
  bool ret = false;

  if (isValidValue(info.W2F5Output2K4K5K_W5F0Date) &&
      isValidValue(info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear) &&
      isValidValue(info.W7F0SnNumH) &&
      isValidValue(info.W7F0SnNumH)) {
    ret = true;
  }

  return ret;
}

inline bool ParseLaserDebugInfo(const LaserDebug &info, device_info &value) {
  bool ret = false;
  uint8_t CustomVerMajor = (static_cast<uint8_t>
                            (info.W3F4CusMajor_W4F0CusMinor) >> 4);
  uint8_t CustomVerMinor = static_cast<uint8_t>
                           (info.W3F4CusMajor_W4F0CusMinor) & 0x0F;
  uint8_t lidarmodel = (static_cast<uint8_t>(info.W4F3Model_W3F0DebugInfTranVer)
                        >> 3);
  uint8_t hardwareVer = static_cast<uint8_t>
                        (info.W3F4HardwareVer_W4F0FirewareMajor) >> 4;
  uint8_t Moth = static_cast<uint8_t>(info.W3F4BoradHardVer_W4F0Moth) & 0x0F;

  uint8_t Date = static_cast<uint8_t>(info.W2F5Output2K4K5K_W5F0Date) & 0x1F;
  uint8_t Year = static_cast<uint8_t>
                 (info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear) & 0x0F;
  uint16_t Number = ((static_cast<uint8_t>(info.W7F0SnNumH) << 7) |
                     static_cast<uint8_t>(info.W7F0SnNumL));

  if (isVersionValid(info) && info.MaxDebugIndex > 0 && Year) {

    if (isSerialNumbValid(info) && info.MaxDebugIndex > 8) {
      value.firmware_version = (CustomVerMajor << 8 | CustomVerMinor);
      value.hardware_version = hardwareVer;
      value.model = lidarmodel;
      uint32_t year = Year + 2015;
      sprintf(reinterpret_cast<char *>(value.serialnum), "%04d", year);
      sprintf(reinterpret_cast<char *>(value.serialnum + 4), "%02d", Moth);
      sprintf(reinterpret_cast<char *>(value.serialnum + 6), "%02d", Date);
      sprintf(reinterpret_cast<char *>(value.serialnum + 8), "%08d", Number);

      for (int i = 0; i < 16; i++) {
        value.serialnum[i] -= 48;
      }

      ret = true;
    }
  }

  return ret;
}

}

