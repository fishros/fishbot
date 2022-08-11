/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
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
#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <angles.h>
#include <numeric>

using namespace std;
using namespace ydlidar;
using namespace impl;
using namespace angles;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 230400;
  m_FixedResolution   = true;
  m_Reversion         = false;
  m_Inverted          = false;//
  m_AutoReconnect     = true;
  m_SingleChannel     = false;
  m_LidarType         = TYPE_TRIANGLE;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 64.0;
  m_MinRange          = 0.01;
  m_SampleRate        = 5;
  defalutSampleRate   = 5;
  m_ScanFrequency     = 10;
  isScanning          = false;
  m_FixedSize         = 720;
  frequencyOffset     = 0.4;
  m_AbnormalCheckCount  = 4;
  Major               = 0;
  Minjor              = 0;
  m_IgnoreArray.clear();
  m_PointTime         = 1e9 / 5000;
  m_OffsetTime        = 0.0;
  m_AngleOffset       = 0.0;
  lidar_model = YDLIDAR_G2B;
  last_node_time = getTime();
  global_nodes = new node_info[YDlidarDriver::MAX_SCAN_NODES];
  m_ParseSuccess = false;
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();

  if (global_nodes) {
    delete[] global_nodes;
    global_nodes = NULL;
  }
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

  isScanning = false;
}

//get zero angle offset value
float CYdLidar::getAngleOffset() const {
  return m_AngleOffset;
}

bool CYdLidar::isAngleOffetCorrected() const {
  return m_isAngleOffsetCorrected;
}

std::string CYdLidar::getSoftVersion() const {
  return m_lidarSoftVer;
}

std::string CYdLidar::getHardwareVersion() const {
  return m_lidarHardVer;
}

std::string CYdLidar::getSerialNumber() const {
  return m_lidarSerialNum;
}

bool CYdLidar::isRangeValid(double reading) const {
  if (reading >= m_MinRange && reading <= m_MaxRange) {
    return true;
  }

  return false;
}

bool CYdLidar::isRangeIgnore(double angle) const {
  bool ret = false;

  for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
    if ((angles::from_degrees(m_IgnoreArray[j]) <= angle) &&
        (angle <= angles::from_degrees(m_IgnoreArray[j + 1]))) {
      ret = true;
      break;
    }
  }

  return ret;
}


/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan,
                                bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(200 / m_ScanFrequency);
    return false;
  }

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;
  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  uint64_t startTs = tim_scan_start;
  result_t op_result =  lidarPtr->grabScanData(global_nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    uint64_t scan_time = m_PointTime * (count - 1);
    tim_scan_end += m_OffsetTime * 1e9;
    tim_scan_end -= m_PointTime;
    tim_scan_end -= global_nodes[0].stamp;
    tim_scan_start = tim_scan_end -  scan_time ;

    if (tim_scan_start < startTs) {
      tim_scan_start = startTs;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if ((last_node_time + m_PointTime) >= tim_scan_start) {
      tim_scan_start = last_node_time + m_PointTime;
      tim_scan_end = tim_scan_start + scan_time;
    }

    last_node_time = tim_scan_end;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    int all_node_count = count;

    outscan.config.min_angle = angles::from_degrees(m_MinAngle);
    outscan.config.max_angle =  angles::from_degrees(m_MaxAngle);
    outscan.config.scan_time =  static_cast<float>(scan_time * 1.0 / 1e9);
    outscan.config.time_increment = outscan.config.scan_time / (double)(count - 1);
    outscan.config.min_range = m_MinRange;
    outscan.config.max_range = m_MaxRange;
    outscan.stamp = tim_scan_start;
    outscan.points.clear();

    if (m_FixedResolution) {
      all_node_count = m_FixedSize;
    }

    outscan.config.angle_increment = (outscan.config.max_angle -
                                      outscan.config.min_angle) / (all_node_count - 1);

    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;

    for (int i = 0; i < count; i++) {
      angle = static_cast<float>((global_nodes[i].angle_q6_checkbit >>
                                  LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) + m_AngleOffset;

      if (isTOFLidar(m_LidarType)) {
        if (isOldVersionTOFLidar(lidar_model, Major, Minjor)) {
          range = static_cast<float>(global_nodes[i].distance_q2 / 2000.f);
        } else {
          range = static_cast<float>(global_nodes[i].distance_q2 / 1000.f);
        }
      } else {
        if (isOctaveLidar(lidar_model)) {
          range = static_cast<float>(global_nodes[i].distance_q2 / 2000.f);
        } else {
          range = static_cast<float>(global_nodes[i].distance_q2 / 4000.f);
        }
      }

      intensity = static_cast<float>(global_nodes[i].sync_quality);
      angle = angles::from_degrees(angle);

      //Rotate 180 degrees or not
      if (m_Reversion) {
        angle = angle + M_PI;
      }

      //Is it counter clockwise
      if (m_Inverted) {
        angle = 2 * M_PI - angle;
      }

      angle = angles::normalize_angle(angle);

      //ignore angle
      if (isRangeIgnore(angle)) {
        range = 0.0;
      }

      //valid range
      if (!isRangeValid(range)) {
        range = 0.0;
        intensity = 0.0;
      }

      if (angle >= outscan.config.min_angle &&
          angle <= outscan.config.max_angle) {
        LaserPoint point;
        point.angle = angle;
        point.range = range;
        point.intensity = intensity;

        if (outscan.points.empty()) {
          outscan.stamp = tim_scan_start + i * m_PointTime;
        }

        outscan.points.push_back(point);
      }

    }
    handleDeviceInfoPackage(count);

    return true;
  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
    }
  }

  return false;

}

void CYdLidar::parsePackageNode(const node_info &node, LaserDebug &info) {
  switch (node.index) {
    case 0://W3F4CusMajor_W4F0CusMinor;
      info.W3F4CusMajor_W4F0CusMinor = node.debug_info[node.index];
      break;

    case 1://W4F3Model_W3F0DebugInfTranVer
      info.W4F3Model_W3F0DebugInfTranVer = node.debug_info[node.index];
      break;

    case 2://W3F4HardwareVer_W4F0FirewareMajor
      info.W3F4HardwareVer_W4F0FirewareMajor = node.debug_info[node.index];
      break;

    case 4://W3F4BoradHardVer_W4F0Moth
      info.W3F4BoradHardVer_W4F0Moth = node.debug_info[node.index];
      break;

    case 5://W2F5Output2K4K5K_W5F0Date
      info.W2F5Output2K4K5K_W5F0Date = node.debug_info[node.index];
      break;

    case 6://W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear
      info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear =
        node.debug_info[node.index];
      break;

    case 7://W7F0SnNumH
      info.W7F0SnNumH = node.debug_info[node.index];
      break;

    case 8://W7F0SnNumL
      info.W7F0SnNumL = node.debug_info[node.index];

      break;

    default:
      break;
  }

  if (node.index > info.MaxDebugIndex && node.index < 100) {
    info.MaxDebugIndex = static_cast<int>(node.index);
  }
}

void CYdLidar::handleDeviceInfoPackage(int count) {
  if (m_ParseSuccess) {
    return;
  }

  LaserDebug debug;
  debug.MaxDebugIndex = 0;

  for (int i = 0; i < count; i++) {
    parsePackageNode(global_nodes[i], debug);
  }

  device_info info;

  if (ParseLaserDebugInfo(debug, info)) {
    if (info.firmware_version != 0 ||
        info.hardware_version != 0) {
      std::string serial_number;

      for (int i = 0; i < 16; i++) {
        serial_number += std::to_string(info.serialnum[i] & 0xff);
      }

      Major = (uint8_t)(info.firmware_version >> 8);
      Minjor = (uint8_t)(info.firmware_version & 0xff);
      std::string softVer =  std::to_string(Major & 0xff) + "." + std::to_string(
                               Minjor & 0xff);
      std::string hardVer = std::to_string(info.hardware_version & 0xff);

      m_lidarSerialNum = serial_number;
      m_lidarSoftVer = softVer;
      m_lidarHardVer = hardVer;

      if (!m_ParseSuccess) {
        printfVersionInfo(info);
      }
    }

  }
}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      lidarPtr->stop();
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  m_ParseSuccess &= !m_SingleChannel;
  m_PointTime = lidarPtr->getPointTime();

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  if (m_SingleChannel && !m_ParseSuccess) {
    handleSingleChannelDevice();
  }

  m_PointTime = lidarPtr->getPointTime();
  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", m_SampleRate);
  printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }

  isScanning = false;
  return true;
}

/*-------------------------------------------------------------
            checkLidarAbnormal
-------------------------------------------------------------*/
bool CYdLidar::checkLidarAbnormal() {

  size_t   count = YDlidarDriver::MAX_SCAN_NODES;
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;
  std::vector<int> data;
  int buffer_count  = 0;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count * 1000);
    }

    float scan_time = 0.0;
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    op_result = RESULT_OK;

    while (buffer_count < 10 && (scan_time < 0.05 ||
                                 !lidarPtr->getSingleChannel()) && IS_OK(op_result)) {
      start_time = getms();
      count = YDlidarDriver::MAX_SCAN_NODES;
      op_result =  lidarPtr->grabScanData(global_nodes, count);
      end_time = getms();
      scan_time = 1.0 * static_cast<int32_t>(end_time - start_time) / 1e3;
      buffer_count++;

      if (IS_OK(op_result)) {
        handleDeviceInfoPackage(count);

        if (CalculateSampleRate(count, scan_time)) {
          if (!lidarPtr->getSingleChannel()) {
            return !IS_OK(op_result);
          }
        }
      }
    }

    if (IS_OK(op_result) && lidarPtr->getSingleChannel()) {
      data.push_back(count);
      int collection = 0;

      while (collection < 5) {
        count = YDlidarDriver::MAX_SCAN_NODES;
        start_time = getms();
        op_result =  lidarPtr->grabScanData(global_nodes, count);
        end_time = getms();


        if (IS_OK(op_result)) {
          if (std::abs(static_cast<int>(data.front() - count)) > 10) {
            data.erase(data.begin());
          }

          handleDeviceInfoPackage(count);
          scan_time = 1.0 * static_cast<int32_t>(end_time - start_time) / 1e3;
          data.push_back(count);

          if (CalculateSampleRate(count, scan_time)) {

          }

          if (scan_time > 0.05 && scan_time < 0.5 && lidarPtr->getSingleChannel()) {
            m_SampleRate = static_cast<int>((count / scan_time + 500) / 1000);
            m_PointTime = 1e9 / (m_SampleRate * 1000);
            lidarPtr->setPointTime(m_PointTime);
          }

        }

        collection++;
      }

      if (data.size() > 1) {
        int total = accumulate(data.begin(), data.end(), 0);
        int mean =  total / data.size(); //mean value
        m_FixedSize = (static_cast<int>((mean + 5) / 10)) * 10;
        printf("[YDLIDAR]:Fixed Size: %d\n", m_FixedSize);
        printf("[YDLIDAR]:Sample Rate: %dK\n", m_SampleRate);
        return false;
      }

    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}


/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() {
  if (!lidarPtr) {
    return false;
  }

  lidarPtr->stop();
  result_t op_result;
  device_health healthinfo;
  printf("[YDLIDAR]:SDK Version: %s\n", YDlidarDriver::getSDKVersion().c_str());
  op_result = lidarPtr->getHealth(healthinfo);

  if (IS_OK(op_result)) {
    printf("[YDLIDAR]:Lidar running correctly ! The health status: %s\n",
           (int)healthinfo.status == 0 ? "good" : "bad");

    if (healthinfo.status == 2) {
      fprintf(stderr,
              "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
      return false;
    } else {
      return true;
    }

  } else {
    fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
    return false;
  }

}

bool CYdLidar::getDeviceInfo() {
  if (!lidarPtr) {
    return false;
  }

  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error\n");
    return false;
  }

  if (!isSupportLidar(devinfo.model)) {
    printf("[YDLIDAR INFO] Current SDK does not support current lidar models[%s]\n",
           lidarModelToString(devinfo.model).c_str());
    return false;
  }

  frequencyOffset     = 0.4;
  std::string model = "G2";
  lidar_model = devinfo.model;
  model = lidarModelToString(devinfo.model);
  bool intensity = hasIntensity(devinfo.model);
  defalutSampleRate = lidarModelDefaultSampleRate(devinfo.model);

  std::string serial_number;
  lidarPtr->setIntensities(intensity);
  printfVersionInfo(devinfo);

  for (int i = 0; i < 16; i++) {
    serial_number += std::to_string(devinfo.serialnum[i] & 0xff);
  }

  if (devinfo.firmware_version != 0 ||
      devinfo.hardware_version != 0) {
    m_lidarSerialNum = serial_number;
    m_lidarSoftVer = std::to_string(Major & 0xff) + "." + std::to_string(
                       Minjor & 0xff);
    m_lidarHardVer = std::to_string(devinfo.hardware_version & 0xff);
  }

  if (hasSampleRate(devinfo.model)) {
    checkSampleRate();
  } else {
    m_SampleRate = defalutSampleRate;
  }

  if (hasScanFrequencyCtrl(devinfo.model)) {
    checkScanFrequency();
  }

  if (hasZeroAngle(devinfo.model)) {
    checkCalibrationAngle(serial_number);
  }

  return true;
}

void CYdLidar::handleSingleChannelDevice() {
  if (!lidarPtr || !lidarPtr->getSingleChannel()) {
    return;
  }

  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(op_result)) {
    return;
  }

  printfVersionInfo(devinfo);
  return;
}

void CYdLidar::printfVersionInfo(const device_info &info) {
  if (info.firmware_version == 0 &&
      info.hardware_version == 0) {
    return;
  }

  m_ParseSuccess = true;
  lidar_model = info.model;
  Major = (uint8_t)(info.firmware_version >> 8);
  Minjor = (uint8_t)(info.firmware_version & 0xff);
  printf("[YDLIDAR] Connection established in [%s][%d]:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         m_SerialPort.c_str(),
         m_SerialBaudrate,
         Major,
         Minjor,
         (unsigned int)info.hardware_version,
         lidarModelToString(lidar_model).c_str());

  for (int i = 0; i < 16; i++) {
    printf("%01X", info.serialnum[i] & 0xff);
  }

  printf("\n");
}

void CYdLidar::checkSampleRate() {
  sampling_rate _rate;
  _rate.rate = 3;
  int _samp_rate = 9;
  int try_count = 0;
  m_FixedSize = 1440;
  result_t ans = lidarPtr->getSamplingRate(_rate);

  if (IS_OK(ans)) {
    _samp_rate = ConvertUserToLidarSmaple(lidar_model, m_SampleRate, _rate.rate);

    while (_samp_rate != _rate.rate) {
      ans = lidarPtr->setSamplingRate(_rate);
      try_count++;

      if (try_count > 6) {
        break;
      }
    }

    _samp_rate = ConvertLidarToUserSmaple(lidar_model, _rate.rate);
  }

  m_SampleRate = _samp_rate;
  defalutSampleRate = m_SampleRate;
}


bool CYdLidar::CalculateSampleRate(int count, double scan_time) {
  if (count < 1) {
    return false;
  }

  if (global_nodes[0].scan_frequence != 0) {
    double scanfrequency  = global_nodes[0].scan_frequence / 10.0;

    if (isTOFLidar(m_LidarType)) {
      if (!isOldVersionTOFLidar(lidar_model, Major, Minjor)) {
        scanfrequency  = global_nodes[0].scan_frequence / 10.0 + 3.0;
      }
    }

    int samplerate = static_cast<int>((count * scanfrequency + 500) / 1000);
    int cnt = 0;

    if (SampleRateMap.find(samplerate) != SampleRateMap.end()) {
      cnt = SampleRateMap[samplerate];
    }

    cnt++;
    SampleRateMap[samplerate] =  cnt;

    if (isValidSampleRate(SampleRateMap) || defalutSampleRate == samplerate) {
      m_SampleRate = samplerate;
      m_PointTime = 1e9 / (m_SampleRate * 1000);
      lidarPtr->setPointTime(m_PointTime);

      if (!m_SingleChannel) {
        m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
        printf("[YDLIDAR]:Fixed Size: %d\n", m_FixedSize);
        printf("[YDLIDAR]:Sample Rate: %dK\n", m_SampleRate);
      }

      return true;
    } else {
      if (SampleRateMap.size() > 1) {
        SampleRateMap.clear();
      }
    }
  } else {
    if (scan_time > 0.04 && scan_time < 0.4) {
      int samplerate = static_cast<int>((count / scan_time + 500) / 1000);

      if (defalutSampleRate == samplerate) {
        m_SampleRate = samplerate;
        m_PointTime = 1e9 / (m_SampleRate * 1000);
        lidarPtr->setPointTime(m_PointTime);
        return true;
      }
    }

  }


  return false;
}
/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency() {
  float frequency = 7.4f;
  scan_frequency _scan_frequency;
  float hz = 0;
  result_t ans = RESULT_FAIL;

  if (isSupportScanFrequency(lidar_model, m_ScanFrequency)) {
    m_ScanFrequency += frequencyOffset;
    ans = lidarPtr->getScanFrequency(_scan_frequency) ;

    if (IS_OK(ans)) {
      frequency = _scan_frequency.frequency / 100.f;
      hz = m_ScanFrequency - frequency;

      if (hz > 0) {
        while (hz > 0.95) {
          lidarPtr->setScanFrequencyAdd(_scan_frequency);
          hz = hz - 1.0;
        }

        while (hz > 0.09) {
          lidarPtr->setScanFrequencyAddMic(_scan_frequency);
          hz = hz - 0.1;
        }

        frequency = _scan_frequency.frequency / 100.0f;
      } else {
        while (hz < -0.95) {
          lidarPtr->setScanFrequencyDis(_scan_frequency);
          hz = hz + 1.0;
        }

        while (hz < -0.09) {
          lidarPtr->setScanFrequencyDisMic(_scan_frequency);
          hz = hz + 0.1;
        }

        frequency = _scan_frequency.frequency / 100.0f;
      }
    }
  } else {
    m_ScanFrequency += frequencyOffset;
    fprintf(stderr, "current scan frequency[%f] is out of range.",
            m_ScanFrequency - frequencyOffset);
  }

  ans = lidarPtr->getScanFrequency(_scan_frequency);

  if (IS_OK(ans)) {
    frequency = _scan_frequency.frequency / 100.0f;
    m_ScanFrequency = frequency;
  }

  m_ScanFrequency -= frequencyOffset;
  m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
  printf("[YDLIDAR INFO] Current Scan Frequency: %fHz\n", m_ScanFrequency);
  return true;
}

/*-------------------------------------------------------------
                        checkCalibrationAngle
-------------------------------------------------------------*/
void CYdLidar::checkCalibrationAngle(const std::string &serialNumber) {
  m_AngleOffset = 0.0;
  result_t ans = RESULT_FAIL;
  offset_angle angle;
  int retry = 0;
  m_isAngleOffsetCorrected = false;

  while (retry < 2) {
    ans = lidarPtr->getZeroOffsetAngle(angle);

    if (IS_OK(ans)) {
      if (angle.angle > 720 || angle.angle < -720) {
        ans = lidarPtr->getZeroOffsetAngle(angle);

        if (!IS_OK(ans)) {
          continue;
          retry++;
        }
      }

      m_isAngleOffsetCorrected = (angle.angle != 720);
      m_AngleOffset = angle.angle / 4.0;
      printf("[YDLIDAR INFO] Successfully obtained the %s offset angle[%f] from the lidar[%s]\n"
             , m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
             serialNumber.c_str());
      return;
    }

    retry++;
  }

  printf("[YDLIDAR INFO] Current %s AngleOffset : %f°\n",
         m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset);
}



/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      fprintf(stderr, "Create Driver fail\n");
      return false;
    }
  }

  if (lidarPtr->isconnected()) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    fprintf(stderr,
            "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  lidarPtr->setSingleChannel(m_SingleChannel);
  lidarPtr->setLidarType(m_LidarType);

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus() {

  if (!checkCOMMs()) {
    return false;
  }

  bool ret = getDeviceHealth();

  if (!ret) {
    delay(2000);
    ret = getDeviceHealth();

    if (!ret) {
      delay(1000);
    }
  }

  if (!getDeviceInfo()) {
    delay(2000);
    ret = getDeviceInfo();

    if (!ret) {
      return false;
    }
  }

  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  if (!checkCOMMs()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check Comms.\n");
    fflush(stderr);
    return false;
  }

  if (!checkStatus()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check status.\n");
    fflush(stderr);
    return false;
  }

  return true;
}
