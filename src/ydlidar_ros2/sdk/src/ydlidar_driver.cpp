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
#include "ydlidar_driver.h"
#include "common.h"
#include <math.h>
using namespace impl;

namespace ydlidar {

YDlidarDriver::YDlidarDriver():
  _serial(NULL) {
  isConnected         = false;
  isScanning          = false;
  //串口配置参数
  m_intensities       = false;
  isAutoReconnect     = true;
  isAutoconnting      = false;
  m_baudrate          = 230400;
  isSupportMotorDtrCtrl  = true;
  scan_node_count     = 0;
  sample_rate         = 5000;
  m_PointTime         = 1e9 / 5000;
  trans_delay         = 0;
  scan_frequence      = 0;
  m_sampling_rate     = -1;
  model               = -1;
  retryCount          = 0;
  has_device_header   = false;
  m_SingleChannel     = false;
  m_LidarType         = TYPE_TOF;

  //解析参数
  PackageSampleBytes  = 2;
  IntervalSampleAngle = 0.0;
  FirstSampleAngle    = 0;
  LastSampleAngle     = 0;
  CheckSum            = 0;
  CheckSumCal         = 0;
  SampleNumlAndCTCal  = 0;
  LastSampleAngleCal  = 0;
  CheckSumResult      = true;
  Valu8Tou16          = 0;

  last_device_byte    = 0x00;
  asyncRecvPos        = 0;
  async_size          = 0;
  headerBuffer = reinterpret_cast<uint8_t *>(&header_);
  infoBuffer = reinterpret_cast<uint8_t *>(&info_);
  healthBuffer = reinterpret_cast<uint8_t *>(&health_);
  get_device_health_success = false;
  get_device_info_success = false;

  package_Sample_Index = 0;
  IntervalSampleAngle_LastPackage = 0.0;
  globalRecvBuffer = new uint8_t[sizeof(node_packages)];
  scan_node_buf = new node_info[MAX_SCAN_NODES];
  package_index = 0;
  has_package_error = false;
}

YDlidarDriver::~YDlidarDriver() {
  {
    isScanning = false;
  }

  isAutoReconnect = false;
  _thread.join();

  ScopedLocker lk(_serial_lock);

  if (_serial) {
    if (_serial->isOpen()) {
      _serial->flush();
      _serial->closePort();
    }
  }

  if (_serial) {
    delete _serial;
    _serial = NULL;
  }

  if (globalRecvBuffer) {
    delete[] globalRecvBuffer;
    globalRecvBuffer = NULL;
  }

  if (scan_node_buf) {
    delete[] scan_node_buf;
    scan_node_buf = NULL;
  }
}

result_t YDlidarDriver::connect(const char *port_path, uint32_t baudrate) {
  ScopedLocker lk(_serial_lock);
  m_baudrate = baudrate;
  serial_port = string(port_path);

  if (!_serial) {
    _serial = new serial::Serial(port_path, m_baudrate,
                                 serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
  }

  {
    ScopedLocker l(_lock);

    if (!_serial->open()) {
      return RESULT_FAIL;
    }

    isConnected = true;

  }

  stopScan();
  delay(100);
  clearDTR();

  return RESULT_OK;
}


void YDlidarDriver::setDTR() {
  if (!isConnected) {
    return ;
  }

  if (_serial) {
    _serial->setDTR(1);
  }

}

void YDlidarDriver::clearDTR() {
  if (!isConnected) {
    return ;
  }

  if (_serial) {
    _serial->setDTR(0);
  }
}
void YDlidarDriver::flushSerial() {
  if (!isConnected) {
    return;
  }

  size_t len = _serial->available();

  if (len) {
    _serial->read(len);
  }

  delay(20);
}


void YDlidarDriver::disconnect() {
  isAutoReconnect = false;

  if (!isConnected) {
    return ;
  }

  stop();
  delay(10);
  ScopedLocker l(_serial_lock);

  if (_serial) {
    if (_serial->isOpen()) {
      _serial->closePort();
    }
  }

  isConnected = false;

}


void YDlidarDriver::disableDataGrabbing() {
  {
    if (isScanning) {
      isScanning = false;
      _dataEvent.set();
    }
  }
  _thread.join();
}

bool YDlidarDriver::isscanning() const {
  return isScanning;
}
bool YDlidarDriver::isconnected() const {
  return isConnected;
}

result_t YDlidarDriver::sendCommand(uint8_t cmd, const void *payload,
                                    size_t payloadsize) {
  uint8_t pkt_header[10];
  cmd_packet *header = reinterpret_cast<cmd_packet * >(pkt_header);
  uint8_t checksum = 0;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (payloadsize && payload) {
    cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
  }

  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd;
  sendData(pkt_header, 2) ;

  if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD) && payloadsize && payload) {
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= cmd;
    checksum ^= (payloadsize & 0xFF);

    for (size_t pos = 0; pos < payloadsize; ++pos) {
      checksum ^= ((uint8_t *)payload)[pos];
    }

    uint8_t sizebyte = (uint8_t)(payloadsize);
    sendData(&sizebyte, 1);

    sendData((const uint8_t *)payload, sizebyte);

    sendData(&checksum, 1);
  }

  return RESULT_OK;
}

result_t YDlidarDriver::sendData(const uint8_t *data, size_t size) {
  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (data == NULL || size == 0) {
    return RESULT_FAIL;
  }

  size_t r;

  while (size) {
    r = _serial->writeData(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::getData(uint8_t *data, size_t size) {
  if (!isConnected) {
    return RESULT_FAIL;
  }

  size_t r;

  while (size) {
    r = _serial->readData(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::waitResponseHeader(lidar_ans_header *header,
    uint32_t timeout) {
  int  recvPos     = 0;
  uint32_t startTs = getms();
  uint8_t  recvBuffer[sizeof(lidar_ans_header)];
  uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
  uint32_t waitTime = 0;
  has_device_header = false;
  last_device_byte = 0x00;

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize = sizeof(lidar_ans_header) - recvPos;
    size_t recvSize = 0;
    result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (!IS_OK(ans)) {
      return ans;
    }

    if (recvSize > remainSize) {
      recvSize = remainSize;
    }

    ans = getData(recvBuffer, recvSize);

    if (IS_FAIL(ans)) {
      return RESULT_FAIL;
    }

    for (size_t pos = 0; pos < recvSize; ++pos) {
      uint8_t currentByte = recvBuffer[pos];

      switch (recvPos) {
        case 0:
          if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
            if (last_device_byte == (PH & 0xFF) && currentByte == (PH >> 8)) {
              has_device_header = true;
            }

            last_device_byte = currentByte;
            continue;
          }

          break;

        case 1:
          if (currentByte != LIDAR_ANS_SYNC_BYTE2) {
            last_device_byte = currentByte;
            recvPos = 0;
            continue;
          }

          break;
      }

      headerBuffer[recvPos++] = currentByte;
      last_device_byte = currentByte;

      if (recvPos == sizeof(lidar_ans_header)) {
        return RESULT_OK;
      }
    }
  }

  return RESULT_FAIL;
}

result_t YDlidarDriver::waitForData(size_t data_count, uint32_t timeout,
                                    size_t *returned_size) {
  size_t length = 0;

  if (returned_size == NULL) {
    returned_size = (size_t *)&length;
  }

  return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
}

result_t YDlidarDriver::checkAutoConnecting() {
  result_t ans = RESULT_FAIL;
  isAutoconnting = true;

  while (isAutoReconnect && isAutoconnting) {
    {
      ScopedLocker l(_serial_lock);

      if (_serial) {
        if (_serial->isOpen() || isConnected) {
          isConnected = false;
          _serial->closePort();
          delete _serial;
          _serial = NULL;
        }
      }
    }
    retryCount++;

    if (retryCount > 100) {
      retryCount = 100;
    }

    delay(100 * retryCount);
    int retryConnect = 0;

    while (isAutoReconnect &&
           connect(serial_port.c_str(), m_baudrate) != RESULT_OK) {
      retryConnect++;

      if (retryConnect > 25) {
        retryConnect = 25;
      }

      delay(200 * retryConnect);
    }

    if (!isAutoReconnect) {
      isScanning = false;
      return RESULT_FAIL;
    }

    if (isconnected()) {
      delay(100);
      {
        ScopedLocker l(_serial_lock);
        ans = startAutoScan();

        if (!IS_OK(ans)) {
          ans = startAutoScan();
        }
      }

      if (IS_OK(ans)) {
        isAutoconnting = false;
        return ans;
      }
    }
  }

  return RESULT_FAIL;

}

int YDlidarDriver::cacheScanData() {
  node_info      local_buf[128];
  size_t         count = 128;
  node_info      local_scan[MAX_SCAN_NODES];
  size_t         scan_count = 0;
  result_t       ans = RESULT_FAIL;
  memset(local_scan, 0, sizeof(local_scan));

  if (m_SingleChannel) {
    waitDevicePackage();
  }

  flushSerial();
  waitScanData(local_buf, count);

  int timeout_count   = 0;
  retryCount = 0;

  while (isScanning) {
    count = 128;
    ans = waitScanData(local_buf, count);

    if (!IS_OK(ans)) {
      if (IS_FAIL(ans) || timeout_count > DEFAULT_TIMEOUT_COUNT) {
        if (!isAutoReconnect) {
          fprintf(stderr, "exit scanning thread!!\n");
          fflush(stderr);
          {
            isScanning = false;
          }
          return RESULT_FAIL;
        } else {
          ans = checkAutoConnecting();

          if (IS_OK(ans)) {
            timeout_count = 0;
            local_scan[0].sync_flag = Node_NotSync;
          } else {
            isScanning = false;
            return RESULT_FAIL;
          }
        }
      } else {
        timeout_count++;
        local_scan[0].sync_flag = Node_NotSync;
        fprintf(stderr, "timout count: %d\n", timeout_count);
        fflush(stderr);
      }
    } else {
      timeout_count = 0;
      retryCount = 0;
    }


    for (size_t pos = 0; pos < count; ++pos) {
      if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
        if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
          _lock.lock();//timeout lock, wait resource copy
          local_scan[0].stamp = local_buf[pos].stamp;
          local_scan[0].scan_frequence = local_buf[pos].scan_frequence;
          memcpy(scan_node_buf, local_scan, scan_count * sizeof(node_info));
          scan_node_count = scan_count;
          _dataEvent.set();
          _lock.unlock();
        }

        scan_count = 0;
      }

      local_scan[scan_count++] = local_buf[pos];

      if (scan_count == _countof(local_scan)) {
        scan_count -= 1;
      }
    }
  }

  isScanning = false;

  return RESULT_OK;
}

result_t YDlidarDriver::checkDeviceInfo(uint8_t *recvBuffer, uint8_t byte,
                                        int recvPos, int recvSize, int pos) {
  if (asyncRecvPos == sizeof(lidar_ans_header)) {
    if ((((pos < recvSize - 1) && byte == LIDAR_ANS_SYNC_BYTE1) ||
         (last_device_byte == LIDAR_ANS_SYNC_BYTE1 && byte == LIDAR_ANS_SYNC_BYTE2)) &&
        recvPos == 0) {
      if ((last_device_byte == LIDAR_ANS_SYNC_BYTE1 &&
           byte == LIDAR_ANS_SYNC_BYTE2)) {
        asyncRecvPos = 0;
        async_size = 0;
        headerBuffer[asyncRecvPos] = last_device_byte;
        asyncRecvPos++;
        headerBuffer[asyncRecvPos] = byte;
        asyncRecvPos++;
        last_device_byte = byte;
        return RESULT_OK;
      } else {
        if (pos < recvSize - 1) {
          if (recvBuffer[pos + 1] == LIDAR_ANS_SYNC_BYTE2) {
            asyncRecvPos = 0;
            async_size = 0;
            headerBuffer[asyncRecvPos] = byte;
            asyncRecvPos++;
            last_device_byte = byte;
            return RESULT_OK;
          }
        }

      }

    }

    last_device_byte = byte;

    if (header_.type == LIDAR_ANS_TYPE_DEVINFO ||
        header_.type == LIDAR_ANS_TYPE_DEVHEALTH) {
      if (header_.size < 1) {
        asyncRecvPos = 0;
        async_size = 0;
      } else {

        if (header_.type == LIDAR_ANS_TYPE_DEVHEALTH) {
          if (async_size < sizeof(health_)) {
            healthBuffer[async_size] = byte;
            async_size++;

            if (async_size == sizeof(health_)) {
              asyncRecvPos = 0;
              async_size = 0;
              get_device_health_success = true;
              last_device_byte = byte;
              return RESULT_OK;
            }

          } else {
            asyncRecvPos = 0;
            async_size = 0;
          }

        } else {

          if (async_size < sizeof(info_)) {
            infoBuffer[async_size] = byte;
            async_size++;

            if (async_size == sizeof(info_)) {
              asyncRecvPos = 0;
              async_size = 0;
              get_device_info_success = true;

              last_device_byte = byte;
              return RESULT_OK;
            }

          } else {
            asyncRecvPos = 0;
            async_size = 0;
          }
        }
      }
    } else if (header_.type == LIDAR_ANS_TYPE_MEASUREMENT) {
      asyncRecvPos = 0;
      async_size = 0;

    }

  } else {

    switch (asyncRecvPos) {
      case 0:
        if (byte == LIDAR_ANS_SYNC_BYTE1 && recvPos == 0) {
          headerBuffer[asyncRecvPos] = byte;
          last_device_byte = byte;
          asyncRecvPos++;
        }

        break;

      case 1:
        if (byte == LIDAR_ANS_SYNC_BYTE2 && recvPos == 0) {
          headerBuffer[asyncRecvPos] = byte;
          asyncRecvPos++;
          last_device_byte = byte;
          return RESULT_OK;
        } else {
          asyncRecvPos = 0;
        }

        break;

      default:
        break;
    }

    if (asyncRecvPos >= 2) {
      if (((pos < recvSize - 1 && byte == LIDAR_ANS_SYNC_BYTE1) ||
           (last_device_byte == LIDAR_ANS_SYNC_BYTE1 && byte == LIDAR_ANS_SYNC_BYTE2)) &&
          recvPos == 0) {
        if ((last_device_byte == LIDAR_ANS_SYNC_BYTE1 &&
             byte == LIDAR_ANS_SYNC_BYTE2)) {
          asyncRecvPos = 0;
          async_size = 0;
          headerBuffer[asyncRecvPos] = last_device_byte;
          asyncRecvPos++;
        } else {
          if (pos < recvSize - 2) {
            if (recvBuffer[pos + 1] == LIDAR_ANS_SYNC_BYTE2) {
              asyncRecvPos = 0;
            }
          }
        }
      }

      headerBuffer[asyncRecvPos] = byte;
      asyncRecvPos++;
      last_device_byte = byte;
      return RESULT_OK;
    }
  }

  return RESULT_FAIL;

}

result_t YDlidarDriver::waitDevicePackage(uint32_t timeout) {
  int recvPos = 0;
  asyncRecvPos = 0;
  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  result_t ans = RESULT_FAIL;

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize = PackagePaidBytes - recvPos;
    size_t recvSize = 0;
    result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (!IS_OK(ans)) {
      return ans;
    }

    ans = RESULT_FAIL;

    if (recvSize > remainSize) {
      recvSize = remainSize;
    }

    getData(globalRecvBuffer, recvSize);

    for (size_t pos = 0; pos < recvSize; ++pos) {
      uint8_t currentByte = globalRecvBuffer[pos];

      if (checkDeviceInfo(globalRecvBuffer, currentByte, recvPos, recvSize,
                          pos) == RESULT_OK) {
        continue;
      }
    }

    if (get_device_info_success) {
      ans = RESULT_OK;
      break;
    }
  }

  flushSerial();
  return ans;

}

result_t YDlidarDriver::waitPackage(node_info *node, uint32_t timeout) {
  int recvPos         = 0;
  uint32_t startTs    = getms();
  uint32_t waitTime   = 0;
  uint8_t  *packageBuffer = (m_intensities) ? (uint8_t *)&package.package_Head :
                            (uint8_t *)&packages.package_Head;
  uint8_t  package_Sample_Num         = 0;
  int32_t  AngleCorrectForDistance    = 0;
  int  package_recvPos    = 0;
  uint8_t package_type    = 0;

  if (package_Sample_Index == 0) {
    recvPos = 0;

    while ((waitTime = getms() - startTs) <= timeout) {
      size_t remainSize   = PackagePaidBytes - recvPos;
      size_t recvSize     = 0;
      result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

      if (!IS_OK(ans)) {
        return ans;
      }

      if (recvSize > remainSize) {
        recvSize = remainSize;
      }

      getData(globalRecvBuffer, recvSize);

      for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = globalRecvBuffer[pos];

        switch (recvPos) {
          case 0:
            if (currentByte == (PH & 0xFF)) {

            } else {
              continue;
            }

            break;

          case 1:
            CheckSumCal = PH;

            if (currentByte == (PH >> 8)) {

            } else {
              recvPos = 0;
              continue;
            }

            break;

          case 2:
            SampleNumlAndCTCal = currentByte;
            package_type = currentByte & 0x01;

            if ((package_type == CT_Normal) || (package_type == CT_RingStart)) {
              if (package_type == CT_RingStart) {
                scan_frequence = (currentByte & 0xFE) >> 1;
              }
            } else {
              has_package_error = true;
              recvPos = 0;
              continue;
            }

            break;

          case 3:
            SampleNumlAndCTCal += (currentByte * 0x100);
            package_Sample_Num = currentByte;
            break;

          case 4:
            if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
              FirstSampleAngle = currentByte;
            } else {
              has_package_error = true;
              recvPos = 0;
              continue;
            }

            break;

          case 5:
            FirstSampleAngle += currentByte * 0x100;
            CheckSumCal ^= FirstSampleAngle;
            FirstSampleAngle = FirstSampleAngle >> 1;
            break;

          case 6:
            if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
              LastSampleAngle = currentByte;
            } else {
              has_package_error = true;
              recvPos = 0;
              continue;
            }

            break;

          case 7:
            LastSampleAngle = currentByte * 0x100 + LastSampleAngle;
            LastSampleAngleCal = LastSampleAngle;
            LastSampleAngle = LastSampleAngle >> 1;

            if (package_Sample_Num == 1) {
              IntervalSampleAngle = 0;
            } else {
              if (LastSampleAngle < FirstSampleAngle) {
                if ((FirstSampleAngle > 270 * 64) && (LastSampleAngle < 90 * 64)) {
                  IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle -
                                                 FirstSampleAngle) / ((
                                                       package_Sample_Num - 1) * 1.0));
                  IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                } else {
                  IntervalSampleAngle = IntervalSampleAngle_LastPackage;
                }
              } else {
                IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) / ((
                                                package_Sample_Num - 1) * 1.0));
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              }
            }

            break;

          case 8:
            CheckSum = currentByte;
            break;

          case 9:
            CheckSum += (currentByte * 0x100);
            break;
        }

        packageBuffer[recvPos++] = currentByte;
      }

      if (recvPos  == PackagePaidBytes) {
        package_recvPos = recvPos;
        break;
      }
    }

    if (PackagePaidBytes == recvPos) {
      startTs = getms();
      recvPos = 0;

      while ((waitTime = getms() - startTs) <= timeout) {
        size_t remainSize = package_Sample_Num * PackageSampleBytes - recvPos;
        size_t recvSize = 0;
        result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

        if (!IS_OK(ans)) {
          return ans;
        }

        if (recvSize > remainSize) {
          recvSize = remainSize;
        }

        getData(globalRecvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
          if (m_intensities) {
            if (recvPos % 3 == 2) {
              Valu8Tou16 += globalRecvBuffer[pos] * 0x100;
              CheckSumCal ^= Valu8Tou16;
            } else if (recvPos % 3 == 1) {
              Valu8Tou16 = globalRecvBuffer[pos];
            } else {
              CheckSumCal ^= globalRecvBuffer[pos];
            }
          } else {
            if (recvPos % 2 == 1) {
              Valu8Tou16 += globalRecvBuffer[pos] * 0x100;
              CheckSumCal ^= Valu8Tou16;
            } else {
              Valu8Tou16 = globalRecvBuffer[pos];
            }
          }

          packageBuffer[package_recvPos + recvPos] = globalRecvBuffer[pos];
          recvPos++;
        }

        if (package_Sample_Num * PackageSampleBytes == recvPos) {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_Sample_Num * PackageSampleBytes != recvPos) {
        return RESULT_FAIL;
      }
    } else {
      return RESULT_FAIL;
    }

    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    if (CheckSumCal != CheckSum) {
      CheckSumResult = false;
      has_package_error = true;
    } else {
      CheckSumResult = true;
    }

  }

  uint8_t package_CT;

  if (m_intensities) {
    package_CT = package.package_CT;
  } else {
    package_CT = packages.package_CT;
  }

  (*node).scan_frequence  = 0;

  if ((package_CT & 0x01) == CT_Normal) {
    (*node).sync_flag = Node_NotSync;
    memset((*node).debug_info, 0xff, sizeof((*node).debug_info));

    if (!has_package_error) {
      if (package_index < 10) {
        (*node).debug_info[package_index] = (package_CT >> 1);
        (*node).index = package_index;
      } else {
        (*node).index = 0xff;
      }

      if (package_Sample_Index == 0) {
        package_index++;
      }
    } else {
      (*node).index = 255;
      package_index = 0;
    }
  } else {
    (*node).sync_flag = Node_Sync;
    (*node).index = 255;
    package_index = 0;

    if (CheckSumResult) {
      has_package_error = false;
      (*node).scan_frequence  = scan_frequence;
    }
  }

  (*node).sync_quality = Node_Default_Quality;
  (*node).stamp = 0;

  if (CheckSumResult) {
    if (m_intensities) {
      (*node).sync_quality = ((uint16_t)((
                                           package.packageSample[package_Sample_Index].PakageSampleDistance
                                           & 0x03) << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT) |
                              (package.packageSample[package_Sample_Index].PakageSampleQuality));
      (*node).distance_q2 =
        package.packageSample[package_Sample_Index].PakageSampleDistance & 0xfffc;
    } else {
      (*node).distance_q2 = packages.packageSampleDistance[package_Sample_Index];
      (*node).sync_quality = ((uint16_t)(0xfc |
                                         packages.packageSampleDistance[package_Sample_Index] &
                                         0x0003)) << LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

    }

    if ((*node).distance_q2 != 0) {
      if (!isTOFLidar(m_LidarType)) {
        if (isOctaveLidar(model)) {
          AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - ((
                                                  *node).distance_q2 / 2.0))) / 155.3) / ((
                                                      *node).distance_q2 / 2.0))) * 180.0 / 3.1415) * 64.0);
        } else  {
          AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - ((
                                                  *node).distance_q2 / 4.0))) / 155.3) / ((
                                                      *node).distance_q2 / 4.0))) * 180.0 / 3.1415) * 64.0);
        }
      }



    } else {
      AngleCorrectForDistance = 0;
    }

    float sampleAngle = IntervalSampleAngle * package_Sample_Index;

    if ((FirstSampleAngle + sampleAngle +
         AngleCorrectForDistance) < 0) {
      (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                    AngleCorrectForDistance + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                  LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
        (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                      AngleCorrectForDistance - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                      AngleCorrectForDistance)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    (*node).sync_flag       = Node_NotSync;
    (*node).sync_quality    = Node_Default_Quality;
    (*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    (*node).distance_q2      = 0;
    (*node).scan_frequence  = 0;
  }


  uint8_t nowPackageNum;

  if (m_intensities) {
    nowPackageNum = package.nowPackageNum;
  } else {
    nowPackageNum = packages.nowPackageNum;
  }

  package_Sample_Index++;

  if (package_Sample_Index >= nowPackageNum) {
    package_Sample_Index = 0;
    CheckSumResult = false;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::waitScanData(node_info *nodebuffer, size_t &count,
                                     uint32_t timeout) {
  if (!isConnected) {
    count = 0;
    return RESULT_FAIL;
  }

  size_t     recvNodeCount    =  0;
  uint32_t   startTs          = getms();
  uint32_t   waitTime         = 0;
  result_t   ans              = RESULT_FAIL;

  while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
    node_info node;
    ans = waitPackage(&node, timeout - waitTime);

    if (!IS_OK(ans)) {
      count = recvNodeCount;
      return ans;
    }

    nodebuffer[recvNodeCount++] = node;

    if (node.sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
      size_t size = _serial->available();
      uint64_t delayTime = 0;
      size_t PackageSize = (m_intensities ? INTENSITY_NORMAL_PACKAGE_SIZE :
                            NORMAL_PACKAGE_SIZE);

      if (size > PackagePaidBytes && size < PackagePaidBytes * PackageSize) {
        size_t packageNum = size / PackageSize;
        size_t Number = size % PackageSize;
        delayTime = packageNum * m_PointTime * PackageSize / 2;

        if (Number > PackagePaidBytes) {
          delayTime += m_PointTime * ((Number - PackagePaidBytes) / 2);
        }

        size = Number;

        if (packageNum > 0 && Number == 0) {
          size = PackageSize;
        }
      }

      nodebuffer[recvNodeCount - 1].stamp = size * trans_delay + delayTime;
      nodebuffer[recvNodeCount - 1].scan_frequence = node.scan_frequence;
      count = recvNodeCount;
      return RESULT_OK;
    }

    if (recvNodeCount == count) {
      return RESULT_OK;
    }
  }

  count = recvNodeCount;
  return RESULT_FAIL;
}


result_t YDlidarDriver::grabScanData(node_info *nodebuffer, size_t &count,
                                     uint32_t timeout) {
  switch (_dataEvent.wait(timeout)) {
    case Event::EVENT_TIMEOUT:
      count = 0;
      return RESULT_TIMEOUT;

    case Event::EVENT_OK: {
      if (scan_node_count == 0) {
        return RESULT_FAIL;
      }

      ScopedLocker l(_lock);
      size_t size_to_copy = min(count, scan_node_count);
      memcpy(nodebuffer, scan_node_buf, size_to_copy * sizeof(node_info));
      count = size_to_copy;
      scan_node_count = 0;
    }

    return RESULT_OK;

    default:
      count = 0;
      return RESULT_FAIL;
  }

}


result_t YDlidarDriver::ascendScanData(node_info *nodebuffer, size_t count) {
  float inc_origin_angle = (float)360.0 / count;
  int i = 0;

  for (i = 0; i < (int)count; i++) {
    if (nodebuffer[i].distance_q2 == 0) {
      continue;
    } else {
      while (i != 0) {
        i--;
        float expect_angle = (nodebuffer[i + 1].angle_q6_checkbit >>
                              LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /
                             64.0f - inc_origin_angle;

        if (expect_angle < 0.0f) {
          expect_angle = 0.0f;
        }

        uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                            LIDAR_RESP_MEASUREMENT_CHECKBIT;
        nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                           LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
      }

      break;
    }
  }

  if (i == (int)count) {
    return RESULT_FAIL;
  }

  for (i = (int)count - 1; i >= 0; i--) {
    if (nodebuffer[i].distance_q2 == 0) {
      continue;
    } else {
      while (i != ((int)count - 1)) {
        i++;
        float expect_angle = (nodebuffer[i - 1].angle_q6_checkbit >>
                              LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /
                             64.0f + inc_origin_angle;

        if (expect_angle > 360.0f) {
          expect_angle -= 360.0f;
        }

        uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                            LIDAR_RESP_MEASUREMENT_CHECKBIT;
        nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                           LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
      }

      break;
    }
  }

  float frontAngle = (nodebuffer[0].angle_q6_checkbit >>
                      LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

  for (i = 1; i < (int)count; i++) {
    if (nodebuffer[i].distance_q2 == 0) {
      float expect_angle =  frontAngle + i * inc_origin_angle;

      if (expect_angle > 360.0f) {
        expect_angle -= 360.0f;
      }

      uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                          LIDAR_RESP_MEASUREMENT_CHECKBIT;
      nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                         LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
    }
  }

  size_t zero_pos = 0;
  float pre_degree = (nodebuffer[0].angle_q6_checkbit >>
                      LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

  for (i = 1; i < (int)count ; ++i) {
    float degree = (nodebuffer[i].angle_q6_checkbit >>
                    LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

    if (zero_pos == 0 && (pre_degree - degree > 180)) {
      zero_pos = i;
      break;
    }

    pre_degree = degree;
  }

  node_info *tmpbuffer = new node_info[count];

  for (i = (int)zero_pos; i < (int)count; i++) {
    tmpbuffer[i - zero_pos] = nodebuffer[i];
  }

  for (i = 0; i < (int)zero_pos; i++) {
    tmpbuffer[i + (int)count - zero_pos] = nodebuffer[i];
  }

  memcpy(nodebuffer, tmpbuffer, count * sizeof(node_info));
  delete[] tmpbuffer;

  return RESULT_OK;
}

/************************************************************************/
/* get health state of lidar                                            */
/************************************************************************/
result_t YDlidarDriver::getHealth(device_health &health, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (m_SingleChannel) {
    if (get_device_health_success) {
      health = this->health_;
      return RESULT_OK;
    }

    health.error_code = 0;
    health.status = 0;
    return RESULT_OK;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(device_health)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&health), sizeof(health));
  }
  return RESULT_OK;
}

/************************************************************************/
/* get device info of lidar                                             */
/************************************************************************/
result_t YDlidarDriver::getDeviceInfo(device_info &info, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (m_SingleChannel) {
    if (get_device_info_success) {
      info = this->info_;
      return RESULT_OK;
    }

    info.model = YDLIDAR_S2;
    info.firmware_version = 0;
    info.hardware_version = 0;
    return RESULT_OK;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(lidar_ans_header)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&info), sizeof(info));
    model = info.model;
  }

  return RESULT_OK;
}

/************************************************************************/
/* the set to signal quality                                            */
/************************************************************************/
void YDlidarDriver::setIntensities(const bool &isintensities) {
  if (m_intensities != isintensities) {
    if (globalRecvBuffer) {
      delete[] globalRecvBuffer;
      globalRecvBuffer = NULL;
    }

    globalRecvBuffer = new uint8_t[isintensities ? sizeof(node_package) : sizeof(
                                                   node_packages)];
  }

  m_intensities = isintensities;

  if (m_intensities) {
    PackageSampleBytes = 3;
  } else {
    PackageSampleBytes = 2;
  }
}
/**
* @brief 设置雷达异常自动重新连接 \n
* @param[in] enable    是否开启自动重连:
*     true	开启
*	  false 关闭
*/
void YDlidarDriver::setAutoReconnect(const bool &enable) {
  isAutoReconnect = enable;
}


void YDlidarDriver::checkTransDelay() {
  //calc stamp
  trans_delay = _serial->getByteTime();
  sample_rate = lidarModelDefaultSampleRate(model) * 1000;

  switch (model) {
    case YDLIDAR_G4://g4
    case YDLIDAR_G4PRO:
    case YDLIDAR_F4PRO:
    case YDLIDAR_G6://g6
    case YDLIDAR_TG15:
    case YDLIDAR_TG30:
    case YDLIDAR_TG50:
      if (m_sampling_rate == -1) {
        sampling_rate _rate;
        _rate.rate = 0;
        getSamplingRate(_rate);
        m_sampling_rate = _rate.rate;
      }

      sample_rate = ConvertLidarToUserSmaple(model, m_sampling_rate);
      sample_rate *= 1000;

      break;

    case YDLIDAR_G2C:
      sample_rate = 4000;
      break;

    case YDLIDAR_G1:
      sample_rate = 9000;
      break;

    case YDLIDAR_G4C:
      sample_rate = 4000;
      break;

    default:
      break;
  }

  m_PointTime = 1e9 / sample_rate;
}

/************************************************************************/
/*  start to scan                                                       */
/************************************************************************/
result_t YDlidarDriver::startScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (isScanning) {
    return RESULT_OK;
  }

  stop();
  checkTransDelay();
  flushSerial();
  delay(30);
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!m_SingleChannel) {

      lidar_ans_header response_header;

      if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
        return ans;
      }

      if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
        return RESULT_FAIL;
      }

      if (response_header.size < 5) {
        return RESULT_FAIL;
      }
    }

    ans = this->createThread();
  }

  if (isSupportMotorCtrl(model)) {
    startMotor();
  }

  return ans;
}


result_t YDlidarDriver::stopScan(uint32_t timeout) {
  UNUSED(timeout);

  if (!isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker l(_lock);
  sendCommand(LIDAR_CMD_FORCE_STOP);
  delay(5);
  sendCommand(LIDAR_CMD_STOP);
  delay(5);
  return RESULT_OK;
}

result_t YDlidarDriver::createThread() {
  _thread = CLASS_THREAD(YDlidarDriver, cacheScanData);

  if (_thread.getHandle() == 0) {
    isScanning = false;
    return RESULT_FAIL;
  }

  isScanning = true;
  return RESULT_OK;
}


result_t YDlidarDriver::startAutoScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  flushSerial();
  delay(10);
  {

    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!m_SingleChannel) {
      lidar_ans_header response_header;

      if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
        return ans;
      }

      if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
        return RESULT_FAIL;
      }

      if (response_header.size < 5) {
        return RESULT_FAIL;
      }
    }

  }

  if (isSupportMotorCtrl(model)) {
    startMotor();
  }

  return RESULT_OK;
}

/************************************************************************/
/*   stop scan                                                   */
/************************************************************************/
result_t YDlidarDriver::stop() {
  if (isAutoconnting) {
    isAutoReconnect = false;
    isScanning = false;
  }

  disableDataGrabbing();
  stopScan();

  if (isSupportMotorCtrl(model)) {
    stopMotor();
  }

  return RESULT_OK;
}

/************************************************************************/
/*  reset device                                                        */
/************************************************************************/
result_t YDlidarDriver::reset(uint32_t timeout) {
  UNUSED(timeout);
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker l(_lock);

  if ((ans = sendCommand(LIDAR_CMD_RESET)) != RESULT_OK) {
    return ans;
  }

  return RESULT_OK;
}

/************************************************************************/
/*  startMotor                                                          */
/************************************************************************/
result_t YDlidarDriver::startMotor() {
  ScopedLocker l(_lock);

  if (isSupportMotorDtrCtrl) {
    setDTR();
    delay(500);
  } else {
    clearDTR();
    delay(500);
  }

  return RESULT_OK;
}

/************************************************************************/
/*  stopMotor                                                           */
/************************************************************************/
result_t YDlidarDriver::stopMotor() {
  ScopedLocker l(_lock);

  if (isSupportMotorDtrCtrl) {
    clearDTR();
    delay(500);
  } else {
    setDTR();
    delay(500);
  }

  return RESULT_OK;
}

/************************************************************************/
/* get the current scan frequency of lidar                              */
/************************************************************************/
result_t YDlidarDriver::getScanFrequency(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_AIMSPEED)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* add the scan frequency by 1Hz each time                              */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAdd(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADD)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* decrease the scan frequency by 1Hz each time                         */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDis(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DIS)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* add the scan frequency by 0.1Hz each time                            */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAddMic(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADDMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* decrease the scan frequency by 0.1Hz each time                       */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDisMic(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DISMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  get the sampling rate of lidar                                      */
/************************************************************************/
result_t YDlidarDriver::getSamplingRate(sampling_rate &rate, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_SAMPLING_RATE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
    m_sampling_rate = rate.rate;
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the set to sampling rate                                            */
/************************************************************************/
result_t YDlidarDriver::setSamplingRate(sampling_rate &rate, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_SAMPLING_RATE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the get to zero offset angle                                        */
/************************************************************************/
result_t YDlidarDriver::getZeroOffsetAngle(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_OFFSET_ANGLE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(offset_angle)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&angle), sizeof(angle));
  }
  return RESULT_OK;
}



std::string YDlidarDriver::getSDKVersion() {
  return SDKVerision;
}
std::map<std::string, std::string>  YDlidarDriver::lidarPortList() {
  std::vector<PortInfo> lst = list_ports();
  std::map<std::string, std::string> ports;

  for (std::vector<PortInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
    std::string port = "ydlidar" + (*it).device_id;
    ports[port] = (*it).port;
  }

  return ports;
}

}
