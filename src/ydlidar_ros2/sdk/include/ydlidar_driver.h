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

/** @mainpage

    <table>
        <tr><th>Library     <td>YDlidarDriver
        <tr><th>File        <td>YDlidarDriver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/sdk
        <tr><th>Version     <td>1.4.5
        <tr><th>Sample      <td>[samples](samples/main.cpp)
    </table>

    Jump to the @link ::ydlidar::YDlidarDriver @endlink and @link ::CYdLidar @endlink interface documentation.

*/
#ifndef YDLIDAR_DRIVER_H
#define YDLIDAR_DRIVER_H
#include <stdlib.h>
#include <atomic>
#include <map>
#include "serial.h"
#include "locker.h"
#include "thread.h"
#include "ydlidar_protocol.h"
#include "help_info.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif


using namespace std;
using namespace serial;

namespace ydlidar {

/*!
* Class that provides a lidar interface.
*/
class YDlidarDriver {
 public:
  PropertyBuilderByName(bool, SingleChannel,
                        private) ///< 是否是单通信雷达
  PropertyBuilderByName(int, LidarType,
                        private) ///< 雷达类型
  PropertyBuilderByName(uint32_t, PointTime,
                        private) ///< 连个激光点之间采样时间间隔
  /*!
  * A constructor.
  * A more elaborate description of the constructor.
  */
  YDlidarDriver();

  /*!
  * A destructor.
  * A more elaborate description of the destructor.
  */
  virtual ~YDlidarDriver();

  /*!
  * @brief 连接雷达 \n
  * 连接成功后，必须使用::disconnect函数关闭
  * @param[in] port_path    串口号
  * @param[in] baudrate    波特率，YDLIDAR-SS雷达波特率：
  *     230400 G2-SS-1
  * @return 返回连接状态
  * @retval 0     成功
  * @retval < 0   失败
  * @note连接成功后，必须使用::disconnect函数关闭
  * @see 函数::YDlidarDriver::disconnect (“::”是指定有连接功能,可以看文档里的disconnect变成绿,点击它可以跳转到disconnect.)
  */
  result_t connect(const char *port_path, uint32_t baudrate);

  /*!
  * @brief 断开雷达连接
  */
  void disconnect();

  /*!
  * @brief 获取当前SDK版本号 \n
  * 静态函数
  * @return 返回当前SKD 版本号
  */
  static std::string getSDKVersion();

  /*!
  * @brief lidarPortList 获取雷达端口
  * @return 在线雷达列表
  */
  static std::map<std::string, std::string> lidarPortList();


  /*!
  * @brief 扫图状态 \n
  * @return 返回当前雷达扫图状态
  * @retval true     正在扫图
  * @retval false    扫图关闭
  */
  bool isscanning() const;

  /*!
  * @brief 连接雷达状态 \n
  * @return 返回连接状态
  * @retval true     成功
  * @retval false    失败
  */
  bool isconnected() const;

  /*!
  * @brief 设置雷达是否带信号质量 \n
  * 连接成功后，必须使用::disconnect函数关闭
  * @param[in] isintensities    是否带信号质量:
  *     true	带信号质量
  *	  false 无信号质量
  * @note只有S4B(波特率是153600)雷达支持带信号质量, 别的型号雷达暂不支持
  */
  void setIntensities(const bool &isintensities);

  /*!
  * @brief 设置雷达异常自动重新连接 \n
  * @param[in] enable    是否开启自动重连:
  *     true	开启
  *	  false 关闭
  */
  void setAutoReconnect(const bool &enable);

  /*!
  * @brief 获取雷达设备健康状态 \n
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
  */
  result_t getHealth(device_health &health, uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 获取雷达设备信息 \n
  * @param[in] info     设备信息
  * @param[in] timeout  超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
  */
  result_t getDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 开启扫描 \n
  * @param[in] force    扫描模式
  * @param[in] timeout  超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       开启成功
  * @retval RESULT_FAILE    开启失败
  * @note 只用开启一次成功即可
  */
  result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /*!
  * @brief 关闭扫描 \n
  * @return 返回执行结果
  * @retval RESULT_OK       关闭成功
  * @retval RESULT_FAILE    关闭失败
  */
  result_t stop();


  /*!
  * @brief 获取激光数据 \n
  * @param[in] nodebuffer 激光点信息
  * @param[in] count      一圈激光点数
  * @param[in] timeout    超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE    获取失败
  * @note 获取之前，必须使用::startScan函数开启扫描
  */
  result_t grabScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT) ;


  /*!
  * @brief 补偿激光角度 \n
  * 把角度限制在0到360度之间
  * @param[in] nodebuffer 激光点信息
  * @param[in] count      一圈激光点数
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 补偿之前，必须使用::grabScanData函数获取激光数据成功
  */
  result_t ascendScanData(node_info *nodebuffer, size_t count);

  /*!
  * @brief 重置激光雷达 \n
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作, 如果在扫描中调用::stop函数停止扫描
  */
  result_t reset(uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 打开电机 \n
    * @return 返回执行结果
    * @retval RESULT_OK       成功
    * @retval RESULT_FAILE    失败
    */
  result_t startMotor();

  /*!
  * @brief 关闭电机 \n
    * @return 返回执行结果
    * @retval RESULT_OK       成功
    * @retval RESULT_FAILE    失败
    */
  result_t stopMotor();

  /*!
  * @brief 获取激光雷达当前扫描频率 \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getScanFrequency(scan_frequency &frequency,
                            uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 设置增加扫描频率1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyAdd(scan_frequency &frequency,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 设置减小扫描频率1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyDis(scan_frequency &frequency,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 设置增加扫描频率0.1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyAddMic(scan_frequency &frequency,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 设置减小扫描频率0.1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyDisMic(scan_frequency &frequency,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 获取激光雷达当前采样频率 \n
  * @param[in] frequency    采样频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getSamplingRate(sampling_rate &rate,
                           uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 设置激光雷达当前采样频率 \n
  * @param[in] rate    　　　采样频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setSamplingRate(sampling_rate &rate,
                           uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 获取激光雷达当前零位角 \n
  * @param[in] angle　　　   零位偏移角
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getZeroOffsetAngle(offset_angle &angle,
                              uint32_t timeout = DEFAULT_TIMEOUT);

 protected:

  /*!
  * @brief 创建解析雷达数据线程 \n
  * @note 创建解析雷达数据线程之前，必须使用::startScan函数开启扫图成功
  */
  result_t createThread();


  /*!
  * @brief 重新连接开启扫描 \n
  * @param[in] force    扫描模式
  * @param[in] timeout  超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       开启成功
  * @retval RESULT_FAILE    开启失败
  * @note sdk 自动重新连接调用
  */
  result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /*!
  * @brief stopScan
  * @param timeout
  * @return
  */
  result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
     * @brief checkDeviceStatus
     * @param byte
     * @return
     */
  result_t checkDeviceInfo(uint8_t *recvBuffer, uint8_t byte, int recvPos,
                           int recvSize, int pos);

  /*!
   * @brief waitDevicePackage
   * @param timeout
   * @return
   */
  result_t waitDevicePackage(uint32_t timeout = DEFAULT_TIMEOUT);
  /*!
  * @brief 解包激光数据 \n
  * @param[in] node 解包后激光点信息
  * @param[in] timeout     超时时间
  */
  result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 发送数据到雷达 \n
  * @param[in] nodebuffer 激光信息指针
  * @param[in] count      激光点数大小
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    失败
  */
  result_t waitScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 激光数据解析线程 \n
  */
  int cacheScanData();

  /*!
  * @brief 发送数据到雷达 \n
  * @param[in] cmd 	 命名码
  * @param[in] payload      payload
  * @param[in] payloadsize      payloadsize
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  */
  result_t sendCommand(uint8_t cmd, const void *payload = NULL,
                       size_t payloadsize = 0);

  /*!
  * @brief 等待激光数据包头 \n
  * @param[in] header 	 包头
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    获取失败
  * @note 当timeout = -1 时, 将一直等待
  */
  result_t waitResponseHeader(lidar_ans_header *header,
                              uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief 等待固定数量串口数据 \n
  * @param[in] data_count 	 等待数据大小
  * @param[in] timeout    	 等待时间
  * @param[in] returned_size   实际数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    获取失败
  * @note 当timeout = -1 时, 将一直等待
  */
  result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                       size_t *returned_size = NULL);

  /*!
  * @brief 获取串口数据 \n
  * @param[in] data 	 数据指针
  * @param[in] size    数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE    获取失败
  */
  result_t getData(uint8_t *data, size_t size);

  /*!
  * @brief 串口发送数据 \n
  * @param[in] data 	 发送数据指针
  * @param[in] size    数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       发送成功
  * @retval RESULT_FAILE    发送失败
  */
  result_t sendData(const uint8_t *data, size_t size);


  /*!
  * @brief checkTransDelay
  */
  void checkTransDelay();

  /*!
  * @brief 关闭数据获取通道 \n
  */
  void disableDataGrabbing();

  /*!
  * @brief 设置串口DTR \n
  */
  void setDTR();

  /*!
  * @brief 清除串口DTR \n
  */
  void clearDTR();

  /*!
   * @brief flushSerial
   */
  void flushSerial();

  /*!
   * @brief checkAutoConnecting
   */
  result_t checkAutoConnecting();


 public:
  std::atomic<bool>     isConnected;  ///< 串口连接状体
  std::atomic<bool>     isScanning;   ///< 扫图状态
  std::atomic<bool>     isAutoReconnect;  ///< 异常自动从新连接
  std::atomic<bool>     isAutoconnting;  ///< 是否正在自动连接中


  enum {
    DEFAULT_TIMEOUT = 2000,    /**< 默认超时时间. */
    DEFAULT_HEART_BEAT = 1000, /**< 默认检测掉电功能时间. */
    MAX_SCAN_NODES = 3600,	   /**< 最大扫描点数. */
    DEFAULT_TIMEOUT_COUNT = 1,
  };

  node_info      *scan_node_buf;    ///< 激光点信息
  size_t         scan_node_count;   ///< 激光点数
  Event          _dataEvent;        ///< 数据同步事件
  Locker         _lock;				///< 线程锁
  Locker         _serial_lock;		///< 串口锁
  Thread 	     _thread;		   ///< 线程id

 private:
  int PackageSampleBytes;            ///< 一个包包含的激光点数
  serial::Serial *_serial;			///< 串口
  bool m_intensities;				///< 信号质量状体
  uint32_t m_baudrate;				///< 波特率
  bool isSupportMotorDtrCtrl;	    ///< 是否支持电机控制
  uint32_t trans_delay;				///< 串口传输一个byte时间
  int m_sampling_rate;              ///< 采样频率
  int model;                        ///< 雷达型号
  int sample_rate;                  ///<

  node_package package;             ///< 带信号质量协议包
  node_packages packages;           ///< 不带信好质量协议包

  uint16_t package_Sample_Index;    ///< 包采样点索引
  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  uint16_t FirstSampleAngle;        ///< 起始采样角
  uint16_t LastSampleAngle;         ///< 结束采样角
  uint16_t CheckSum;                ///< 校验和
  uint8_t scan_frequence;           ///< 协议中雷达转速

  uint16_t CheckSumCal;
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  bool CheckSumResult;
  uint16_t Valu8Tou16;

  std::string serial_port;///< 雷达端口
  uint8_t *globalRecvBuffer;
  int retryCount;
  bool has_device_header;
  uint8_t last_device_byte;
  int         asyncRecvPos;
  uint16_t    async_size;

  //singleChannel
  device_info info_;
  device_health health_;
  lidar_ans_header header_;
  uint8_t  *headerBuffer;
  uint8_t  *infoBuffer;
  uint8_t  *healthBuffer;
  bool     get_device_info_success;
  bool     get_device_health_success;

  int package_index;
  bool has_package_error;

};

}// namespace ydlidar

#endif // YDLIDAR_DRIVER_H
