// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "laserMapping.h"

namespace robosense {
namespace slam {

void FastLivoSlam::resetStatics()
{
#ifdef ADAPTIVE_INIT
  livo_initial::PreIntergration::resetStatics();
#endif
  lidar_selection::Feature::resetStatics();
  lidar_selection::Frame::resetStatics();
  lidar_selection::Point::resetStatics();
}

bool FastLivoSlam::IsValid(int type, double ts, double new_iter_ts) {
  if (ts < new_iter_ts - delay_t) {
    // static ofstream log_file;
    if (!IsValid_log_file_.is_open()) {
      IsValid_log_file_.open(log_dir + "/drop_msg.txt");
    }
    IsValid_log_file_ << fixed << "drop msg: " << type << " " << ts << " " << new_iter_ts
             << std::endl;
    return false;
  }
  return true;
}

void FastLivoSlam::standard_pcl_cbk(const CloudPtr &msg, double ts) {
  SensorData data;
  data.type = SensorType::LIDAR;
  data.timestamp = ts;
  // data.cloud = std::make_shared<PointCloudXYZI>(*msg);
  data.cloud.reset(new PointCloudXYZI(*msg));

  std::lock_guard<std::mutex> lk(mtx_raw_msg_buffer);
  if (ts_msg_buffer.size() &&
      !IsValid(1, ts, ts_msg_buffer.rbegin()->second.timestamp))
    return;

  if (ts_msg_buffer.find(ts) == ts_msg_buffer.end()) {
    ts_msg_buffer[ts] = data;
  } else {
    ts_msg_buffer[ts + 0.000001] = data;
  }
  // 通知buffer线程
  cv_raw_msg.notify_one();
}

// 旋转矩阵，从IMU(右下前坐标系)到lidar(前左上坐标系)
void FastLivoSlam::imu_cbk(const slam::Imu::ConstPtr &msg_in) {
  SensorData data;
  data.type = SensorType::IMU;
  data.imu = msg_in;

  std::lock_guard<std::mutex> lk(mtx_raw_msg_buffer);
  double ts = msg_in->header.ToSec();
  data.timestamp = ts;

  // 非空则维护最新数据早前的一段窗口，窗口外的数据丢弃
  if (ts_msg_buffer.size() &&
      !IsValid(0, ts, ts_msg_buffer.rbegin()->second.timestamp))
    return;

  if (ts_msg_buffer.find(ts) == ts_msg_buffer.end()) {
    ts_msg_buffer[ts] = data;
  } else {
    ts_msg_buffer[ts + 0.000001] = data;
  }
  cv_raw_msg.notify_one();
}

void FastLivoSlam::img_cbk(const std::shared_ptr<cv::Mat> &msg,
                           double timestamp) {
  SensorData data;
  data.type = SensorType::CAMERA;
  data.timestamp = timestamp;
  data.mat = *msg;

  std::lock_guard<std::mutex> lk(mtx_raw_msg_buffer);
  double ts = timestamp;
  if (ts_msg_buffer.size() &&
      !IsValid(2, ts, ts_msg_buffer.rbegin()->second.timestamp))
    return;
  if (ts_msg_buffer.find(ts) == ts_msg_buffer.end()) {
    ts_msg_buffer[ts] = data;
    // TODO: 做一些防呆，和降采样
  } else {
    ts_msg_buffer[ts + 0.000001] = data;
  }
  cv_raw_msg.notify_one();
}

cv::Mat FastLivoSlam::getImageFromMsg(const cv::Mat img_msg) {
  cv::Mat img = img_msg;
  if (img_scaling_ratio < 1.0) {
    cv::Mat resized_img;
    int new_width = static_cast<int>(img.cols * img_scaling_ratio);
    int new_height = static_cast<int>(img.rows * img_scaling_ratio);
    cv::resize(img, resized_img, cv::Size(new_width, new_height));
    cv::Mat img_undist;
    lidar_selector->cam->undistortImage(resized_img, img_undist);
    return img_undist;
  } else {
    cv::Mat img_undist;
    lidar_selector->cam->undistortImage(img_msg, img_undist);
    return img_undist;
  }
}

void FastLivoSlam::ProcessMsgBufferLoop() {
  while (run_flag_) {

    // 队列空则等待原始消息触发
    if (ts_msg_buffer.empty()) {
      std::unique_lock<std::mutex> lk(mtx_raw_msg_buffer);
      // cv_raw_msg.wait(lk); // FIXME: 会阻塞线程退出
      cv_raw_msg.wait_for(lk, std::chrono::milliseconds(500));
      lk.unlock();
      continue;
    }

    while (true) {
      // 处理窗口内最早的数据
      mtx_raw_msg_buffer.lock();
      auto front_iter = ts_msg_buffer.begin();
      SensorData data = front_iter->second;
      if (data.timestamp > ts_msg_buffer.rbegin()->second.timestamp - delay_t) {
        mtx_raw_msg_buffer.unlock();
        break;
      }
      ts_msg_buffer.erase(front_iter);
      mtx_raw_msg_buffer.unlock();

      double timestamp = data.timestamp;
      SensorType msg_type = data.type;
      switch (msg_type) {
      case SensorType::IMU: {
        slam::Imu::ConstPtr msg_in = data.imu;

        // imu消息变到lidar系
        slam::Imu::ConstPtr msg;
        if (trans_imu_to_lidar) {
          slam::Imu imu_L;
          imu_L.header = msg_in->header;
          rotateVector(msg_in->angular_velocity, R_L_I, imu_L.angular_velocity);
          rotateVector(msg_in->linear_acceleration, R_L_I,
                      imu_L.linear_acceleration);
          msg = std::make_shared<slam::Imu>(imu_L);
        } else {
          msg = msg_in;
        }

        mtx_buffer.lock();
        if (save_log) {
          if (last_timestamp_imu < 0)
            f_sensor_buf << fixed << " first imu " << timestamp << endl;
          else
            f_sensor_buf << fixed << " imu " << timestamp << " cnt: " << process_msg_buffer_loop_imu_cnt_++
                         << endl;
        }
        
        double ts = msg->header.ToSec();
        if (ts < last_timestamp_imu) {
          printf("imu loop back, clear buffer");
          f_sensor_buf << "imu loop back, clear buffer\n";
          imu_buffer.clear();
          flg_imu_reset_ = true;
        }
        last_timestamp_imu = ts;

        imu_buffer.push_back(msg);
        mtx_buffer.unlock();
#ifdef ADAPTIVE_INIT
      if(livo_init_ptr && img_en)
      {
        livo_init_ptr->addImu(msg->header.ToSec(), 
          Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z), 
          Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z)
          );
      }
#endif

        mtx_loop.lock();
        sig_buffer.notify_all();
        mtx_loop.unlock();
        break;
      }

      case SensorType::LIDAR: {
        CloudPtr msg = data.cloud;

        mtx_buffer.lock();
        f_sensor_buf << std::fixed << "lid " << timestamp << std::endl;
        if (timestamp < last_timestamp_lidar) {
          printf("lidar loop back, clear buffer");
          f_sensor_buf << "lidar loop back, clear buffer\n";
          lidar_buffer.clear();
        }

        lidar_buffer.push_back(msg);
        time_buffer.push_back(timestamp);
        last_timestamp_lidar = timestamp; // 帧尾
        mtx_buffer.unlock();
#ifdef ADAPTIVE_INIT
        if (livo_init_ptr && img_en) {
          // livo_init_ptr->addLidar(timestamp, msg);
        }
#endif
        break;
      }
      
      case SensorType::CAMERA: {
        cv::Mat msg = data.mat;

        mtx_buffer.lock();
        f_sensor_buf << std::fixed << "cam " << timestamp << " ";
        if (!img_en) {
          f_sensor_buf << "\n";
          mtx_buffer.unlock();
          break;
        }
        // static double last_msg_header_time = 0;
        double msg_header_time = timestamp + delta_time;
        // NOTE: 避免偶发的时间戳早的图像被晚接受导致的积分崩溃
        if (msg_header_time < last_timestamp_lidar && last_timestamp_lidar > 0) {
          LERROR << "img timestamp is earlier than lidar timestamp. "
                    "msg_header_time: "
                << msg_header_time
                << " last_timestamp_lidar: " << last_timestamp_lidar << REND;
          f_sensor_buf << "msg_header_time < last_timestamp_lidar: "
                      << msg_header_time << " " << last_timestamp_lidar << "\n";
          mtx_buffer.unlock();
          break;
        }
        process_msg_buffer_loop_cam_cnt_++;
        f_sensor_buf << "img_cnt:" << process_msg_buffer_loop_cam_cnt_ << " " << cam_keep_period << " "
                     << p_imu->imu_need_init_ << " ";
        if (cam_keep_period != -1 && p_imu->imu_need_init_ == false) {
          if (process_msg_buffer_loop_cam_cnt_ % cam_keep_period != 0) {
            f_sensor_buf << "downsample dropping\n";
            mtx_buffer.unlock();
            break;
          }
        }
        if (msg_header_time < last_timestamp_lidar) {
          f_sensor_buf << "last_msg_header_time < last_timestamp_lidar\n";
          mtx_buffer.unlock();
          break;
        }

        double time_dif = (last_timestamp_lidar + 0.1) - msg_header_time;
        double img_minus_last_img = msg_header_time - process_msg_buffer_loop_last_msg_header_time_;
        f_sensor_buf << "img-lidar"
                    << " " << time_dif << " img-last img: " << img_minus_last_img
                    << " imu: " << imu_buffer.size() << std::endl;
        printf("[ ProcessMsgBufferLoop ] get img at time: %.3f. img-lidar: %.3f. img-last_img: "
              "%.3f. imu: %ld\n",
              msg_header_time, time_dif, msg_header_time - process_msg_buffer_loop_last_msg_header_time_,
              imu_buffer.size());
        process_msg_buffer_loop_last_msg_header_time_ = msg_header_time;
        if (msg_header_time < last_timestamp_img) {
          printf("img loop back, clear buffer");
          img_buffer.clear();
          img_time_buffer.clear();
          img_valid_buffer.clear();
          f_sensor_buf << "img loop back, clear buffer";
        }

        // mtx_buffer.lock();
        img_buffer.push_back(getImageFromMsg(msg));
        img_time_buffer.push_back(msg_header_time);
        if(cam_keep_period != -1 && process_msg_buffer_loop_cam_cnt_ % cam_keep_period != 0)
        {
          img_valid_buffer.push_back(false);
        }
        else
        {
          img_valid_buffer.push_back(true);
        }
        last_timestamp_img = msg_header_time;
        mtx_buffer.unlock();
#ifdef ADAPTIVE_INIT
        if (livo_init_ptr && img_en) {
          livo_init_ptr->addImage(msg_header_time, msg);
        }
#endif
        break;
      }
      default: {
        break;
      }
      } // end switch
    }
  }
}

bool FastLivoSlam::CombineSensorMsgs(LidarMeasureGroup &meas) {
  if (run_flag_ == false) {
    return false;
  }
  if ((lidar_buffer.empty() && img_buffer.empty())) {
    return false;
  }
  if (imu_buffer.size() < 3) {
    return false;
  }
  
#ifdef ADAPTIVE_INIT
  if(p_imu->init_data_ready_ && img_en)
  {
    static int count{0};
    if(count<10)
    {
      mtx_buffer.lock();
      int dst{0};
      for(int i=0; i<img_buffer.size(); i++)
      {
        if(img_time_buffer[i]<=p_imu->init_time_ || img_valid_buffer[i])
        {
          img_buffer[dst] = img_buffer[i];
          img_time_buffer[dst] = img_time_buffer[i];
          img_valid_buffer[dst] = img_valid_buffer[i];
          dst++;
        }
      }
      img_buffer.resize(dst);
      img_time_buffer.resize(dst);
      img_valid_buffer.resize(dst);
      mtx_buffer.unlock();
    }
    count++;
  }
#endif

  if (meas.is_lidar_end) { // If meas.is_lidar_end==true, means it just after
                           // scan end, clear all buffer in meas.
    // 上一帧在处理LIO, 假设V频率高于L, 所以这一帧处理VIO
    meas.measures.clear();
    meas.is_lidar_end = false;
  } else {
    // 上一帧在处理VIO
  }

  // lidar消息未被记录
  if (!lidar_pushed) { // If not in lidar scan, need to generate new meas
    if (lidar_buffer.empty()) {
      return false;
    }
    meas.lidar = lidar_buffer.front(); // push the firsrt lidar topic
    if (meas.lidar->points.size() <= 1) {
      mtx_buffer.lock();
      if (img_buffer.size() >
          0) // temp method, ignore img topic when no lidar points, keep sync
      {
        lidar_buffer.pop_front();
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        img_valid_buffer.pop_front();
      }
      mtx_buffer.unlock();
      LERROR << "meas.lidar->points.size() <= 1" << REND;
      return false;
    }
    sort(meas.lidar->points.begin(), meas.lidar->points.end(),
         time_list); // sort by sample timestamp

    meas.last_lidar_time = lidar_end_time;
    if (lidar_time_is_tail)
      meas.lidar_beg_time =
          time_buffer.front() - meas.lidar->points.back().curvature /
                                    double(1000); // generate lidar_beg_time
    else
      meas.lidar_beg_time = time_buffer.front();
    lidar_end_time =
        meas.lidar_beg_time + meas.lidar->points.back().curvature /
                                  double(1000); // calc lidar scan end time
    meas.lidar_end_time = lidar_end_time;
    lidar_pushed = true; // 表示最早的lidar消息被记录
    cout << "[ CombineSensorMsgs ] beg time: " << meas.lidar_beg_time
         << " first pt rel t(ms): " << meas.lidar->points[0].curvature
         << " last rel t(ms): " << meas.lidar->points.back().curvature << REND;
  }
  // 保证imu消息在lidar之后
  if (imu_buffer.back()->header.ToSec() < lidar_end_time + 2.5e-3) {
    return false;
  }

  // 没有img topic，只处理lidar topic
  if (img_buffer.empty()) { // no img topic, means only has lidar topic
    if (last_timestamp_imu <
        lidar_end_time + 2.5e-3) { // imu message needs to be larger than lidar_end_time,
                          // keep complete propagate.
      return false;
    }
    struct MeasureGroup m; // standard method to keep imu message.
    double imu_time = imu_buffer.front()->header.ToSec();
    m.imu.clear();
    mtx_buffer.lock();
    // 取出lidar帧尾前的imu消息
    LTITLE << "[ CombineSensorMsgs ] LIO meas.measures lidar t(s) from:"
           << meas.lidar_beg_time << " to " << lidar_end_time << REND;
    LTITLE << "[ CombineSensorMsgs ] LIO get imu from buffer, t(s) from:"
           << imu_buffer.front()->header.ToSec() << " to "
           << imu_buffer.back()->header.ToSec() << REND;

    while ((!imu_buffer.empty() && (imu_time < lidar_end_time))) {
      imu_time = imu_buffer.front()->header.ToSec();
      if (imu_time > lidar_end_time)
        break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }

    // 丢弃已经记录的lidar消息， 记录下一帧imu消息
    CombineSensorMsgs_last_lidar_t_ = lidar_end_time;
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    if (imu_buffer.empty()) {
      LERROR << "=============imu_buffer empty" << REND;
      m.imu_next = m.imu.back();
    } else {
      m.imu_next = imu_buffer.front(); // RS
    }
    mtx_buffer.unlock();

    lidar_pushed = false; // sync one whole lidar scan.
    meas.is_lidar_end = true;
    meas.measures.push_back(m);

    // if(meas.measures.size())
    // {
    //     LTITLE<<"meas.is_lidar_end:"<<meas.is_lidar_end << "
    //     measures,size()"<< meas.measures.size()<<REND; for(auto
    //     &meas_i:meas.measures)
    //     {
    //         if(meas_i.imu.size())
    //             LTITLE<<std::fixed<<"meas imu t(s)
    //             from:"<<meas_i.imu.front()->header.stamp.toSec()<<"
    //             to:"<<meas_i.imu.back()->header.stamp.toSec() << "
    //             total:"<<meas_i.imu.back()->header.stamp.toSec()-meas_i.imu.front()->header.stamp.toSec()<<
    //             " "<<meas_i.imu.size()<<REND;
    //         else
    //             LERROR<<"no imu"<<REND;
    //     }
    // }
    return true;
  }

  // 有img topic，需要处理lidar或img topic
  struct MeasureGroup m;
  cout << "[ CombineSensorMsgs ] buffer size, lidar: " << lidar_buffer.size()
       << " img: " << img_buffer.size() << " imu: " << imu_buffer.size()
       << endl;
  double lidar_t_minus_img_t = lidar_end_time - img_time_buffer.front();
  cout << "[ CombineSensorMsgs ] first img: " << img_time_buffer.front()
       << " lidar: " << lidar_end_time << " imu: " << last_timestamp_imu
       << " lidar-img(s): " << lidar_t_minus_img_t << endl;
  // lidar在前，仅处理lidar
  if ((img_time_buffer.front() > lidar_end_time)) {
    LTITLE << "[ CombineSensorMsgs ] ===only process lidar, lidar-img(s): "
           << lidar_t_minus_img_t << REND;
    if (last_timestamp_imu < lidar_end_time + 2.5e-3) {
      return false;
    }
    double imu_time = imu_buffer.front()->header.ToSec();
    m.imu.clear();
    mtx_buffer.lock();
    while ((!imu_buffer.empty() && (imu_time < lidar_end_time))) {
      imu_time = imu_buffer.front()->header.ToSec();
      if (imu_time > lidar_end_time)
        break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    CombineSensorMsgs_last_lidar_t_ = lidar_end_time;
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    m.imu_next = imu_buffer.front(); // RS
    mtx_buffer.unlock();

    lidar_pushed = false;
    meas.is_lidar_end = true;
    meas.measures.push_back(m);
  } else {
    if (img_time_buffer.front() < CombineSensorMsgs_last_lidar_t_ ||
        img_time_buffer.front() < CombineSensorMsgs_last_cam_t_) {
      LERROR << "WRONG CAM TIME:" << std::fixed << img_time_buffer.front()
             << " last_lidar_t: " << CombineSensorMsgs_last_lidar_t_
             << " last_cam_t: " << CombineSensorMsgs_last_cam_t_ << " \n\n\n\n\n"
             << REND;
      mtx_buffer.lock();
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      img_valid_buffer.pop_front();
      mtx_buffer.unlock();
      return false; // NOTE: 避免相机时间戳逆序（相对lidar）
    }
    // img在前，仅处理img
    LDEBUG << "[ CombineSensorMsgs ] ===only process img, lidar-img(s): "
           << lidar_t_minus_img_t << REND;
    double img_start_time =
        img_time_buffer.front(); // process img topic, record timestamp
    if (last_timestamp_imu < img_start_time + 2.5e-3) {
      return false;
    }
    double imu_time = imu_buffer.front()->header.ToSec();
    m.imu.clear();
    m.img_offset_time =
        img_start_time -
        meas.lidar_beg_time; // record img offset time, it shoule be the Kalman
                             // update timestamp.
    m.img = img_buffer.front();
    m.last_img_time = img_start_time;
    mtx_buffer.lock();
    // 取出img前的imu消息
    while ((!imu_buffer.empty() && (imu_time < img_start_time))) {
      imu_time = imu_buffer.front()->header.ToSec();
      if (imu_time > img_start_time)
        break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    CombineSensorMsgs_last_cam_t_ = img_start_time;
    img_buffer.pop_front();
    img_time_buffer.pop_front();
    img_valid_buffer.pop_front();
    m.imu_next = imu_buffer.front(); // RS
    mtx_buffer.unlock();

    meas.is_lidar_end = false;
    meas.measures.push_back(m);
  }
  return true;
}

/**
 * [功能描述]：FastLivoSlam的核心处理循环，负责传感器数据融合和SLAM状态估计
 * 主要功能包括：
 * 1. 传感器数据同步和组合
 * 2. IMU状态预测和点云去畸变
 * 3. VIO（视觉惯性里程计）处理
 * 4. LIO（激光惯性里程计）处理
 * 5. K-D树地图更新和结果发布
 * @return 无返回值
 */
void FastLivoSlam::CoreLoop() {
  LINFO << "FastLivoSlam Core loop start. " << REND;
  
  // 主循环：持续运行直到程序停止标志被设置
  while (run_flag_) {

    // 等待传感器数据缓冲区有新数据到达
    std::unique_lock<std::mutex> cb_mutex_lg(mtx_loop);
    sig_buffer.wait(cb_mutex_lg); // 阻塞等待数据信号
    cb_mutex_lg.unlock();

    // 内层循环：处理所有可用的传感器数据
    while (true) {
      // 尝试组合传感器消息（IMU、LiDAR、相机）
      if (!CombineSensorMsgs(LidarMeasures)) {
        break; // 如果没有足够的数据组合，退出内层循环
      }
    
      // 检查IMU是否需要重置
      if (flg_imu_reset_) {
        p_imu->Reset(); // 重置IMU状态
        flg_imu_reset_ = false;
        continue; // 跳过当前循环，重新开始
      }

      // 初始化各模块的时间统计变量
      match_time = kdtree_search_time = kdtree_search_counter = solve_time =
          solve_const_H_time = 0;

      // 记录当前循环开始时间
      loop_beg_t = omp_get_wtime();

      // 自适应初始化模块（仅在编译时启用ADAPTIVE_INIT时有效）
  #ifdef ADAPTIVE_INIT
      // 如果启用图像且IMU需要初始化但数据还未准备好
      if (img_en && p_imu->imu_need_init_ && !p_imu->init_data_ready_) {
        double tgt_time = 0.; // 目标时间戳
        
        // 根据当前数据类型确定目标时间
        if (LidarMeasures.is_lidar_end) {
          // 如果是LiDAR数据结束，计算LiDAR扫描结束时间
          tgt_time = LidarMeasures.lidar_beg_time + LidarMeasures.lidar->points.back().curvature / double(1000);
        } else {
          // 如果是图像数据，使用图像偏移时间
          tgt_time = LidarMeasures.lidar_beg_time + LidarMeasures.measures.back().img_offset_time;
        }

        // 等待初始化完成
        while (run_flag_ && livo_init_ptr && p_imu) {
          // 尝试获取初始化结果
          p_imu->init_data_ready_ = livo_init_ptr->getInitResult(p_imu->init_time_, p_imu->init_R_, p_imu->init_bg_, p_imu->init_v_, p_imu->init_g_);
          if (p_imu->init_time_ >= tgt_time) {
            break; // 初始化时间达到目标时间，退出等待
          } else {
            cv::waitKey(2); // 短暂等待，避免过度占用CPU
          }
        }
        // 清空初始化缓冲区
        if (livo_init_ptr && p_imu && p_imu->init_data_ready_) {
          livo_init_ptr->clearBuf();
        }
      }
#endif

      // IMU状态预测和点云去畸变
      double process2_t_start = omp_get_wtime();
      // 使用IMU数据预测状态并对点云进行运动补偿
      p_imu->PredicteStateAndUndistortCloud(LidarMeasures, state,
                                            feats_undistort);
      state_propagat = state; // 保存传播后的状态用于后续处理
      process2_t = omp_get_wtime() - process2_t_start;
      livo_result_->time_cost->imu_process_t = process2_t; // 记录IMU处理时间

      // 调试模式下显示测量数据信息
      if (lidar_selector->debug) {
        LidarMeasures.debug_show();
      }

      // 检查LIO初始化条件
      if (feats_undistort->empty() || (feats_undistort == nullptr)) {
        if (!fast_lio_is_ready) {
          // 如果FAST-LIO还未准备好，记录首次LiDAR时间
          first_lidar_time = LidarMeasures.lidar_beg_time;
          p_imu->first_lidar_time = first_lidar_time;
          LidarMeasures.measures.clear();
          LERROR << "[ IMU initialing] FAST-LIO not ready because of no lidar points. " << REND;
          // 发布结果回调
          if(full_result_cb_){
            full_result_cb_(*livo_result_);
          }
          continue; // 跳过当前循环
        }
      }
      
      // 标记FAST-LIO准备就绪
      fast_lio_is_ready = true;
      // 判断EKF是否已初始化（需要运行0.5秒以上）
      flg_EKF_inited = LidarMeasures.lidar_beg_time - first_lidar_time < 0.5 ? false : true;

      // 获取K-D树点云数据（用于VIO地图投影或LIO地图显示）
      if ((!LidarMeasures.is_lidar_end && use_map_proj) ||
          (LidarMeasures.is_lidar_end &&
          (show_lidar_map || boundary_point_remove))) {
        auto tt = omp_get_wtime();
        // 清空存储并重新展平K-D树
        PointVector().swap(ikdtree.PCL_Storage);
        ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
        featsFromMap->points = ikdtree.PCL_Storage; // 获取地图点云
        flatten_t = omp_get_wtime() - tt;
        livo_result_->cloud_process_result->current_size = featsFromMap->size();
      }

      // VIO处理分支：当前数据不是LiDAR扫描结束时
      if (!LidarMeasures.is_lidar_end) {
  #if 1
        // TODO: 完善VIO初始化条件
  #else
        // 预留的VIO初始化逻辑（当前被禁用）
        if (core_loop_first_) {
          first_lidar_time_ = LidarMeasures.lidar_beg_time;
          core_loop_first_ = false;
        }
        time_past = LidarMeasures.lidar_beg_time - first_lidar_time_;
        if (time_past < VIO_wait_time) {
          LDEBUG << "[ VIO ] return for init" << REND;
          if (show_lidar_map)
            publish_map(pubLaserCloudMap);
          continue;
        }
  #endif
        
        // 如果启用了图像处理
        if (img_en) {
          // VIO状态估计
          auto tt = omp_get_wtime();
          EstimateVIOState(); // 估计视觉惯性里程计状态
          livo_result_->time_cost->vio_t = omp_get_wtime() - tt;

          // 发布VIO结果
          tt = omp_get_wtime();
          PublishVIOResult(); // 发布VIO估计结果
          livo_result_->time_cost->pub_vio_t = omp_get_wtime() - tt;
        }
        
        loop_end_t = omp_get_wtime();
        continue; // VIO处理完成，继续下一轮循环
      }

      // LIO处理分支：当前数据是LiDAR扫描结束时
      
      // LIO预处理步骤
      /*** 裁剪K-D树，保留LiDAR视野内的点 ***/
      CropKdTree(state);

      /*** 移除扫描中的噪声点，降采样扫描，计算VIO权重 ***/
      PreprocessCloud(feats_undistort);

      /*** 初始化地图K-D树 ***/
      if (ikdtree.Root_Node == nullptr) {
        BuildKdTree(feats_down_body); // 构建初始K-D树
        continue; // 跳过当前循环，等待下一帧数据
      }

      // LIO状态更新
      /*** 迭代状态估计 ***/
      double t_update_start = omp_get_wtime();
      if (lidar_en) {
        EstimateLIOState(); // 估计激光惯性里程计状态
      }
      update_LIO_state_t = omp_get_wtime() - t_update_start;
      livo_result_->time_cost->lio_t = update_LIO_state_t;

      /*** K-D树增长，将特征点添加到地图K-D树中 ***/
      double kdtree_grow_t_start = omp_get_wtime();
      KdTreeGrow(); // 更新地图K-D树
      kdtree_grow_t = omp_get_wtime() - kdtree_grow_t_start;
      loop_end_t = omp_get_wtime();

      // 发布LIO结果
      PublishLIOResult(); // 发布激光惯性里程计结果
      livo_result_->time_cost->pub_lio_t = omp_get_wtime() - loop_end_t;
    }
  }

  /**************** 保存地图 ****************/
  SaveMap(); // 程序结束时保存地图到文件

  // 关闭所有日志文件
  fout_out.close();
  fout_pre.close();
  f_state_utm.close();
  f_lidar_state_utm.close();
  
  LINFO << "FastLivoSlam Core loop exit." << REND;
}

//
void FastLivoSlam::PreprocessCloud(CloudPtr feats_undistort) {
  auto tt = omp_get_wtime();

  // 去除点云拖点
  int feats_size = feats_undistort->size();
  if (boundary_point_remove && feats_size > 100 &&
      featsFromMap->points.size() > 0) {
#if 0
            // 计算source法向量
            int num_near = 5;
            vector<PointVector> nearest_feat_vec; 
            nearest_feat_vec.resize(feats_size);
            int near_map_cnt{0}, far_map_cnt{0};
#ifdef MP_EN
                omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
            for (int i = 0; i < feats_size; ++i)
            {
                PointType &point_body = feats_undistort->points[i];
                PointType point_world;
                pointBodyToWorld(&point_body, &point_world, state_propagat);
                vector<float> sq_dis_near(num_near);
                ikdtree.Nearest_Search(point_world, num_near,
                                       nearest_feat_vec[i], sq_dis_near);
                bool kdtree_condition = nearest_feat_vec[i].size() == 0 || sq_dis_near[0] > 5;
                if (kdtree_condition) {
                    point_body.curvature = 999; // 远离地图的点
                    ++far_map_cnt;
                } else {
                    double angle = CalcDiffNormalAngle(nearest_feat_vec[i], sq_dis_near) * R2D;
                    point_body.curvature = angle > 99 ? 99 : angle; // 平均法向量夹角
                    ++near_map_cnt;
                }                 
            }
#else
    for (int i = 0; i < feats_size; ++i) {
      feats_undistort->points[i].curvature = 99;
    }
#endif
    T_W_L_pose = Pose(state_propagat.rot_end, state_propagat.pos_end,
                      LidarMeasures.lidar_end_time);
    T_W_L_pose.updatePoseRight(T_I_L_pose);
    lidar_odometry_->AddLiDAR(feats_undistort, T_W_L_pose);
    *feats_undistort = lidar_odometry_->GetVlidaCloud();

    auto seg_res = lidar_odometry_->GetSegResult();
    auto filter_res = lidar_odometry_->GetFilterResult();

    auto& pro_res =  livo_result_->cloud_process_result;
    pro_res->valid_size = seg_res->fullCloudIndex.size();
    pro_res->valid_seg_size = seg_res->segmentedCloud.size();
    pro_res->boundary_semantic_size = filter_res->segmentedCloud.size();
    pro_res->boundary_size = filter_res->outlierCloud.size();
    pro_res->non_boundary_size = filter_res->validCloud.size();    
  }

  // 过滤远处点
  feats_size = feats_undistort->size();
  CloudPtr rm_far_pt_cloud(new pcl::PointCloud<PointType>);
  rm_far_pt_cloud->reserve(feats_size);
  for (int i = 0; i < feats_size; ++i) {
    PointType &pt = feats_undistort->points[i];
    double range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (range < blind_max) {
      rm_far_pt_cloud->push_back(pt);
    }
  }
  *feats_undistort = *rm_far_pt_cloud;

  /*** downsample the feature points in a scan ***/
  if (downsample_source_cloud) {
    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);
  } else {
    *feats_down_body = *feats_undistort;
  }
  feats_down_size = feats_down_body->points.size();
  {
    for (auto &pt : feats_down_body->points) {
      double range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (range < 0.3) {
        pt.curvature = 0.05;
      } else if (range < 1.) {
        pt.curvature = 0.05 + (0.02 - 0.05) * (range - 0.3) / (1. - 0.3);
      } else {
        pt.curvature = 0.02;
      }
    }
  }

  preprocess_t = omp_get_wtime() - tt;
  livo_result_->time_cost->cloud_process_t = preprocess_t;
  livo_result_->cloud_process_result->undistort_size = feats_size;
  livo_result_->cloud_process_result->feats_down_size = feats_down_size;
}

// LIO
void FastLivoSlam::BuildKdTree(CloudPtr feats_down_body) {
  if (feats_down_size > 5) {
    ikdtree.set_downsample_param(filter_size_map_min);
    feats_down_world->resize(feats_down_size);
    for (int i = 0; i < feats_down_size; i++) {
      pointBodyToWorld(&(feats_down_body->points[i]),
                       &(feats_down_world->points[i]));
    }
    auto normals = getNormals(feats_down_world, 5, 0);
    for (int i = 0; i < feats_down_size; i++) {
      feats_down_world->points[i].normal_x = normals->points[i].normal_x;
      feats_down_world->points[i].normal_y = normals->points[i].normal_y;
      feats_down_world->points[i].normal_z = normals->points[i].normal_z;
    }
    ikdtree.Build(feats_down_world->points);

    livo_result_->cloud_process_result->build_size = feats_down_world->size();
  }
}

void FastLivoSlam::CropKdTree(const StatesGroup &state) {
  cub_needrm.clear();
  kdtree_delete_counter = 0;
  kdtree_delete_time = 0.0;
  const V3D& pos_LiD = state.pos_end;
  bool need_move = false;
#if 1 // RS, simply remove near 26 voxel where min = half_len and max = lid_pos + max_range
  std::vector<Eigen::Vector3i> far_directions;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        if (x || y || z)
          far_directions.push_back(Eigen::Vector3i(x, y, z));
      }
    }
  }
  double half_len = cube_len / 2.0;
  BoxPointType rm_box;
  for (const auto &dir : far_directions) {
    for (int i = 0; i < 3; ++i) {
      rm_box.vertex_min[i] = pos_LiD(i) + half_len * dir[i];
      rm_box.vertex_max[i] =
          pos_LiD(i) + std::max(cube_len * 10., 150.) * dir[i]; // cover max_range
      if (rm_box.vertex_min[i] > rm_box.vertex_max[i])
        std::swap(rm_box.vertex_min[i], rm_box.vertex_max[i]);

      if (dir[i] == 0) {
        rm_box.vertex_min[i] = pos_LiD(i) - half_len;
        rm_box.vertex_max[i] = pos_LiD(i) + half_len;
      }
    }
    cub_needrm.push_back(rm_box);
  }
  livo_result_->cloud_process_result->need_move = need_move = true;
#else // bug in official
  PRINT("Local Map is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n",
          LocalMap_Points.vertex_min[0], LocalMap_Points.vertex_max[0],
          LocalMap_Points.vertex_min[1], LocalMap_Points.vertex_max[1],
          LocalMap_Points.vertex_min[2], LocalMap_Points.vertex_max[2]);
  if (!Localmap_Initialized) {
    for (int i = 0; i < 3; i++) {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
    }
    Localmap_Initialized = true;
    return;
  }
  float dist_to_map_edge[3][2];
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
        dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
      livo_result_->cloud_process_result->need_move = need_move = true;
  }
  if (!need_move)
    return;
  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  New_LocalMap_Points = LocalMap_Points;
  float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                       double(DET_RANGE * (MOV_THRESHOLD - 1)));
  for (int i = 0; i < 3; i++) {
    tmp_boxpoints = LocalMap_Points;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
      // PRINT("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n",
      // tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
      // PRINT("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n",
      // tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
    }
  }
  LocalMap_Points = New_LocalMap_Points;
#endif

  RemovePointsFromKdTree();
  double delete_begin = omp_get_wtime();
  if (cub_needrm.size() > 0)
    kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
  kdtree_delete_time = omp_get_wtime() - delete_begin;

  // PRINT("Delete time: %0.6f, delete size: %d\n", kdtree_delete_time,
  //       kdtree_delete_counter);
  // PRINT("Delete Box: %d\n", int(cub_needrm.size()));

  livo_result_->cloud_process_result->need_move= need_move;
  livo_result_->cloud_process_result->kdtree_delete_counter = kdtree_delete_counter;
  livo_result_->cloud_process_result->kdtree_delete_time = kdtree_delete_time;
}

void FastLivoSlam::EstimateLIOState() {
  int featsFromMapNum = ikdtree.size();
  LDEBUG << "[ LIO ] Raw feature num: " << feats_undistort->points.size()
         << " feats_down_size: " << feats_down_size
         << " Map num: " << featsFromMapNum << "." << REND;

  euler_cur = RotMtoEuler(state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_update_time - first_lidar_time
           << " " << euler_cur.transpose() * R2D << " "
           << state.pos_end.transpose() << " " << state.vel_end.transpose()
           << " " << state.bias_g.transpose() << " " << state.bias_a.transpose()
           << " " << state.gravity.transpose() << endl;

  if (boundary_point_remove && feats_down_size < 200 ||
      !boundary_point_remove && feats_down_size < 500) {
    LERROR << "feats_down_size too small" << REND;
    // return; // TODO: 完善退出条件
  }
#ifdef MP_EN
  printf("[ LIO ]: Using multi-processor, used core number: %d.\n",
         MP_PROC_NUM);
#endif
  /*** ICP and iterated Kalman filter update ***/
  normvec->resize(feats_down_size);
  feats_down_world->resize(feats_down_size);
  res_last.resize(feats_down_size, 1000.0);

  point_selected_surf.resize(feats_down_size, 1);
  pointSearchInd_surf.resize(feats_down_size);
  Nearest_Points.resize(feats_down_size);
  int rematch_num = 0;
  bool nearest_search_en = true;

  for (iterCount = -1; iterCount < NUM_MAX_ITERATIONS && flg_EKF_inited;
       iterCount++) {
    lidar_selector->lidar_iter_num = iterCount + 2;
    match_start = omp_get_wtime();
    PointCloudXYZI().swap(*laserCloudOri);
    PointCloudXYZI().swap(*corr_normvect);
    valid_source_indices.reset(new pcl::PointIndices);
    invalid_source_indices.reset(new pcl::PointIndices);
    invalid_reason.clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) {
      PointType &point_body = feats_down_body->points[i];
      PointType &point_world = feats_down_world->points[i];
      point_world.normal_x = point_world.normal_y = point_world.normal_z = 99;
      V3D p_body(point_body.x, point_body.y, point_body.z);
      pointBodyToWorld(&point_body, &point_world);
      vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
      auto &points_near = Nearest_Points[i];
      uint8_t search_flag = 0;
      double search_start = omp_get_wtime();
      if (nearest_search_en) {
        /** Find the closest surfaces in the map **/
        ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
                               pointSearchSqDis);
        point_selected_surf[i] =
            pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? 0 : 1;
        kdtree_search_time += omp_get_wtime() - search_start;
        kdtree_search_counter++;
      }
      if (!point_selected_surf[i] || points_near.size() < NUM_MATCH_POINTS)
        continue;

      VF(4) pabcd;
      point_selected_surf[i] = 0;
      if (esti_plane(pabcd, points_near, 0.1f)) //(planeValid)
      {
        float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                    pabcd(2) * point_world.z + pabcd(3);
        float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

        if (s > 0.9) {
          point_selected_surf[i] = 1;
          normvec->points[i].x = pabcd(0);
          normvec->points[i].y = pabcd(1);
          normvec->points[i].z = pabcd(2);
          normvec->points[i].intensity = pd2;
          res_last[i] = abs(pd2);
        }
        point_world.normal_x = pabcd(0);
        point_world.normal_y = pabcd(1);
        point_world.normal_z = pabcd(2);
        point_body.normal_x = pabcd(0);
        point_body.normal_y = pabcd(1);
        point_body.normal_z = pabcd(2);
      }
    }
    // cout<<"pca time test: "<<pca_time1<<" "<<pca_time2<<endl;
    effct_feat_num = 0;
    laserCloudOri->resize(feats_down_size);
    corr_normvect->resize(feats_down_size);
    valid_source_indices->indices.reserve(feats_down_size);
    invalid_source_indices->indices.reserve(feats_down_size);
    invalid_reason.resize(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
      if (point_selected_surf[i] && (res_last[i] <= p2plane_thresh)) {
        laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
        // feats_down_body->points[i].curvature = abs(res_last[i]);
        // feats_down_world->points[i].curvature = abs(res_last[i]);
        corr_normvect->points[effct_feat_num] = normvec->points[i];
        valid_source_indices->indices.push_back(i);
        total_residual += res_last[i];
        effct_feat_num++;
      } else {
        invalid_source_indices->indices.push_back(i);
        if (!point_selected_surf[i])
          invalid_reason[i] = 10;
        else if (res_last[i] > 0.5)
          invalid_reason[i] = 20;
      }
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    LTITLE << "[ LIO mapping ] Effective feature num: " << effct_feat_num
           << " res_mean_last " << res_mean_last << REND;

    solve_start = omp_get_wtime();
    lidar_selector->lidar_feat_num = effct_feat_num;

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    MatrixXd Hsub(effct_feat_num, 6);
    VectorXd meas_vec(effct_feat_num);
    // LTITLE<<"iterCount: "<<iterCount+1<<", effct_feat_num:
    // "<<effct_feat_num<<REND;
    for (int i = 0; i < effct_feat_num; i++) {
      const PointType &laser_p = laserCloudOri->points[i];
      V3D point_this(laser_p.x, laser_p.y, laser_p.z);
      point_this = Lidar_rot_to_IMU * point_this + Lidar_offset_to_IMU;
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/
      const PointType &norm_p = corr_normvect->points[i];
      V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

      /*** calculate the Measuremnt Jacobian matrix H ***/
      V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
      Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

      /*** Measuremnt: distance to the closest surface/corner ***/
      meas_vec(i) = -norm_p.intensity;
    }
    solve_const_H_time += omp_get_wtime() - solve_start;

    MatrixXd K(DIM_STATE, effct_feat_num);

    EKF_stop_flg = false;
    flg_EKF_converged = false;

    /*** Iterative Kalman Filter Update ***/
    if (!flg_EKF_inited) {
      cout << "||||||||||Initiallizing LiDar||||||||||" << endl;
      /*** only run in initialization period ***/
      MatrixXd H_init(MD(9, DIM_STATE)::Zero());
      MatrixXd z_init(VD(9)::Zero());
      H_init.block<3, 3>(0, 0) = M3D::Identity();
      H_init.block<3, 3>(3, 3) = M3D::Identity();
      H_init.block<3, 3>(6, 15) = M3D::Identity();
      z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
      z_init.block<3, 1>(0, 0) = -state.pos_end;

      auto H_init_T = H_init.transpose();
      auto &&K_init =
          state.cov * H_init_T *
          (H_init * state.cov * H_init_T + 0.0001 * MD(9, 9)::Identity())
              .inverse();
      solution = K_init * z_init;
      state.resetpose();
      EKF_stop_flg = true;
    } else {
      auto &&Hsub_T = Hsub.transpose();
      auto &&HTz = Hsub_T * meas_vec;
      H_T_H.block<6, 6>(0, 0) = Hsub_T * Hsub;
      // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
      MD(DIM_STATE, DIM_STATE) &&K_1 =
          (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse();
      G.block<DIM_STATE, 6>(0, 0) =
          K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
      auto vec = state_propagat - state;
      solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec -
                 G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
      state += solution;

      rot_add = solution.block<3, 1>(0, 0);
      t_add = solution.block<3, 1>(3, 0);

      if ((rot_add.norm() * R2D < 0.01) && (t_add.norm() * 100 < 0.015)) {
        flg_EKF_converged = true;
      }

      deltaR = rot_add.norm() * R2D;
      deltaT = t_add.norm() * 100;
    }
    euler_cur = RotMtoEuler(state.rot_end);

    /*** Rematch Judgement ***/
    nearest_search_en = false;
    if (flg_EKF_converged ||
        ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2)))) {
      nearest_search_en = true;
      rematch_num++;
    }

    /*** Convergence Judgements and Covariance Update ***/
    if (!EKF_stop_flg &&
        (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))) {
      if (flg_EKF_inited) {
        /*** Covariance Update ***/
        // G.setZero();
        // G.block<DIM_STATE,6>(0,0) = K * Hsub;
        state.cov = (I_STATE - G) * state.cov;
        total_distance += (state.pos_end - position_last).norm();
        position_last = state.pos_end;

        VD(DIM_STATE) K_sum = K.rowwise().sum();
        VD(DIM_STATE) P_diag = state.cov.diagonal();
        // cout<<"K: "<<K_sum.transpose()<<endl;
        // cout<<"P: "<<P_diag.transpose()<<endl;
        // cout<<"position: "<<state.pos_end.transpose()<<" total distance:
        // "<<total_distance<<endl;
      }
      EKF_stop_flg = true;

      // 方向退化检测
      if (detect_lidar_degenetate) {
        Eigen::MatrixXd t_J = Hsub.rightCols(3);
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(t_J.transpose() *
                                                          t_J);
        Eigen::VectorXd eigen_values = es.eigenvalues();
        double min_eigen_value = eigen_values.minCoeff() / t_J.rows();
        auto is_degenerate_ = false;
        if (min_eigen_value < lidar_degenerate_thresh) {
          is_degenerate_ = true;
          LERROR << "[ LIO mapping ] lidar degenerate, " << min_eigen_value << " per point"
                 << REND;
        }
        lidar_selector->is_lidar_degenerate = is_degenerate_;
        lidar_selector->lidar_degenerate_score = min_eigen_value;
        if (is_degenerate_ && reset_state_when_degenerate) {
          state = state_propagat;
          // TODO: 仅重置退化方向状态
        }
      }
    }
    solve_time += omp_get_wtime() - solve_start;

    if (EKF_stop_flg)
      break;
  }
  // cout<<"[ mapping ]: iteration count: "<<iterCount+1<<endl;

  // log
  // opt
  livo_result_->lio_result->log->flg_EKF_inited = flg_EKF_inited;
  livo_result_->lio_result->log->opt_iter = iterCount+1;
  livo_result_->lio_result->log->valid_size = effct_feat_num;
  livo_result_->lio_result->log->res_mean_last = res_mean_last;
  livo_result_->lio_result->log->solution = solution;
  livo_result_->lio_result->state = state;
  // eskf
  // livo_result_->lio_result->log->state_pos = state.pos_end;

  if (rs_debug && feats_down_world->size()) {
    auto source_normal_cloud = DrawCloudNormal(feats_down_world);
    pcl::PCDWriter pcd_writer;
    if (source_normal_cloud->size())
      pcd_writer.writeBinary("/apollo/data/log/vis-test2.pcd",
                             *source_normal_cloud);
  }
}

/**
 * [功能描述]：将当前帧的特征点添加到K-D树地图中，实现地图的增长更新
 * 采用降采样策略避免地图点过于密集，提高查询效率
 * @return 无返回值
 */
void FastLivoSlam::KdTreeGrow() {
  // 检查是否有足够的降采样特征点，如果没有则直接返回
  if (!feats_down_size)
    return;
  
  // 获取地图降采样的体素大小
  int size = filter_size_map_min;
  
#if 1 // 使用 fast-lio2 的地图增长策略
  // 定义两个点云容器：需要降采样的点和不需要降采样的点
  PointVector PointToAdd;           // 需要降采样添加的点
  PointVector PointNoNeedDownsample; // 不需要降采样直接添加的点
  
  // 预分配内存空间，提高效率
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);
  
  // 遍历当前帧的每个降采样特征点
  for (int i = 0; i < feats_down_size; i++) {
    /* 将点从机体坐标系转换到世界坐标系 */
    pointBodyToWorld(&(feats_down_body->points[i]),
                     &(feats_down_world->points[i]));
    
    /* 判断是否需要添加到地图中 */
    // 如果该点有最近邻点且EKF已初始化，则进行降采样判断
    if (!Nearest_Points[i].empty() && flg_EKF_inited) {
      const PointVector &points_near = Nearest_Points[i]; // 获取最近邻点集
      bool need_add = true; // 是否需要添加该点的标志
      BoxPointType Box_of_Point; // 点的包围盒（未使用）
      PointType downsample_result, mid_point; // 降采样结果点和体素中心点
      
      // 计算当前点所在体素的中心点坐标
      mid_point.x =
          floor(feats_down_world->points[i].x / size) * size + 0.5 * size;
      mid_point.y =
          floor(feats_down_world->points[i].y / size) * size + 0.5 * size;
      mid_point.z =
          floor(feats_down_world->points[i].z / size) * size + 0.5 * size;
      
      // 计算当前点到体素中心的距离
      float dist = calc_dist(feats_down_world->points[i], mid_point);
      
      // 如果最近邻点在体素外部，则直接添加不进行降采样
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * size &&
          fabs(points_near[0].y - mid_point.y) > 0.5 * size &&
          fabs(points_near[0].z - mid_point.z) > 0.5 * size) {
        PointNoNeedDownsample.push_back(feats_down_world->points[i]);
        continue;
      }
      
      // 检查最近邻点中是否有点比当前点更接近体素中心
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
        // 确保有足够的最近邻点
        if (points_near.size() < NUM_MATCH_POINTS)
          break;
        
        // 如果存在更接近体素中心的点，则不添加当前点（避免冗余）
        if (calc_dist(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      
      // 如果需要添加，则加入到降采样点集中
      if (need_add)
        PointToAdd.push_back(feats_down_world->points[i]);
    } else {
      // 如果没有最近邻或EKF未初始化，直接添加该点
      PointToAdd.push_back(feats_down_world->points[i]);
    }
  }

  // 将需要降采样的点添加到K-D树中（启用降采样）
  int add_point_size = ikdtree.Add_Points(PointToAdd, true);
  
  // 将不需要降采样的点添加到K-D树中（禁用降采样）
  ikdtree.Add_Points(PointNoNeedDownsample, false);
  
  // 计算总的添加点数（包含降采样和非降采样的点）
  add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();

#else // fast-livo 的简单策略（已废弃）
  // 简单地将所有点转换到世界坐标系
  for (int i = 0; i < feats_down_size; i++) {
    /* 将点从机体坐标系转换到世界坐标系 */
    pointBodyToWorld(&(feats_down_body->points[i]),
                     &(feats_down_world->points[i]));
  }
  // 直接添加所有点到K-D树（启用降采样）
  ikdtree.Add_Points(feats_down_world->points, true);
#endif
}

/**
 * [功能描述]：执行VIO（视觉惯性里程计）状态估计，将LiDAR点云投影到图像进行视觉特征匹配
 * 主要步骤包括：
 * 1. 过滤当前帧点云，保留视野范围内的有效点
 * 2. 过滤地图点云，扩大视野范围
 * 3. 根据法线角度进行点云筛选
 * 4. 调用视觉检测器进行特征匹配和状态估计
 * @return 无返回值
 */
void FastLivoSlam::EstimateVIOState() {
  auto tt = omp_get_wtime(); // 记录开始时间
  
  // 获取用于VIO的有效点云数据
  const auto &valid_source_cloud_for_VIO = valid_source_cloud;
  
  // 检查点云是否为空，如果为空则直接返回
  if (valid_source_cloud_for_VIO->empty()) {
    LERROR << "[ VIO ] continue, source size not enough: "
           << valid_source_cloud_for_VIO->size() << REND;
    return;
  }
  
  // 计算并记录先验状态信息
  euler_cur = RotMtoEuler(state.rot_end); // 将旋转矩阵转换为欧拉角
  // 将先验状态信息写入日志文件
  fout_pre << setw(20) << LidarMeasures.last_update_time - first_lidar_time
           << " " << euler_cur.transpose() * R2D << " "           // 欧拉角（度）
           << state.pos_end.transpose() << " "                   // 位置
           << state.vel_end.transpose() << " "                   // 速度
           << state.bias_g.transpose() << " "                    // 陀螺仪偏置
           << state.bias_a.transpose() << " "                    // 加速度计偏置
           << state.gravity.transpose() << endl;                 // 重力向量

  // 初始化噪声点云容器，用于存储满足特定条件的点
  lidar_selector->noise_cloud.reset(new PointCloudXYZI());
  int scan_size = valid_source_cloud_for_VIO->size();
  // 预分配内存空间，包含扫描点和地图点的总数
  lidar_selector->noise_cloud->reserve(scan_size + featsFromMap->size());
  
  // 获取当前IMU的位置和旋转状态
  V3D pos_I = state.pos_end;  // IMU位置
  M3D rot_I = state.rot_end;  // IMU旋转矩阵

  // ========== 过滤当前帧扫描点云 ==========
  std::vector<int> is_scan_selected(scan_size, 0);  // 扫描点选择标志
  std::vector<PointType> scan_vec(scan_size);       // 扫描点副本容器
  
  // 判断是否启用法线过滤（需要EKF已初始化且启用法线过滤）
  bool normal_filter = flg_EKF_inited && enable_normal_filter;
  
#ifdef MP_EN
  // 启用多线程并行处理
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  // 遍历每个扫描点进行过滤
  for (int i = 0; i < scan_size; ++i) {
    const auto &pt = valid_source_cloud_for_VIO->points[i];
    
    // 计算点相对于IMU的光束向量
    V3D pt_beam{pt.x - pos_I.x(), pt.y - pos_I.y(), pt.z - pos_I.z()};
    
    // 将光束向量转换到IMU坐标系
    M3D T_I_W = rot_I.transpose(); // 世界到IMU的旋转矩阵
    V3D pt_beam_I = T_I_W * (pt_beam);
    
    // 检查点是否在IMU视野范围内（x、y、z方向的阈值检查）
    if (fabs(pt_beam_I.z()) > z_thresh || fabs(pt_beam_I.y()) > y_thresh ||
        fabs(pt_beam_I.x()) > x_thresh) {
      continue; // 超出视野范围，跳过该点
    }
    
    // 创建点的副本并设置默认权重
    auto pt_copy = pt;
    pt_copy.intensity = 1.0; // 设置默认强度权重
    
    // 如果启用法线过滤
    if (normal_filter) {
      // 检查点的法线是否有效（normal_x > 1表示无效点）
      if (pt.normal_x > 1) // LIO中标记的无效点
        continue;
      
      // 计算点的法线向量
      V3D pt_normal{pt.normal_x, pt.normal_y, pt.normal_z};
      pt_beam.normalize(); // 归一化光束向量
      
      // 计算法线与光束向量的夹角余弦值
      double dot = fabs(pt_normal.dot(pt_beam));
      
      // 如果夹角大于阈值，说明是大角度点，给予特殊权重
      if (dot < cos(norm_thresh)) {
        pt_copy.intensity = weight; // 设置大角度点的权重
        is_scan_selected[i] = 2; // 标记为大角度点
      }
    }
    
    is_scan_selected[i] += 1; // 增加选择标志
    scan_vec[i] = pt_copy;    // 保存处理后的点
  }
  
  // 根据选择标志整理过滤后的扫描点云
  CloudPtr scan_W_filtered(new PointCloudXYZI());
  scan_W_filtered->reserve(scan_size);
  int scan_large_angle_cnt{0}; // 大角度点计数器
  
  for (int i = 0; i < scan_size; ++i) {
    if (is_scan_selected[i] == 0)
      continue; // 未选中的点跳过
    else if (is_scan_selected[i] == 1) {
      // 普通点：添加到过滤后的点云
      scan_W_filtered->push_back(scan_vec[i]);
    } else if (is_scan_selected[i] == 3) {
      // 大角度点：既添加到过滤点云，也添加到噪声点云
      scan_W_filtered->push_back(scan_vec[i]);
      lidar_selector->noise_cloud->push_back(scan_vec[i]);
      ++scan_large_angle_cnt;
    }
  }
  filter_scan_t = omp_get_wtime() - tt; // 记录扫描过滤时间

  // ========== 过滤地图点云 ==========
  tt = omp_get_wtime();
  int map_size = featsFromMap->points.size();
  CloudPtr map_W_filtered(new PointCloudXYZI());
  std::vector<int> map_out_FOV_vec(map_size, 0); // 地图点视野外标志
  int out_FOV_cnt{0}, map_large_angle_cnt{0}; // 视野外点和大角度点计数器
  
  // 如果启用地图投影且地图不为空
  if (use_map_proj && map_size) {
    std::vector<int> is_map_selected(map_size, 0); // 地图点选择标志
    std::vector<PointType> map_vec(map_size);      // 地图点副本容器
    
#ifdef MP_EN
    // 启用多线程并行处理
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    // 遍历每个地图点进行过滤
    for (int i = 0; i < map_size; ++i) {
      const auto &pt = featsFromMap->points[i];
      
      // 计算地图点相对于IMU的光束向量
      V3D pt_beam{pt.x - pos_I.x(), pt.y - pos_I.y(), pt.z - pos_I.z()};
      
      // 将光束向量转换到IMU坐标系
      M3D T_I_W = rot_I.transpose();
      V3D pt_beam_I = T_I_W * (pt_beam);
      
      // 检查地图点是否在视野范围内
      if (fabs(pt_beam_I.z()) > z_thresh || fabs(pt_beam_I.y()) > y_thresh ||
          fabs(pt_beam_I.x()) > x_thresh) {
        map_out_FOV_vec[i] = 1; // 标记为视野外
        continue;
      }

      // 创建地图点的副本
      auto pt_copy = pt;
      pt_copy.intensity = 1.0;
      
      // 如果启用法线过滤
      if (normal_filter) {
        // 检查地图点的法线是否有效
        if (pt.normal_x > 1)
          continue;
        
        // 计算地图点的法线向量和夹角
        V3D pt_normal{pt.normal_x, pt.normal_y, pt.normal_z};
        pt_beam.normalize();
        double dot = fabs(pt_normal.dot(pt_beam));
        
        // 大角度地图点处理
        if (dot < cos(norm_thresh)) {
          pt_copy.intensity = weight; // 设置权重
          is_map_selected[i] = 2;     // 标记为大角度点
        }
      }
      
      is_map_selected[i] += 1; // 增加选择标志
      map_vec[i] = pt_copy;    // 保存处理后的点
    }

    // 根据选择标志整理过滤后的地图点云
    map_W_filtered->reserve(map_size);
    for (int i = 0; i < map_size; ++i) {
      if (is_map_selected[i] == 0)
        continue; // 未选中的点跳过
      else if (is_map_selected[i] == 1) {
        // 普通地图点
        map_W_filtered->push_back(map_vec[i]);
      } else if (is_map_selected[i] == 3) {
        // 大角度地图点：既添加到过滤点云，也添加到噪声点云
        map_W_filtered->push_back(map_vec[i]);
        lidar_selector->noise_cloud->push_back(map_vec[i]);
        ++map_large_angle_cnt;
      }
      
      // 统计视野外的点
      if (map_out_FOV_vec[i])
        ++out_FOV_cnt;
    }
  }
  filter_map_t = omp_get_wtime() - tt; // 记录地图过滤时间

  // 输出过滤统计信息
  LDEBUG << "[ VIO filter scan and map] size scan: " << feats_undistort->size()
         << " downsampled scan: " << feats_down_body->size()
         << " valid size: " << valid_source_cloud->size()
         << " scan_filtered: " << scan_W_filtered->size()
         << " scan_large_angle_cnt:" << scan_large_angle_cnt
         << ", map: " << featsFromMap->size()
         << " map_filtered:" << map_W_filtered->size()
         << " out_FOV_cnt:" << out_FOV_cnt
         << " map_large_angle_cnt:" << map_large_angle_cnt
         << " noise: " << lidar_selector->noise_cloud->size() << REND;

  // ========== 执行状态估计 ==========
  tt = omp_get_wtime();
  // 调用视觉选择器进行特征检测和状态估计
  // 参数：图像时间戳、图像数据、过滤后的扫描点云、过滤后的地图点云
  lidar_selector->detect(LidarMeasures.measures.back().last_img_time,
                         LidarMeasures.measures.back().img, 
                         scan_W_filtered,
                         map_W_filtered);
  detect_t = omp_get_wtime() - tt; // 记录检测时间
}

void FastLivoSlam::PublishVIOResult() {
  publish_visual_world_sub_map(lidar_selector);
  if (rgb_image_cb_) {
    rgb_image_cb_(lidar_selector->img_cp);
  }
  if (noise_image_cb_ && lidar_selector->pub_noise_cloud) {
    noise_image_cb_(lidar_selector->img_noise);
  }
  if (raw_image_cb_){// && lidar_selector->pub_all_cloud) {
    raw_image_cb_(lidar_selector->img_raw);
  }
  if (all_cloud_image_cb_ && lidar_selector->pub_all_cloud) {
    all_cloud_image_cb_(lidar_selector->img_all_cloud);
  }
  // rgb cloud
  publish_frame_world_rgb(lidar_selector);
  // odometry
  Eigen::Vector3d t = state.pos_end;
  Eigen::Quaterniond q(state.rot_end);
  Pose pose(q, t, LidarMeasures.measures.back().last_img_time);
  if (odom_aft_mapped_cb_) {
    odom_aft_mapped_cb_(pose);
  }
  if (path_cb_) {
    path_cb_(pose);
  }
  // full result
  livo_result_->update_ts = LidarMeasures.measures.back().last_img_time;
  livo_result_->source = SensorType::CAMERA;
  livo_result_->vio_result->cam_pose = pose;
  livo_result_->lio_result->rgb_scan = rgb_scan;
  livo_result_->lio_result->rgb_map = rgb_map;

  livo_result_->vio_result->img_cp = lidar_selector->img_cp;
  livo_result_->vio_result->img_noise = lidar_selector->img_noise;
  livo_result_->vio_result->img_raw = lidar_selector->img_raw;
  livo_result_->vio_result->img_all_cloud = lidar_selector->img_all_cloud;
  if (full_result_cb_) {
    full_result_cb_(*livo_result_);
  }

  // state
  euler_cur = RotMtoEuler(state.rot_end);
  fout_out << setw(20) << LidarMeasures.is_lidar_end << " "
           << LidarMeasures.last_update_time - first_lidar_time << " "
           << euler_cur.transpose() * R2D << " " << state.pos_end.transpose()
           << " " << state.vel_end.transpose() << " "
           << state.bias_g.transpose() << " " << state.bias_a.transpose() << " "
           << state.gravity.transpose() << " " << feats_undistort->points.size()
           << endl;

  // pose
  f_state_utm << std::fixed << std::setprecision(6)
              << LidarMeasures.measures.back().last_img_time << " " << t.x()
              << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y()
              << " " << q.z() << " " << q.w() << std::endl;

  // time-cost
  // VIO_total_t = omp_get_wtime() - t_VIO_beg;
  cout << "[ VIO time-cost(s)]" << std::fixed << std::setprecision(3)
       << " pre: " << process2_t << " flatten: " << flatten_t
       << " filter scan: " << filter_scan_t << " filter_map_t: " << filter_map_t
       << " detect: " << detect_t << " VIO total: " << VIO_total_t
       << std::setprecision(6) << REND;

  if (rs_debug && lidar_selector->noise_cloud->size()) {
    double t1 = omp_get_wtime();
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary("/apollo/data/log/vis-test3.pcd",
                           *lidar_selector->noise_cloud);
  }
  if (1) {
    double img_ts = LidarMeasures.measures.back().last_img_time;
    livo_result_->vio_result->update_ts = img_ts;
    livo_result_->vio_result->log->update_ts = img_ts;
    if (print_log) {
      livo_result_->Print(0, 0, 0, 1, 0);
    }
    if (save_log) {
      livo_result_->SaveLog(0, 0, 0, 1, 0);
    }
    livo_result_->Print(0, 0, 0, 0, 1);
  }
}

void FastLivoSlam::PublishLIOResult() {
  /******* Publish LIO odometry *******/
  {
    Eigen::Quaterniond q(state.rot_end);
    Pose pose(q, state.pos_end, LidarMeasures.lidar_end_time);
    if (odom_aft_mapped_cb_) {
      odom_aft_mapped_cb_(pose);
    }
    if (path_cb_) {
      path_cb_(pose);
    }
    livo_result_->update_ts = LidarMeasures.lidar_end_time;
    livo_result_->source = SensorType::LIDAR;
    livo_result_->lio_result->lidar_pose = pose;
    livo_result_->lio_result->scan = xyzi_scan;
    livo_result_->lio_result->kdtree_map = featsFromMap;
    if (full_result_cb_) {
      full_result_cb_(*livo_result_);
    }
  }

  /******* Publish points *******/
  CloudPtr cloud_registered_I(feats_down_body);
  int size = cloud_registered_I->points.size();
  CloudPtr cloud_registered_W(new PointCloudXYZI(size, 1));
  for (int i = 0; i < size; i++) {
    RGBpointBodyToWorld(&cloud_registered_I->points[i],
                        &cloud_registered_W->points[i]);
    cloud_registered_W->points[i].curvature =
        cloud_registered_I->points[i].curvature;
    cloud_registered_W->points[i].normal_x =
        feats_down_world->points[i].normal_x;
    cloud_registered_W->points[i].normal_y =
        feats_down_world->points[i].normal_y;
    cloud_registered_W->points[i].normal_z =
        feats_down_world->points[i].normal_z;
  }
  if (downsample_source_cloud && pub_dense_map) {
    int size = feats_undistort->points.size();
    CloudPtr full_cloud_registered_W(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++) {
      RGBpointBodyToWorld(&feats_undistort->points[i],
                          &full_cloud_registered_W->points[i]);
    }
    xyzi_scan = full_cloud_registered_W;
  } else {
    xyzi_scan = cloud_registered_W;
  }

  valid_source_cloud->clear();
  pcl::copyPointCloud(*cloud_registered_W, *valid_source_indices,
                      *valid_source_cloud);

  if (lidar_selector->pub_all_cloud) {
    lidar_selector->all_cloud = cloud_registered_W;
    lidar_selector->all_cloud_L = cloud_registered_I;
  }

  if (rs_debug) {
    pcl::copyPointCloud(*cloud_registered_W, *invalid_source_indices,
                        *invalid_source_cloud);
    for (int i = 0; i < invalid_source_cloud->points.size(); ++i) {
      invalid_source_cloud->points[i].normal_x = invalid_reason[i];
    }
    pcl::PCDWriter pcd_writer;
    if (invalid_source_cloud->size())
      pcd_writer.writeBinary("/apollo/data/log/vis-test4.pcd",
                             *invalid_source_cloud);
  }

  if (!show_rgb_map || !img_en)
    publish_frame_world();
  // publish_visual_world_map(pubVisualCloud);
  publish_effect_world();
  if (show_lidar_map)
    publish_map();

  // save time-cost to log
  // total_t = kdtree_delete_time + update_LIO_state_t + pubcloud + kdtree_grow_t
  // update_LIO_state_t = match_time + solve_time
  // solve_time = solve_const_H_time + 退化检测
  ++frame_num;
  double total_t = (loop_end_t - loop_beg_t);
  aver_total_t = aver_total_t * (frame_num - 1) / frame_num + total_t / frame_num;
  aver_update_lio_state_t = aver_update_lio_state_t * (frame_num - 1) / frame_num +
                  update_LIO_state_t / frame_num;
  aver_time_match =
      aver_time_match * (frame_num - 1) / frame_num + match_time / frame_num;
  aver_time_solve =
      aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
  aver_time_const_H_time =
      aver_time_const_H_time * (frame_num - 1) / frame_num +
      solve_const_H_time / frame_num;

  // T1[time_log_counter] = LidarMeasures.lidar_end_time;
  // s_plot[time_log_counter] = aver_total_t;
  // s_plot2[time_log_counter] = kdtree_grow_t;
  // s_plot3[time_log_counter] = kdtree_search_time / kdtree_search_counter;
  // s_plot4[time_log_counter] = featsFromMapNum;
  // s_plot5[time_log_counter] = t5 - t0;
  // kdtree_delete_time
  time_log_counter++;

  if (1) {
    // time-cost
    printf("[ LIO time-cost]: lidar_ts: %f PredicteStateAndUndistortCloud: "
           "%0.3f preprocess_t: %0.3f total=(crop+update+grow+pub): %0.3f "
           "CropKdTree: %0.3f "
           "ave_match: "
           "%0.3f ave_solve: "
           "%0.3f EstimateLIOState: %0.3f KdTreeGrow: %0.3f ave total: %0.3f "
           "ave update state : %0.3f "
           "ave construct H: %0.3f.\n",
           LidarMeasures.lidar_end_time, process2_t, preprocess_t, total_t,
           kdtree_delete_time, aver_time_match, aver_time_solve,
           update_LIO_state_t, kdtree_grow_t, aver_total_t,
           aver_update_lio_state_t, aver_time_const_H_time); 
    f_LIO_t << std::fixed << std::setprecision(6)
            << LidarMeasures.last_lidar_time << " " << process2_t << " "
            << preprocess_t << " " << total_t << " " << aver_time_match << " "
            << aver_time_solve << " " << update_LIO_state_t << " "
            << kdtree_grow_t << " " << aver_total_t << " "
            << aver_update_lio_state_t << " " << aver_time_const_H_time << endl;

    // state
    euler_cur = RotMtoEuler(state.rot_end);
    fout_out << setw(20) << LidarMeasures.is_lidar_end << " "
             << LidarMeasures.last_update_time - first_lidar_time << " "
             << euler_cur.transpose() * R2D << " " << state.pos_end.transpose()
             << " " << state.vel_end.transpose() << " "
             << state.bias_g.transpose() << " " << state.bias_a.transpose()
             << " " << state.gravity.transpose() << " "
             << feats_undistort->points.size() << endl;

    // pose
    Eigen::Vector3d t = state.pos_end;
    Eigen::Quaterniond q(state.rot_end);
    f_state_utm << std::fixed << std::setprecision(6)
                << LidarMeasures.lidar_end_time << " " << t.x() << " " << t.y()
                << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z()
                << " " << q.w() << std::endl;

    f_lidar_state_utm << std::fixed << std::setprecision(6)
                      << LidarMeasures.lidar_end_time << " " << t.x() << " "
                      << t.y() << " " << t.z() << " " << q.x() << " " << q.y()
                      << " " << q.z() << " " << q.w() << std::endl;
  }
  if (1) {
    livo_result_->cloud_process_result->update_ts =
        LidarMeasures.lidar_end_time;
    livo_result_->lio_result->update_ts = LidarMeasures.lidar_end_time;
    livo_result_->lio_result->log->update_ts = LidarMeasures.lidar_end_time;

    if (print_log) {
      livo_result_->Print(0, 1, 1, 0, 0);
    }
    if (save_log) {
      livo_result_->SaveLog(0, 1, 1, 0, 1);
    }
    livo_result_->Print(0, 0, 0, 0, 1);
  }
}

void FastLivoSlam::SaveMap() {
  /* 1. make sure you have enough memories
  /* 2. pcd save will largely influence the real-time performences **/
  if (rgb_map->size() > 0 && pcd_save_en) {
    string map_dir(pcd_dir + "/rgb_map.pcd");
    pcl::PCDWriter pcd_writer;
    LINFO << "rgb map saved" << REND;
    pcd_writer.writeBinary(map_dir, *rgb_map);

    if (ply_save_en) {
      pcl::io::savePLYFileBinary(pcd_dir + "/rgb_map.ply", *rgb_map);
    }
    if (voxel_size) {
      auto rgb_map_ds = uniformSample<PointTypeRGB>(rgb_map, voxel_size);
      string file_name = "rgb_map_voxel_" + std::to_string(voxel_size) + ".pcd";
      pcd_writer.writeBinary(pcd_dir + file_name, *rgb_map_ds);

      if (ply_save_en) {
        pcl::io::savePLYFileBinary(pcd_dir + "rgb_map_voxel_" +
                                       std::to_string(voxel_size) + ".ply",
                                   *rgb_map_ds);
      }
    }
  }

  if (xyzi_map->size() > 0 && pcd_save_en) {
    string map_dir(pcd_dir + "intensity_map.pcd");
    pcl::PCDWriter pcd_writer;
    LINFO << "intensity map saved" << REND;
    pcd_writer.writeBinary(map_dir, *xyzi_map);
    if (voxel_size) {
      auto xyzi_map_ds = uniformSample<PointType>(xyzi_map, voxel_size);
      string file_name =
          "intensity_map_voxel_" + std::to_string(voxel_size) + ".pcd";
      pcd_writer.writeBinary(pcd_dir + file_name, *xyzi_map_ds);
      if (ply_save_en) {
        pcl::io::savePLYFileBinary(pcd_dir + "intensity_map_voxel_" +
                                       std::to_string(voxel_size) + ".ply",
                                   *xyzi_map_ds);
      }
    }
  }
}

} // namespace slam
} // namespace robosense
