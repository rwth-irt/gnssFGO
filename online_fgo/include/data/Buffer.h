//
// Created by haoming on 21.02.22.
//

#ifndef ONLINE_FGO_BUFFER_H
#define ONLINE_FGO_BUFFER_H

#pragma once

#include <mutex>
#include <shared_mutex>
#include <rclcpp/time.hpp>
#include <boost/circular_buffer.hpp>


namespace fgo::buffer {
  typedef const std::lock_guard<std::mutex> ExecutiveMutexLock;

  template<typename BufferType>
  struct CircularDataBuffer {
    boost::circular_buffer<BufferType> buffer;
    boost::circular_buffer<BufferType> buffer_tmp;
    boost::circular_buffer<rclcpp::Time> time_buffer;
    boost::circular_buffer<rclcpp::Time> time_buffer_tmp;
    boost::circular_buffer<double> duration_buffer;
    boost::circular_buffer<double> duration_buffer_tmp;
    std::mutex mutex_;

    void clean() {
      ExecutiveMutexLock lock(mutex_);
      clean_();
    };

    void cleanBeforeTime(const double& time)
    {
      ExecutiveMutexLock lock(mutex_);
      auto timeIter = time_buffer.begin();
      auto bufferIter = buffer.begin();
      while(timeIter != time_buffer.end())
      {
        if(timeIter->seconds() <= time)
        {
          timeIter = time_buffer.erase(timeIter);
          bufferIter = buffer.erase(bufferIter);
          continue;
        }
        timeIter++;
        bufferIter++;
      }
    }

    int size() {
      //ExecutiveMutexLock lock(mutex_);
      return buffer.size();
    }

    void resize_buffer(uint new_size) {
      ExecutiveMutexLock lock(mutex_);
      buffer.set_capacity(new_size);
      buffer_tmp.set_capacity(new_size);
      time_buffer.set_capacity(new_size);
      time_buffer_tmp.set_capacity(new_size);
      duration_buffer.set_capacity(new_size);
      duration_buffer_tmp.set_capacity(new_size);
      clean_();
      buffer_size = new_size;
    }

    void update_buffer(BufferType new_buffer, const rclcpp::Time &t_msg, const rclcpp::Time &t_now = rclcpp::Time(0, 0, RCL_ROS_TIME)) {
      if (mutex_.try_lock()) {
        check_temp_buffer();
        buffer.push_back(new_buffer);
        counter++;
        time_buffer.push_back(t_msg);
        if (t_now.seconds() != 0) {
          duration_buffer.push_back((t_now - t_msg).seconds());
        }
        mutex_.unlock();
      } else {
        buffer_tmp.push_back(new_buffer);
        time_buffer_tmp.push_back(t_now);
        if (t_now.seconds() != 0) {
          duration_buffer_tmp.push_back((t_now - t_msg).seconds());
        }
      }
    }

    void check_temp_buffer() {
      if (buffer_tmp.size()) {
        for (size_t i = 0; i < buffer_tmp.size(); i++) {
          buffer.push_back(buffer_tmp[i]);
          time_buffer.push_back(time_buffer_tmp[i]);
          duration_buffer.push_back(duration_buffer_tmp[i]);
          counter++;
        }
        buffer_tmp.clear();
        time_buffer_tmp.clear();
        duration_buffer_tmp.clear();
      }
    }

    BufferType get_first_buffer() {
        ExecutiveMutexLock lock(mutex_);
        // check_temp_buffer();
        return buffer.front();
      }

    BufferType get_first_buffer_and_pop()
    {
      ExecutiveMutexLock lock(mutex_);
      auto data = buffer.front();
      buffer.pop_front();
      time_buffer.pop_front();
      duration_buffer.pop_front();
      return data;
    }

    std::tuple<BufferType, rclcpp::Time> get_first_buffer_time_pair_and_pop()
    {
      ExecutiveMutexLock lock(mutex_);
      auto data = buffer.front();
      buffer.pop_front();
      auto data_timestamp = time_buffer.front();
      time_buffer.pop_front();
      duration_buffer.pop_front();
      return {data, data_timestamp};
    }

    BufferType get_last_buffer() {
      ExecutiveMutexLock lock(mutex_);
     // check_temp_buffer();
      return buffer.back();
    }

    rclcpp::Time get_last_time() {
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();
      return time_buffer.back();
    }

    rclcpp::Time get_first_time() {
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();
      return time_buffer.front();
    }

    BufferType get_buffer(const rclcpp::Time &t) {
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();

      if(buffer.empty())
      {
        BufferType T;
        return T;
      }

      std::vector<std::pair<double, BufferType>> buffer_map;

      for (uint i = 0; i < buffer_size; i++) {
        buffer_map.template emplace_back(std::make_pair(abs(t.seconds() - time_buffer[i].seconds()), buffer[i]));

      }
      std::sort(buffer_map.begin(), buffer_map.end(), [](std::pair<double, BufferType> &a,
                                                        std::pair<double, BufferType> &b) {
          return a.first > b.first;
      });
      return buffer_map.back().second;
    }

    BufferType get_buffer(const double &t) {
        ExecutiveMutexLock lock(mutex_);
        //check_temp_buffer();

        if(buffer.empty())
        {
          BufferType T;
          return T;
        }

        std::vector<std::pair<double, BufferType>> buffer_map;

        for (uint i = 0; i < buffer_size; i++) {
          buffer_map.template emplace_back(std::make_pair(abs(t - time_buffer[i].seconds()), buffer[i]));

        }
        std::sort(buffer_map.begin(), buffer_map.end(), [](std::pair<double, BufferType> &a,
                                                           std::pair<double, BufferType> &b) {
            return a.first > b.first;
        });
        return buffer_map.back().second;
      }

    BufferType get_buffer_from_id(uint id) {
      ExecutiveMutexLock lock(mutex_);
      if (id > buffer.size()) {
        std::cout << "Wrong Buffer ID: " << id << " last buffer is returned!" << std::endl;
        return buffer.back();
      }
      try
      {
        return buffer.at(id);
      }
      catch(std::exception& ex)
      {
        std::cout << "INVALID ID: " << id << " last buffer is returned!" << std::endl;
        return buffer.back();
      }
    }

    std::vector<std::pair<rclcpp::Time, BufferType>> get_all_time_buffer_pair()
    {
      std::vector<std::pair<rclcpp::Time, BufferType>> pairs;
      ExecutiveMutexLock lock(mutex_);
      for(size_t i = 0; i < buffer.size(); i++)
      {
        pairs.template emplace_back(std::make_pair(time_buffer[i], buffer[i]));
      }
      return pairs;
    }

      std::vector<std::pair<rclcpp::Time, BufferType>> get_all_time_buffer_pair_and_clean()
      {
        std::vector<std::pair<rclcpp::Time, BufferType>> pairs;
        ExecutiveMutexLock lock(mutex_);
        for(size_t i = 0; i < buffer.size(); i++)
        {
          pairs.template emplace_back(std::make_pair(time_buffer[i], buffer[i]));
        }
        clean_();
        return pairs;
      }

    std::vector<BufferType> get_all_buffer() {
      std::vector<BufferType> buffers;
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();
      for (const auto &b: buffer) {
        buffers.template emplace_back(b);
      }
      return buffers;
    }

    std::vector<BufferType> get_all_buffer_and_clean() {
      std::vector<BufferType> buffers;
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();
      for (const auto &b: buffer) {
        buffers.template emplace_back(b);
      }
      clean_();
      return buffers;
    }

    std::atomic<uint64_t> counter = 0;
    uint buffer_size = 3;

  private:
    void clean_() {
      buffer.clear();
      time_buffer.clear();
      duration_buffer.clear();
      buffer_tmp.clear();
      time_buffer_tmp.clear();
      duration_buffer_tmp.clear();
    }
  };
}


#endif //ONLINE_FGO_BUFFER_H
