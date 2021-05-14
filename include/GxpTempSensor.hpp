/*
// Copyright (c) 2021 Hewlett-Packard Development Company, L.P.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#pragma once
#include "sensor.hpp"

#include <boost/asio/streambuf.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/container/flat_map.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

struct GxpTempSensor :
  public Sensor,
  public std::enable_shared_from_this<GxpTempSensor>
{
  public:
    GxpTempSensor(boost::asio::io_service& io,
                  std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
                  sdbusplus::asio::object_server& objectServer,
                  const std::string& name,
                  const std::string& configurationPath,
                  std::vector<thresholds::Threshold>&& thresholdData,
                  const std::string& objectType,
                  const std::string& path);

    ~GxpTempSensor();
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objectServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string path; //sysfs path of hwmon

    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};
