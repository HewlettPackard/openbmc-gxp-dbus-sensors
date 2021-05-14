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

#include "GxpTachSensor.hpp"

#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <math.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <filesystem>
#include <fstream>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

constexpr bool debug = true;

namespace fs = std::filesystem;


static constexpr unsigned int sensorPollMs = 500;
static constexpr unsigned int sensorScaleFactor = 1000;
static constexpr double maxReading = 255;
static constexpr double minReading = 0;

GxpTachSensor::GxpTachSensor(
                            boost::asio::io_service& io,
                            std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
                            sdbusplus::asio::object_server& objectServer,
                            const std::string& sensorName,
                            const std::string& configurationPath,
                            std::vector<thresholds::Threshold>&& thresholdData,
                            const std::string& path,
                            const std::string& objectType,
                            const std::string& fail,
                            const std::string& inst,
                            const PowerState& powerState) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdData), configurationPath,
           objectType, maxReading,
           minReading, powerState),
    objectServer(objectServer), waitTimer(io), path(path),
    inputDev(io, open(path.c_str(), O_RDONLY)), unit("xyz.openbmc_project.Sensor.Value.Unit.Percent"), failLine(gpiod::find_line(fail)), instLine(gpiod::find_line(inst))
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/fan_tach/" + name,
        "xyz.openbmc_project.Sensor.Value");

    sensorInterface->register_property("Unit", unit);

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/fan_tach/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }

    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/fan_tach/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/fan_tach/" + name,
        association::interface);

    if(failLine)
    {
        try{
            failLine.request({"GxpFanSensor", gpiod::line_request::DIRECTION_INPUT, 0});
        }
        catch (std::system_error&)
        {
            std::cerr << "GxpFanSensor error requesting gpio pin name: " << fail << "\n";
        }
    }
    else {
        std::cerr << "Error requesting gpio: " << fail << "\n";
    }

    if(instLine)
    {
        try{
            instLine.request({"GxpFanSensor", gpiod::line_request::DIRECTION_INPUT, 0});
        }
        catch (std::system_error&)
        {
            std::cerr << "GxpFanSensor error requesting gpio pin name: " << inst << "\n";
        }
    }
    else {
        std::cerr << "Error requesting gpio: " << inst << "\n";
    }

    setInitialProperties(dbusConnection);
    setupRead();
}

GxpTachSensor::~GxpTachSensor()
{
    inputDev.close();
    waitTimer.cancel();
    objectServer.remove_interface(thresholdInterfaceWarning);
    objectServer.remove_interface(thresholdInterfaceCritical);
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void GxpTachSensor::setupRead(void) {
    boost::asio::async_read_until(inputDev, readBuf, '\n',
        [&](const boost::system::error_code& ec, std::size_t /*bytes_transfered*/)
        {
            handleResponse(ec);
        }
    );
}

void GxpTachSensor::handleResponse(const boost::system::error_code& err)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        std::cerr << "bad file descriptor\n";
        return; // we're being destroyed
    }

    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        try
        {
            double percent = 0.0;
            bool inst = instLine.get_value();
            bool fail = failLine.get_value();
            if(inst && !fail)
            {
                std::getline(responseStream, response);
                rawValue = std::stod(response);
                percent = 100.0 * (rawValue / maxValue);
            }
            updateValue(percent);
        }
        catch (const std::invalid_argument&)
        {
            std::cerr << "Invalid temp reading\n";
            incrementError();
        }
    }
    else
    {
        std::cerr << "Invalid temp reading\n";
        incrementError();
    }

    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([&](const boost::system::error_code& ec)
    {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        setupRead();
    });
}

void GxpTachSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
