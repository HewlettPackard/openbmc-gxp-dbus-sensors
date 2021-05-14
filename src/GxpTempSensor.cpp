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

#include "GxpTempSensor.hpp"

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

static constexpr std::array<const char*, 1> sensorTypes = {
    "xyz.openbmc_project.Configuration.GxpTempSensor"};
static constexpr unsigned int sensorPollMs = 500;
static constexpr unsigned int sensorScaleFactor = 1000;
static constexpr double maxReading = 127;
static constexpr double minReading = -127;

GxpTempSensor::GxpTempSensor(
                            boost::asio::io_service& io,
                            std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
                            sdbusplus::asio::object_server& objectServer,
                            const std::string& sensorName,
                            const std::string& configurationPath,
                            std::vector<thresholds::Threshold>&& thresholdData,
                            const std::string& objectType,
                            const std::string& path) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdData), configurationPath,
           objectType, maxReading,
           minReading),
    std::enable_shared_from_this<GxpTempSensor>(),
    objectServer(objectServer), waitTimer(io), path(path),
    inputDev(io, open(path.c_str(), O_RDONLY))
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }

    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        association::interface);

    setInitialProperties(dbusConnection);
}

GxpTempSensor::~GxpTempSensor()
{
    inputDev.close();
    waitTimer.cancel();
    objectServer.remove_interface(thresholdInterfaceWarning);
    objectServer.remove_interface(thresholdInterfaceCritical);
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void GxpTempSensor::setupRead(void) {
    std::weak_ptr<GxpTempSensor> weakRef = weak_from_this();
    boost::asio::async_read_until(inputDev, readBuf, '\n',
        [weakRef](const boost::system::error_code& ec, std::size_t /*bytes_transfered*/)
        {
            std::shared_ptr<GxpTempSensor> self = weakRef.lock();
            if (self)
            {
                self->handleResponse(ec);
            }
        }
    );
}

void GxpTempSensor::handleResponse(const boost::system::error_code& err)
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
        std::getline(responseStream, response);
        try
        {
            double nvalue = std::stod(response);
            nvalue /= sensorScaleFactor;
            updateValue(nvalue);
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
    std::weak_ptr<GxpTempSensor> weakRef = weak_from_this();
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec)
    {
        std::shared_ptr<GxpTempSensor> self = weakRef.lock();
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        if (self)
        {
            self->setupRead();
        }
    });
}

void GxpTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

bool isGxpTemp(const fs::path& parentPath)
{
    std::string devicePath = fs::read_symlink(parentPath / "device").filename();
    if (boost::ends_with(devicePath, "c0000130.coretemp")) {
        return true;
    }
    std::cerr << devicePath << "is not matched gxptempsensor\n";
    return false;
}

void createSensor(
    boost::asio::io_service& io,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<GxpTempSensor>& sensor)
{
    if (!dbusConnection)
    {
        std::cerr << "dbusConnection not created\n";
        return;
    }

    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &dbusConnection, &objectServer, &sensor](const ManagedObjectType& sensorConfigurations){
            //find all temperature hwmon
            std::vector<fs::path> temps;
            if (!findFiles(fs::path("/sys/class/hwmon"), R"(temp\d+_input)", temps))
            {
                std::cerr << "No temperature sensors in system\n";
                return;
            }

            //iterate all temp hwmon to match configuration
            for (auto& path : temps)
            {
                if (!isGxpTemp(path.parent_path())) {
                    continue;
                }
                //validate configruation
                const SensorData* sensorData = nullptr;
                const std::string* configurationPath = nullptr;
                const char* sensorType = nullptr;
                const SensorBaseConfiguration* baseConfiguration = nullptr;
                const SensorBaseConfigMap* baseConfigMap = nullptr;
                for (const std::pair<sdbusplus::message::object_path, SensorData>& sensorConfiguration : sensorConfigurations)
                {
                    sensorData = &(sensorConfiguration.second);
                    for (const char* type : sensorTypes)
                    {
                        auto sensorBase = sensorData->find(type);
                        if (sensorBase != sensorData->end()) {
                            baseConfiguration = &(*sensorBase);
                            sensorType = type;
                            break;
                        }
                    }
                    if (baseConfiguration == nullptr)
                    {
                        std::cerr << "error finding base configruation for GxpTemp\n";
                        continue;
                    }
                    baseConfigMap = &(baseConfiguration->second);
                    configurationPath = &(sensorConfiguration.first.str);
                    break;
                }
                if (configurationPath == nullptr)
                {
                    std::cerr << "failed to find match for GxpTemp\n";
                    continue;
                }

                //find name
                auto findName = baseConfigMap->find("Name");
                if(findName == baseConfigMap->end()) {
                    std::cerr << "could not determine configruation name for GxpTemp\n";
                    continue;
                }
                std::string sensorName = std::get<std::string>(findName->second);

                //find threshold data
                std::vector<thresholds::Threshold> sensorThresholds;
                if (!parseThresholdsFromConfig(*sensorData, sensorThresholds)) {
                    std::cerr << "error populating thresholds for GxpTemp\n";
                }

                if constexpr (debug)
                {
                    std::cerr
                    << "Configuration parsed for GxpTempSensor\n\t"
                    << "\n"
                    << "with\n"
                    << "\tName: " << sensorName << "\n"
                    << "\tPath: " << path.string() << "\n"
                    << "\tsensorType: " << sensorType << "\n"
                    << "\tconfigurationPath: " << *configurationPath << "\n";
                }

                //discard created sensor
                sensor = nullptr;

                //create dbus sensor for matched hwmon
                sensor = std::make_shared<GxpTempSensor>(
                            io, dbusConnection, objectServer, sensorName, *configurationPath,
                            std::move(sensorThresholds), sensorType,
                            path.string());

                sensor->setupRead();
            }
        }
    );
    getter->getConfiguration(std::vector<std::string>(sensorTypes.begin(), sensorTypes.end()));
}

int main()
{
    boost::asio::io_service io;
    auto dbusConnection = std::make_shared<sdbusplus::asio::connection>(io);
    dbusConnection->request_name("xyz.openbmc_project.GxpTempSensor");
    sdbusplus::asio::object_server objectServer(dbusConnection);
    std::shared_ptr<GxpTempSensor> sensor = nullptr;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;

    io.post([&]()
    {
        createSensor(io, dbusConnection, objectServer, sensor);
        if (sensor == nullptr)
        {
            std::cout << "Configuration not detected\n";
        }
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)>
    eventHandler = [&](sdbusplus::message::message& message)
    {
        if (message.is_method_error())
        {
            std::cerr << "callback method error\n";
            return;
        }
        // this implicitly  cancels the timer
        filterTimer.expires_from_now(boost::posix_time::seconds(1));

        // create a timer because normally multiple properties change
        filterTimer.async_wait([&](const boost::system::error_code& ec)
        {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }
            else if (ec)
            {
                std::cerr << "timer error\n";
                return;
            }
            createSensor(io, dbusConnection, objectServer, sensor);
            if (sensor == nullptr)
            {
                std::cout << "Configuration not detected\n";
            }
        });
    };

    for (const char* type : sensorTypes)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*dbusConnection),
            "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + type + "'",
            eventHandler);
        matches.emplace_back(std::move(match));
    }
    io.run();
}
