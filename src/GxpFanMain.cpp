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

#include "GxpPwmSensor.hpp"
#include "GxpTachSensor.hpp"

#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <math.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
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
    "xyz.openbmc_project.Configuration.GxpFan"};

bool isGxpFanCtrl(const fs::path& parentPath)
{
    fs::path linkPath = parentPath / "device";
    std::string canonical = fs::read_symlink(linkPath);

    if (boost::ends_with(canonical, "c1000c00.fanctrl"))
    {
        std::cerr << "(o)isGxpFanCtrl: " << canonical << "\n";
        return true;
    }
    std::cerr << "(X)isGxpFanCtrl: " << canonical << "\n";
    return false;
}

void createSensors(
    boost::asio::io_service& io,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<GxpPwmSensor>>& pwmSensors,
    boost::container::flat_map<std::string, std::unique_ptr<GxpTachSensor>>& tachSensors,
    size_t retries = 0)
{
    if(!dbusConnection)
    {
        std::cerr << "Connection not created\n";
        return;
    }

    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &dbusConnection, &objectServer, &pwmSensors, &tachSensors] (const ManagedObjectType& configurations) {
            //fanInputNumbers tuple<index, configPath, name, thresholds, sensorType, fail, inst, powerState>
            std::vector<std::tuple<uint8_t, std::string, std::string, std::vector<thresholds::Threshold>, std::string, std::string, std::string, PowerState>> tachNumbers;
            std::vector<std::tuple<uint8_t, std::string, std::string>> pwmNumbers;

            // find matched configuration from Entity-Manager
            for(const auto& [configurationPath, sensorData] : configurations)
            {
                const char* baseType = nullptr;
                const SensorBaseConfiguration* baseConfiguration = nullptr;

                for(const auto type : sensorTypes)
                {
                    auto t = sensorData.find(type);
                    if(t != sensorData.end())
                    {
                        baseConfiguration = &(*t);
                        baseType = type;
                        break;
                    }
                }
                if(baseConfiguration == nullptr)
                {
                    std::cerr << "cound not find baseConfiguration for GxpSensor\n";
                    continue;
                }

                // get threshold data
                std::vector<thresholds::Threshold> thresholds;
                if(!parseThresholdsFromConfig(sensorData, thresholds))
                {
                    std::cerr << "error populating threhsolds for " << configurationPath.str << "\n";
                }

                // get name
                auto findName = baseConfiguration->second.find("Name");
                if(findName == baseConfiguration->second.end())
                {
                    std::cerr << "missing name for " << configurationPath.str << "\n";
                    continue;
                }
                std::string cfgName = std::get<std::string>(findName->second);

                // get power state
                auto powerState = PowerState::on;
                auto findPowerState = baseConfiguration->second.find("PowerState");
                if(findPowerState == baseConfiguration->second.end())
                {
                    auto ptrPower = std::get_if<std::string>(&(findPowerState->second));
                    if(ptrPower)
                    {
                        setReadState(*ptrPower, powerState);
                    }
                }

                std::string cfgFail;
                std::string cfgInst;
                size_t cfgPwm;
                auto connector = sensorData.find(baseType + std::string(".Connector"));
                if(connector != sensorData.end())
                {
                    //get pwm index from connector
                    auto findPwm = connector->second.find("Pwm");
                    if(findPwm == connector->second.end())
                    {
                        std::cerr << "Connector missing pwm\n";
                        continue;
                    }

                    auto findFail = connector->second.find("Fail");
                    if(findFail == connector->second.end())
                    {
                        std::cerr << "Connector missing Fail\n";
                        continue;
                    }
                    cfgFail = std::get<std::string>(findFail->second);

                    auto findInst = connector->second.find("Inst");
                    if(findFail == connector->second.end())
                    {
                        std::cerr << "Connector missing Inst\n";
                        continue;
                    }
                    cfgInst = std::get<std::string>(findInst->second);

                    cfgPwm = std::visit(VariantToUnsignedIntVisitor(), findPwm->second);
                    if constexpr (debug)
                    {
                        std::cerr << "pwm index:" << cfgPwm << " configPath:" << configurationPath.str << "\n";
                    }
                    pwmNumbers.emplace_back(cfgPwm, configurationPath.str, "Pwm_" + std::to_string(cfgPwm));
                }

                //tachNumbers tuple<index, configPath, name, thresholds, sensorType, powerState, fail, inst>
                if constexpr (debug)
                {
                    std::cerr << "tach name:" << cfgName << " configPath:"<< configurationPath.str << " baseType:" << baseType << "\n";
                }
                tachNumbers.emplace_back(cfgPwm, configurationPath.str, cfgName, std::move(thresholds), baseType, cfgFail, cfgInst, powerState);
            }

            std::vector<fs::path> pwms;
            if (findFiles(fs::path("/sys/class/hwmon"), R"(pwm\d+$)", pwms))
            {
                //create fan sensor
                //fanInputNumbers tuple<index, configPath, name, thresholds, sensorType, powerState>
                for(auto& [index, configPath, name, thresholds, objectType, fail, inst, powerState] : tachNumbers)
                {
                    for(auto& pwm : pwms) {
                        if(!isGxpFanCtrl(pwm.parent_path()))
                        {
                            continue;
                        }
                        if(!boost::ends_with(pwm.string(), std::to_string(index))) {
                            continue;
                        }
                        if(tachSensors.find(name) != tachSensors.end())
                        {
                            //has been created
                            continue;
                        }
                        if constexpr (debug)
                        {
                            std::cerr << "create GxpTachSensor index=" << index << "sysPath ="<< pwm.string()<<"\n";
                        }
                        tachSensors.insert(
                            std::pair<std::string, std::unique_ptr<GxpTachSensor>>(
                                name,
                                std::make_unique<GxpTachSensor>(
                                    io, dbusConnection, objectServer, name, configPath, std::move(thresholds), pwm.string(), objectType, fail, inst, powerState
                                )
                            )
                        );
                    }
                }
                //create pwm sensor
                for(auto& pwm : pwms)
                {
                    for(const auto& [index, configPath, name] : pwmNumbers)
                    {
                        if(!isGxpFanCtrl(pwm.parent_path()))
                        {
                            continue;
                        }
                        if(!boost::ends_with(pwm.string(), std::to_string(index)))
                        {
                            continue;
                        }
                        if(pwmSensors.find(pwm.string()) != pwmSensors.end())
                        {
                            //has been created
                            continue;
                        }
                        if constexpr (debug)
                        {
                            std::cerr << "create pwmSensor index=" << index << "sysPath ="<<pwm.string()<<"\n";
                        }
                        pwmSensors.insert(
                            std::pair<std::string, std::unique_ptr<GxpPwmSensor>>(
                                pwm.string(),
                                std::make_unique<GxpPwmSensor>(objectServer, name, configPath, pwm.string())
                            )
                        );
                    }
                }
            }
            else
            {
                std::cerr << "No pwm in system\n";
            }
        }
    );
    getter->getConfiguration(std::vector<std::string>{sensorTypes.begin(), sensorTypes.end()}, retries);
}

int main()
{
    boost::asio::io_service io;
    auto dbusConnection = std::make_shared<sdbusplus::asio::connection>(io);
    dbusConnection->request_name("xyz.openbmc_project.GxpFanSensor");
    sdbusplus::asio::object_server objectServer(dbusConnection);
    boost::container::flat_map<std::string, std::unique_ptr<GxpPwmSensor>> pwmSensors;
    boost::container::flat_map<std::string, std::unique_ptr<GxpTachSensor>> tachSensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;

    io.post([&]()
    {
        createSensors(io, dbusConnection, objectServer, pwmSensors, tachSensors);
        if (pwmSensors.empty())
        {
            std::cout << "Configuration not detected\n";
        }
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler = [&](sdbusplus::message::message& message)
    {
        if (message.is_method_error())
        {
            std::cerr << "callback method error\n";
            return;
        }
        // this implicitly cancels the timer
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
            createSensors(io, dbusConnection, objectServer, pwmSensors, tachSensors, 5);
            if (pwmSensors.empty())
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
