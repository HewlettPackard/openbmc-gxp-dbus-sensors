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

#include "Utils.hpp"

#include <boost/asio/read_until.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

static constexpr size_t pwmMax = 255;
static constexpr double defaultPwm = 30.0;

GxpPwmSensor::GxpPwmSensor(
                            sdbusplus::asio::object_server& objectServer,
                            const std::string& name,
                            const std::string& configurationPath,
                            const std::string& path) :
    path(path), objectServer(objectServer), name(name)
{
    uint32_t pwmValue = GetValue(false);
    if(!pwmValue)
    {
        pwmValue = static_cast<uint32_t>(pwmMax * (defaultPwm / 100));
        SetValue(pwmValue);
    }

    controlInterface = objectServer.add_interface(
        "/xyz/openbmc_project/control/fan_pwm/" + name,
        "xyz.openbmc_project.Control.FanPwm");

    controlInterface->register_property(
        "Target", static_cast<uint64_t>(pwmValue),
        [this](const uint64_t& req, uint64_t rsp) {
            if(req > pwmMax)
            {
                throw std::runtime_error("Value out of range");
                return -1;
            }
            if(req != rsp)
            {
                SetValue(req);
                rsp = req;
            }
            return 1;
        },
        [this](uint64_t& curVal) {
            uint64_t value = GetValue();
            if(curVal != value)
            {
                curVal = value;
                controlInterface->signal_property("Target");
            }
            return curVal;
        }
    );

    controlInterface->initialize();

    association = objectServer.add_interface(
        "/xyz/openbmc_project/control/fan_pwm/" + name,
        association::interface);
    createAssociation(association, configurationPath);
}

GxpPwmSensor::~GxpPwmSensor()
{
	objectServer.remove_interface(controlInterface);
	objectServer.remove_interface(association);
}

void GxpPwmSensor::SetValue(uint32_t value)
{
	std::ofstream ref(path);
	if(!ref.good())
	{
		throw std::runtime_error("Bad write file");
	}
	ref << value;
}

uint32_t GxpPwmSensor::GetValue(bool errThrow)
{
	std::ifstream ref(path);
	if(!ref.good())
	{
		return -1;
	}

	std::string line;
	if(!std::getline(ref, line))
	{
		return -1;
	}

	try
	{
		return std::stoi(line);
	}
	catch(std::invalid_argument&)
	{
		std::cerr << "Error reading pwm at "<< path << "\n";
		if(errThrow)
		{
			throw std::runtime_error("Bad read");
		}
	}
	return 0;
}
