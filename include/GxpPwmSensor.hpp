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

#include <boost/asio/streambuf.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>
#include <memory>

class GxpPwmSensor
{
	public:
		GxpPwmSensor(
							sdbusplus::asio::object_server& objectServer,
							const std::string& name,
							const std::string& configurationPath,
							const std::string& path);
		~GxpPwmSensor();

	private:
		sdbusplus::asio::object_server& objectServer;
		std::string name;
		std::string path;

    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> controlInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
		void SetValue(uint32_t value);
		uint32_t GetValue(bool errThrow = true);
};