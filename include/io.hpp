#pragma once

#include <string>
#include <thread>
#include <unordered_map>

#include "can.hpp"
#include "utils.hpp"

using CAN = IO::Can_interface;

namespace IO
{
    extern std::vector<std::thread> io_handles;
    template<typename T>
    class IO
    {
       private:
        std::unordered_map<std::string, T*> data;

       public:
        T* operator[](const std::string& key) const {
            auto p = data.find(key);
            if (p == data.end()) {
                LOG_ERR("IO error: no device named %s\n", key.c_str());
                return nullptr;
            }
            return p->second;
        }

        void insert(T& device) {
            auto& p = data[device.name];
            if (p != nullptr) {
                LOG_ERR("IO error: double register device named %s\n", device.name.c_str());
            }
            p = &device;
            io_handles.emplace_back(std::thread([&]() { device.task(); }));
        }
    };

    template<typename T>
    IO<T> io;
}  // namespace IO
