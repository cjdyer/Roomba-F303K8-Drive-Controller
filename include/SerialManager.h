#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include <vector>
#include <string>

class SerialManager
{
public:
    template <typename T> void queueSend(const T _value, const bool _seperator = false);
    void run();

private:
    static std::vector<std::string> send_queue_;
};

#endif