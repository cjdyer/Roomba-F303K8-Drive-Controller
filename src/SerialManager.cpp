#include "SerialManager.h"
#include "Arduino.h"
#include <iterator>

template <typename T>
void SerialManager::queueSend(const T _value, const bool seperator) 
{
    if (seperator) { send_queue_.push_back("-----------------------------"); }
    send_queue_.push_back(String(_value).c_str());
}

void SerialManager::run()
{
    std::vector<std::string>::iterator vector_iterator;
    for (vector_iterator = send_queue_.begin(); vector_iterator < send_queue_.end(); vector_iterator++)
    {
        Serial.println(String((*vector_iterator).c_str()));
    }
}