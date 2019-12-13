#pragma once

#include <cstdint>

namespace comms
{

// Using template specification for each message type
// TODO: Do std::is_fundamental + std::enable_if. But it will definitely confuse people
// TODO: We can make it recursive
namespace message
{

template<typename T>   // primary template
constexpr uint32_t size_of_message()
{
    assert(false && strcat("template specification not implement for",typeid(T).name()));
    return 0;
}

template<typename T>
void serialize(const T &obj, char * const buffer)
{
    assert(false && strcat("template specification not implement for",typeid(T).name()));
}

template<typename T>
void deserialize(char const * const buffer, T &obj)
{
    assert(false && strcat("template specification not implement for",typeid(T).name()));  
}

} // message

} // comms