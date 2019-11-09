#include "../comms_utils.h"


namespace comms
{
namespace message
{

// the message type here is just int32_t

template<>   // primary template
constexpr size_t size_of_message<int32_t>()
{
    return sizeof(int32_t);
}

template<>
void serialize<int32_t>(const int32_t &obj, char * const buffer)
{
    // assume same platform
    memcpy(buffer, &obj, sizeof(int32_t));
}

template<>
void deserialize<int32_t>(char const * const buffer, int32_t &obj)
{
    // assume same platform
    memcpy(&obj, buffer, sizeof(int32_t));
}

}
}