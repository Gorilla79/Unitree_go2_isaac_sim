#include <iostream>
#include <unistd.h>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#define TOPIC_LOWSTATE "rt/lowstate"

using namespace unitree::robot;

void LowStateHandler(const void* message)
{
    auto state = *(unitree_go::msg::dds_::LowState_*)message;

    // 각 다리의 각도 및 속도 출력 (12개 관절)
    for (int i = 0; i < 12; ++i)
    {
        float pos = state.motor_state()[i].q();   // 각도 (rad)
        float vel = state.motor_state()[i].dq();  // 속도 (rad/s)
        std::cout << "Joint " << i << " -> q: " << pos << ", dq: " << vel << std::endl;
    }

    std::cout << "=== end ===" << std::endl;
}

int main()
{
    std::string netInterface = "eth0";  
    ChannelFactory::Instance()->Init(0, netInterface);

    ChannelSubscriber<unitree_go::msg::dds_::LowState_> suber(TOPIC_LOWSTATE);
    suber.InitChannel(LowStateHandler);

    while (true)
    {
        usleep(20000);
    }

    return 0;
}
