#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>

static void sleep_ms(int ms){ std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

int main(int argc, char** argv){
    // 사용법:
    //   go2_grand_ICT_v3 move vx vy vyaw
    //   go2_grand_ICT_v3 stop
    //   go2_grand_ICT_v3 scrape
    //   go2_grand_ICT_v3 sit
    //   go2_grand_ICT_v3 hello
    if(argc < 2){
        std::cerr << "Usage: " << argv[0] << " {move vx vy vyaw|stop|scrape|sit|hello}\n";
        return -1;
    }

    std::string cmd = argv[1];

    // DDS 초기화 (eth0 고정 – 본 환경 가정)
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

    unitree::robot::go2::SportClient sport;
    sport.SetTimeout(2.0f);
    sport.Init();

    // (선택) 조명/음향 제어가 필요하면 VUI 사용
    unitree::robot::go2::VuiClient vui;
    vui.SetBrightness(0);

    auto ok = [](int32_t r){ return r==0; };

    if(cmd == "move"){
        if(argc < 5){
            std::cerr << "Usage: " << argv[0] << " move vx vy vyaw\n";
            return -2;
        }
        float vx = std::stof(argv[2]);
        float vy = std::stof(argv[3]);
        float vyaw = std::stof(argv[4]);

        // 항상 균형기립에서 이동
        sport.BalanceStand();
        sleep_ms(50);
        sport.Move(vx, vy, vyaw);
        return 0;
    }
    else if(cmd == "stop"){
        // 이동 정지 + 균형기립으로 안정화
        sport.Move(0.f, 0.f, 0.f);
        sleep_ms(50);
        sport.BalanceStand();
        return 0;
    }
    else if(cmd == "scrape"){
        // 인사(인사각) 동작이 SDK에 Scrape()로 노출되어 있다고 가정
        int32_t r = sport.Scrape();
        std::cout << "Scrape ret=" << r << std::endl;
        return ok(r) ? 0 : r;
    }
    else if(cmd == "sit"){
        // 앉기
        int32_t r = sport.Sit();
        std::cout << "Sit ret=" << r << std::endl;
        return ok(r) ? 0 : r;
    }
    else if(cmd == "hello"){
        int32_t r = sport.Hello();
        std::cout << "Hello ret=" << r << std::endl;
        return ok(r) ? 0 : r;
    }
    else{
        std::cerr << "Unknown cmd: " << cmd << std::endl;
        return -3;
    }
}
