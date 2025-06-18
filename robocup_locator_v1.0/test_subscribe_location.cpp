#include <iostream>

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include "LocationModule.hpp"


int main(int argc, char *argv[]) {

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    std::shared_ptr<unitree::robot::SubscriptionBase<LocationModule::LocationResult>> locState;
    locState = std::make_shared<unitree::robot::SubscriptionBase<LocationModule::LocationResult>>("rt/locationresults");

    while(true) {

        LocationModule::LocationResult location = locState -> msg_;
        std::cout << location.robot2field_x() << ", " << location.robot2field_y() << ", " << location.robot2field_theta() << std::endl;
        
        sleep(0.1);
    }
    return 0;
}