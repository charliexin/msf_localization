
#include "msf_localization/data_process/data_process_flow.hpp"

using namespace msf_localization;

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "data_process_node");

    std::shared_ptr<DataProcessFlow> data_process_flow_ptr;
    data_process_flow_ptr = std::make_shared<DataProcessFlow>();

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_process_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}
