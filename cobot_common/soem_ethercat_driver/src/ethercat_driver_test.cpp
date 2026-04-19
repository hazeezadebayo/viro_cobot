#include <soem_ethercat_driver/ethercat_driver.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr soem_ethercat_node = rclcpp::Node::make_shared("soem_ethercat_test");
    soem_ethercat_driver::EtherCatDriver ed("enx207bd21fa0da", soem_ethercat_node);
    RCLCPP_INFO(soem_ethercat_node->get_logger(),"soem_ethercat_test start");
    rclcpp::spin(soem_ethercat_node);
}