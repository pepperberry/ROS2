/**
 * Credits
 * - Edouardo Renard
 * - Ezekiel A. Mitchell
*/

#include <iostream>
#include "rclcpp/rclcpp.hpp" // edit c_cpp_properties.json

class MyCustomNode : public rclcpp::Node { // modift name
    public:
        MyCustomNode() : Node("node_name") // modify name
        {
            // source code
            RCLCPP_INFO(this->get_logger(), "Node has started");
        }

    private:
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // modify name
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}