// Include the header file for the controller and add the namespace definition
#include "zoe2_controllers/passive_steering_drive_controller.hpp"
namespace passive_steering_drive_controller
{
// Implement the init method
controller_interface::CallbackReturn PassiveSteeringDriveController::on_init()
{
    try
    {
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch(const std::exception& e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

// Implement the on_configure method. Parameters are usually read here, and everything is prepared so that the controller can be started.
controller_interface::CallbackReturn PassiveSteeringDriveController::on_configure(const rclcpp_lifecycle::State &previous_state)
{

}

// Implement command_interface_configuration and state_interface_configuration where required interfaces are defined.
controller_interface::InterfaceConfiguration PassiveSteeringDriveController::command_interface_configuration() const
{

}

controller_interface::InterfaceConfiguration PassiveSteeringDriveController::state_interface_configuration() const
{

}

// Implement the on_activate method. This is where the controller is started.
controller_interface::CallbackReturn PassiveSteeringDriveController::on_activate(const rclcpp_lifecycle::State &previous_state)
{


}

// Implement the on_deactivate method. This is where the controller is stopped.
controller_interface::CallbackReturn PassiveSteeringDriveController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{


}

// Implement the update method. This is where the controller logic is implemented.
controller_interface::CallbackReturn PassiveSteeringDriveController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{


}

} // namespace passive_steering_drive_controller
// Add the macro to export the controller
#include "class_loader/register_macro.hpp"
CLASS_LOADER_REGISTER_CLASS(
    passive_steering_drive_controller::PassiveSteeringDriveController, controller_interface::ControllerInterface)