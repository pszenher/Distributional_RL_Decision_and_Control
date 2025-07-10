#pragma once

#include <gz/gui/Application.hh>
#include <gz/utils/ImplPtr.hh>
#include <rclcpp/node.hpp>

namespace gzilla
{

class GzGui : public rclcpp::Node
{
public:
  explicit GzGui(const rclcpp::NodeOptions & options);
  ~GzGui();
  void OnStart();

  std::unique_ptr<gz::gui::Application> CreateGui(std::string world_name);

private:
  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};

}  // namespace gzilla
