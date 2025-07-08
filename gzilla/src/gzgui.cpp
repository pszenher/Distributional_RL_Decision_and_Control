#include "gzilla/gzgui.hpp"
#include "gzilla/GuiRunner.hh"


#include <gz/common/Console.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/sim/gui/Gui.hh>
#include "gz/sim/InstallationDirectories.hh"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace gzilla
{

  class GzGui::Implementation
{

public:
  std::thread thread;
};

  GzGui::GzGui(const rclcpp::NodeOptions & options)
  : Node("gzgui", options), dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
  {
    this->dataPtr->thread = std::thread(std::bind(&GzGui::OnStart, this));
  }

  GzGui::~GzGui()
  {
    if (this->dataPtr->thread.joinable()) {
      this->dataPtr->thread.join();
    }
  }

  std::unique_ptr<gz::gui::Application> GzGui::CreateGui( std::string worldName ) {
    static std::array argv{ const_cast<char *>("gz-sim-gui") };
    static int argc = argv.size();

    RCLCPP_INFO(this->get_logger(), "Pre-app");
    
    auto app =
      std::make_unique<gz::gui::Application>(argc, argv.data(),
					     gz::gui::WindowType::kDialog,
					     // gz::gui::WindowType::kMainWindow,
					     nullptr /* _renderEngineGuiApiBackend */);

    RCLCPP_INFO(this->get_logger(), "Post-app");
    
    app->AddPluginPath(gz::sim::getGUIPluginInstallDir());
    app->Engine()->addImportPath(gz::sim::getGUIPluginInstallDir().c_str());

    RCLCPP_INFO(this->get_logger(), "Plugins+info done");
    
    // std::string packageName = "gz_sim_vendor";
    // std::string defaultConfig;
    // std::string configPrefix = ament_index_cpp::get_package_prefix( packageName );
    
    // if (!ament_index_cpp::get_resource( "vendor_packages", packageName, defaultConfig, &configPrefix )) {
    //   RCLCPP_ERROR(this->get_logger(),
    // 		   "Failed resolve ROS package '%s', cannot load default 'gui.config' file",
    // 		   packageName.c_str());
    //   return nullptr;
    // }

    // // FIXME: last component should generalize over version (currently hardcodes `gz-sim8`)
    // defaultConfig = configPrefix + "/" + defaultConfig + "/share/gz/gz-sim8/gui/gui.config";

    std::string defaultConfig = ament_index_cpp::get_package_share_directory("gzilla") + "/config/gui_minimal.config";

    RCLCPP_INFO(this->get_logger(), "Default config resolved");
    
    // gz::common::env(GZ_HOMEDIR, defaultConfig);
    // defaultConfig = gz::common::joinPaths(defaultConfig, ".gz", "sim", GZ_SIM_MAJOR_VERSION_STR, "gui.config");

    app->SetDefaultConfigPath(defaultConfig);

    // auto mainWin = app->findChild<gz::gui::MainWindow *>();
    // RCLCPP_INFO(this->get_logger(), "queried window");

    
    // auto win = mainWin->QuickWindow();
    // win->setProperty("title", "Gazebo Sim GUI");
    // RCLCPP_INFO(this->get_logger(), "set quickwindow title");

    if (!app->CreateMainWindow()) {
      RCLCPP_ERROR(this->get_logger(),
		   "Failed to initialize main gazebo window");
      return nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "main window init");

    // Skip all the negotiation nonsense, just load the default config
    if (!app->LoadConfig(defaultConfig)) {
      RCLCPP_ERROR(this->get_logger(),
		   "Failed load config file '%s'",
		   defaultConfig.c_str());
      return nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "loaded config");

    auto runner = new gz::sim::GuiRunner(worldName);
    RCLCPP_INFO(this->get_logger(), "spawned runner");
    runner->setParent(gz::gui::App());
    RCLCPP_INFO(this->get_logger(), "set runner parent");

    RCLCPP_INFO(this->get_logger(), "returning");
    return app;
  }
  
  void GzGui::OnStart()
  {
    auto world_name = this->declare_parameter("world_name", "");
    auto verbosity  = this->declare_parameter("verbosity", 1);

    if (world_name.empty()) {
      RCLCPP_ERROR( this->get_logger(),
		    "Must specify 'world_name'" );
      rclcpp::shutdown();
      return;
    }

    if (verbosity < 1 || 5 < verbosity) {
      auto input_verbosity = verbosity;
      verbosity = std::clamp(verbosity, 1l, 5l);
      RCLCPP_WARN(this->get_logger(),
		  "Out-of-range verbosity level provided (verbosity=%ld);  clamping to nearest:  %ld",
		  input_verbosity,
		  verbosity);
    }
    gz::common::Console::SetVerbosity(verbosity);

    // TODO: generalize default world name
    auto app = GzGui::CreateGui( world_name );
    
    if (!app) {
      RCLCPP_ERROR(this->get_logger(),
		   "Failed to create Gazebo GUI instance");
      rclcpp::shutdown();
      return;
    }

    // Execute GUI Qt application
    // Blocks until GUI window closure or SIGINT
    app->exec();
    rclcpp::shutdown();
  }
  
}  // namespace gzilla

RCLCPP_COMPONENTS_REGISTER_NODE(gzilla::GzGui)
