class RLActionPlanner : public ActionPlanner
{
public:
  RL_agent(const std::string& robot_name, const std::string& model_path, const std::string& agent_type)
  : ActionPlannerNode(robot_name),
    agent_type(agent_type)
  ;

private:
  void actionPlanningCallback(const virelex_msgs::msg::State::SharedPtr msg) override;
  
  void computeAction(const virelex_msgs::msg::State::SharedPtr msg);

  static bool compareByDistance(const std::vector<double>& v1, const std::vector<double>& v2);

  torch::jit::script::Module model;

  // Max number of objects spawned, to be used in querying torch model
  int max_obj_num;

  // Scaling factor applied to thrust actions received from agent
  double action_scale;

  // String name of RL agent type, one of:  { "AC-IQN", "IQN", "DDPG", "DQN", "SAC", "Rainbow" }
  std::string agent_type;

};



class DiscreteRLActionPlanner : public RLActionPlanner
{
public:
  
private:
  // Actions of discrete control agents:  {"IQN", "DQN", "Rainbow"}
  // Left thruster discrete change value set
  std::vector<float> left_thrust_change;
  // Right thruster discrete change value set
  std::vector<float> right_thrust_change;
  // Permutation set of left/right thrust valueset pairs from above
  std::vector<std::vector<float>> actions;
  // Index into `actions` vector representing current discrete action
  int current_action_d;
};

class ContinuousRLActionPlanner : public RLActionPlanner
{
public:
private:
  // Actions of continuous control agents:  { "AC-IQN", "DDPG", "SAC" }
  // ================================================================
  // Scalar bias applied to continuous thrust actions of "AC-IQN", "DDPG", "SAC" agent types
  float action_mean;
  // Scaling factor applied to continuous thrust actions of "AC-IQN", "DDPG", "SAC" agent types
  float action_amplitude;
  // Current thrust action
  std::vector<float> current_action_c; 

};

class RainbowActionPlanner : public DiscreteRLActionPlanner
{
private:
  // parameters of the Rainbow agent
  int atoms;
  double Vmin;
  double Vmax;
  torch::Tensor support; 
}
