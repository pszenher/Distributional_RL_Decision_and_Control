
namespace virelex
{
  class ActionPlanner {
  
    virtual void reachedGoal(const double * goal_dist_x, const double * goal_dist_y);

  private:

    double goal_threshold{ 2.0 };
  
  }
} // namespace virelex
