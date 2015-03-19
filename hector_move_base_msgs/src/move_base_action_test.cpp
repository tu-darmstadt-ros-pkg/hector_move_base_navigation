#include <hector_move_base_msgs/move_base_action.h>

using namespace hector_move_base_msgs;

int main(int argc, char **argv) {
  MoveBaseActionGoal action;

  setAction(action, MoveBaseGoal());
  if (hasType<MoveBaseGoal>(action)) {
    std::cout << "Action is a Goal action: " << *getAction<MoveBaseGoal>(action) << std::endl;
  }

  setAction(action, MoveBasePath());
  if (hasType<MoveBasePath>(action)) {
    std::cout << "Action is a Path action: " << *getAction<MoveBasePath>(action) << std::endl;
  }

  setAction(action, MoveBaseExplore());
  if (hasType<MoveBaseExplore>(action)) {
    std::cout << "Action is a Explore action: " << *getAction<MoveBaseExplore>(action) << std::endl;
  }

  return 0;
}
