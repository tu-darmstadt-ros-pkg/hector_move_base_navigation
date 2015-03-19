#ifndef HANDLER_HECTOR_ACTION_GOAL_H_
#define HANDLER_HECTOR_ACTION_GOAL_H_

#include <hector_nav_core/hector_move_base_handler.h>

namespace hector_move_base_handler {

class HectorActionGoalHandler : hector_nav_core::HectorMoveBaseHandler {

    hector_nav_core::RESULT handle();

    void abort();

    void notifyGoalChanged();

    void notifyControllerFeedback();

};

}

#endif
