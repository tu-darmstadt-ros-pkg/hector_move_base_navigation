#ifndef HECTOR_NAV_CORE_MOVE_BASE_HANDLER_H_
#define HECTOR_NAV_CORE_MOVE_BASE_HANDLER_H_

#include <hector_nav_core/hector_move_base_interface.h>
#include <hector_move_base_msgs/MoveBaseActionResult.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <map>

namespace hector_move_base_handler {

/**
 * @abstract HectorMoveBaseHandler
 * @brief Provides an abstract class for implementation of HectorMoveBase modules
 */
class HectorMoveBaseHandler
{
protected:
    hector_move_base::IHectorMoveBase* hectorMoveBaseInterface;

    HectorMoveBaseHandler(hector_move_base::IHectorMoveBase* interface) {
        hectorMoveBaseInterface = interface;
    }

public:
    virtual hector_move_base::RESULT handle() = 0;

    virtual void abort() = 0;
};

}

#endif
