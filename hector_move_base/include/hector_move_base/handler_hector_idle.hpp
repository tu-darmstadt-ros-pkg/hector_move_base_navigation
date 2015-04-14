#ifndef HANDLER_HECTOR_IDLE_H_
#define HANDLER_HECTOR_IDLE_H_

#include <hector_nav_core/hector_move_base_handler.h>

namespace hector_move_base_handler {

class HectorIdleHandler : public HectorMoveBaseHandler {
public:
    HectorIdleHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface){
	
    }

    hector_move_base::RESULT handle()
    {
        return hector_move_base::NEXT;
    }

    void abort(){
    }
};
}
#endif
