#include <hector_move_base/hector_move_base_state_machine.h>

namespace hector_move_base {

HectorMoveBaseStateMachine::HectorMoveBaseStateMachine() {
    nextStateMapping = std::map<boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>,
            std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > >();
}

HectorMoveBaseStateMachine::~HectorMoveBaseStateMachine() {

}

void HectorMoveBaseStateMachine::addHandlerMapping(const boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> handler,
                                                   const std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > mappingForHandler) {
    nextStateMapping[handler] = mappingForHandler;
}

void HectorMoveBaseStateMachine::overrideMapping(std::map<boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>,
                                                 std::map<RESULT, boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > > mapping) {
    nextStateMapping = mapping;
}

boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> HectorMoveBaseStateMachine::getNextActionForHandler(
        const boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> handler, const RESULT result) {
    return nextStateMapping[handler][result];
}

void HectorMoveBaseStateMachine::setNextActionForHandler(const boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> handler,
                                                         const RESULT result, const boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> nextHandler) {
    nextStateMapping[handler][result] = nextHandler;
}

}
