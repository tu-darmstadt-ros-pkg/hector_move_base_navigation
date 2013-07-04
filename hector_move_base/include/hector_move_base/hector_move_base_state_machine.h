#ifndef HECTOR_MOVE_BASE_STATE_MACHINE_H_
#define HECTOR_MOVE_BASE_STATE_MACHINE_H_

#include <hector_nav_core/hector_move_base_handler.h>

#include <boost/shared_ptr.hpp>

#include <deque>
#include <map>

namespace hector_move_base {

class HectorMoveBaseStateMachine {
private:
    std::map<boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>,
    std::map<RESULT,boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > > nextStateMapping;

public:
    HectorMoveBaseStateMachine();
    virtual ~HectorMoveBaseStateMachine();

    void addHandlerMapping(const boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>,
                           const std::map<RESULT,boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> >);

    void overrideMapping(std::map<boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>,
                         std::map<RESULT,boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> > >);

    boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> getNextActionForHandler(const boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>, const RESULT);

    void setNextActionForHandler(const boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>,
                                 const RESULT, const boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler>);

};

}

#endif
