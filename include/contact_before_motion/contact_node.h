#pragma once

#include <graph_search/node.h>
#include <contact_before_motion/contact_state.h>

namespace contact_before_motion{
  class ContactNode : public graph_search::Node {
  public:
    const ContactState& state() const {return state_;}
    ContactState& state() {return state_;}
    const unsigned int& level() const {return level_;}
    unsigned int& level() {return level_;}
  private:
    ContactState state_;
    unsigned int level_ = 0;
  };
}
