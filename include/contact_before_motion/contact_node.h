#pragma once

#include <graph_search/node.h>
#include <contact_before_motion/contact_state.h>

namespace contact_before_motion{
  class ContactNode : public graph_search::Node {
  public:
    const ContactState& state() const {return state_;}
    ContactState& state() {return state_;}
  private:
    ContactState state_;
  };
}
