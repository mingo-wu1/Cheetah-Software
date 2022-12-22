#include "behavior_controller.h"

namespace BZL{

/**
 * @description: 
 * @param {*}
 * @return {*}
 */
SwitchNode::SwitchNode(const std::string& name, const BT::NodeConfiguration& config) 
: BaseSwitchNode<BZL::NODE_SIZE>(name, config)
{
  for(auto state : states){
    states_.insert(std::pair<const int, BZL::SequenceNode*>(state, new BZL::SequenceNode(std::to_string(state))));
    addChild(states_.at(state));
  }
}

SwitchNode::~SwitchNode(){
  for(auto& state : states_){
    delete state.second;
  }
}

/**
 * @brief override SwitchNode tick
 * 
 * @return BT::NodeStatus 
 */
// BT::NodeStatus SwitchNode::tick()
// {
//     constexpr const char * case_port_names[9] = {
//       "case_1", "case_2", "case_3", "case_4", "case_5", "case_6", "case_7", "case_8", "case_9"};

//     if( childrenCount() != BZL::NODE_SIZE+1)
//     {
//         throw BT::LogicError("Wrong number of children in SwitchNode; "
//                          "must be (num_cases + default)");
//     }

//     std::string variable;
//     std::string value;
//     int child_index = BZL::NODE_SIZE; // default index;

//     if (getInput("variable", variable)) // no variable? jump to default
//     {
//         // check each case until you find a match
//         for (unsigned index = 0; index < BZL::NODE_SIZE; ++index)
//         {
//             bool found = false;
//             if( index < 9 )
//             {
//                 found = (bool)getInput(case_port_names[index], value);
//             }
//             else{
//                 char case_str[20];
//                 sprintf(case_str, "case_%d", index+1);
//                 found = (bool)getInput(case_str, value);
//             }

//             if (found && variable == value)
//             {
//                 child_index = index;
//                 break;
//             }
//         }
//     }

//     // if another one was running earlier, halt it
//     int next_child_index = running_child_;
//     if(BZL::TransLock::Instance().done == true){
//       next_child_index = child_index;
//     }else{
//       BZL::TransLock::Instance().done = true;
//     }

//     if( running_child_ != -1 && running_child_ != next_child_index)
//     {
//         BZL::TransLock::Instance().done = true;
//         haltChild(running_child_);
//     }

//     auto& selected_child = children_nodes_[next_child_index];
    
//     ret = selected_child->executeTick();

//     if( ret == BT::NodeStatus::RUNNING)
//     {
//         running_child_ = next_child_index;
//     }
//     else{
//         BZL::TransLock::Instance().done = true;
//         haltChildren();
//         running_child_ = -1;
//     }
//     return ret;
// }

/**
 * @description: 
 * @param {*}
 * @return {*}
 */
SwitchConfig::SwitchConfig():prefix_("case_")
{
  input_.insert(std::make_pair("variable", std::string("{") + BZL::FSM_STATE + std::string("}")));
  for(auto state : states){
    Insert(state);
  }

  switch_config_.blackboard = BT::Blackboard::create();
  switch_config_.input_ports = input_;
  switch_config_.blackboard->set(BZL::FSM_STATE, std::to_string(BZL::PASSIVE));
}

void SwitchConfig::Insert(int state){
  input_.insert(std::make_pair(prefix_ + std::to_string(state), std::to_string(state)));
  switch_config_.input_ports = input_;
}

BT::NodeConfiguration SwitchConfig::GetConfig() const
{ 
  return switch_config_;
}

/**
 * @description: 
 * @param {string} &stateStringIn
 * @return {*}
 */
ActionNode::ActionNode(const std::string &stateStringIn) : BT::StatefulActionNode(stateStringIn, {})
{

}

/**
 * @description: 
 * @param {*}
 * @return {*}
 */
GamepadSwitchNode::GamepadSwitchNode(){
  for(auto state : states){
    Insert(state, state);
  }
}

GamepadSwitchNode::~GamepadSwitchNode(){
  stateMap_.clear();
}

void GamepadSwitchNode::Insert(int state, int rc_mode){
  stateMap_.insert(std::make_pair(rc_mode, std::to_string(state)));
}

std::string GamepadSwitchNode::SwitchFSMState(int fsmState){
  if(curState == BZL::PASSIVE)
    switch (fsmState) {
      case BZL::RECOVERY_STAND:
        curState = fsmState;
        break;
      case BZL::STAND_UP:
        curState = fsmState;
        break;
      case BZL::JOINT_PD:
        curState = fsmState;
        break;
  }
  else if(curState == BZL::RECOVERY_STAND){
    switch (fsmState) {
      case BZL::LOCOMOTION:
        curState = fsmState;
        break;
      case BZL::PASSIVE:
        curState = fsmState;
        break;
      case BZL::BALANCE_STAND:
        curState = fsmState;
        break;
      case BZL::BACKFLIP:
        curState = fsmState;
        break;
      case BZL::FRONTJUMP:
        curState = fsmState;
        break;
      case BZL::PRONE:
        curState = fsmState;
        break;
      // case BZL::TWOLEG_STAND:
      //   curState = fsmState;
      //   break;
      // case BZL::HANDSHAKE:
      //   curState = fsmState;
      //   break;
      // case BZL::KANGAROOSWING:
      //   curState = fsmState;
      //   break;
    }
  }
  else if(curState == BZL::STAND_UP){
    switch (fsmState) {
      case BZL::PASSIVE:
        curState = fsmState;
        break;
      case BZL::RECOVERY_STAND:
        curState = fsmState;
        break;
      case BZL::LOCOMOTION:
        curState = fsmState;
        break;
      case BZL::BALANCE_STAND:
        curState = fsmState;
        break;
    }
  }
  else if(curState == BZL::BALANCE_STAND){
    switch (fsmState) {
      case BZL::RECOVERY_STAND:
        curState = fsmState;
        break;
      case BZL::PASSIVE:
        curState = fsmState;
        break;
    }
  }
    else if(curState == BZL::LOCOMOTION){
    switch (fsmState) {
      case BZL::RECOVERY_STAND:
        curState = fsmState;
        break;
      case BZL::PASSIVE:
        curState = fsmState;
        break;
      case BZL::STAND_UP:
        curState = fsmState;
        break;
    }
  }
    else if(curState == BZL::PRONE){
    switch (fsmState) {
      case BZL::PASSIVE:
        curState = fsmState;
        break;
      case BZL::RECOVERY_STAND:
        curState = fsmState;
        break;
    }
  }
    else if(curState == BZL::BACKFLIP){
    switch (fsmState) {
      case BZL::RECOVERY_STAND:
        curState = fsmState;
        break;
      case BZL::PASSIVE:
        curState = fsmState;
        break;
    }
  }
    else if(curState == BZL::FRONTJUMP){
    switch (fsmState) {
      case BZL::RECOVERY_STAND:
        curState = fsmState;
        break;
      case BZL::PASSIVE:
        curState = fsmState;
        break;
    }
  }
  //   else if(curState == BZL::TWOLEG_STAND){
  //   switch (fsmState) {
  //     case BZL::RECOVERY_STAND:
  //       curState = fsmState;
  //       break;
  //     case BZL::PASSIVE:
  //       curState = fsmState;
  //       break;
  //   }
  // }
  //   else if(curState == BZL::HANDSHAKE){
  //   switch (fsmState) {
  //     case BZL::RECOVERY_STAND:
  //       curState = fsmState;
  //       break;
  //     case BZL::PASSIVE:
  //       curState = fsmState;
  //       break;
  //   }
  // }
  //   else if(curState == BZL::KANGAROOSWING){
  //   switch (fsmState) {
  //     case BZL::RECOVERY_STAND:
  //       curState = fsmState;
  //       break;
  //     case BZL::PASSIVE:
  //       curState = fsmState;
  //       break;
  //   }
  // }

  else{}

  return stateMap_.at(curState);
}

/**
 * @description: 
 * @param {ActionNode} *fsm_state
 * @return {*}
 */
SequenceNode::SequenceNode(const std::string& name):BT::SequenceNode(name)
{

}

ActionNodeBase::ActionNodeBase(const std::string& name) 
  : BT::ActionNodeBase(name, {})
{

}

/**
 * @description: 
 * @param {TreeNode} *root_node
 * @return {*}
 */
StatusChangeLogger::StatusChangeLogger(BT::TreeNode *root_node) : BT::StatusChangeLogger(root_node)
{
  bool expected = false;
  if (!ref_count.compare_exchange_strong(expected, true))
  {
    throw BT::LogicError("Only one instance of StatusChangeLogger shall be created");
  }
}

StatusChangeLogger::~StatusChangeLogger()
{
  ref_count.store(false);
}

void StatusChangeLogger::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                      BT::NodeStatus status)
{
  using namespace std::chrono;
  constexpr const char* whitespaces = "                         ";
  constexpr const size_t ws_count = 25;
  double since_epoch = duration<double>(timestamp).count();
  printf("[%.3f]: %s%s %s -> %s \n",
         since_epoch, node.name().c_str(),
         &whitespaces[std::min(ws_count, node.name().size())],
         toStr(prev_status, true).c_str(),
         toStr(status, true).c_str() );
}

void StatusChangeLogger::flush()
{
  std::cout << std::flush;
	ref_count = false;
}

}

