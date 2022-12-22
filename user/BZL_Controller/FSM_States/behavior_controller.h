#ifndef BEHAVIOR_CONTROLLER_H
#define BEHAVIOR_CONTROLLER_H

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

/*
#define K_PASSIVE 0
#define K_STAND_UP 1
#define K_BALANCE_STAND 3
#define K_LOCOMOTION 4
#define K_LOCOMOTION_TEST 5
#define K_RECOVERY_STAND 6
#define K_VISION 8
#define K_BACKFLIP 9
#define K_FRONTJUMP 11
#define K_PRONE 12
#define K_JOINT_PD 51
#define K_IMPEDANCE_CONTROL 52
#define K_INVALID 100

enum class FSM_StateName {
  PASSIVE,
  STAND_UP,
  BALANCE_STAND,
  RECOVERY_STAND,
  PRONE,
  LOCOMOTION,
  BACKFLIP,
  FRONTJUMP,
  VISION,
  JOINT_PD,
  IMPEDANCE_CONTROL,
  INVALID,
};

namespace RC_mode{
  constexpr int OFF = 0;
  constexpr int STAND_UP = 1;
  constexpr int QP_STAND = 3;
  constexpr int BACKFLIP_PRE = 4;
  constexpr int BACKFLIP = 5;
  constexpr int VISION = 6;
  constexpr int LOCOMOTION = 11;
  constexpr int RECOVERY_STAND = 12;
  constexpr int PRONE = 14;
  constexpr int TWO_LEG_STANCE_PRE = 20;
  constexpr int TWO_LEG_STANCE = 21;
};
*/

// Normal robot states
#define K_PASSIVE 0
#define K_STAND_UP 1
#define K_BALANCE_STAND 3
#define K_LOCOMOTION 4
#define K_LOCOMOTION_TEST 5
#define K_RECOVERY_STAND 6
#define K_VISION 8
#define K_BACKFLIP 9
#define K_FRONTJUMP 11
#define K_PRONE 12
// Specific control states
#define K_JOINT_PD 51
#define K_IMPEDANCE_CONTROL 52
#define K_INVALID 100

namespace BZL{

const std::string FSM_STATE = "fsm_state";

const int STAND_UP = 1;
const int BALANCE_STAND = 2;
const int RECOVERY_STAND = 3;
const int PRONE = 4;
const int LOCOMOTION = 5;
const int BACKFLIP = 6;
const int FRONTJUMP = 7;
const int VISION = 8;
const int JOINT_PD = 9;
const int IMPEDANCE_CONTROL = 10;
const int PASSIVE = 100;

static const int NODE_SIZE = 10;
const int states[NODE_SIZE+1] = 
{
  STAND_UP, //1
  BALANCE_STAND, //2
  RECOVERY_STAND, //3
  PRONE, //4
  LOCOMOTION, //5
  BACKFLIP, //6
  FRONTJUMP, //7
  VISION, //8
  JOINT_PD, //9
  IMPEDANCE_CONTROL, //10
  PASSIVE //100
};

/**
 * @brief Singleton of lock for hehavior transform
 * 
 */
class TransLock
{
public:
    ~TransLock()=default;
    TransLock(const TransLock&)=delete;
    TransLock& operator=(const TransLock&)=delete;
    static TransLock& Instance(){
        static TransLock instance;
        return instance;
    }
    bool done{true};
private:
    TransLock()=default;
};

class SequenceNode : public BT::SequenceNode
{
  public:
    SequenceNode(const std::string& name);
};

class ActionNodeBase : public BT::ActionNodeBase{
  public:
    ActionNodeBase(const std::string& name);
};

template <size_t NUM_CASES>
class BaseSwitchNode : public BT::ControlNode
{
  public:
    BaseSwitchNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ControlNode::ControlNode(name, config ),
      running_child_(-1)
    {
        setRegistrationID("Switch");
    }

    virtual ~BaseSwitchNode() override = default;

    void halt() override
    {
        running_child_ = -1;
        BT::ControlNode::halt();
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports;
        ports.insert( BT::InputPort<std::string>("variable") );
        for(unsigned i=0; i < NUM_CASES; i++)
        {
            char case_str[20];
            sprintf(case_str, "case_%d", i+1);
            ports.insert( BT::InputPort<std::string>(case_str) );
        }
        return ports;
    }

private:
    int running_child_;
    virtual BT::NodeStatus tick() override;
    BT::NodeStatus ret{BT::NodeStatus::SUCCESS};
};

template<size_t NUM_CASES> inline
BT::NodeStatus BaseSwitchNode<NUM_CASES>::tick()
{
    constexpr const char * case_port_names[9] = {
      "case_1", "case_2", "case_3", "case_4", "case_5", "case_6", "case_7", "case_8", "case_9"};

    if( childrenCount() != NUM_CASES+1)
    {
        throw BT::LogicError("Wrong number of children in SwitchNode; "
                         "must be (num_cases + default)");
    }

    std::string variable;
    std::string value;
    int child_index = NUM_CASES; // default index;

    if (getInput("variable", variable)) // no variable? jump to default
    {
        // check each case until you find a match
        for (unsigned index = 0; index < NUM_CASES; ++index)
        {
            bool found = false;
            if( index < 9 )
            {
                found = (bool)getInput(case_port_names[index], value);
            }
            else{
                char case_str[20];
                sprintf(case_str, "case_%d", index+1);
                found = (bool)getInput(case_str, value);
            }

            if (found && variable == value)
            {
                child_index = index;
                break;
            }
        }
    }

    // if another one was running earlier, halt it
    int next_child_index = running_child_;
    if(BZL::TransLock::Instance().done == true){
      next_child_index = child_index;
    }else{
      BZL::TransLock::Instance().done = true;
    }

    if( running_child_ != -1 && running_child_ != next_child_index)
    {
        BZL::TransLock::Instance().done = true;
        haltChild(running_child_);
    }

    auto& selected_child = children_nodes_[next_child_index];
    
    ret = selected_child->executeTick();

    if( ret == BT::NodeStatus::RUNNING)
    {
        running_child_ = next_child_index;
    }
    else{
        BZL::TransLock::Instance().done = true;
        haltChildren();
        running_child_ = -1;
    }
    return ret;
}

class SwitchNode : public BaseSwitchNode<BZL::NODE_SIZE>{
  public:
    SwitchNode(const std::string& name, const BT::NodeConfiguration& config);

    ~SwitchNode();

  protected:
    std::map<const int, BZL::SequenceNode*, std::less<const int> > states_;
};

class SwitchConfig{
  public:
    SwitchConfig();

    BT::NodeConfiguration GetConfig() const;
  
  private:
    void Insert(int state);

  private:
    BT::NodeConfiguration switch_config_;
    BT::PortsRemapping input_;
    std::string prefix_;
};

class ActionNode : public BT::StatefulActionNode{
  public:
    ActionNode(const std::string &stateStringIn);
};

class GamepadSwitchNode{
  public:
    GamepadSwitchNode();

    ~GamepadSwitchNode();

    std::string SwitchFSMState(int fsmState);

  private:
    void Insert(int state, int rc_mode);

  private:
    std::map<int, std::string> stateMap_;
    int curState{BZL::PASSIVE};
};

static std::atomic<bool> ref_count(false);
class StatusChangeLogger : public BT::StatusChangeLogger
{
  public:
    StatusChangeLogger(BT::TreeNode *root_node);

    ~StatusChangeLogger();

    virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                          BT::NodeStatus status) override final;

    virtual void flush() override final;
};

}
#endif  // BEHAVIOR_CONTROLLER_H
