#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include <unistd.h>

#define test 1

using BT::NodeStatus;
using std::chrono::milliseconds;
using namespace BT;

namespace BT
{
static std::atomic<bool> ref_count(false);

class MyStdCoutLogger : public StatusChangeLogger
{

  public:
    MyStdCoutLogger(TreeNode *root_node) : StatusChangeLogger(root_node)
    {
        bool expected = false;
        if (!ref_count.compare_exchange_strong(expected, true))
        {
            throw LogicError("Only one instance of StdCoutLogger shall be created");
        }
    }

    ~MyStdCoutLogger() override
    {
        ref_count.store(false);
    }


    virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status) override
    {
        using namespace std::chrono;

        constexpr const char* whitespaces = "                         ";
        constexpr const size_t ws_count = 25;

        double since_epoch = duration<double>(timestamp).count();
        printf("[%.3f]: %s%s %s -> %s",
               since_epoch, node.name().c_str(),
               &whitespaces[std::min(ws_count, node.name().size())],
               toStr(prev_status, true).c_str(),
               toStr(status, true).c_str() );
        std::cout << std::endl;
    }

    virtual void flush() override
    {
        std::cout << std::flush;
    	ref_count = false;
    }
};

}   // end namespace

namespace BT
{
class ConditionTestNode : public ConditionNode
{
  public:
    
    ConditionTestNode(const std::string& name) : ConditionNode::ConditionNode(name, {})
    {
        expected_result_ = NodeStatus::SUCCESS;
        tick_count_ = 0;
    }

    void setExpectedResult(NodeStatus res)    
    {
        expected_result_ = res;
    }

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override
    {
        tick_count_++;
        return expected_result_;
    }

    int tickCount() const
    {
        return tick_count_;
    }

  private:
    NodeStatus expected_result_;
    int tick_count_;
};
}

namespace BT
{
class SyncActionTest : public SyncActionNode
{
  public:
    SyncActionTest(const std::string& name) :
        SyncActionNode(name, {})
    {
        tick_count_ = 0;
        expected_result_ = NodeStatus::SUCCESS;
    }

    BT::NodeStatus tick() override
    {
        tick_count_++;
        return expected_result_;
    }

    void setExpectedResult(NodeStatus res)
    {
        expected_result_ = res;
    }

    int tickCount() const
    {
        return tick_count_;
    }

    void resetTicks()
    {
        tick_count_ = 0;
    }

  private:
    NodeStatus expected_result_;
    int tick_count_;
};

class AsyncActionTest : public AsyncActionNode
{
  public:
    AsyncActionTest(const std::string& name, BT::Duration deadline_ms = std::chrono::milliseconds(100)):
        AsyncActionNode(name, {}),
        success_count_(0),
        failure_count_(0)
    {
        expected_result_ = NodeStatus::SUCCESS;
        time_ = deadline_ms;
        tick_count_ = 0;
    }

    virtual ~AsyncActionTest()
    {
        halt();
    }

    // The method that is going to be executed by the thread
    BT::NodeStatus tick() override
    {
        /// Add Begin, test 
        static int testcount = 0;
        while(testcount < 5){
            testcount++;
            // usleep(1000000);
            std::cout<<"action run"<<std::endl;
        }
        testcount = 0;
        /// Add End
        using std::chrono::high_resolution_clock;
        tick_count_++;

        auto initial_time = high_resolution_clock::now();

        // we simulate an asynchronous action that takes an amount of time equal to time_
        while (!isHaltRequested() && high_resolution_clock::now() < initial_time + time_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // check if we exited the while(9 loop because of the flag stop_loop_
        if( isHaltRequested() ){
            return NodeStatus::IDLE;
        }

        if( expected_result_ == NodeStatus::SUCCESS){
            success_count_++;
        }
        else if( expected_result_ == NodeStatus::FAILURE){
            failure_count_++;
        }

        return expected_result_;
    }

    void setTime(BT::Duration time);

    // The method used to interrupt the execution of the node
    virtual void halt() override
    {
    // do more cleanup here if necessary
        AsyncActionNode::halt();
    }

    void setExpectedResult(NodeStatus res)
    {
        expected_result_ = res;
    }

    int tickCount() const {
        return tick_count_;
    }

    int successCount() const {
        return success_count_;
    }

    int failureCount() const {
        return failure_count_;
    }

    void resetCounters() {
        success_count_ = 0;
        failure_count_ = 0;
        tick_count_ = 0;
    }

  private:
    // using atomic because these variables might be accessed from different threads
    BT::Duration time_;
    std::atomic<NodeStatus> expected_result_;
    std::atomic<int> tick_count_;
    int success_count_;
    int failure_count_;

};

// class MyAsyncActionTest : public SyncActionNode

class MyAsyncActionTest : public ActionNodeBase
{
  private:
        int count_ = 0;
        int iters_ = 0;
  public:
    MyAsyncActionTest(const std::string& name, int iters):
        ActionNodeBase(name, {}), iters_(iters)
    {
        name_ = name;
    }

    virtual ~MyAsyncActionTest()
    {
        halt();
    }

    // The method that is going to be executed by the thread
    // BT::NodeStatus executeTick(){
    //     return tick();
    // }
    std::string name_;
    BT::NodeStatus tick() override
    {
        // do{
        count_++;
        std::cout<<name_<<std::endl;
        std::cout<<std::endl;
        // }while(count_ < iters_);

        if(count_ >= iters_)
            return BT::NodeStatus::SUCCESS;
        else
            return BT::NodeStatus::RUNNING;
        usleep(10);
        // check if we exited the while(9 loop because of the flag stop_loop_
        // if( isHaltRequested() ){
        //     return NodeStatus::IDLE;
        // }

        // if( expected_result_ == NodeStatus::SUCCESS){
        //     success_count_++;
        // }
        // else if( expected_result_ == NodeStatus::FAILURE){
        //     failure_count_++;
        // }

        // return expected_result_;
    }

    void setExpectedResult(NodeStatus res)
    {
        expected_result_ = res;
    }

    // The method used to interrupt the execution of the node
    virtual void halt() override
    {
    // do more cleanup here if necessary

        // SyncActionNode::halt();
    }

  private:
    // using atomic because these variables might be accessed from different threads
    std::atomic<NodeStatus> expected_result_;
    std::atomic<int> tick_count_;
    int success_count_;
    int failure_count_;

};


}


// static const char* xml_text = R"(

// <root main_tree_to_execute = "MainTree" >

//     <BehaviorTree ID="MainTree">
//         <Switch3 name="simple_switch" variable="{my_var}"  case_1="1" case_2="42 case_3="666" >
//             <AsyncActionTest name="action_1"/>
//             <AsyncActionTest name="action_42"/>
//             <AsyncActionTest name="action_666"/>
//             <AsyncActionTest name="action_default"/>
//         </Switch3>
//     </BehaviorTree>
// </root>
//         )";

struct SimpleSequenceTest
{
    BT::SequenceNode root;
    BT::MyAsyncActionTest action;
    BT::MyAsyncActionTest condition;
    // BT::ConditionTestNode condition;

    SimpleSequenceTest() :
      root("root_sequence")
      , action("action_fuck", 10)
      , condition("condition_fuck", 10)
    {
        root.addChild(&condition);
        root.addChild(&action);
    }
    ~SimpleSequenceTest()
    {

    }
};

struct SwitchTest
{
    
    using Switch2 = BT::SwitchNode<2>;
    std::unique_ptr<Switch2> root;
    BT::MyAsyncActionTest action_1;
    // BT::MyAsyncActionTest action_42;
    SimpleSequenceTest sequenceTest;
    BT::MyAsyncActionTest action_def;
    BT::Blackboard::Ptr bb = BT::Blackboard::create();
    BT::NodeConfiguration simple_switch_config_;

    SwitchTest() :
      action_1("action_1", 10),
    //   action_42("action_42", 1000),
      action_def("action_default", 10)
    {
        BT::PortsRemapping input;
        input.insert(std::make_pair("variable", "{my_var}"));
        input.insert(std::make_pair("case_1", "1"));
        input.insert(std::make_pair("case_2", "42"));

        BT::NodeConfiguration simple_switch_config_;
        simple_switch_config_.blackboard = bb;
        simple_switch_config_.input_ports = input;

        // std::unique_ptr<BT::SwitchNode<2> > root;
        root = std::make_unique<BT::SwitchNode<2> >("simple_switch", simple_switch_config_);

        root->addChild(&action_1);
        root->addChild(&sequenceTest.root);
        root->addChild(&action_def);
    }
    ~SwitchTest()
    {
        root->halt();
    }
};

#if test==1
int main(){
    SimpleSequenceTest sequenceTest;
    SwitchTest switchTest;
    BT::MyStdCoutLogger logger_cout(switchTest.root.get());
    BT::NodeStatus status = switchTest.root->executeTick();
    switchTest.bb->set("my_var", "1");
    static int fuck = 1;
    do{
        status = switchTest.root->executeTick();
        if(status == BT::NodeStatus::RUNNING){std::cout<<"RUNNING"<<std::endl;}
        if(status == BT::NodeStatus::IDLE){std::cout<<"IDLE"<<std::endl;}
        if(status == BT::NodeStatus::SUCCESS){std::cout<<"SUCCESS"<<std::endl;}
        usleep(1000);
        fuck++;
        if(fuck == 5){
            switchTest.bb->set("my_var", "42");
            std::cout<<"---------------------------------------------"<<std::endl;
        }
    }while(status != BT::NodeStatus::SUCCESS);

    // switchTest.bb->set("my_var", "42");
    // status = switchTest.root->executeTick();
    // switchTest.bb->set("my_var", "42");
	return 0;
}
#endif

#if test==0
int main(){
    SwitchTest switchTest;
    BT::MyStdCoutLogger logger_cout(switchTest.root.get());
    BT::NodeStatus status = switchTest.root->executeTick();
    switchTest.bb->set("my_var", "1");
    do{
        status = switchTest.root->executeTick();
        if(status == BT::NodeStatus::RUNNING){std::cout<<"RUNNING"<<std::endl;}
        if(status == BT::NodeStatus::IDLE){std::cout<<"IDLE"<<std::endl;}
        if(status == BT::NodeStatus::SUCCESS){std::cout<<"SUCCESS"<<std::endl;}
        usleep(1000000);
    }while(status != BT::NodeStatus::SUCCESS);

    switchTest.bb->set("my_var", "42");
    status = switchTest.root->executeTick();
	return 0;
}
#endif

#if test==2

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

using namespace BT;

class BB_TestNode: public SyncActionNode
{
  public:
    BB_TestNode(const std::string& name, const NodeConfiguration& config):
      SyncActionNode(name, config)
    { }

    NodeStatus tick()
    {
        int value = 0;
        auto res = getInput<int>("in_port");
        if(!res)
        {
            throw RuntimeError("BB_TestNode needs input", res.error());
        }
        value = res.value()*2;
        if( !setOutput("out_port", value) )
        {
            throw RuntimeError("BB_TestNode failed output");
        }
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        return { BT::InputPort<int>("in_port"),
                 BT::OutputPort<int>("out_port") };
    }
};

class BB_TypedTestNode: public SyncActionNode
{
  public:
    BB_TypedTestNode(const std::string& name, const NodeConfiguration& config):
      SyncActionNode(name, config)
    { }

    NodeStatus tick()
    {
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        return { BT::InputPort("input"),
                 BT::InputPort<int>("input_int"),
                 BT::InputPort<std::string>("input_string"),

                 BT::OutputPort("output"),
                 BT::OutputPort<int>("output_int"),
                 BT::OutputPort<std::string>("output_string") };
    }
};

int main(){
    auto bb = Blackboard::create();

    NodeConfiguration config;
    assignDefaultRemapping<BB_TestNode>( config );

    config.blackboard = bb;
    bb->set("in_port", 11 );

    BB_TestNode node("good_one", config);

    // this should read and write "my_entry" in tick()
    node.executeTick();

    if(22 == bb->get<int>("out_port")){
        std::cout<<"good"<<std::endl;
    }
}

#endif