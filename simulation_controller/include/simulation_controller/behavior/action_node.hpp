#ifndef ENTITY_BEHAVIOR__ACTION_NODE_H_INCLUDED
#define ENTITY_BEHAVIOR__ACTION_NODE_H_INCLUDED

#include <behaviortree_cpp_v3/action_node.h>

namespace entity_behavior
{
    class BehaviorTreeRuntimeError : public std::runtime_error
    {
    public:
        BehaviorTreeRuntimeError(const char *message, int res=0) : error_info_(res), runtime_error(message) {};
    private:
        int error_info_;
    };

    class ActionNode : public BT::ActionNodeBase
    {
    public:

        ActionNode(const std::string& name, const BT::NodeConfiguration& config);
        ~ActionNode() override = default;

        /// throws if the derived class return RUNNING.
        virtual BT::NodeStatus executeTick() override;

        /// You don't need to override this
        virtual void halt() override final
        {
            setStatus(BT::NodeStatus::IDLE);
        }
    };
}  // namespace entity_behavior

#endif  // ENTITY_BEHAVIOR__ACTION_NODE_H_INCLUDED