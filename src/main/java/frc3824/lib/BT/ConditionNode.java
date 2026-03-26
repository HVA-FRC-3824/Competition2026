package frc3824.lib.BT;

import java.util.function.Supplier;

public class ConditionNode extends Node
{    
    public ConditionNode(Supplier<Boolean> condition) 
    {
        m_conditionChecker = condition;
    }

    @Override
    public NodeStatus update()
    {
        return m_conditionChecker.get() ? NodeStatus.Success : NodeStatus.Failure;
    }
    
    private Supplier<Boolean> m_conditionChecker;
}
