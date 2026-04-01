package frc.robot.lib.BT;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.XboxController;

public class ConditionNode extends Node
{    
    public ConditionNode(Supplier<Boolean> condition) 
    {
        m_conditionChecker = condition;
    }

    @Override
    public NodeStatus update(XboxController controller)
    {
        return (m_inversed != m_conditionChecker.get()) ? NodeStatus.Success : NodeStatus.Failure;
    }

    // DECORATORS

    public Node inverse()
    {
        m_inversed = !m_inversed;

        return this;
    }
    
    private Supplier<Boolean> m_conditionChecker;
    
    protected boolean m_inversed = false;
}
