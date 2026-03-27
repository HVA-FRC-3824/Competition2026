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
        boolean checked = m_conditionChecker.get(); 

        if (m_toggleCheck)
            checked = false;

        m_toggleCheck = checked;
        
        return checked ? NodeStatus.Success : NodeStatus.Failure;
    }
    
    private Supplier<Boolean> m_conditionChecker;
    private boolean           m_toggleCheck = false;
}
