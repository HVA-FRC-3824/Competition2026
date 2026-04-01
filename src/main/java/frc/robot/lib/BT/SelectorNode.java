package frc.robot.lib.BT;

import edu.wpi.first.wpilibj.XboxController;
import java.util.Arrays;
import java.util.List;

public class SelectorNode extends Node
{
    public SelectorNode(Node ...nodes)
    {
        m_nodes = Arrays.asList(nodes);
    }
    
    @Override
    public NodeStatus update(XboxController controller)
    {
        for (Node node : m_nodes) 
        {
            if (node.update(controller) == NodeStatus.Success)
                return NodeStatus.Success;
        }
        return NodeStatus.Failure;
    }

    List<Node> m_nodes;
}
