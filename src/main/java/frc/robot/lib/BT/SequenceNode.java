package frc.robot.lib.BT;

import edu.wpi.first.wpilibj.XboxController;
import java.util.Arrays;
import java.util.List;

public class SequenceNode extends Node
{

    public SequenceNode(Node ...nodes) {
        m_nodes = Arrays.asList(nodes);
    }
    
    @Override
    public NodeStatus update(XboxController controller)
    {
        for (Node node : m_nodes) 
        {
            if (node.update(controller) == NodeStatus.Failure)
                return NodeStatus.Failure;
        }

        return NodeStatus.Success;
    }

    private List<Node> m_nodes;
    private int m_currentIndex = 0;
}
