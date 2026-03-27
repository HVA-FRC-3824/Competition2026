package frc.robot.lib.BT;

import edu.wpi.first.wpilibj.XboxController;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RootNode extends Node
{
    public RootNode(Node ...nodes)
    {
        m_nodes = Arrays.asList(nodes);
    }   
    
    @Override
    public NodeStatus update(XboxController controller)
    {
        for (Node node : m_nodes)
        {
            node.update(controller);
        }
        return NodeStatus.Success;
    }

    List<Node> m_nodes;
}
