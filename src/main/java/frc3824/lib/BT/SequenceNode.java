package frc3824.lib.BT;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequenceNode extends Node
{

    public SequenceNode(Node ...nodes) {
        m_nodes = Arrays.asList(nodes);
    }
    
    @Override
    public NodeStatus update()
    {
        for (Node node : m_nodes) 
        {
            if (node.update() == NodeStatus.Failure)
                return NodeStatus.Failure;
        }

        return NodeStatus.Success;
    }

    private List<Node> m_nodes;
    private int m_currentIndex = 0;
}
