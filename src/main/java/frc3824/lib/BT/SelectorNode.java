package frc3824.lib.BT;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SelectorNode extends Node
{
    public SelectorNode(Node ...nodes)
    {
        m_nodes = Arrays.asList(nodes);
    }
    
    @Override
    public NodeStatus update()
    {
        for (Node node : m_nodes) 
        {
            if (node.update() == NodeStatus.Success)
                return NodeStatus.Success;
        }
        return NodeStatus.Failure;
    }

    List<Node> m_nodes;
    private int m_currentIndex = 0;
}
