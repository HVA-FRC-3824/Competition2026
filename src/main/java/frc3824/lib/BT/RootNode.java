package frc3824.lib.BT;

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
    public NodeStatus update()
    {
        for (Node node : m_nodes)
        {
            node.update();
        }
        return NodeStatus.Success;
    }

    List<Node> m_nodes;
}
