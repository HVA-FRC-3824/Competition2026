package frc.robot.lib.BT;

import edu.wpi.first.wpilibj.XboxController;

// ControllerNode behaves like a ConditionNode but implements its own
// update so we can treat POV presses the same as button presses
// (i.e. trigger once on press, not while held).
public class ControllerNode extends Node
{
    private final int m_button;

    // Store previous raw POV value for edge detection (-1 means no POV)
    private int m_prevPov = -1;

    public ControllerNode(int button)
    {
        m_button = button;
    }

    @Override
    public NodeStatus update(XboxController controller)
    {
        // ** Normal Button Logic ** //

        if (!(m_button == 0 || m_button > 10))
            return controller.getRawButtonPressed(m_button) ? NodeStatus.Success : NodeStatus.Failure;


        // ** Logic for POVs ** //
        // Because there isn't a simple "getPOVPressed" there's a bit more logic to standardize it

        int currentPov = controller.getPOV();
        if (currentPov == -1)
        {
            m_prevPov = currentPov;
            return NodeStatus.Failure;
        }
        boolean povPressed = (currentPov == m_button) && (m_prevPov != m_button);

        m_prevPov = currentPov;

        return povPressed ? NodeStatus.Success : NodeStatus.Failure;
    }
}
