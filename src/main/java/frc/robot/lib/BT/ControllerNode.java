package frc.robot.lib.BT;

import edu.wpi.first.wpilibj.XboxController;

// ControllerNode behaves like a ConditionNode but implements its own
// update so we can treat POV presses the same as button presses
// (i.e. trigger once on press, not while held).
public class ControllerNode extends ConditionNode
{
    private final int m_button;

    public ControllerNode(int button)
    {
        // Not used
        super(() -> false);
        m_button = button;
    }

    @Override
    public NodeStatus update(XboxController controller)
    {
        // ** Normal Button Logic ** //

        if (!(m_button == 0 || m_button > 10))
            return controller.getRawButton(m_button) ? NodeStatus.Success : NodeStatus.Failure;

        // ** Logic for POVs ** //
        // Because there isn't a simple "getPOVPressed" there's a bit more logic to standardize it

        int currentPov = controller.getPOV();
        if (currentPov == -1)
        {
            return NodeStatus.Failure;
        }
        boolean povPressed = (currentPov == m_button);// && (m_prevPov != m_button);

        return (m_inversed ? !povPressed : povPressed) ? NodeStatus.Success : NodeStatus.Failure;
    }
}
