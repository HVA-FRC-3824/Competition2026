package frc3824.lib.BT;

import edu.wpi.first.wpilibj.XboxController;

public class ControllerNode extends ConditionNode
{
    public ControllerNode(XboxController controller, int button)
    { 
        super(() -> {
            if (button == 0 || button > 10)
            {
                return controller.getPOV() == button;
            }
            else
            {
                return controller.getRawButton(button);
            }
        });
    }
}
