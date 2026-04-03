package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BT.Node;
import frc.robot.lib.BT.RootNode;
import frc.robot.RobotState.TowerState;
import frc.robot.lib.BT.ActionNode;
import frc.robot.lib.BT.SequenceNode;
import frc.robot.lib.BT.SelectorNode;
import frc.robot.lib.BT.ConditionNode;
import frc.robot.lib.BT.ControllerNode;

public class RobotContainer 
{  
  private final XboxController m_driver   = new XboxController(0);
  private final XboxController m_operator = new XboxController(1);

  private final SendableChooser<Command> m_autoChooser;

  private RootNode m_driverBT;
  private RootNode m_operatorBT;

  public RobotContainer() 
  {
    new RobotState();

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    configureBindings();
  }

  private void configureBindings() 
  {
    Node shootingNode =
      (Node) new SelectorNode(
        (Node) new SequenceNode( // Shoot
          (Node) new ConditionNode(() -> m_driver.getRightTriggerAxis() >= 0.5),
          (Node) new ActionNode(RobotState.midShootTowerCommand),
          (Node) new ActionNode(RobotState.driveModeCommand),
          (Node) new ActionNode(RobotState.spinUpTowerCommand)
        ),
        (Node) new SequenceNode( // Shoot
          (Node) new ConditionNode(() -> m_driver.getLeftTriggerAxis() >= 0.5),
          (Node) new ActionNode(RobotState.midShootTowerCommand),
          (Node) new ActionNode(RobotState.driveModeCommand),
          (Node) new ActionNode(RobotState.spinUpTowerCommand)
        ),
        (Node) new SequenceNode( // Shoot
          (Node) new ControllerNode(Constants.Controller.RightBumper),
          (Node) new ActionNode(RobotState.longShootTowerCommand),
          (Node) new ActionNode(RobotState.driveModeCommand),
          (Node) new ActionNode(RobotState.spinUpTowerCommand)
        ),
        (Node) new SequenceNode( // Default/Spin Down
          (Node) new ActionNode(RobotState.spinDownTowerCommand),
          (Node) new ActionNode(RobotState.driveModeCommand)
        )
      );

    Node intakeNode = new SelectorNode(
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.RightPaddle),
          (Node) new ActionNode(RobotState.deployIntakeCommand),
          (Node) new ActionNode(RobotState.startIntakeCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.A),
          (Node) new ActionNode(RobotState.jiggleIntakeCommand)
        ),
        (Node) new ActionNode(RobotState.stopIntakeCommand)
      );

    Node driveNode =
      (Node) new SelectorNode(
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.X),
          (Node) new ActionNode(RobotState.xModeOnCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ActionNode(() -> RobotState.setDrive(-m_driver.getLeftY(), -m_driver.getLeftX(), -m_driver.getRightX())),
          (Node) new ActionNode(RobotState.xModeOffCommand)
        )
      );

    Node indexNode = (Node) new SelectorNode(
        (Node) new SequenceNode(
          (Node) new ConditionNode(() -> RobotState.isSpunUp()),
          (Node) new SelectorNode(
            (Node) new ControllerNode(Constants.Controller.RightBumper),
            (Node) new ConditionNode(() -> m_driver.getRightTriggerAxis() >= 0.5)
          ),
          (Node) new ActionNode(RobotState.indexingCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.LeftPaddle),
          (Node) new ActionNode(RobotState.indexingCommand),
          (Node) new ActionNode(()->Logger.recordOutput("Doo the index MANUEL", "Are we index manuel?"))
        ),
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.Y),
          (Node) new ActionNode(RobotState.backwardsIndexingCommand)
        ),
        (Node) new ActionNode(RobotState.notIndexingCommand)
      );

    m_driverBT = new RootNode(
      driveNode,
      intakeNode,
      shootingNode,
      indexNode
    );

    Node shootingModeNodes = 
      (Node) new SelectorNode(
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.A),
          (Node) new ActionNode(RobotState.lowShootTowerCommand),
            (Node) new ActionNode(RobotState.spinUpTowerCommand)
        ),        
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.B),
          (Node) new ActionNode(RobotState.midShootTowerCommand),
            (Node) new ActionNode(RobotState.spinUpTowerCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.Y),
          (Node) new ActionNode(RobotState.longShootTowerCommand),
          (Node) new ActionNode(RobotState.spinUpTowerCommand)
        ),
        (Node) new SequenceNode( // skepticism
          (Node) new ControllerNode(Constants.Controller.LeftPaddle),
          (Node) new ActionNode(RobotState.spinUpTowerCommand),
          (Node) new ActionNode(RobotState.autoTowerCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ConditionNode(() -> m_operator.getRightTriggerAxis() >= 0.5),
          (Node) new ActionNode(RobotState.spinUpTowerCommand),
          (Node) new ActionNode(RobotState.manualTowerCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ConditionNode(() -> RobotState.m_towerState != TowerState.Auto),
          (Node) new ActionNode(RobotState.spinDownTowerCommand)
        )
      );

    Node manualModeNodes = 
      (Node) new SelectorNode(
        (Node) new SequenceNode( // insure its only on press
          (Node) new ConditionNode(()->m_operator.getRawButtonPressed(Constants.Controller.RightBumper)),
          (Node) new ActionNode(RobotState.increaseManualTowerCommand)
        ),
        (Node) new SequenceNode( // insure its only on press
          (Node) new ConditionNode(()->m_operator.getRawButtonPressed(Constants.Controller.LeftBumper)),
          (Node) new ActionNode(RobotState.decreaseManualTowerCommand)
        )
      );

    Node intakeJoggingNode =
      (Node) new SequenceNode(
        (Node) new ConditionNode(() -> m_operator.getLeftTriggerAxis() >= 0.5),
        (Node) new ActionNode(RobotState.jiggleIntakeCommand)
      );

    m_operatorBT = new RootNode(
        manualModeNodes,
        shootingModeNodes,
        intakeJoggingNode
      );
  }

  public void tick()
  {
    // Mid match swap
    // m_driverBT.update((Timer.getMatchTime() - ((2 * 60) + 30) / 2 > 0) ? m_driver : m_operator);

    m_operatorBT.update(m_operator);
    m_driverBT.update(m_driver);
  }

  public void log()
  {
    RobotState.Periodic();
  }

  public Command getAutonomousCommand() 
  {
    return m_autoChooser.getSelected();
  }
}
