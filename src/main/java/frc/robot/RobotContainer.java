package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BT.Node;
import frc.robot.lib.BT.RootNode;
import frc.robot.Constants.Controller;
import frc.robot.lib.BT.ActionNode;
import frc.robot.lib.BT.SequenceNode;
import frc.robot.lib.BT.SelectorNode;
import frc.robot.lib.BT.ConditionNode;
import frc.robot.lib.BT.ControllerNode;

public class RobotContainer 
{
  RobotState m_stateController = new RobotState();
  
  private final XboxController m_driver   = new XboxController(0);
  private final XboxController m_operator = new XboxController(1);

  private final SendableChooser<Command> m_autoChooser;

  private RootNode m_driverBT;

  public RobotContainer() 
  {
    m_autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
  }

  private void configureBindings() 
  {
    Node shootingNode =
      (Node) new SelectorNode(
        (Node) new SequenceNode( // Low
          (Node) new ControllerNode(Constants.Controller.A),
          (Node) new ActionNode(m_stateController.lowShootTowerCommand),
          (Node) new ActionNode(m_stateController.autoAimCommand)
        ),
        (Node) new SequenceNode( // Mid
          (Node) new ControllerNode(Constants.Controller.B),
          (Node) new ActionNode(m_stateController.midShootTowerCommand),
          (Node) new ActionNode(m_stateController.autoAimCommand)
        ),
        (Node) new SequenceNode( // Long
          (Node) new ControllerNode(Constants.Controller.Y),
          (Node) new ActionNode(m_stateController.longShootTowerCommand),
          (Node) new ActionNode(m_stateController.autoAimCommand)
        ),
        (Node) new SequenceNode( // Auto
          new ConditionNode(() -> m_driver.getRightTriggerAxis() >= 0.5),
          (Node) new ActionNode(m_stateController.autoTowerCommand),
          (Node) new ActionNode(m_stateController.autoAimCommand)
        ),
        (Node) new SequenceNode( // Warm Up
          (Node) new ControllerNode(Constants.Controller.X),
          (Node) new ActionNode(m_stateController.midShootTowerCommand)
        ),
        (Node) new SequenceNode( // Default/Spin Down
          (Node) new ActionNode(m_stateController.spinDownTowerCommand),
          (Node) new ActionNode(m_stateController.driveModeCommand)
        )
      );

    Node intakeNode = new SelectorNode(
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.LeftBumper),
          (Node) new ActionNode(m_stateController.deployIntakeCommand),
          (Node) new ActionNode(m_stateController.startIntakeCommand)
        ),
        (Node) new ActionNode(m_stateController.stopIntakeCommand)
      );

    Node driveNode =
      (Node) new SelectorNode(
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.RightStickButton),
          (Node) new ActionNode(() -> m_stateController.setDrive(-m_driver.getLeftY() / 2, -m_driver.getLeftX() / 2, -m_driver.getRightX() / 2))
        ),
        (Node) new ActionNode(() -> m_stateController.setDrive(-m_driver.getLeftY(), -m_driver.getLeftX(), -m_driver.getRightX()))
      );

    Node indexNode = 
      (Node) new SelectorNode( // Indexing
        (Node) new SequenceNode(
          (Node) new ConditionNode(() -> m_stateController.GetIsReady()),
          (Node) new ActionNode(m_stateController.indexingCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ControllerNode(Constants.Controller.LeftStickButton),
          (Node) new ActionNode(m_stateController.indexingCommand)
        ),
        // Default
        (Node) new ActionNode(m_stateController.notIndexingCommand)
      );

    m_driverBT = new RootNode(
      // Driving
      driveNode,
      shootingNode,
      indexNode,
      intakeNode
    );
  }

  public void tick()
  {
    // Mid match swap
    // m_driverBT.update((Timer.getMatchTime() - ((2 * 60) + 30) / 2 > 0) ? m_driver : m_operator);

    m_driverBT.update(m_driver);
  }

  public void log()
  {
    m_stateController.Periodic();
  }

  public Command getAutonomousCommand() 
  {
    return m_autoChooser.getSelected();
  }
}
