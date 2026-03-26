package frc3824;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc3824.lib.BT.Node;
import frc3824.lib.BT.RootNode;
import frc3824.Constants.Controller;
import frc3824.lib.BT.ActionNode;
import frc3824.lib.BT.SequenceNode;
import frc3824.lib.BT.SelectorNode;
import frc3824.lib.BT.ConditionNode;
import frc3824.lib.BT.ControllerNode;

public class RobotContainer {

  
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
    Node shootingButtons  =
      (Node) new SelectorNode(
        (Node) new SequenceNode(
          (Node) new ControllerNode(m_driver, Constants.Controller.A),
          (Node) new ActionNode(RobotState.GetInstance().lowShootTowerCommand),
          (Node) new ActionNode(RobotState.GetInstance().autoAimCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ControllerNode(m_driver, Constants.Controller.B),
          (Node) new ActionNode(RobotState.GetInstance().midShootTowerCommand),
          (Node) new ActionNode(RobotState.GetInstance().autoAimCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ControllerNode(m_driver, Constants.Controller.Y),
          (Node) new ActionNode(RobotState.GetInstance().longShootTowerCommand),
          (Node) new ActionNode(RobotState.GetInstance().autoAimCommand)
        ),
        (Node) new SequenceNode(
          new ConditionNode(() -> m_driver.getRightTriggerAxis() >= 0.5),
          (Node) new ActionNode(RobotState.GetInstance().autoTowerCommand),
          (Node) new ActionNode(RobotState.GetInstance().autoAimCommand)
        ),
        // Default
        (Node) new SequenceNode(
          (Node) new ActionNode(RobotState.GetInstance().spinDownTowerCommand),
          (Node) new ActionNode(RobotState.GetInstance().driveModeCommand)
        )
      );

    m_driverBT = new RootNode(
      // Driving
      (Node) new ActionNode(() -> RobotState.GetInstance().setDrive(m_driver.getLeftY(), m_driver.getLeftX(), m_driver.getRightX())),
      // Chassis Commands
      (Node) new SequenceNode(
        (Node) new ControllerNode(m_driver, Constants.Controller.Pov_0),
        (Node) new ActionNode(RobotState.GetInstance().xModeCommand)
      ),
      (Node) new SequenceNode(
        (Node) new ControllerNode(m_driver, Constants.Controller.Pov_90),
        (Node) new ActionNode(RobotState.GetInstance().slowModeCommand)
      ),
      // Warm up
      (Node) new SequenceNode(
        (Node) new ControllerNode(m_driver, Constants.Controller.Pov_180),
        (Node) new ActionNode(RobotState.GetInstance().midShootTowerCommand)
      ),
      shootingButtons,
      // Indexing
      (Node) new SelectorNode(
        (Node) new SequenceNode(
          (Node) new ConditionNode(() -> RobotState.GetInstance().GetIsReady()),
          (Node) new ActionNode(RobotState.GetInstance().indexingCommand)
        ),
        (Node) new SequenceNode(
          (Node) new ControllerNode(m_driver, Constants.Controller.LeftBumper),
          (Node) new ActionNode(RobotState.GetInstance().indexingCommand)
        ),
        (Node) new ActionNode(RobotState.GetInstance().notIndexingCommand)
      ),
      (Node) new SequenceNode(
        (Node) new ControllerNode(m_driver, Constants.Controller.Pov_270)
      )
    );
  }

  public void tick()
  {
    m_driverBT.update();
  }

  public Command getAutonomousCommand() 
  {
    return m_autoChooser.getSelected();
  }
}
