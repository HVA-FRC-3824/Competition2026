package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.HubActivePeriod;
import frc.robot.lib.motor.talonFX.OrchestraOrchestrator;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

  // ADVANTAGE KIT
  if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
  } else {
    // For now, all we want in sim is network tables
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
  }

  Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  
  LoggedPowerDistribution.getInstance(Constants.CanIds.PdhId, ModuleType.kRev); // Example: PDH on CAN ID 50

  // PATHPLANNER

  Pathfinding.setPathfinder(new LocalADStar());  
  // I'm not sure which one to use... This is what's in the docs though
  // CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

  m_robotContainer = new RobotContainer();

  OrchestraOrchestrator.playSong();
  }

  @Override
  public void robotPeriodic() 
  {
  // GOD remove this ABSOLUTE GARBAGE, absolute SCUM OF THE EARTH ARCHITECTURE
  CommandScheduler.getInstance().run();

  // Beautiful, absolutely perfect, handling inputs from the controller GUHHHH
  m_robotContainer.modulePeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
  m_autonomousCommand = m_robotContainer.getAutoCommand();
  CommandScheduler.getInstance().schedule(m_autonomousCommand);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    HubActivePeriod.initialize();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.resetState();
  }

  @Override
  public void teleopPeriodic() 
  {
  
  }

  @Override
  public void teleopExit() {}

  @Override
  public void simulationInit() 
  {
  // for (int x = 7 * 6; x <= 10 * 6; x++)
  // {
  //   for (int y = 2 * 6; y <= 6 * 6; y++) 
  //   {
  //     RebuiltFuelOnField ball = new RebuiltFuelOnField(new Translation2d(x / 6.0, y / 6.0));
  //     SimulatedArena.getInstance().addGamePiece(ball);
  //   }
  // }

  // for (int x = 0 * 6; x <= 0.5 * 6; x++) 
  // {
  //   for (int y = (int)(5.5 * 6); y <= 6.5 * 6; y++) 
  //   {
  //     RebuiltFuelOnField ball = new RebuiltFuelOnField(new Translation2d(x / 6.0, y / 6.0));
  //     SimulatedArena.getInstance().addGamePiece(ball);
  //   }
  // }
  }

  // simulation period method in your Robot.java
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    
    Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    
  }

  @Override
  public void testInit() 
  {
  CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
