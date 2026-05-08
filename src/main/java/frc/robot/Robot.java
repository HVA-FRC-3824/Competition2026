package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.lib.HubActivePeriod;
import frc.robot.lib.motor.OrchestraOrchestrator;
import frc.robot.lib.motor.OrchestraOrchestrator.Song;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand = Commands.none();

  private final ControlLayer m_robotContainer;

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
    
    LoggedPowerDistribution.getInstance(Constants.CanIds.PdhId, ModuleType.kCTRE);

    // PATHPLANNER

    Pathfinding.setPathfinder(new LocalADStar());  
    // I'm not sure which one to use... This is what's in the docs though
    // CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    m_robotContainer = new ControlLayer();

    SmartDashboard.putData(CommandScheduler.getInstance());
    
    OrchestraOrchestrator.playSong(Song.Tetris);
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    
    CommandScheduler.getInstance().schedule(m_robotContainer.getSwerveZero().andThen(m_robotContainer.getAutoCommand()).withName("AUTO"));
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    HubActivePeriod.initialize();

    m_autonomousCommand.cancel();

    CommandScheduler.getInstance().schedule(m_robotContainer.getSwerveZero());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void simulationInit() {

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
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
