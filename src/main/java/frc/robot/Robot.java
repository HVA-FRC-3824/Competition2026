package frc.robot;

import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms o
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot
{
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() 
  {

    // ADVANTAGE KIT

    if (isReal()) 
    {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else 
    {
    //   setUseTiming(false); // Run as fast as possible
    //  String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //  Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //  Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      
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
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
    m_robotContainer.log();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() 
  {
    if (m_autonomousCommand != null)
      m_autonomousCommand.cancel();
  }

  @Override
  public void teleopPeriodic() 
  {
    m_robotContainer.tick();
  }

  @Override
  public void teleopExit() {}

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
