// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.EstimatePose;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.intake.schedulerChangesCall();
    CommandScheduler.getInstance().schedule(new EstimatePose(m_robotContainer.lights,m_robotContainer.drivetrain,false));
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot Disabled");
    m_robotContainer.disabledLights();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledLights();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    System.out.println("Autonomous Started");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.initLights();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    System.out.println("Teleop Started");
    m_robotContainer.TeleopLights();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
