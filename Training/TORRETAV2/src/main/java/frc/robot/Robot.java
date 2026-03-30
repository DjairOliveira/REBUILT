// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static GenericEntry TurretExposure;
  public static GenericEntry TurretBlackLevel;
  public static GenericEntry TurretSensorGain;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Turret.mInclinaVertical.getEncoder().setPosition(0);
    Turret.mInclinaHorizontal.getEncoder().setPosition(0);

    // dashSets();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // SmartDashboard.putNumber("Turret Exposure", TurretExposure.getDouble(50));
    // SmartDashboard.putNumber("Turret Black Level", TurretBlackLevel.getDouble(50));
    // SmartDashboard.putNumber("Turret Sensor Gain", TurretSensorGain.getDouble(50));

    SmartDashboard.putNumber("Inclina-Horizontal", Turret.getInclinaHorizontal());
    SmartDashboard.putNumber("Inclina-Vertical", Turret.getInclinaVertical());

    SmartDashboard.putNumber("Turret TX", LimelightHelpers.getTX("limelight-turret"));
    SmartDashboard.putNumber("Turret TY", LimelightHelpers.getTY("limelight-turret"));
    SmartDashboard.putNumber("Turret TA", LimelightHelpers.getTA("limelight-turret"));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  private void dashSets() {
    TurretExposure = Shuffleboard.getTab("Limelight")
        .add("Turret Exposition", 50)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 2, "max", 3300))
        .getEntry();

    TurretBlackLevel = Shuffleboard.getTab("Limelight")
        .add("Turret Black Level", 2)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 40))
        .getEntry();
    TurretSensorGain = Shuffleboard.getTab("Limelight")
        .add("Turret Black Level", 2)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 1, "max", 45))
        .getEntry();

  }
}
