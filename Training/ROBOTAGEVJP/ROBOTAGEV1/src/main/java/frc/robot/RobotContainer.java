// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.BooleanSupplier;

import swervelib.SwerveInputStream;

public class RobotContainer
{
  public final double inclinationMax = 6;
  final CommandXboxController Cmdriver = new CommandXboxController(0);
  public XboxController driver = new XboxController(0);
  public final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/MK4i"));
  
  double shooterMov=0;

  private int intakectn=0;

  private final SendableChooser<Command> autoChooser;

  // private Robot mRobot;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> Cmdriver.getLeftY() * -normalized(Robot.velocityRobot) * Cmdriver.getRightTriggerAxis(),
                                                                () -> Cmdriver.getLeftX() * -normalized(Robot.velocityRobot) * Cmdriver.getRightTriggerAxis())
                                                            .withControllerRotationAxis(() -> Cmdriver.getRightX() * -normalized(Robot.velocityRobot))
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(Cmdriver::getRightX,
                                                                                             Cmdriver::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -Cmdriver.getLeftY(),
                                                                        () -> -Cmdriver.getLeftX())
                                                                    .withControllerRotationAxis(() -> Cmdriver.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                    .withControllerHeadingAxis(() ->Math.sin(Cmdriver.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                    () ->Math.cos(Cmdriver.getRawAxis(2) * Math.PI) *(Math.PI * 2))
                    .headingWhile(true).translationHeadingOffset(true)
                    .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    new EventTrigger("INTAKE_ON").onTrue(new SequentialCommandGroup(  //intakectn
      Commands.runOnce(()-> Intake.setInclina(-21.9,0.15,1)),
      Commands.waitSeconds(0.3),
      Commands.runOnce(()-> Intake.setIntakeSpeed(1))));
    
    NamedCommands.registerCommand("INTAKE_OFF", new SequentialCommandGroup(
      Commands.runOnce(()-> Intake.setInclina(0,0.15,1)),
      Commands.runOnce(()-> Intake.setIntakeSpeed(0)),
      new InstantCommand(()-> intakectn=0)));

    new EventTrigger("TURRET_OFF").onTrue(new SequentialCommandGroup(  // DESLIGA TORRETA
      Commands.runOnce(() -> Turret.end()),
      Commands.runOnce(() -> Turret.stopSpeed()),
      Commands.runOnce(() -> Turret.stopEngatilhar()),
      Commands.runOnce(() -> Intake.stopOrganizador())));
      
    NamedCommands.registerCommand("TURRET_HUB", new Turret());

    new EventTrigger("CLIMBER_UP").onTrue(Commands.runOnce(() -> Climber.setPosition(-350, 1)));
    new EventTrigger("CLIMBER_PUSH").onTrue(Commands.runOnce(() -> Climber.setPosition(-100, 1)));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    Cmdriver.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    /********** INTAKE **************************/
    BooleanSupplier protectIntake = ()-> Intake.getInclina() > -15 && Intake.getInclina() < -3. && Intake.getVelocityInclina() > 0 && intakectn==1;
    BooleanSupplier safeInclina = ()-> Intake.getInclina() < -15 && intakectn==1;
    BooleanSupplier coletarsafe = ()-> Intake.getInclina() > -3 || (Intake.getIntakeSpeed() == 0 && Intake.getInclina()<-15);
    BooleanSupplier indexer = ()-> Intake.getInclina() < -15 &&  (driver.getLeftBumperButtonReleased() || !driver.getLeftBumperButton());
    
    activateCommandOnCondition(()-> driver.getAButton(), new InstantCommand(()-> intakectn++));
    activateCommandOnCondition(indexer, new SequentialCommandGroup(
      new InstantCommand(()-> Intake.setSpeedOrganizador1(0.35)),
      new InstantCommand(()-> Intake.setSpeedOrganizador(-0.15))));


    activateCommandOnCondition(()-> intakectn==1, new SequentialCommandGroup(
      Commands.runOnce(()-> Intake.setInclina(-21.9,0.15,0.8)),
      Commands.waitSeconds(0.3),
      Commands.runOnce(()-> Intake.setIntakeSpeed(1)),
      Commands.runOnce(()-> Intake.setSpeedEsteira(0))));

    activateCommandOnCondition(()-> intakectn==2, new SequentialCommandGroup(
      Commands.runOnce(()-> Intake.setInclina(0,0.3,0.25)),
      new InstantCommand(()-> Intake.setSpeedOrganizador1(0)),
      new InstantCommand(()-> Intake.setSpeedOrganizador(0)),
      new InstantCommand(()-> intakectn=0)));

    activateCommandOnCondition(protectIntake, new InstantCommand(()-> intakectn=2));
    activateCommandOnCondition(safeInclina, Commands.runOnce(()-> Intake.stopInclina()));
    activateCommandOnCondition(coletarsafe, Commands.runOnce(()-> Intake.setIntakeSpeed(0)));

    /********** TURRET **************************/
    Cmdriver.b().onTrue(new SequentialCommandGroup(
      Commands.runOnce(() -> Turret.stopSpeed()),
      Commands.runOnce(() -> Turret.stopEngatilhar()),
      Commands.runOnce(() -> Intake.stopOrganizador()),
      Commands.runOnce(() -> Turret.setHorizontal(0)),
      Commands.runOnce(() -> Intake.setSpeedOrganizador1(0))));

    // Cmdriver.povDown().onTrue(Commands.runOnce(() -> Turret.setVertical(0)));
    // Cmdriver.povLeft().onTrue(Commands.runOnce(() -> Turret.setVertical(inclinationMax * 0.3333)));
    // Cmdriver.povRight().onTrue(Commands.runOnce(() -> Turret.setVertical(inclinationMax * 0.6666)));
    // Cmdriver.povUp().onTrue(Commands.runOnce(() -> Turret.setVertical(inclinationMax)));

    Cmdriver.leftBumper().whileTrue(new Turret().repeatedly());
    Cmdriver.leftBumper().onFalse(Commands.runOnce(() -> Turret.end()));

    Cmdriver.y().onTrue(Commands.runOnce(() -> Climber.setPosition(0, 1))); //359 max alto
    Cmdriver.povUp().onTrue(Commands.runOnce(() -> Climber.setPosition(-350, 1)));
    Cmdriver.povDown().onTrue(Commands.runOnce(() -> Climber.setPosition(-100, 1)));


    // Cmdriver.back().onTrue(new SequentialCommandGroup(
    //   Commands.runOnce(() -> Turret.stopSpeed()),
    //   Commands.runOnce(() -> Turret.stopEngatilhar()),
    //   Commands.runOnce(() -> Intake.stopOrganizador()),
    //   Commands.runOnce(() -> Turret.setHorizontal(0)),
    //   Commands.runOnce(() -> Intake.setSpeedOrganizador1(0))));
    
  }

  private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  // private void activateCommandOnConditionWhile(BooleanSupplier condition, Command command) {
  //   new Trigger(condition).whileTrue(command);
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public static double normalized(GenericEntry velocityRobot){
    return velocityRobot.getDouble(0) > 1 ? 1 : velocityRobot.getDouble(0) < 0 ? 0 : velocityRobot.getDouble(0);
  }
}
