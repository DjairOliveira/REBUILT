// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.networktables.GenericEntry;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.BooleanSupplier;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public final double inclinationMax = 6;
  final CommandXboxController Cmdriver = new CommandXboxController(0);
  public XboxController driver = new XboxController(0);
  
  public final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/MK4i"));
  private Turret mTurret = new Turret(drivebase);
  private Climber mClimber = new Climber(drivebase);

  double shooterMov=0;

  private int intakectn=0;
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocityJose = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> Cmdriver.getLeftY() * -normalized(Robot.velocityRobot) * Cmdriver.getRightTriggerAxis(),
                                                                () -> Cmdriver.getLeftX() * -normalized(Robot.velocityRobot) * Cmdriver.getRightTriggerAxis())
                                                            .withControllerRotationAxis(() -> (Cmdriver.getRightX() * -normalized(Robot.velocityRobot)  * 0.5))
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityYuri = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> Cmdriver.getLeftY() * -normalized(Robot.velocityRobot),
                                                                () -> Cmdriver.getLeftX() * -normalized(Robot.velocityRobot))
                                                            .withControllerRotationAxis(() -> (Cmdriver.getRightX() * -normalized(Robot.velocityRobot)) * 0.8)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  
    new EventTrigger("INTAKE_ON").onTrue(new SequentialCommandGroup(
      Commands.runOnce(()-> Intake.setInclina(-21.9,0.15,1)),
      Commands.waitSeconds(0.3),
      Commands.runOnce(()-> Intake.setIntakeSpeed(1)),
      Commands.runOnce(()-> Intake.setSpeedEsteira(0)),
      Commands.runOnce(()-> Intake.setSpeedOrganizador(-0.08)),
      Commands.runOnce(()-> Turret.setEngatilha(-0.05))));
      
    NamedCommands.registerCommand("INTAKE_OFF", new SequentialCommandGroup(
      Commands.runOnce(()-> Intake.setInclina(-14,0.3,0.25)),
      new InstantCommand(()-> Intake.setSpeedOrganizador(0)),
      Commands.runOnce(()-> Turret.setEngatilha(0)),
      Commands.runOnce(()-> Intake.setIntakeSpeed(1))));

    new EventTrigger("TURRET_OFF").onTrue(new SequentialCommandGroup(
      Commands.runOnce(() -> Turret.end()),
      Commands.runOnce(() -> mTurret.cancel())));

    new EventTrigger("TURRET_MOVE").onTrue(new Turret(drivebase));
      
    NamedCommands.registerCommand("TURRET_HUB", new Turret(drivebase));

    new EventTrigger("CLIMBER_UP").onTrue(Commands.runOnce(() -> mClimber.setPosition(-350, 1)));
    new EventTrigger("CLIMBER_PUSH").onTrue(Commands.runOnce(() -> mClimber.setPosition(0, 1)));

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
    // Command driverJose = drivebase.driveFieldOriented(driveAngularVelocityJose);
    Command driverYuri = drivebase.driveFieldOriented(driveAngularVelocityYuri);

    drivebase.setDefaultCommand(driverYuri);
    Cmdriver.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // Cmdriver.leftBumper().onTrue((Commands.runOnce(drivebase::lock, drivebase)));  APENAS PARADO

    /********** INTAKE **************************/
    BooleanSupplier protectIntake = ()-> Intake.getInclina() > -15 && Intake.getInclina() < -3. && Intake.getVelocityInclina() > 0 && intakectn==1;
    BooleanSupplier safeInclina = ()-> Intake.getInclina() < -15 && intakectn==1;
    BooleanSupplier coletarsafe = ()-> Intake.getInclina() > -3;
    
    activateCommandOnCondition(()-> driver.getAButton(), new InstantCommand(()-> intakectn++));

    activateCommandOnCondition(()-> intakectn==1, new SequentialCommandGroup(
      Commands.runOnce(()-> Intake.setInclina(-21.9,0.15,1)),
      Commands.waitSeconds(0.3),
      Commands.runOnce(()-> Intake.setIntakeSpeed(1)),
      Commands.runOnce(()-> Intake.setSpeedEsteira(0)),
      Commands.runOnce(()-> Intake.setSpeedOrganizador(-0.08)),
      Commands.runOnce(()-> Turret.setEngatilha(-0.05))));

    // activateCommandOnCondition(()-> intakectn==2, Commands.runOnce(()-> Intake.setIntakeSpeed(0)));

    // activateCommandOnCondition(()-> intakectn==3, new SequentialCommandGroup(
    //   Commands.runOnce(()-> Intake.setInclina(0,0.3,0.25)),
    //   new InstantCommand(()-> Intake.setSpeedOrganizador(0)),
    //   Commands.runOnce(()-> Turret.setEngatilha(0)),
    //   Commands.runOnce(()-> Intake.setIntakeSpeed(1)),
    //   new InstantCommand(()-> intakectn=0)));

    activateCommandOnCondition(()-> intakectn>=2, new SequentialCommandGroup(
      Commands.runOnce(()-> Intake.setInclina(0,0.3,0.25)),
      new InstantCommand(()-> Intake.setSpeedOrganizador(0)),
      Commands.runOnce(()-> Turret.setEngatilha(0)),
      Commands.runOnce(()-> Intake.setSpeedEsteira(0)),
      Commands.runOnce(()-> Intake.setIntakeSpeed(1)),
      new InstantCommand(()-> intakectn=0)));

    activateCommandOnCondition(protectIntake, new InstantCommand(()-> intakectn=2));
    activateCommandOnCondition(safeInclina, Commands.runOnce(()-> Intake.stopInclina()));
    activateCommandOnCondition(coletarsafe, Commands.runOnce(()-> Intake.setIntakeSpeed(0)));

    /********** TURRET **************************/
    Cmdriver.b().onTrue(new SequentialCommandGroup(
      Commands.runOnce(() -> Turret.stopSpeed()),
      Commands.runOnce(() -> Turret.stopEngatilhar()),
      Commands.runOnce(() -> Intake.stopOrganizador()),
      Commands.runOnce(() -> Intake.setSpeedEsteira(0)),
      Commands.runOnce(() -> Turret.setHorizontal(0))));

    Cmdriver.leftBumper().whileTrue(mTurret.repeatedly());
    Cmdriver.leftBumper().onFalse(Commands.runOnce(() -> Turret.end()));

    Cmdriver.rightBumper().whileTrue(new SequentialCommandGroup(
      Commands.runOnce(() -> Turret.setEngatilha(0.4)),
      Commands.runOnce(() -> Turret.setShotter(0.4)),
      Commands.runOnce(() -> Intake.setSpeedOrganizador(0.2)),
      Commands.runOnce(() -> Intake.setSpeedEsteira(0.2)),
      Commands.runOnce(() -> Intake.setIntakeSpeed(1))));

    Cmdriver.rightBumper().onFalse(new SequentialCommandGroup(
      Commands.runOnce(() -> Turret.setEngatilha(0)),
      Commands.runOnce(() -> Turret.setShotter(0)),
      Commands.runOnce(() -> Intake.setSpeedOrganizador(0)),
      Commands.runOnce(() -> Intake.setSpeedEsteira(0)),
      Commands.runOnce(() -> Intake.setIntakeSpeed(0))));

    /*  CLIMBER  */
    
    Cmdriver.povDown().onTrue(Commands.runOnce(() -> mClimber.setPosition(0, 1))); //359 max alto
    Cmdriver.povUp().onTrue(Commands.runOnce(() -> mClimber.setPosition(-350, 1)));

  } 

  private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }
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
    return Math.max(0, Math.min(1, velocityRobot.getDouble(0)));
  }
}
