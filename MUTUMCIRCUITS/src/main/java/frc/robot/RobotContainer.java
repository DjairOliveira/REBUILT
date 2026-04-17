// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    // velocidade de translação e rotação. definidas na Tuner Constants
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final PIDController headingPid = new PIDController(3.0, 0.0, 0.0);

    private Rotation2d lockedHeading = new Rotation2d();
    private boolean isHeadingLocked = false;


    // // o robo ira dirigir de acordo com o campo.
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // @SuppressWarnings("unused")
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    final CommandXboxController Cmdriver = new CommandXboxController(0);
    public XboxController driver = new XboxController(0);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    // public CommandSwerveDrivetrain1 drivetrain = TunerConstants1.createDrivetrain();   // my
    private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public Hood mHood = new Hood(drivetrain);
    public Intake mIntake = new Intake();
    public Climber mClimber = new Climber();

    private int intakectn = 0;

    public double colisionProtect = 1;

    public RobotContainer() {

        configureBindings();

        // // Configuração do autochooser e definição das opções de auto
        // autoChooser = new SendableChooser<>();
        // autoChooser.setDefaultOption("TRENCH RIGHT", AutoBuilder.buildAuto("TrenchRight"));

        NamedCommands.registerCommand("updateFrontCAM",
            Commands.runOnce(() -> drivetrain.odometryUpdateAutonomo(drivetrain.mt2Front)));

        NamedCommands.registerCommand("HOOD_HUB", new Hood(drivetrain));

        // new EventTrigger("CLIMBER_UP").onTrue(Commands.runOnce(() -> mClimber.setPosition(-350, 1)));
        // new EventTrigger("CLIMBER_PUSH").onTrue(Commands.runOnce(() -> mClimber.setPosition(0, 1)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);

    }

    private void configureBindings() {

        // Command driveNormal = drivetrain.driveFieldOrientedScaled(
        //     () -> -driver.getLeftY() * MaxSpeed * drivetrain.getColision() * driver.getRightTriggerAxis(),
        //     () -> -driver.getLeftX() * MaxSpeed * drivetrain.getColision() * driver.getRightTriggerAxis(),
        //     () -> -drivetrain.getOmegaCmd() * MaxAngularRate * drivetrain.getColision(), 1, 1);

        // Command driveHood = drivetrain.driveFieldOrientedScaled(
        //     () -> -driver.getLeftY() * MaxSpeed * drivetrain.getColision() * driver.getRightTriggerAxis(),
        //     () -> -driver.getLeftX() * MaxSpeed * drivetrain.getColision() * driver.getRightTriggerAxis(),
        //     () -> -mHood.getOmega(), 1, 1);

        // BooleanSupplier driverShooter = () -> driver.getLeftBumperButtonPressed();
        // BooleanSupplier defaultMove = () -> driver.getLeftBumperButtonReleased() || Robot.elapsedTime < 1;
        // Command driveMode = driveNormal;

        // activateCommandOnCondition(defaultMove, driveMode = driveNormal);
        // activateCommandOnCondition(driverShooter, driveMode = driveHood);

        // drivetrain.setDefaultCommand(driveMode);
        // Cmdriver.start().onTrue((Commands.runOnce(drivetrain::zeroGyro)));
        // Cmdriver.start().onTrue(Commands.runOnce(drivetrain::configAngleInit));


        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
    

            // if(Math.abs(driver.getRightX()) < 0.1 && !driver.getYButton()){
            //     if (!isHeadingLocked) {
            //         lockedHeading = Rotation2d.fromDegrees(anguloRealModulado);
            //         headingPid.reset(); 
            //         isHeadingLocked = true;
            //     }

            //     double calcTrava = headingPid.calculate(
            //         Math.toRadians(anguloRealModulado),
            //         lockedHeading.getRadians()
            //     );

            //     rotOutput = headingPid.atSetpoint() ? 0.0 : calcTrava;
            //     rotOutput = MathUtil.clamp(rotOutput, -Constants.LIMITE_ROTACAO, Constants.LIMITE_ROTACAO);
            // }

            // else{
            //     rotOutput = -driver.getRightX();
            // }

            return drive
                .withVelocityX(-driver.getLeftY() * MaxSpeed * driver.getRightTriggerAxis())
                .withVelocityY(-driver.getLeftX() * MaxSpeed * driver.getRightTriggerAxis())
                .withRotationalRate(drivetrain.getOmegaCmd() * MaxSpeed);
        }));


        Cmdriver.start().onTrue(Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.configAngleInit()),
                Commands.waitSeconds(0.1),
                drivetrain.runOnce(() -> {
                    headingPid.reset();
                    isHeadingLocked = false;
                })
            )
        );


        // /********** INTAKE **************************/
        activateCommandOnCondition(() -> driver.getAButton(), new InstantCommand(() -> intakectn++));

        activateCommandOnCondition(() -> intakectn == 1, new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(3000)),
            Commands.runOnce(() -> mIntake.setArticulated(0.1, 22.394, 0.5)),
            Commands.runOnce(() -> mClimber.setPosition(110, 1))

            // Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setIntakeVelocity(4)),
            // Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubIntake(20, 3, 0, 20)),
            // Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubClimber(drivetrain.getPose(), 100, 5, 0, 100))
            ));

        activateCommandOnCondition(() -> intakectn >= 2, new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> mIntake.setArticulated(0.05, 0, 0.5)),

            // Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubIntake(0, 3, 0, 20)),
            // Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setIntakeVelocity(0)),
            new InstantCommand(() -> intakectn = 0)));

        // // Cmdriver.povLeft().onTrue(Commands.runOnce(() -> drivetrain.m_SubSystemSIM.intakeVelocityCurrent(1)));
        // // Cmdriver.povLeft().onTrue(Commands.runOnce(() -> drivetrain.m_SubSystemSIM.shooterVelocityCurrent(3.2)));


        activateCommandOnCondition(() -> mHood.getarticulaAux(), new SequentialCommandGroup(
            new InstantCommand(() -> mIntake.setArticulated(0.1, 10, 0.5)),
            Commands.runOnce(() -> mIntake.setIntakeRPM(5000))));

        activateCommandOnCondition(() -> !mHood.getarticulaAux(), new SequentialCommandGroup(
            new InstantCommand(() -> mIntake.setArticulated(0.05, 22.394, 0.5)),
            Commands.runOnce(() -> mIntake.setIntakeRPM(5000))));
        // activateCommandOnCondition(() -> mHood.getarticulaAux(),
        //         new InstantCommand(() -> mSubSystemSIM.setSubClimber(0, 1, 0, 100)));

        // /********** HOOD **************************/
        Cmdriver.b().onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> Hood.stopShooterSpeed()),
            Commands.runOnce(() -> Hood.stopIndexSpeed()),
            Commands.runOnce(() -> Hood.stopBelt()),
            Commands.runOnce(() -> mIntake.setIntakeRPM(0))
        ));

        Cmdriver.leftBumper().whileTrue(mHood.repeatedly());
        Cmdriver.leftBumper().onFalse(Commands.runOnce(() -> mHood.end()));

        // /* CLIMBER */

        // Cmdriver.povUp().onTrue(Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubClimber(drivetrain.getPose(), 100, 5, 0, 100)));
        // Cmdriver.povDown().onTrue(Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubClimber(drivetrain.getPose(), 0, 5, 0, 100)));
        
        Cmdriver.povUp().onTrue(Commands.runOnce(() -> mClimber.setPosition(110, 1)));
        Cmdriver.povDown().onTrue(Commands.runOnce(() -> mClimber.setPosition(0, 1)));

    }

    public Command getAutonomousCommand() {
        // Primeiro o robô reseta o giroscópio de forma segura e DEPOIS lê a trajetória
        // que irá realizar
        return Commands.sequence(
                // prepararInicioDePartidaCommand(),
                autoChooser.getSelected());
    }

    private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
        new Trigger(condition).onTrue(command);
    }
}