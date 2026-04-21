// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    // velocidade de translação e rotação. definidas na Tuner Constants
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // private final PIDController headingPid = new PIDController(3.0, 0.0, 0.0);

    // // o robo ira dirigir de acordo com o campo.
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // @SuppressWarnings("unused")
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private boolean HoodOK = false;

    final CommandXboxController Cmdriver = new CommandXboxController(0);
    public XboxController driver = new XboxController(0);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public Hood mHood = new Hood();
    public Intake mIntake = new Intake();
    public Climber mClimber = new Climber();
    public SubSystemSIM mSim = new SubSystemSIM();

    private int intakectn = 0;
    private int climberMove = 0;

    public double colisionProtect = 1;

    public RobotContainer() {

        configureBindings();

        // NamedCommands.registerCommand("updateFrontCAM",
        // Commands.runOnce(() -> drivetrain.odometryUpdateAutonomo(drivetrain.mt2Front)));   bode bode
        NamedCommands.registerCommand(
            "HOOD_SHOOT",
            Commands.sequence(
                Commands.runOnce(() -> HoodOK = true),

                Commands.waitUntil(() ->
                    Hood.getAligned() && Hood.getIndexando()
                ),

                Commands.waitSeconds(4),

                Commands.runOnce(() -> HoodOK = false),
                Commands.runOnce(() -> mHood.end())
            )
        );

        NamedCommands.registerCommand("HOOD_ON", Commands.runOnce(() -> HoodOK = true));
        NamedCommands.registerCommand("HOOD_OFF", new SequentialCommandGroup(
            Commands.runOnce(() -> HoodOK = false),
            Commands.runOnce(() -> mHood.end()),
            Commands.runOnce(() -> mHood.cancel())));

        NamedCommands.registerCommand(
            "HOOD_AIN",
            Commands.defer(() -> {
                boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red;

                double targetX = red ? 11.914 : 4.624;
                double targetY = 4.044;

            return drivetrain.alignToTargetCommand(targetX, targetY);
            }, java.util.Set.of(drivetrain))
        );

        // NamedCommands.registerCommand("HOOD_ON", new InstantCommand(()-> drivetrain.stop), null)new Hood(drivetrain));
        // new EventTrigger("HOOD_OFF").onTrue(new SequentialCommandGroup(
        //     Commands.runOnce(() -> mHood.end()),
        //     Commands.runOnce(() -> mHood.cancel())));

        // // Configuração do autochooser e definição das opções de auto
        // autoChooser = new SendableChooser<>();
        // autoChooser.setDefaultOption("TRENCH RIGHT", AutoBuilder.buildAuto("TrenchRight"));

        new EventTrigger("SET_SHOTTER").onTrue(Commands.runOnce(() -> mHood.setShooterRPM(3000)));
        new EventTrigger("INTAKE_ON").onTrue(Commands.runOnce(() -> intakectn=1));
        new EventTrigger("INTAKE_AUX").onTrue(Commands.runOnce(() -> intakectn=1));
        new EventTrigger("CLIMBER_DOWN").onTrue(Commands.runOnce(() -> mClimber.setPosition(drivetrain.getPose(), 0, 1)));
        // new EventTrigger("CLIMBER_PUSH").onTrue(Commands.runOnce(() -> mClimber.setPosition(0, 1)));

        // activateCommandOnCondition(()-> Hood.getIndexando() && driver.getRightBumperButton(),
        //     new SequentialCommandGroup(
        //     Commands.runOnce(() -> Intake.setArticulated(0.0035, 0, 0.2)),
        //     Commands.runOnce(() -> mIntake.setIntakeRPM(2500))));

        // activateCommandOnCondition(()-> Intake.getArticulatedPosition() > 6 && Intake.getArticulatedPosition() < 8 && climberMove == 1, new SequentialCommandGroup(
        //     // Commands.runOnce(() -> mClimber.setPosition(drivetrain.getPose(), 0, 1)),
        //     Commands.runOnce(() -> climberMove = 0)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);

    }

    private void configureBindings() {

        /* SWERVE DRIVE */
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> { return drive
            .withVelocityX(-driver.getLeftY() * MaxSpeed * driver.getRightTriggerAxis() * drivetrain.getColision())
            .withVelocityY(-driver.getLeftX() * MaxSpeed * driver.getRightTriggerAxis() * drivetrain.getColision())
            .withRotationalRate(drivetrain.getOmegaCmd() * MaxSpeed);
        }));

        Cmdriver.start().onTrue(drivetrain.runOnce(() -> drivetrain.configAngleInit()));

        whileCommandOnCondition(()-> Hood.getAligned() && Hood.getIndexando() && driver.getRightBumperButton(), drivetrain.brakeX()); 

        /********** INTAKE **********/
        activateCommandOnCondition(() -> driver.getLeftBumperButton(), new InstantCommand(() -> intakectn++));

        activateCommandOnCondition(() -> intakectn == 1, new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(3500)),
            Commands.runOnce(() -> Intake.setArticulated(0.1, 22.394, 0.5)),
            Commands.runOnce(() -> mClimber.setPosition(drivetrain.getPose(), 110, 1))));

        activateCommandOnCondition(() -> intakectn >= 2, new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            new InstantCommand(() -> intakectn = 0)));

        activateCommandOnCondition(()-> driver.getAButton() && Intake.getArticulatedPosition() > 19 ,new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(-3500)),
            Commands.runOnce(() -> mHood.setBeltRPM(-4000)),
            Commands.runOnce(() -> mHood.setIndexRPM(-3000))));
        
        Cmdriver.a().onFalse(new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> mHood.setBeltRPM(0)),
            Commands.runOnce(() -> mHood.setIndexRPM(0))));

        /** AUXILIO INDEXER ***************/
        activateCommandOnCondition(()-> Hood.getIndexando() && driver.getRightBumperButton() && driver.getXButton(),
            new SequentialCommandGroup(
            Commands.runOnce(() -> Intake.setArticulated(0.015, 0, 0.1)),
            Commands.runOnce(() -> mIntake.setIntakeRPM(2500))));

        activateCommandOnCondition(()-> Intake.getArticulatedPosition() > 2 && Intake.getArticulatedPosition() < 8 && climberMove == 1, new SequentialCommandGroup(
            Commands.runOnce(() -> mClimber.setPosition(drivetrain.getPose(), 0, 1)),
            Commands.runOnce(() -> climberMove = 0)));

        // activateCommandOnCondition(()-> Climber.getPosition() < 20 && Hood.getIndexando() && driver.getRightBumperButton() && driver.getXButton(), 
        // Commands.runOnce(() -> Intake.setArticulated(0.015, 0, 0.1)));
        
        Cmdriver.x().onFalse(new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> Intake.setArticulated(0.1, 22.394, 0.5)),
            Commands.runOnce(() -> intakectn = 0)));

        /** HOOD **************************/
        Cmdriver.rightBumper().onTrue(Commands.runOnce(() -> climberMove = 1));
        
        Cmdriver.rightBumper().onFalse(new SequentialCommandGroup(
            Commands.runOnce(() -> mHood.end()),
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> Intake.setArticulated(0.1, 22.394, 0.5)),
            Commands.runOnce(() -> intakectn = 0)));

        // whileCommandOnCondition(()-> HoodOK, mHood.repeatedly());

        activateCommandOnCondition(() -> mHood.isFinished(),  Commands.runOnce(() -> HoodOK = false));

        Cmdriver.rightBumper().whileTrue(createHoodCommand());

        /*****    AUTONOMO  *****/
        whileCommandOnCondition(() -> HoodOK, createHoodCommand());

        whileCommandOnCondition(() -> HoodOK && Hood.getIndexando(), new SequentialCommandGroup(
            Commands.runOnce(() -> Intake.setArticulated(0.0035, 0, 0.2)),
            Commands.runOnce(() -> mIntake.setIntakeRPM(2500))));
        
        activateCommandOnCondition(()-> !HoodOK, new SequentialCommandGroup(
            Commands.runOnce(() -> mHood.end()),
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> Intake.setArticulated(0.1, 22.394, 0.5)),
            Commands.runOnce(() -> intakectn = 0)));
        /*****    AUTONOMO  *****/

        /** CLIMBER ***********************/
        Cmdriver.povUp().onTrue(Commands.runOnce(() -> mClimber.setPosition(drivetrain.getPose(), 110, 1)));
        Cmdriver.povDown().onTrue(Commands.runOnce(() -> mClimber.setPosition(drivetrain.getPose(), 0, 1)));

        /*  STOP ALL MOTOR */
        Cmdriver.b().onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> Hood.stopShooterSpeed()),
            Commands.runOnce(() -> Hood.stopIndexSpeed()),
            Commands.runOnce(() -> Hood.stopBelt()),
            Commands.runOnce(() -> mIntake.setIntakeRPM(0))
        ));
        
        activateCommandOnCondition(()-> Climber.getPosition() < 10 && driver.getBButton(), 
        Commands.runOnce(() -> Intake.setArticulated(0.1, 0, 0.5)));
        
        /*  SIMULATION */
        activateCommandOnCondition(() -> intakectn == 1, new SequentialCommandGroup(
            Commands.runOnce(() -> mSim.setIntakeVelocity(4)),
            Commands.runOnce(() -> mSim.setSubArticula(22.394, 3)),
            Commands.runOnce(() -> mSim.setSubClimber(drivetrain.getPose(), 110, 5))));

        activateCommandOnCondition(() -> intakectn >= 2, new SequentialCommandGroup(
            Commands.runOnce(() -> mSim.setSubArticula(0, 3)),
            Commands.runOnce(() -> mSim.setIntakeVelocity(0)),
            new InstantCommand(() -> intakectn = 0)));

        Cmdriver.povUp().onTrue(Commands.runOnce(() -> mSim.setSubClimber(drivetrain.getPose(), 110, 5)));
        Cmdriver.povDown().onTrue(Commands.runOnce(() -> mSim.setSubClimber(drivetrain.getPose(), 0, 5)));
        


        /* PUNHETADA POR POSIÇÃO Ajuda indexer */
        // activateCommandOnCondition(()-> Hood.getIndexando() && driver.getRightBumperButton(),
        //     new SequentialCommandGroup(
        //     Commands.runOnce(() -> Intake.setArticulated(0.0035, 0, 0.2)),
        //     Commands.runOnce(() -> mIntake.setIntakeRPM(2500))));

        // activateCommandOnCondition(()-> Intake.getArticulatedPosition() > 6 && Intake.getArticulatedPosition() < 8 && climberMove == 1, new SequentialCommandGroup(
        //     // Commands.runOnce(() -> mClimber.setPosition(drivetrain.getPose(), 0, 1)),
        //     Commands.runOnce(() -> climberMove = 0)));

        // activateCommandOnCondition(()-> Climber.getPosition() < 20 && Hood.getIndexando() && driver.getRightBumperButton(), 
        // Commands.runOnce(() -> Intake.setArticulated(0.015, 0, 0.1)));


        /*PUNHETADA POR TEMPO*/
        // activateCommandOnCondition(() -> mHood.getarticulaAux(), new SequentialCommandGroup(
        //     new InstantCommand(() -> Intake.setArticulated(0.1, 10, 0.5)),
        //     Commands.runOnce(() -> mIntake.setIntakeRPM(5000))));

        // activateCommandOnCondition(() -> !mHood.getarticulaAux(), new SequentialCommandGroup(
        //     new InstantCommand(() -> Intake.setArticulated(0.05, 22.394, 0.5)),
        //     Commands.runOnce(() -> mIntake.setIntakeRPM(5000))));
        /* */

        // activateCommandOnCondition(() -> mHood.getarticulaAux(),
        //         new InstantCommand(() -> mSubSystemSIM.setSubClimber(0, 1, 0, 100)));


    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.configAngleInit()),
                autoChooser.getSelected());
    }

    private Command createHoodCommand() {
        return Commands.defer(() -> new Hood(), java.util.Set.of());
    }

    public void configInit(){
        drivetrain.configAngleInit();
    }

    public boolean getHoodOK(){
        return HoodOK;
    }

    private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
        new Trigger(condition).onTrue(command);
    }

    private void whileCommandOnCondition(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

    public void setshotOk(boolean shotOk){
        drivetrain.shotOk = shotOk;
    }

}