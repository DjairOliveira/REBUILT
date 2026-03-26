// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot
{
  private static Robot instance;

  // private static Turret mTurret = new Turret();
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static double linearVelocity=0;

  public static final Pigeon2 mPigeon2 = new Pigeon2(13);

  private Timer disabledTimer;
  private double teleopStartTime = 0;
  public static double elapsedTime = 0;

  Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
  private NetworkTable limelightBack = NetworkTableInstance.getDefault().getTable("limelight-back");
  private NetworkTable limelightFront = NetworkTableInstance.getDefault().getTable("limelight-front");

  public static GenericEntry velocityRobot;
  public static GenericEntry velocityTiro;
  public static GenericEntry setInclina;

  public static GenericEntry pipeline;
  public static GenericEntry auxiliar;
  public static GenericEntry Driver;

  public static double[] pose = new double[3];

  public Robot()
  {
    instance = this;

    velocityRobot = Shuffleboard.getTab("CONFIG")
        .add("Velocity", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

    velocityTiro = Shuffleboard.getTab("CONFIG")
        .add("TIRO", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

    setInclina = Shuffleboard.getTab("CONFIG")
        .add("INCLINATION", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();
        

    auxiliar = Shuffleboard.getTab("CONFIG")
        .add("Auxiliar", -0.1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 0, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

    pipeline = Shuffleboard.getTab("CONFIG")
        .add("Pipeline", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    Driver = Shuffleboard.getTab("CONFIG")
        .add("Driver Mode", 2)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    initializeRobot();

    Logger.recordMetadata("ProjectName", "MeuRobo");

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
    } else {
        // SIMULAÇÃO
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
  }
// roborio-9168-frc.local lvuser@roborio-XXXX:~$
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    pose = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Field").getEntry("Robot").getDoubleArray(new double[3]);
    NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("OdometryRobot").setDoubleArray(new double[] {
      pose[0],
      pose[1],
      Math.toRadians(pose[2])
    });

    NetworkTableInstance.getDefault().getTable("PERIODIC INTAKE").getEntry("Inclina Position").setDouble(Intake.getInclina());
    NetworkTableInstance.getDefault().getTable("PERIODIC INTAKE").getEntry("Coletor Position").setDouble(Intake.getIntake());
    NetworkTableInstance.getDefault().getTable("PERIODIC INTAKE").getEntry("Inclina Velocity").setDouble(Intake.mInclina.getVelocity().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable("PERIODIC INTAKE").getEntry("Spindex Velocity").setDouble(Intake.mOrganizador.getEncoder().getVelocity());
    NetworkTableInstance.getDefault().getTable("PERIODIC INTAKE").getEntry("Esteira Velocity").setDouble(Intake.getSpeedEsteira());

    Logger.recordOutput("Intake/InclinaPosition", Intake.getInclina());
    Logger.recordOutput("Intake/ColetorPosition", Intake.getIntake());
    Logger.recordOutput("Intake/InclinaVelocity", Intake.mInclina.getVelocity().getValueAsDouble());
    Logger.recordOutput("Intake/Spindex Velocity", Intake.mOrganizador.getEncoder().getVelocity());
    Logger.recordOutput("Intake/Esteira Velocity", Intake.getSpeedEsteira());
    Logger.recordOutput("Intake/InclinaPosition", Intake.getInclina());

    NetworkTableInstance.getDefault().getTable("PERIODIC CLIMBER").getEntry("Position").setDouble(Climber.getPosition());

    NetworkTableInstance.getDefault().getTable("PERIODIC ROBOT").getEntry("TIMER").setDouble(elapsedTime);

    limelightBack.getEntry("pipeline").setNumber(pipeline.getDouble(0));
    limelightFront.getEntry("pipeline").setNumber(pipeline.getDouble(0));

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }
  
  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      // m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //Print the selected autonomous command upon autonomous init
    System.out.println("Auto selected: " + m_autonomousCommand);

    // schedule the autonomous command selected in the autoChooser
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }

    initializeRobot();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }

    teleopStartTime = 0;
    elapsedTime = 0;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    elapsedTime = Timer.getFPGATimestamp() - teleopStartTime;
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }

  void initializeRobot(){
    Intake.mIntake.getEncoder().setPosition(0);
    // Intake.mIntake.setPosition(0);
    Intake.mInclina.setPosition(0);
    Intake.mOrganizador.getEncoder().setPosition(0);
    Turret.mShooter.setPosition(0);
    Turret.mShooterFlw.setPosition(0);
    Turret.mVertical.getEncoder().setPosition(0);
    Turret.mHorizontal.getEncoder().setPosition(0);
    Turret.mShooter.setPosition(0);
    mPigeon2.setYaw(0);
    Climber.mclimber.setPosition(0);

    Turret.configHorizontal(0.02, -0.5, 0.5);
    Turret.configVertical(0.1, -0.1, 0.2);
    Turret.configEngatilha(NeutralModeValue.Coast);
    Turret.configShooter(NeutralModeValue.Coast);
    Intake.configInclina(0.15, -1, 1, NeutralModeValue.Brake);
  }

}