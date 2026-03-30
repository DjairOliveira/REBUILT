// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Map;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;

  private static Turret mTurret = new Turret();
  private Command m_autonomousCommand;

  private XboxController driver = new XboxController(0);

  private RobotContainer m_robotContainer;

  public static double linearVelocity=0;
  public static double velX=0, velY=0;
  // public ChassisSpeeds speeds = m_robotContainer.drivebase.getRobotVelocity();

  // private double pose2dGetX, pose2dGetY, poseYaw;

  public static double[] pose = new double[3];

  public static final Pigeon2 mPigeon2 = new Pigeon2(13);

  private Timer disabledTimer;

  Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
  private NetworkTable limelightBack = NetworkTableInstance.getDefault().getTable("limelight-back");
  private NetworkTable limelightFront = NetworkTableInstance.getDefault().getTable("limelight-front");

  public static GenericEntry velocityRobot;
  public static GenericEntry velocityTiro;
  public static GenericEntry setInclina, HubAngle, RobotAngle;

  public static GenericEntry pipeline;
  public static GenericEntry teste;


  public Robot() {
    instance = this;

    velocityRobot = Shuffleboard.getTab("CONFIG")
        .add("Velocity", 0.5)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

    velocityTiro = Shuffleboard.getTab("CONFIG")
        .add("TIRO", 0.5)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

    setInclina = Shuffleboard.getTab("CONFIG")
        .add("INCLINATION", 0.5)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();
        

    teste = Shuffleboard.getTab("CONFIG")
        .add("Teste", -0.1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 0, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

    HubAngle = Shuffleboard.getTab("CONFIG")
        .add("HubAngle", -0.1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 0, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

    RobotAngle = Shuffleboard.getTab("CONFIG")
        .add("RobotAngle", -0.1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 0, "orientation", "VERTICAL"))
        .withSize(2, 6)
        .getEntry();

    pipeline = Shuffleboard.getTab("CONFIG")
        .add("Pipeline", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();



  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    initializeRobot();
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    pose = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Field").getEntry("Robot").getDoubleArray(new double[3]);
    NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("OdometryRobot").setDoubleArray(new double[] {
      pose[0],
      pose[1],
      Math.toRadians(pose[2])
    });

    // if(Turret.getHorizontal()>160 || Turret.getHorizontal()<-160){      //Proteção contra erros inesperados
    //   Turret.end();
    // }
    // if(driver.getLeftBumperButton()) Turret.periodic();
    // else Turret.end();
    

    // Turret.mShotter.set(Robot.teste.getDouble(0));
    // Turret.mShotterFlw.set(Robot.teste.getDouble(0));


    ChassisSpeeds speeds = m_robotContainer.drivebase.getRobotVelocity();
    linearVelocity = Math.hypot(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond);
    velX=speeds.vxMetersPerSecond;
    velY=speeds.vyMetersPerSecond;

    NetworkTableInstance.getDefault().getTable("INTAKE").getEntry("Inclina Position").setDouble(Intake.getInclina());
    NetworkTableInstance.getDefault().getTable("INTAKE").getEntry("Coletor Position").setDouble(Intake.getIntake());
    NetworkTableInstance.getDefault().getTable("INTAKE").getEntry("Inclina Velocity").setDouble(Intake.mInclina.getVelocity().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable("INTAKE").getEntry("Spindex Velocity").setDouble(Intake.mOrganizador.getEncoder().getVelocity());
    NetworkTableInstance.getDefault().getTable("INTAKE").getEntry("Esteira Velocity").setDouble(Intake.getSpeedEsteira());

    NetworkTableInstance.getDefault().getTable("CLIMBER").getEntry("Position").setDouble(Climber.getPosition());

    NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Horizontal Position").setDouble(Turret.getHorizontal());
    NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Horizontal Velocity").setDouble(Turret.getHorizontalVelocity());
    NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Vertical Position").setDouble(Turret.getVertical());
    NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Horizontal Position").setDouble(Turret.getHorizontal());
    NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Shooter Right").setDouble(Turret.mShooter.getVelocity().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Shooter Left").setDouble(Turret.mShooterFlw.getVelocity().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Is Finish").setBoolean(mTurret.isFinished());

    NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("Velocity X ms").setDouble(velX);
    NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("Velocity Y ms").setDouble(velY);
    NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("Linear Velocity").setDouble(linearVelocity);
    NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("Linear Velocity").setDouble(linearVelocity);
    NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("Angular Velocity").setDouble(speeds.omegaRadiansPerSecond);
    NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("YAW").setDouble(mPigeon2.getYaw().getValueAsDouble());

    limelightBack.getEntry("pipeline").setNumber(pipeline.getDouble(0));
    limelightFront.getEntry("pipeline").setNumber(pipeline.getDouble(0));

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    // m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
    
    // Turret.end();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      // m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    System.out.println("Auto selected: " + m_autonomousCommand);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    initializeRobot();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }

  void initializeRobot(){
    Intake.mIntake.getEncoder().setPosition(0);
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
