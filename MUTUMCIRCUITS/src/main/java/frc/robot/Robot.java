// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
//   private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public static final Pigeon2 mPigeon2 = new Pigeon2(13);

  private double teleopStartTime = 0;
  public static double elapsedTime = 0;

  public static GenericEntry RPMShooter;
  public static GenericEntry KVShooter;
  public static GenericEntry setHmax;

  public static GenericEntry pipeline;
  public static GenericEntry auxiliar;
  public static GenericEntry auxiliar2;
  public static GenericEntry Driver;

//   public static double[] pose = new double[3];

//   private Hood mHood = new Hood();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
            
        teleopStartTime = Timer.getFPGATimestamp();

        RPMShooter = Shuffleboard.getTab("CONFIG").add("RPMShooter", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(10, 10)
        .getEntry();

        setHmax = Shuffleboard.getTab("CONFIG").add("Hmax", 1)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
            
        auxiliar = Shuffleboard.getTab("CONFIG").add("Auxiliar", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

        auxiliar2 = Shuffleboard.getTab("CONFIG").add("Auxiliar2", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

        pipeline = Shuffleboard.getTab("CONFIG").add("setPipeline", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

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

        m_robotContainer.mIntake.setZeroArticulated();
        m_robotContainer.mHood.setZeroHood();
        m_robotContainer.mClimber.setZeroClimber();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        elapsedTime = Timer.getFPGATimestamp() - teleopStartTime;

        // // Pose2d currentPose = m_robotContainer.drivetrain.getPose();

        // // Rotation2d heading = currentPose.getRotation();
        // // ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_robotContainer.drivetrain.getState().Speeds, heading);

        // // double speedX = fieldSpeeds.vxMetersPerSecond;
        // // double speedY = fieldSpeeds.vyMetersPerSecond;

        // // if(m_robotContainer.mSubSystemSIM.getSubClimber() >= 30){
        // //     if(currentPose.getX() >= 3.7 && currentPose.getX() <= 5.5){
        // //         if((currentPose.getY() >= 0 && currentPose.getY() <= 1.4)){
        // //             if((currentPose.getX() - 4.628) < 0 && speedX > 0.5 || (currentPose.getX() - 4.628) > 0 && speedX < -0.5) m_robotContainer.colisionProtect = 0.25; 
        // //             if((currentPose.getX() - 4.628) < 0 && speedX < -0.5 || (currentPose.getX() - 4.628) > 0 && speedX > 0.5) m_robotContainer.colisionProtect = 1;
        // //         }
        // //         else if((currentPose.getY() >= 6.393 && currentPose.getY() <= 8.2)){
        // //             if((currentPose.getX() - 4.628) < 0 && speedX > 0.5 || (currentPose.getX() - 4.628) > 0 && speedX < -0.5) m_robotContainer.colisionProtect = 0.25; 
        // //             if((currentPose.getX() - 4.628) < 0 && speedX < -0.5 || (currentPose.getX() - 4.628) > 0 && speedX > 0.5) m_robotContainer.colisionProtect = 1;
        // //         }
        // //         else{
        // //             m_robotContainer.colisionProtect = 1;
        // //         }
        // //     }
        // // }
        // // else{
        // //     m_robotContainer.colisionProtect = 1;
        // // }

        double speedHood[] = Hood.getShooterVelocity();
        double speed[] = Intake.getIntakeVelocity();

        Logger.recordOutput("HOOD/v1", speedHood[0] * 60);
        Logger.recordOutput("HOOD/v2", speedHood[1] * 60);

        Logger.recordOutput("INTAKE/Coletor1Speed", speed[0] * 60);
        Logger.recordOutput("INTAKE/Coletor2Speed", speed[1] * 60);
        Logger.recordOutput("INTAKE/Articulation", Intake.getArticulatedPosition());
        Logger.recordOutput("Hood/Position", Hood.getHoodPositon());
        Logger.recordOutput("Hood/Index", Hood.getIndexVelocity());
        Logger.recordOutput("Hood/Shooter", Hood.getShooterVelocity());
        Logger.recordOutput("Climber", Climber.getPosition());

    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        
        // Inicialização segura no período autônomo
        // m_robotContainer.prepararInicioDePartida();
        
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
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        // Não pode executar durante a partida, 
        //m_robotContainer.prepararInicioDePartida();
        elapsedTime = 0;
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
