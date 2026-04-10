package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class SubSystemSIM extends SubsystemBase {

    private static double timerIntake;
    private static double timerShooter;
    private static boolean isArmazemFull = false;
    private static boolean isArmazemCleam = false;

    private double subGavetaPositon = 0.0;
    private double subIntakeAngle = 120;
    private double subIntakeVelocity = 0;
    private static double subShooterVelocity = 0;
    private double subClimberPositon = 0.0;

    private double intakeAngleSim = 0.0;
    private double climberPositionSim = 0.0;
    private double intakeVelocitySim = 0.0;
    private double shooterVelocitySim = 0.0;

    private PIDController intakePID = new PIDController(2, 0, 0.0);
    private PIDController climberPID = new PIDController(2, 0, 0.0);
    private PIDController intakeVelocity = new PIDController(1.5, 0, 0.0);
    private PIDController shooterVelocity = new PIDController(10, 0, 0.0);

    private final SwerveSubsystem swerve;
    
    public SubSystemSIM(SwerveSubsystem swerve) {
        this.swerve=swerve;
    }

    public void setIntakeVelocity(double speed){
        subIntakeVelocity = speed;
    }

    public void setSubIntake(double angle, double min, double max) {
        subIntakeAngle = Hood.map(angle, min, max, 120, 0);
    }

    public double getSubIntake(){
        return intakeAngleSim;
    }

    public void setSubClimber(double position, double min, double max) {
        double blueX = 4.298; // Aliança
        double redX = 12.41; // Aliança

        Pose2d robotPose = swerve.getPose();

        if((robotPose.getX() >= blueX-0.2 && robotPose.getX() <= (blueX-0.2)+1.2) || (robotPose.getX() >= (redX+0.2) - 1.2 && robotPose.getX() <= redX+0.2)){
            if(!isArmazemFull){
                if(position < 20)subClimberPositon = Hood.map(position, min, max, -0.1, 0.2);
            }
        }
        else{
            if(!isArmazemFull){
                subClimberPositon = Hood.map(position, min, max, -0.1, 0.2);
            }
            else{
                if(position >= 30) subClimberPositon = Hood.map(position, min, max, -0.1, 0.2);
            }
        }
    }

    public double getSubClimber(){
        return climberPositionSim;
    }

    public double getIntakeVelocity(){
        return intakeVelocitySim;
    }

    public void intakeVelocityCurrent(double speedAtual){
        intakeVelocitySim = speedAtual;
    }

    public double getShooterVelocity(){
        return shooterVelocitySim;
    }

    public static void setShooterVelocity(double speed){
        subShooterVelocity = speed;
    }
    
    public void shooterVelocityCurrent(double speedAtual){
        shooterVelocitySim = speedAtual;
    }

    @Override
    public void simulationPeriodic() {

        double intakeOutput = intakePID.calculate(intakeAngleSim, subIntakeAngle);
        intakeAngleSim += intakeOutput * 0.02;
        intakeAngleSim = MathUtil.clamp(intakeAngleSim, 0, 120);

        subGavetaPositon = Hood.map(intakeAngleSim, 0, 120, 0.28, 0);

        double climberOutput = climberPID.calculate(climberPositionSim, subClimberPositon);
        climberPositionSim += climberOutput * 0.02;

        double intakeVelocityOutput = intakeVelocity.calculate(intakeVelocitySim, subIntakeVelocity);
        intakeVelocitySim += intakeVelocityOutput * 0.02;

        double shooterVelocityOutput = shooterVelocity.calculate(shooterVelocitySim, subShooterVelocity);
        shooterVelocitySim += shooterVelocityOutput * 0.02;

        if (getIntakeVelocity() >= 1 && getIntakeVelocity() <= 3.5 && getSubIntake() < 3){ // Simulação do offset da velocidade
            timerIntake = Timer.getFPGATimestamp();  //Resistencia do intake
        }

        if (getShooterVelocity() >= 3.1 && getShooterVelocity() <= 3.4){ // talvez pegando a velocidade de Belt
            timerShooter = Timer.getFPGATimestamp();
        }

        isArmazemFull = Timer.getFPGATimestamp() - timerIntake > 1.25 ? false : true;
        isArmazemCleam = (Timer.getFPGATimestamp() - timerShooter) > 2 ? true : false;

        NetworkTableInstance.getDefault().getTable("INTAKESIM").getEntry("Speed").setDouble(intakeVelocitySim);
        NetworkTableInstance.getDefault().getTable("SHOOTERSIM").getEntry("Speed").setDouble(shooterVelocitySim);
        NetworkTableInstance.getDefault().getTable("INTAKESIM").getEntry("ArmazemCheio").setBoolean(isArmazemFull);
        NetworkTableInstance.getDefault().getTable("SHOOTERSIM").getEntry("ArmazemVazio").setBoolean(isArmazemCleam);

        Logger.recordOutput("RobotPose", new Pose2d());

        // subShooterAngle = Hood.map(mControl.getRightTriggerAxis(), 0, 1, -70, -110);

        Logger.recordOutput("SubSystemHood", new Pose3d[] {new Pose3d(
            0.345, 0, 0.43, new Rotation3d(0.0, Math.toRadians(Hood.getAngleHood()), Math.PI))});
        
        Logger.recordOutput("SubSystemIntake", new Pose3d[] {new Pose3d(
            -0.24, 0, 0.178, new Rotation3d(0.0, Math.toRadians(intakeAngleSim), 0))});

        Logger.recordOutput("SubSystemGaveta", new Pose3d[] {new Pose3d(
            0.0552 - subGavetaPositon, 0, 0.179, new Rotation3d(0.0, 0, 0))});

        Logger.recordOutput("SubSystemClimber", new Pose3d[] {new Pose3d(
           0.356, 0, climberPositionSim, new Rotation3d(0.0, 0, 0))});
    }
}
