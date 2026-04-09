package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Optional;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.BateryFilter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import org.littletonrobotics.junction.Logger;

public class Hood extends Command {

    private static double startTime;

    public static final TalonFX mShooterR = new TalonFX(17);
    public static final TalonFX mShooterL = new TalonFX(16);
    public static final TalonFXConfiguration cfgShooter = new TalonFXConfiguration();
    public static final PositionDutyCycle PID = new PositionDutyCycle(0);

    public static final SparkMax mHood = new SparkMax(18, MotorType.kBrushless);
    public static final SparkMaxConfig cfgHood = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrHood;

    public static TalonFXS mIndex = new TalonFXS(21);
    public static TalonFXSConfiguration cfgIndex = new TalonFXSConfiguration();
    public static PositionDutyCycle pidCtrIndex = new PositionDutyCycle(0);

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    public static double[] pose = new double[3];

    public static double OmegaCmd = 0;
    public static double angleTurretSim = 0;

    public ChassisSpeeds speeds;

    double[] distances = {0.7, 1.05, 1.35, 1.65, 2, 2.551, 3.089, 3.52, 4.027, 4.635, 5.19, 5.7};
    double[] speed = {-0.4,-0.42,-0.44, -0.46, -0.485, -0.5, -0.525, -0.54, -0.5625, -0.59, -0.63, -0.65};

    private final SwerveSubsystem swerve;
    
    public Field2d field = new Field2d();

    public Hood(SwerveSubsystem swerve) {
        this.swerve=swerve;
        configHood(0.15, -0.15, 0.3);
        configIndex(NeutralModeValue.Coast);
        configShooter(NeutralModeValue.Coast);
    }

    public void execute() {

        double blueX = 4.298;   // Aliança
        double redX = 12.41;    // Aliança
        double targetX = 0;
        double targetY = 0;

        double poseHood = 0;
        double speedShooter = 0, speedBelt = 0, speedIndex = 0;

        Pose2d robotPose = swerve.getPose();
        boolean mode = false;
        boolean dispOk = false;


        if((!isRedAlliance() && robotPose.getX() <= blueX) || (isRedAlliance() && robotPose.getX() >= redX)){
            targetX = isRedAlliance() ? 11.914 : 4.624;
            targetY = 4.044;
            mode = true;
        }
        else if((!isRedAlliance() && robotPose.getX() > blueX + 1.4) || (isRedAlliance() && robotPose.getX() < redX - 1.4)){
            if((robotPose.getY() - 4.044) >= 0) targetY = 6;
            else targetY = 1.829;

            targetX = isRedAlliance() ? 14.32 : 2.331;
            mode = true;
        }
        else{
            if((robotPose.getY() - 4.044) >= 0) targetY = 6;
            else targetY = 1.829;

            targetX = isRedAlliance() ? 14.32 : 2.331;
            mode = false;
        }

        double distanceHood = hoodAling(targetX, targetY);

        if(mode){
            poseHood = map(parabola(distanceHood), 45, 80, 6, 0);
            speedShooter = Math.max(-0.7, Math.min(-0.52, interpolate(distanceHood, distances, speed) * Robot.velocityTiro.getDouble(0)));
        } else{
            poseHood = 0;
            speedShooter = -0.2;
            speedIndex = 0;
            speedBelt = 0;
        }

        setHood(poseHood);
        
        /*  PERMITIR DISPARO SOMENTE QUANDO O HOOD TIVER ALINHADO E NA VELOCIDADE CERTA */
        // if (Math.abs(TurretAngle + getHorizontal()) > 50) {
        //     speedShooter = -0.2;
        //     speedEgatilha = 0;
        //     speedEsteira = 0;
        //     speedOrganiza = 0;
        //     dispOk = false;
        // } else {
        //     dispOk = true;
        // }
        
        /*  AJUDA NO AUTOMO PARA PARAR DE DISPARAR */
        // if (dispOk && (getShooterVelocity() > (speedShooter * 100) - 3.5) && (getShooterVelocity() < (speedShooter * 100) + 3.5)){ // Testarrr
        //     speedEgatilha = -0.7;
        //     speedEsteira = -0.2;
        //     speedOrganiza = -0.7;
        // }
        // else {
        //     speedEgatilha = 0;
        //     speedEsteira = 0;
        //     speedOrganiza = 0;
        //     startTime = Timer.getFPGATimestamp();
        // }

        // setShotter(calculatePID(-2, speedShooter, (getShooterVelocity()/100), speedShooter, -1, -0.45));
        setFuel(speedIndex,speedBelt);

        // SmartDashboard.putNumber("value pid", calculatePID(-2, speedShooter, (getShooterVelocity()/100), speedShooter, -1, -0.52));

        NetworkTableInstance.getDefault().getTable("HOOD").getEntry("Inter speed").setDouble(interpolate(distanceHood, distances, speed));
        NetworkTableInstance.getDefault().getTable("HOOD").getEntry("Distance").setDouble(distanceHood);
        // NetworkTableInstance.getDefault().getTable("HOOD").getEntry("Angle").setDouble(TurretAngle);
        // NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Robot_X").setDouble(RobotX);
        // NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Robot_Y").setDouble(RobotY);
        // NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Robot_YAW").setDouble(RobotYAW);
        // NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Turret_X").setDouble(Turret_X);
        // NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Turret_Y").setDouble(Turret_Y);
        // NetworkTableInstance.getDefault().getTable("TURRET").getEntry("speedPower").setDouble(speedShooter);

        setLogger();
    }

    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 2 ? true : false;
    }

    public void end() {
        setHood(0);
        mShooterL.set(-0.2);
        mShooterR.set(-0.2);
        setIndex(0);
        Intake.setSpeedOrganizador(0);
        Intake.setSpeedEsteira(0);
    }

    private void setLogger(){
        Logger.recordOutput("Turret/VerticalPosition", getHood());
        Logger.recordOutput("Turret/ShooterRight", mShooterL.getVelocity().getValueAsDouble());
        Logger.recordOutput("Turret/ShooterLeft", mShooterR.getVelocity().getValueAsDouble());
        
    }

    private void setFuel (double engatilha, double esteira){
        setIndex(engatilha);
        Intake.setSpeedEsteira(esteira);

    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        double result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return result;
    }

    public static double parabola(double distturrettohub) {
        double delta_T = 0, a_T = 0, b_T = 0, c_T = 1.105;
        double tan_angle_T = 0;

        a_T = (Math.pow(distturrettohub, 2) / 9.22);
        b_T = -distturrettohub;
        delta_T = (Math.pow(b_T, 2) - 4 * a_T * c_T);
        tan_angle_T = (distturrettohub + Math.sqrt(delta_T)) / (2 * a_T);
        return Math.toDegrees(Math.atan(tan_angle_T));
    }

    private double hoodAling (double Target_X, double Target_Y){
        Pose2d robot_getValues = swerve.getPose();
        Rotation2d Robot_Yaw = robot_getValues.getRotation();
        Translation2d robot_pose = robot_getValues.getTranslation();

        Translation2d pose_Hub = new Translation2d(Target_X , Target_Y);
        Translation2d offsetHood = new Translation2d(0, -0.19719);  ///// Verificar se precisa pegar a distancia correta do Hood
        Translation2d poseHood = robot_pose.plus(offsetHood.rotateBy(Robot_Yaw));
        double distanceHood = poseHood.getDistance(pose_Hub);

        Translation2d angleRobot = pose_Hub.minus(robot_pose);
        Rotation2d targetAngle = angleRobot.getAngle();

        double error = MathUtil.angleModulus(targetAngle.minus(Robot_Yaw.plus(new Rotation2d(Math.PI))).getRadians());
        double kP = 1;
        OmegaCmd = kP * error;

        OmegaCmd = MathUtil.clamp(OmegaCmd, -3, 3);

        Translation2d roboHood = robot_pose.plus(new Translation2d(poseHood.getX(), poseHood.getY()));
        Translation2d robotToHUB = pose_Hub.minus(roboHood);
        Rotation2d robotAngle= robotToHUB.getAngle();

        // angleTurretSim = turret_toHub_FUTURE.getAngle().getRadians() - (Robot_Yaw.getRadians() + Math.PI);
        
        NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("Hood").setDoubleArray(new double[] {
        roboHood.getX(), roboHood.getY(), robotAngle.getRadians()});
        
        NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("OdometryRobot").setDoubleArray(new double[] {
        robot_pose.getX(), robot_pose.getY(), Robot_Yaw.getRadians()});

        NetworkTableInstance.getDefault().getTable("FIELD").getEntry("Target").setDoubleArray(new double[] {Target_X, Target_Y, Math.toRadians(0)});
        
        Pose2d robot = new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), new Rotation2d(swerve.getPose().getRotation().getRadians()));
        Pose2d target = new Pose2d(Target_X, Target_Y, new Rotation2d(0));

        
        SmartDashboard.putData("FIELD", field);
        field.getObject("Robot").setPose(robot);
        Logger.recordOutput("Robo/Odometry", robot);
        Logger.recordOutput("Turret/Target", target);

        return distanceHood;
    }

    public ChassisSpeeds getSpeeds() {
        return speeds;
    }

    public double getOmega(){
        return OmegaCmd;
    }

    public static double getAngleTurretSim(){
        return angleTurretSim;
    }

    // private static double calculatePID(double kP, double setpoint, double measurement, double baseOutput, double outputMin, double outputMax) {
    //     double kI = 0.0, kD = 0.0;

    //     double error = (setpoint - measurement) * (-1);

    //     integral += error * 0.02;
    //     double derivative = (error - previousError) / 0.02;
    //     previousError = error;

    //     double output = baseOutput + (kP * error) + (kI * integral) + (kD * derivative);
    //     output = Math.max(outputMin, Math.min(outputMax, output));

    //     return output;
    // }

    public double interpolate(double x, double[] xs, double[] ys) {

    for(int i=0;i<xs.length-1;i++){
        if(x >= xs[i] && x <= xs[i+1]){

            double ratio =
                (x - xs[i]) /
                (xs[i+1] - xs[i]);

            return ys[i] + ratio * (ys[i+1] - ys[i]);
        }
    }
    return ys[ys.length-1];
}

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public static void configShooter(NeutralModeValue kMode) {
        cfgShooter.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgShooter.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgShooter.MotorOutput.NeutralMode = kMode;
        cfgShooter.CurrentLimits.SupplyCurrentLimit = 200;  //200
        cfgShooter.CurrentLimits.SupplyCurrentLimitEnable = false; // false
        cfgShooter.Feedback.SensorToMechanismRatio = 1.0;
        cfgShooter.MotorOutput.PeakForwardDutyCycle = 1;
        cfgShooter.MotorOutput.PeakReverseDutyCycle = -1;
        cfgShooter.MotorOutput.DutyCycleNeutralDeadband = 0; ///0.1
        cfgShooter.Voltage.PeakForwardVoltage = 12;
        cfgShooter.Voltage.PeakReverseVoltage = -12;

        mShooterL.getConfigurator().apply(cfgShooter);
        mShooterR.getConfigurator().apply(cfgShooter);
    }

    static double getShooter() {
        return mShooterL.getPosition().getValueAsDouble();
    }

    static double getShooterVelocity(){
        return mShooterL.getVelocity().getValueAsDouble();
    }

    static public void stopShooterSpeed() {
        configShooter(NeutralModeValue.Coast);
        mShooterL.stopMotor();
        mShooterR.stopMotor();
    }

    static public void setShooter(double speedShooter){
        mShooterL.set(speedShooter);
        mShooterR.set(speedShooter);
    }

    public static void configHood(double P, double OutMin, double OutMax) {
        cfgHood.inverted(false).idleMode(IdleMode.kBrake);
        cfgHood.closedLoopRampRate(0.2);
        cfgHood.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgHood.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(P, 0.0, 0.0)
                .outputRange(OutMin, OutMax);

        mHood.configure(cfgHood, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrHood = mHood.getClosedLoopController();
    }

    static public double getHood() {
        return mHood.getEncoder().getPosition();
    }

    static public void setHood(double position) {
        pidCtrHood.setSetpoint(position, ControlType.kPosition);
    }

    public static void configIndex(NeutralModeValue kMode) {
        cfgIndex.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        cfgIndex.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgShooter.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgIndex.MotorOutput.NeutralMode = kMode;
        cfgIndex.CurrentLimits.SupplyCurrentLimit = 80;
        cfgIndex.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIndex.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIndex.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIndex.Voltage.PeakForwardVoltage = 12;
        cfgIndex.Voltage.PeakReverseVoltage = -12;
        cfgIndex.MotorOutput.DutyCycleNeutralDeadband = 0;

        mIndex.getConfigurator().apply(cfgIndex);
    }

    static public double getIndex() {
        return mIndex.getPosition().getValueAsDouble();
    }

    static public void setIndex(double speed) {
        mIndex.set(speed);
    }

    static public void stopIndex() {
        mIndex.setPosition(0);
        mIndex.set(0);
    }
}