package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Optional;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.BateryFilter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Turret extends Command {

    private static double startTime;
    private static int contIntake = 0;
    // private static double BatFilter=12.5;

    private static double integral = 0;
    private static double previousError = 0;

    public static final TalonFX mShooterFlw = new TalonFX(17);
    public static final TalonFX mShooter = new TalonFX(16);
    public static final TalonFXConfiguration cfgShooter = new TalonFXConfiguration();
    public static final PositionDutyCycle PID = new PositionDutyCycle(0);

    public static final SparkMax mVertical = new SparkMax(18, MotorType.kBrushless);
    public static final SparkMaxConfig cfgVertical = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrVertical;

    public static final SparkMax mHorizontal = new SparkMax(19, MotorType.kBrushless);
    public static final SparkMaxConfig cfgHorizontal = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrHorizontal;

    public static TalonFXS mEngatilha = new TalonFXS(21);
    public static TalonFXSConfiguration cfgEngatilha = new TalonFXSConfiguration();
    public static PositionDutyCycle pidCtrEngatilha = new PositionDutyCycle(0);

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    BateryFilter mBateryFilter = new BateryFilter();

    public static double[] pose = new double[3];

    double[] distances = {2, 2.551, 3.089, 3.52, 4.027, 4.635, 5.19, 5.7};
    double[] speed = {-0.485, -0.5, -0.525, -0.54, -0.5625, -0.59, -0.63, -0.65};

    private final SwerveSubsystem swerve;

    public Turret(SwerveSubsystem swerve) {
    this.swerve=swerve;
        configHorizontal(0.012, -1, 1);
        configVertical(0.15, -0.15, 0.3);
        configEngatilha(NeutralModeValue.Coast);
        configShooter(NeutralModeValue.Coast);
    }

    public void execute() {

        double blueX = 4.298; // Aliança
        double redX = 12.41; // Aliança
        double targetX = 0;
        double targetY = 0;

        double poseHorizontal = 0, poseVertical = 0;
        double speedShooter = 0, speedOrganiza = 0, speedEsteira = 0, speedEgatilha = 0;

        Pose2d robotPose = swerve.getPose();
        boolean mode = false;
        boolean dispOk = false;


        if((!isRedAlliance() && robotPose.getX() <= blueX) || (isRedAlliance() && robotPose.getX() >= redX)){
            targetX = isRedAlliance() ? 11.914 : 4.624;
            targetY = 4.044;
            mode = true;
        }
        else if((!isRedAlliance() && robotPose.getX() > blueX + 1.4) || (isRedAlliance() && robotPose.getX() < redX - 1.4)){
            if((robotPose.getY() - 4.044) >= 0){
                targetX = !isRedAlliance() ? 2.331 : 14.32;
                targetY = 6;
            } else{
                targetX = !isRedAlliance() ? 2.331 : 14.32;
                targetY = 1.829;
            }
            mode = true;
        }
        else{
            mode = false;
        }

        double DispMove[] = futureMove(targetX, targetY);

        double TurretDistance = DispMove[0];
        double TurretAngle= DispMove[1];
        double RobotX = DispMove[2];
        double RobotY = DispMove[3];
        double RobotYAW = DispMove[4];
        double Turret_X = DispMove[5];
        double Turret_Y = DispMove[6];


        NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("OdometryRobot").setDoubleArray(new double[] {
            RobotX, RobotY, Math.toRadians(RobotYAW-180)});

        NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("TurretPose").setDoubleArray(new double[] {
            Turret_X, Turret_Y, Math.toRadians(0)});


        if(mode){
            poseHorizontal = MathUtil.inputModulus(-DispMove[1], -165, 180);
            // poseVertical = Math.max(0, Math.min(2.8, interpolate(DispMove[0], distances, angle) * Robot.setInclina.getDouble(0)));
            poseVertical = map(Para_bola(TurretDistance), 45, 80, 6, 0);
            speedShooter = Math.max(-0.7, Math.min(-0.52, interpolate(TurretDistance, distances, speed) * Robot.velocityTiro.getDouble(0)));
            // speedShooter = Robot.velocityTiro.getDouble(0);
        } else{
            poseHorizontal = getHorizontal();
            poseVertical = 0;
            speedShooter = -0.2;
            speedEgatilha = 0;
            speedEsteira = 0;
            speedOrganiza = 0;
        }

        setHorizontal(poseHorizontal);
        setVertical(poseVertical);
        
        contIntake++;

        if (Math.abs(TurretAngle + getHorizontal()) > 50) {
            speedShooter = -0.2;
            speedEgatilha = 0;
            speedEsteira = 0;
            speedOrganiza = 0;
            dispOk = false;
        } else {
            dispOk = true;
        }
        

        if (dispOk && (getShooterVelocity() > (speedShooter * 100) - 3.5) && (getShooterVelocity() < (speedShooter * 100) + 3.5)){ // Testarrr
            speedEgatilha = -0.7;
            speedEsteira = -0.2;
            speedOrganiza = -0.7;
            if((!isRedAlliance() && robotPose.getX() <= blueX) || (isRedAlliance() && robotPose.getX() >= redX)){
                int timer = 5;
                if(contIntake < 7){
                    Intake.setInclina(-21, 0.2, 0.8);
                    Intake.setIntakeSpeed(1);
                }
                if(contIntake >= timer && contIntake < 10){
                    Intake.setInclina(-12, 0.2, 0.8);
                    Intake.setIntakeSpeed(1);
                }
                if(contIntake >= 10) contIntake = 0;

            }

        }
        else {
            speedEgatilha = 0;
            speedEsteira = 0;
            speedOrganiza = 0;
            startTime = Timer.getFPGATimestamp();
        }

        setShotter(calculatePID(-2, speedShooter, (getShooterVelocity()/100), speedShooter, -1, -0.45));
        setDisparo(speedEgatilha,speedOrganiza,speedEsteira);
        SmartDashboard.putNumber("value pid", calculatePID(-2, speedShooter, (getShooterVelocity()/100), speedShooter, -1, -0.52));

        // mBateryFilter.updateAndGetGain();
        // BatFilter = mBateryFilter.getFilteredVoltage();
        // SmartDashboard.putNumber("BATERIA", BatFilter);

        // NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Inter angle").setDouble(interpolate(TurretDistance, distances, angle));
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Inter speed").setDouble(interpolate(TurretDistance, distances, speed));
        // NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Inter future angle").setDouble(interpolate(DispMove[0], distances, angle));
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Inter future speed").setDouble(interpolate(DispMove[0], distances, speed));
        
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("TurretDistance").setDouble(TurretDistance);
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("TurretAngle").setDouble(TurretAngle);
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Robot_X").setDouble(RobotX);
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Robot_Y").setDouble(RobotY);
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Robot_YAW").setDouble(RobotYAW);
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Turret_X").setDouble(Turret_X);
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("Turret_Y").setDouble(Turret_Y);
        NetworkTableInstance.getDefault().getTable("TURRET").getEntry("speedPower").setDouble(speedShooter);

        SmartDashboard.putNumber("contador", contIntake);
        

    }

    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 2 ? true : false;
    }

    public static void end() {
        setHorizontal(getHorizontal());
        setVertical(0);
        mShooter.set(-0.2);
        mShooterFlw.set(-0.2);
        setEngatilha(0);
        Intake.setSpeedOrganizador(0);
        Intake.setSpeedEsteira(0);
    }

    private void setDisparo (double engatilha, double organizador, double esteira){
        setEngatilha(engatilha);
        Intake.setSpeedOrganizador(organizador);
        Intake.setSpeedEsteira(esteira);
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        double result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return result;
    }

    public static double Para_bola(double distturrettohub) {
        double delta_T = 0, a_T = 0, b_T = 0, c_T = 1.105;
        double tan_angle_T = 0;

        a_T = (Math.pow(distturrettohub, 2) / 9.22);
        b_T = -distturrettohub;
        delta_T = (Math.pow(b_T, 2) - 4 * a_T * c_T);
        tan_angle_T = (distturrettohub + Math.sqrt(delta_T)) / (2 * a_T);
        return Math.toDegrees(Math.atan(tan_angle_T));
    }

    private double[] futureMove (double Hub_X, double Hub_Y){
 
        Pose2d robot_getValues = swerve.getPose();
        ChassisSpeeds robo_speed = swerve.getFieldVelocity();

        Rotation2d Robot_Yaw = robot_getValues.getRotation();
        // Pose2d robotPose = swerve.getPose();
        Translation2d robot_pose = robot_getValues.getTranslation();

        ChassisSpeeds fielSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robo_speed, Robot_Yaw);

        double VelX = isRedAlliance() ? fielSpeeds.vxMetersPerSecond* (-1) : fielSpeeds.vxMetersPerSecond;
        double VelY = isRedAlliance() ? fielSpeeds.vyMetersPerSecond * (-1) : fielSpeeds.vyMetersPerSecond;

        Translation2d pose_Hub = new Translation2d(Hub_X , Hub_Y);
        Translation2d offsetTurret = new Translation2d(0.1425, 0.1535);   //tudo negativo//////////////************************************** */
        Translation2d turretOffSet = robot_pose.plus(offsetTurret.rotateBy(Robot_Yaw)); // offsetTurret.rotateBy(Robot_Yaw)
        double turretDistance_hub = turretOffSet.getDistance(pose_Hub);

        // Transform2d turretTransform = new Transform2d(offsetTurret, new Rotation2d());
        // Pose2d turretPose = robotPose.transformBy(turretTransform);
        
        NetworkTableInstance.getDefault().getTable("FIELD").getEntry("HUB").setDoubleArray(new double[] {Hub_X, Hub_Y, Math.toRadians(0)});

        double omega = robo_speed.omegaRadiansPerSecond;
        Translation2d turretOffsetField = offsetTurret.rotateBy(Robot_Yaw);
        double rotVelX = -omega * turretOffsetField.getY();
        double rotVelY = omega * turretOffsetField.getX();
        double TotalVelX = -VelX + rotVelX;
        double TotalVelY = -VelY + rotVelY;

        double kTimer = Math.max(1, Math.min(1.8, map(turretDistance_hub, 0.7, 7, 1, 1.8))) ; //1 e 1,75 good   
        double futureX = TotalVelX * kTimer;
        double futurey = TotalVelY * kTimer;
        Translation2d TurretFuture = turretOffSet.plus(new Translation2d(futureX, futurey));
        
        Translation2d turret_toHub_FUTURE = pose_Hub.minus(TurretFuture);
        Rotation2d angTurretToHub_FUTURE = turret_toHub_FUTURE.getAngle();
        Rotation2d turretTargetAngle_FUTURE = angTurretToHub_FUTURE.minus(Robot_Yaw);
        turretTargetAngle_FUTURE = turretTargetAngle_FUTURE.minus(Rotation2d.fromDegrees(180));
        
        double distance_FUTURE = turret_toHub_FUTURE.getNorm();

        return new double[] {
            distance_FUTURE, MathUtil.inputModulus(turretTargetAngle_FUTURE.getDegrees(), -165, 180),
            robot_getValues.getX(), robot_getValues.getY(), Robot_Yaw.getDegrees(),
            turretOffSet.getX(), turretOffSet.getY()};
    }

    private static double calculatePID(double kP, double setpoint, double measurement, double baseOutput, double outputMin, double outputMax) {
        double kI = 0.0, kD = 0.0;

        double error = (setpoint - measurement) * (-1);

        integral += error * 0.02;
        double derivative = (error - previousError) / 0.02;
        previousError = error;

        double output = baseOutput + (kP * error) + (kI * integral) + (kD * derivative);
        output = Math.max(outputMin, Math.min(outputMax, output));

        return output;
    }

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

        mShooter.getConfigurator().apply(cfgShooter);
        mShooterFlw.getConfigurator().apply(cfgShooter);
    }

    static double getShooter() {
        return mShooter.getPosition().getValueAsDouble();
    }

    static double getShooterVelocity(){
        return mShooter.getVelocity().getValueAsDouble();
    }

    static public void stopSpeed() {
        configShooter(NeutralModeValue.Coast);
        mShooter.stopMotor();
        mShooterFlw.stopMotor();
    }

    static public void setShotter(double speedShooter){
        mShooter.set(speedShooter);
        mShooterFlw.set(speedShooter);
    }

    public static void configHorizontal(double P, double OutMin, double OutMax) {
        cfgHorizontal.inverted(true).idleMode(IdleMode.kBrake);
        cfgHorizontal.closedLoopRampRate(0.075);
        cfgHorizontal.encoder.positionConversionFactor(15.619753397420465525766759383663).velocityConversionFactor(1);
        cfgHorizontal.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(P, 0.0, 0.0)
                .outputRange(OutMin, OutMax)
                .positionWrappingEnabled(false)
                .positionWrappingInputRange(-180, 180);

        mHorizontal.configure(cfgHorizontal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrHorizontal = mHorizontal.getClosedLoopController();
    }

    static public double getHorizontal() {
        return mHorizontal.getEncoder().getPosition();
    }

    static public double getHorizontalVelocity() {
        return mHorizontal.getEncoder().getVelocity();
    }

    static public void setHorizontal(double positionDegres) {
        pidCtrHorizontal.setSetpoint(positionDegres, ControlType.kPosition);
    }

    public static void configVertical(double P, double OutMin, double OutMax) {
        cfgVertical.inverted(false).idleMode(IdleMode.kBrake);
        cfgVertical.closedLoopRampRate(0.2);
        cfgVertical.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgVertical.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(P, 0.0, 0.0)
                .outputRange(OutMin, OutMax);

        mVertical.configure(cfgVertical, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrVertical = mVertical.getClosedLoopController();
    }

    static public double getVertical() {
        return mVertical.getEncoder().getPosition();
    }

    static public void setVertical(double position) {
        pidCtrVertical.setSetpoint(position, ControlType.kPosition);
    }

    public static void configEngatilha(NeutralModeValue kMode) {
        cfgEngatilha.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        cfgEngatilha.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgShooter.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgEngatilha.MotorOutput.NeutralMode = kMode;
        cfgEngatilha.CurrentLimits.SupplyCurrentLimit = 80;
        cfgEngatilha.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgEngatilha.MotorOutput.PeakForwardDutyCycle = 1;
        cfgEngatilha.MotorOutput.PeakReverseDutyCycle = -1;
        cfgEngatilha.Voltage.PeakForwardVoltage = 12;
        cfgEngatilha.Voltage.PeakReverseVoltage = -12;
        cfgEngatilha.MotorOutput.DutyCycleNeutralDeadband = 0;

        mEngatilha.getConfigurator().apply(cfgEngatilha);
    }

    static public double getEngatilha() {
        return mEngatilha.getPosition().getValueAsDouble();
    }

    static public void setEngatilha(double speed) {
        mEngatilha.set(speed);
    }

    static public void stopEngatilhar() {
        mEngatilha.setPosition(0);
        mEngatilha.set(0);
    }
}