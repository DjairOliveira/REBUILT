package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Turret extends Command {
    public static double linearVelocity = 0;
    private static double startTime;

    private static double integral = 0;
    private static double previousError = 0;
    public static double lastVelocity = 0;
    public static int stableLoops = 0;
    public static double distanceHUB = 0;

    public static final TalonFX mShotterFlw = new TalonFX(17);
    public static final TalonFX mShotter = new TalonFX(16);
    public static final TalonFXConfiguration cfgShotter = new TalonFXConfiguration();
    public static final PositionDutyCycle PID = new PositionDutyCycle(0);
    private final TorqueCurrentFOC torque = new TorqueCurrentFOC(0);

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

    private static final InterpolatingDoubleTreeMap inclineMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();

    public Turret() {
        // configHorizontal(0.2, -1, 1);
        configHorizontal(0.00955, -1, 1);
        configVertical(0.15, -0.15, 0.3);
        configEngatilha(NeutralModeValue.Coast);

        // inclineMap.put(1.259, 0.5);
        // inclineMap.put(1.93, 0.5);
        // inclineMap.put(2.04, 1.0);
        // inclineMap.put(2.36, 1.0);
        // inclineMap.put(2.65, 1.0);
        // inclineMap.put(2.92, 1.65);
        // inclineMap.put(3.23, 1.5);
        // inclineMap.put(3.38, 1.75);
        // inclineMap.put(3.82, 1.6);
        // inclineMap.put(4.011, 1.7);
        // inclineMap.put(4.192, 1.5);
        // inclineMap.put(4.20, 1.5);
        // inclineMap.put(4.42, 1.5);
        // inclineMap.put(4.51, 1.85);
        // inclineMap.put(4.99, 1.6);
        // inclineMap.put(5.62, 1.85);

        // shooterMap.put(1.259, -0.5);
        // shooterMap.put(1.93, -0.52);
        // shooterMap.put(2.04, -0.5);
        // shooterMap.put(2.36, -0.54);
        // shooterMap.put(2.65, -0.54);
        // shooterMap.put(2.92, -0.56);
        // shooterMap.put(3.23, -0.57);
        // shooterMap.put(3.38, -0.55);
        // shooterMap.put(3.82, -0.6);
        // shooterMap.put(4.011, -0.63);
        // shooterMap.put(4.192, -0.65);
        // shooterMap.put(4.20, -0.62);
        // shooterMap.put(4.42, -0.62);
        // shooterMap.put(4.51, -0.615);
        // shooterMap.put(4.99, -0.65);
        // shooterMap.put(5.62, -0.67);

        // timeMap.put(1.259, 1.0);
        // timeMap.put(1.93, 1.25);
        // timeMap.put(2.04, 1.0);
        // timeMap.put(2.36, 1.0);
        // timeMap.put(2.65, 1.125);
        // timeMap.put(2.92, 1.0);
        // timeMap.put(3.23, 1.5);
        // timeMap.put(3.38, 1.125);
        // timeMap.put(3.82, 1.375);
        // timeMap.put(4.011, 1.25);
        // timeMap.put(4.192, 1.75);
        // timeMap.put(4.20, 1.375);
        // timeMap.put(4.42, 1.75);
        // timeMap.put(4.51, 1.375);
        // timeMap.put(4.99, 1.25);
        // timeMap.put(5.62, 1.625);


        inclineMap.put(1.259, 0.5);
        inclineMap.put(1.93, 0.5);
        inclineMap.put(2.04, 0.85);
        inclineMap.put(2.36, 0.95);
        inclineMap.put(2.65, 1.10);
        inclineMap.put(2.92, 1.35);
        inclineMap.put(3.23, 1.50);
        inclineMap.put(3.38, 1.60);
        inclineMap.put(3.82, 1.60);
        inclineMap.put(4.011, 1.60);
        inclineMap.put(4.192, 1.55);
        inclineMap.put(4.20, 1.55);
        inclineMap.put(4.42, 1.60);
        inclineMap.put(4.51, 1.70);
        inclineMap.put(4.99, 1.72);
        inclineMap.put(5.62, 1.85);

        shooterMap.put(1.259, -0.50);
        shooterMap.put(1.93, -0.51);
        shooterMap.put(2.04, -0.52);
        shooterMap.put(2.36, -0.54);
        shooterMap.put(2.65, -0.545);
        shooterMap.put(2.92, -0.555);
        shooterMap.put(3.23, -0.565);
        shooterMap.put(3.38, -0.57);
        shooterMap.put(3.82, -0.60);
        shooterMap.put(4.011, -0.615);
        shooterMap.put(4.192, -0.62);
        shooterMap.put(4.20, -0.62);
        shooterMap.put(4.42, -0.622);
        shooterMap.put(4.51, -0.625);
        shooterMap.put(4.99, -0.645);
        shooterMap.put(5.62, -0.67);

        timeMap.put(1.259, 1.00);
        timeMap.put(1.93, 1.05);
        timeMap.put(2.04, 1.08);
        timeMap.put(2.36, 1.10);
        timeMap.put(2.65, 1.12);
        timeMap.put(2.92, 1.18);
        timeMap.put(3.23, 1.24);
        timeMap.put(3.38, 1.28);
        timeMap.put(3.82, 1.35);
        timeMap.put(4.011, 1.40);
        timeMap.put(4.192, 1.45);
        timeMap.put(4.20, 1.45);
        timeMap.put(4.42, 1.50);
        timeMap.put(4.51, 1.52);
        timeMap.put(4.99, 1.57);
        timeMap.put(5.62, 1.63);


        // // DISTÂNCIA -> INCLINAÇÃO
        // inclineMap.put(1.2242, 0.0);   // A
        // inclineMap.put(2.3633, 0.0);   // H
        // inclineMap.put(2.6000, 0.5);   // B
        // inclineMap.put(3.3808, 1.0);   // I
        // inclineMap.put(3.8186, 1.25);  // G
        // inclineMap.put(3.9719, 1.0);   // C
        // inclineMap.put(4.4179, 1.5);   // F
        // inclineMap.put(4.4531, 1.0);   // D
        // inclineMap.put(5.5465, 1.5);   // E

        // // DISTÂNCIA -> SHOOTER
        // shooterMap.put(1.2242, -0.42);   // A
        // shooterMap.put(2.3633, -0.55);   // H
        // shooterMap.put(2.6000, -0.51);   // B
        // shooterMap.put(3.3808, -0.55);   // I
        // shooterMap.put(3.8186, -0.575);  // G
        // shooterMap.put(3.9719, -0.60);   // C
        // shooterMap.put(4.4179, -0.60);   // F
        // shooterMap.put(4.4531, -0.60);   // D
        // shooterMap.put(5.5465, -0.65);   // E

        // // DISTÂNCIA -> TEMPO DE VIAGEM
        // timeMap.put(1.2242, 0.75);   // A
        // timeMap.put(2.3633, 1.375);  // H
        // timeMap.put(2.6000, 1.25);   // B
        // timeMap.put(3.3808, 1.25);   // I
        // timeMap.put(3.8186, 1.375);  // G
        // timeMap.put(3.9719, 1.375);  // C
        // timeMap.put(4.4179, 1.375);  // F
        // timeMap.put(4.4531, 1.50);   // D
        // timeMap.put(5.5465, 1.50);   // E
    }

    public void execute() {
        // Robot.pose[0] = Posição X do robô na arena
        // Robot.pose[1] = Posição Y do robô na arena
        // Robot.pose[2] = Posição Yaw do robô na arena
        // Muito importante a atualizaçã de odometria

        double HubX = 0; // POSIX BLUE
        double HubY = 0; // POSIY BLUE
        double dx = 0;
        double dy = 0;

        double blueX = 4.298; // Aliança
        double redX = 12.41; // Aliança

        HubX = isRedAlliance() ? 11.914 : 4.624;
        HubY = 4.044;

        double Robot_X = Robot.pose[0] - ((Robot.velX) * getInterpolatedTime(distanceHUB));
        double Robot_Y = Robot.pose[1] - ((Robot.velY) * getInterpolatedTime(distanceHUB)); // 1.18
        double Robot_YAW = Robot.pose[2]; // graus

        if ((!isRedAlliance() && Robot_X <= blueX) || (isRedAlliance() && Robot_X >= redX)) {
            double speedPower = 0.6;
            double positionVertical = 0;

            double XT = 0.14; // frente do robô
            double YT = 0.20; // esquerda do robô (se for direita, use -0.20)

            double theta = Math.toRadians(Robot_YAW + 180);

            double Turret_X = Robot_X + (XT * Math.cos(theta) - YT * Math.sin(theta));
            double Turret_Y = Robot_Y + (XT * Math.sin(theta) + YT * Math.cos(theta));

            distanceHUB = Math.hypot(Turret_X - HubX, Turret_Y - HubY);

            dx = Turret_X - HubX;
            dy = Turret_Y - HubY;

            double HubAngle = -Math.toDegrees(Math.atan(dy / dx));

            double compensation = 0;
            double RobotAngle = 0;
            if (!isRedAlliance()) {
                RobotAngle = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(Robot_YAW - 179.999)));
                if((Robot_Y - HubY) > 0) compensation = 4;
                else compensation = 0;
            }
            else {
                RobotAngle = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(Robot_YAW)));
                if((Robot_Y - HubY) > 0) compensation = 0;
                else compensation = 4;
            }

            double TurretSetpoint = MathUtil.inputModulus(HubAngle + RobotAngle + compensation, -135, 180);  //5
            setHorizontal(TurretSetpoint);

            NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("TURRET").setDoubleArray(new double[] {
            Turret_X, Turret_Y, Math.toRadians(Robot_YAW)});
            
            /* DE 1 PARA 1 OK com bateria boa */
            double incline = getInterpolatedIncline(distanceHUB);
            speedPower = getInterpolatedShooter(distanceHUB) * Robot.velocityTiro.getDouble(0);

            setVertical(incline * Robot.setInclina.getDouble(0));

            /* VALIDAÇÃO MANUAL */
            // positionVertical = Robot.setInclina.getDouble(0);
            // outputVertical = calculatePID(0.125, positionVertical - getVertical(), -0.1, 0.3);
            // setHorizontal(MathUtil.inputModulus((Robot.HubAngle.getDouble(0) - Robot.RobotAngle.getDouble(0)), -135, 180));

            double velocityplus = Math.max(1, Math.min(1.175, map(Robot.linearVelocity, 0, 2.5, 1, 1.175)));
            // double velocityplus=1;

            if (Math.abs(TurretSetpoint - getHorizontal()) < 5) {
                mShotter.set(speedPower * velocityplus);
                mShotterFlw.set(speedPower * velocityplus);
            } else {
                mShotter.set(-0.2);
                mShotterFlw.set(-0.2);
                setEngatilha(0);
                Intake.setSpeedOrganizador(0);
                Intake.setSpeedOrganizador1(0);
                // Intake.setSpeedEsteira(0);
            }

            double speedEngatilha = -0.7;

            if (mShotter.getVelocity().getValueAsDouble() < (speedPower * 100) + 10) {
                setEngatilha(speedEngatilha);
                Intake.setSpeedOrganizador(-0.7);
                Intake.setSpeedOrganizador1(0.6);
                // Intake.setSpeedEsteira(-0.15);
            } else {
                setEngatilha(0);
                Intake.setSpeedOrganizador(0);
                Intake.setSpeedOrganizador1(0);
                // Intake.setSpeedEsteira(0);
                startTime = Timer.getFPGATimestamp();
            }

            SmartDashboard.putNumber("speedpower", speedPower);
            SmartDashboard.putNumber("Hub Angle", HubAngle);
            SmartDashboard.putNumber("Robot Angle", RobotAngle);
            SmartDashboard.putNumber("TURRET Vertical", positionVertical);
            SmartDashboard.putNumber("TURRET Position", MathUtil.inputModulus(HubAngle - RobotAngle, -179.999, 179.999));
        }

        else if ((!isRedAlliance() && Robot_X > blueX + 1.4) || (isRedAlliance() && Robot_X < redX - 1.4)) {
            double positionVertical = 0;
            double speedPower = 0;
            double powerMin = 0.6, powerMax = 1;
            double inclinaMin = 2, inclinaMax = 4;

            if (!isRedAlliance()) {
                setHorizontal(MathUtil.inputModulus(Robot_YAW, -135, 180));
                if(Robot_Y >= 3.33 && Robot_Y <= 4.753){
                    positionVertical = Math.max(0, Math.min(2, map(Robot_X, blueX + 1.4, 11, 0, 2)));
                    setVertical(positionVertical);
                    speedPower = -map(Robot_X, blueX + 1.4, 11, 0.8, 1);
                }
                else {
                    positionVertical = Math.max(inclinaMin, Math.min(inclinaMax, map(Robot_X, blueX + 1.4, 11, inclinaMin, inclinaMax)));
                    setVertical(positionVertical);
                    speedPower = -map(Robot_X, blueX + 1.4, 11, powerMin, powerMax);
                }

            } else { //// FALTANDO RED

                setHorizontal(MathUtil.inputModulus(Robot_YAW + 180, -135, 180));
                positionVertical = map(Robot_X, 5.5, redX - 1.4, inclinaMin, inclinaMax);
                positionVertical = Math.max(inclinaMin, Math.min(inclinaMax, positionVertical));
                setVertical(positionVertical);

                speedPower = -map(Robot_X, 5.5, redX - 1.4, powerMin, powerMax);
            }

            // positionVertical = Math.max(inclinaMin, Math.min(inclinaMax, positionVertical));
            speedPower = -Math.max(powerMin, Math.min(powerMax, speedPower));

            if (mShotter.getVelocity().getValueAsDouble() < (speedPower * 100) + 5) {
                setEngatilha(-0.5);
                Intake.setSpeedOrganizador(-0.4);
                Intake.setSpeedOrganizador1(0.4);
            } else {
                startTime = Timer.getFPGATimestamp();
            }

            mShotter.set(speedPower);
            mShotterFlw.set(speedPower);

            SmartDashboard.putNumber("speedpower", speedPower);
        } else {
            mHorizontal.set(calculatePID(0.0085, getHorizontal(), -1, 1));
            mVertical.set(calculatePID(0.1, 0 - getVertical(), -0.1, 0.3));
            mShotter.set(0);
            mShotterFlw.set(0);
            setEngatilha(0);
            Intake.setSpeedOrganizador(0);
            Intake.setSpeedOrganizador1(0);
        }

        SmartDashboard.putNumber("Turret Distance", distanceHUB);
    }

    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 1.0 ? true : false;
    }

    public static void end() {
        setHorizontal(getHorizontal());
        setVertical(0);
        mShotter.set(-0.2);
        mShotterFlw.set(-0.2);
        setEngatilha(0);
        Intake.setSpeedOrganizador(0);
        Intake.setSpeedOrganizador1(0);
        Intake.setSpeedEsteira(0);
    }

    private static double calculatePID(double kP, double error, double outputMin, double outputMax) {
        double kI = 0.0, kD = 0.0;

        integral += error * 0.02;
        double derivative = (error - previousError) / 0.02;
        previousError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Math.max(outputMin, Math.min(outputMax, output));

        return output;
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        double result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return result;
    }

    public static double distanceHUB(double Robot_X, double Robot_Y, double Hub_X, double Hub_Y) {
        return Math.hypot(Robot_X - Hub_X, Robot_Y - Hub_Y);
    }

    public static double getInterpolatedIncline(double distance) {
        return inclineMap.get(distance);
    }

    public static double getInterpolatedShooter(double distance) {
        return shooterMap.get(distance);
    }

    public static double getInterpolatedTime(double distance) {
        return timeMap.get(distance);
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public static void configShotter(NeutralModeValue kMode) {
        cfgShotter.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgShotter.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgShotter.MotorOutput.NeutralMode = kMode;
        cfgShotter.CurrentLimits.SupplyCurrentLimit = 200;  //200
        cfgShotter.CurrentLimits.SupplyCurrentLimitEnable = false; // false
        cfgShotter.Feedback.SensorToMechanismRatio = 1.0;
        cfgShotter.MotorOutput.PeakForwardDutyCycle = 1;
        cfgShotter.MotorOutput.PeakReverseDutyCycle = -1;
        cfgShotter.MotorOutput.DutyCycleNeutralDeadband = 0; ///0.1
        cfgShotter.Voltage.PeakForwardVoltage = 12;
        cfgShotter.Voltage.PeakReverseVoltage = -12;

        mShotter.getConfigurator().apply(cfgShotter);
        mShotterFlw.getConfigurator().apply(cfgShotter);

    }

    static double getShotter() {
        return mShotter.getPosition().getValueAsDouble();
    }

    static public void stopSpeed() {
        configShotter(NeutralModeValue.Coast);
        mShotter.stopMotor();
        mShotterFlw.stopMotor();
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
        cfgShotter.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

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