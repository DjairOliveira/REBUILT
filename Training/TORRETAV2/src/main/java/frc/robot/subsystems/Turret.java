package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Map;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Turret extends Command {
    private final double offSetTX = 0;
    private final double offSetTY = 0;

    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double integral = 0;
    private double previousError = 0;

    private static boolean ctrTurret = true;
    private boolean ctrLimitP = false;
    private boolean ctrLimitN = false;

    public static final TalonFX mShotter = new TalonFX(17);
    public static final TalonFXConfiguration cfgShotter = new TalonFXConfiguration();
    public static final PositionDutyCycle PID = new PositionDutyCycle(0);

    public static final SparkMax mInclinaVertical = new SparkMax(18, MotorType.kBrushless);
    public static final SparkMaxConfig cfgInclinaVertical = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrInclinaVertical;

    public static final SparkMax mInclinaHorizontal = new SparkMax(19, MotorType.kBrushless);
    public static final SparkMaxConfig cfgInclinaHorizontal = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrInclinaHorizontal;

    public static final SparkMax mEngatilha = new SparkMax(20, MotorType.kBrushless);
    public static final SparkMaxConfig cfgEngatilha = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrEngatilha;

    public Turret() {
        configInclinaHorizontal(0.1, -0.1, 0.1);
        configInclinaVertical(0.1, -0.1, 0.2);
    }

    public void execute() {

        double tagTX = LimelightHelpers.getTX("limelight-turret");
        double tagTY = LimelightHelpers.getTY("limelight-turret");

        double output = 0;

        if (tagTX != 0 && tagTY != 0) {
            double motor = 0.1;
            double angulador = 45;
            if (ctrTurret) {
                if ((getInclinaHorizontal() >= -(180 + angulador)) && getInclinaHorizontal() <= (180 + angulador)) {
                    output = calculatePID(tagTX + offSetTX, motor, 0);
                }
                if (getInclinaHorizontal() < -(180 + angulador)) {
                    output = calculatePID(tagTX + offSetTX, motor, 1);
                    if (tagTX < -1) {
                        ctrTurret = false;
                    }
                }
                if (getInclinaHorizontal() > (180 + angulador)) {
                    output = calculatePID(tagTX + offSetTX, motor, 2);
                    if (tagTX > 1) {
                        ctrTurret = false;
                    }
                }
                mInclinaHorizontal.set(output); /// TESTAR COM MAP
            } else {
                if (getInclinaHorizontal() > (180 + angulador) && tagTX >= 1) {
                    setInclinaHorizontal(-(180 - angulador));
                }
                if (getInclinaHorizontal() < -(180 + angulador) && tagTX <= -1) {
                    setInclinaHorizontal((180 - angulador));
                }
                if (inRange(getInclinaHorizontal(), -(180 - angulador), 2.5) || inRange(getInclinaHorizontal(), (180 - angulador), 2.5)) {
                    ctrTurret = true;
                }
            }
            setInclinaVertical(map(tagTY, -25, 25, 6, 0));
        }

        SmartDashboard.putBoolean("CONTROL Sentido posi", ctrTurret);
        SmartDashboard.putBoolean("CONTROL Sentido nega", ctrLimitN);
    }

    public boolean isFinished() {
        return inRange(LimelightHelpers.getTX("limelight-turret"), 0, 0.5)
                || LimelightHelpers.getTX("limelight-turret") == 0 ? true : false;
    }

    public static void end() {
        setInclinaHorizontal(getInclinaHorizontal());
        setInclinaVertical(getInclinaVertical());
        ctrTurret = true;
    }

    boolean inRange(double read, double setpoint, double range) {
        return (((read - range) < setpoint) && ((read + range) > setpoint)) ? true : false;
    }

    private double calculatePID(double error, double outputRange, int restrict) {
        integral += error * 0.02; // supondo 20ms loop
        double derivative = (error - previousError) / 0.02;
        previousError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        if (restrict == 0)
            output = Math.max(-outputRange, Math.min(outputRange, output)); // Limita velocidade da torreta
        if (restrict == 1)
            output = Math.max(0, Math.min(outputRange, output)); // Limita velocidade da torreta
        if (restrict == 2)
            output = Math.max(-outputRange, Math.min(0, output)); // Limita velocidade da torreta

        return output;
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    static void configShotter(double KP, double OutMin, double OutMax) {
        cfgShotter.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgShotter.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 2;
        // cfg_shutter.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //inverted
        cfgShotter.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfgShotter.CurrentLimits.SupplyCurrentLimit = 40;
        cfgShotter.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfgShotter.Feedback.SensorToMechanismRatio = 1.0;
        cfgShotter.MotorOutput.PeakForwardDutyCycle = OutMax;
        cfgShotter.MotorOutput.PeakReverseDutyCycle = OutMin;

        Slot0Configs slot0 = cfgShotter.Slot0;
        slot0.kP = KP;
        slot0.kI = 0.0;
        slot0.kD = 0;

        mShotter.getConfigurator().apply(cfgShotter);
    }

    static double getShotter(){
        return mShotter.getPosition().getValueAsDouble();
    }

    static void setShotter(double position, double speed){
        configShotter(0.5, -speed, speed);
        mShotter.setControl(PID.withPosition(position));
    }

    static public void disparar(double speed) {
        // mShotter.set(-speed);
        mShotter.setPosition(0);
        setShotter(-20000, speed);
    }

    static public void stop() {
        mShotter.stopMotor();
    }

    static void configInclinaHorizontal(double P, double OutMin, double OutMax) {
        cfgInclinaHorizontal.inverted(true).idleMode(IdleMode.kBrake);
        cfgInclinaHorizontal.closedLoopRampRate(0.075);
        cfgInclinaHorizontal.encoder.positionConversionFactor(15.55564181842682).velocityConversionFactor(1);
        cfgInclinaHorizontal.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(P, 0.0, 0.0)
                .outputRange(OutMin, OutMax)
                .positionWrappingEnabled(false) /////// **/*/*/*/*/ */ */ */ */ */
                .positionWrappingInputRange(-180, 180);

        mInclinaHorizontal.configure(cfgInclinaHorizontal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrInclinaHorizontal = mInclinaHorizontal.getClosedLoopController();
    }

    static public double getInclinaHorizontal() {
        return mInclinaHorizontal.getEncoder().getPosition();
    }

    static public void setInclinaHorizontal(double positionDegres) {
        configInclinaHorizontal(0.025, -0.45, 0.45);
        pidCtrInclinaHorizontal.setSetpoint(positionDegres, ControlType.kPosition);
    }

    static void configInclinaVertical(double P, double OutMin, double OutMax) {
        cfgInclinaVertical.inverted(false).idleMode(IdleMode.kBrake);
        cfgInclinaVertical.closedLoopRampRate(0.2);
        cfgInclinaVertical.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgInclinaVertical.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(P, 0.0, 0.0)
                .outputRange(OutMin, OutMax);

        mInclinaVertical.configure(cfgInclinaVertical, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrInclinaVertical = mInclinaVertical.getClosedLoopController();
    }

    static public double getInclinaVertical() {
        return mInclinaVertical.getEncoder().getPosition();
    }

    static public void setInclinaVertical(double position) {
        configInclinaVertical(0.1, -0.1, 0.2);
        pidCtrInclinaVertical.setSetpoint(position, ControlType.kPosition);
    }

    static void configEngatilha(double P, double OutMin, double OutMax) {
        cfgEngatilha.inverted(true).idleMode(IdleMode.kBrake);
        cfgEngatilha.closedLoopRampRate(0.2);
        cfgEngatilha.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgEngatilha.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(P, 0.0, 0.0)
                .outputRange(OutMin, OutMax);

        mEngatilha.configure(cfgEngatilha, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrEngatilha = mEngatilha.getClosedLoopController();
    }

    static public double getEngatilha() {
        return mEngatilha.getEncoder().getPosition();
    }

    static public void setEngatilha(double position) {
        configEngatilha(0.3, -0.75, 0.75);
        pidCtrEngatilha.setSetpoint(position, ControlType.kPosition);
    }

    static public void engatilhar(){
        mEngatilha.getEncoder().setPosition(0);
        setEngatilha(10000);
    }

    static public void stopEngatilhar(){
        mEngatilha.getEncoder().setPosition(0);
        setEngatilha(0);
    }

}