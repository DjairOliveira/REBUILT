package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.Slot0Configs;
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

public class Intake {

    public static final SparkMax mIntake = new SparkMax(15, MotorType.kBrushless);
    public static final SparkMaxConfig cfgIntake = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrIntake;

    // public static final TalonFX mIntake = new TalonFX(15);
    // public static final TalonFXConfiguration cfgIntake = new TalonFXConfiguration();

    public static TalonFXS mInclina = new TalonFXS(14);
    public static TalonFXSConfiguration cfgInclina = new TalonFXSConfiguration();
    public static PositionDutyCycle pidCtrInclina = new PositionDutyCycle(0);

    public static final SparkMax mOrganizador = new SparkMax(20, MotorType.kBrushless);
    public static final SparkMaxConfig cfgOrganizador = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrOrganizador;

    public static final SparkMax mEsteira = new SparkMax(24, MotorType.kBrushless);
    public static final SparkMaxConfig cfgEsteira = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrEsteira;



    public Intake(){
        configInclina(0.15, -1, 1, NeutralModeValue.Brake);
        configOrganizador(0.3, -1, 1);
        configEsteira(0.3, -1, 1);
    }

    static void configIntake(double P, double OutMin, double OutMax){
        cfgIntake.inverted(false).idleMode(IdleMode.kBrake);
        cfgIntake.openLoopRampRate(0.1);
        cfgIntake.closedLoopRampRate(0.1);
        cfgIntake.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgIntake.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(OutMin, OutMax);

        mIntake.configure(cfgIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrIntake = mIntake.getClosedLoopController();
    }

    static public double getIntake(){
        return mIntake.getEncoder().getPosition();
    }
    static public double getIntakeSpeed(){
        return mIntake.getEncoder().getVelocity();
    }

    static public void setIntake(double position){
        mIntake.getEncoder().setPosition(0);
        configIntake(0.4, -0.4, 0.4);
        pidCtrIntake.setSetpoint(position, ControlType.kPosition);
    }

    static public void setIntakeSpeed(double speed){
        configIntake(2, -speed, speed);
        mIntake.set(speed);
    }

    static public void IntakeStop(){
        mIntake.getEncoder().setPosition(0);
        configIntake(0.25, -0.15, 0.15);
        pidCtrIntake.setSetpoint(0, ControlType.kPosition);
        mIntake.getEncoder().setPosition(0);
    }

    // public static void configIntake(NeutralModeValue kMode) {
    //     cfgIntake.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //     cfgIntake.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

    //     cfgIntake.MotorOutput.NeutralMode = kMode;
    //     cfgIntake.CurrentLimits.SupplyCurrentLimit = 80;
    //     cfgIntake.CurrentLimits.SupplyCurrentLimitEnable = false;
    //     cfgIntake.Feedback.SensorToMechanismRatio = 1.0;
    //     cfgIntake.MotorOutput.PeakForwardDutyCycle = 1;
    //     cfgIntake.MotorOutput.PeakReverseDutyCycle = -1;
    //     cfgIntake.MotorOutput.DutyCycleNeutralDeadband = 0;
    //     cfgIntake.Voltage.PeakForwardVoltage = 12;
    //     cfgIntake.Voltage.PeakReverseVoltage = -12;

    //     mIntake.getConfigurator().apply(cfgIntake);
    // }

    // public static double getIntake() {
    //     return mIntake.getPosition().getValueAsDouble();
    // }

    // static double getIntakeSpeed(){
    //     return mIntake.getVelocity().getValueAsDouble();
    // }

    // static public void setIntakeSpeed(double speed) {
    //     mIntake.set(speed);
    // }

    // static public void IntakeStop() {
    //     configIntake(NeutralModeValue.Coast);
    //     mIntake.stopMotor();
    // }

    public static void configInclina(double KP, double OutMin, double OutMax, NeutralModeValue kMode) {
        cfgInclina.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        cfgInclina.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgInclina.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;

        cfgInclina.MotorOutput.NeutralMode = kMode;
        cfgInclina.CurrentLimits.SupplyCurrentLimit = 40;
        cfgInclina.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfgInclina.Audio.BeepOnConfig = true;
        cfgInclina.MotorOutput.PeakForwardDutyCycle = OutMax;
        cfgInclina.MotorOutput.PeakReverseDutyCycle = OutMin;
        cfgInclina.Voltage.PeakForwardVoltage = 12;
        cfgInclina.Voltage.PeakReverseVoltage = -12;

        Slot0Configs slot0 = cfgInclina.Slot0;
        slot0.kP = KP;
        slot0.kI = 0.0;
        slot0.kD = 0;

        mInclina.getConfigurator().apply(cfgInclina);

    }

    public static double getInclina() {
        return mInclina.getPosition().getValueAsDouble();
    }

    public static double getVelocityInclina() {
        return mInclina.getVelocity().getValueAsDouble();
    }

    public static void setInclina(double position, double kP, double speed) {
        configInclina(kP, -speed, speed, NeutralModeValue.Brake);
        mInclina.setControl(pidCtrInclina.withPosition(position));
    }

    static public void stopInclina(){
        configInclina(0, 0, 0, NeutralModeValue.Coast);
        mInclina.set(0);
    }

    static void configOrganizador(double P, double OutMin, double OutMax){
        cfgOrganizador.inverted(false).idleMode(IdleMode.kBrake);
        cfgOrganizador.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgOrganizador.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(OutMin, OutMax);

        mOrganizador.configure(cfgOrganizador, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrOrganizador = mOrganizador.getClosedLoopController();
    }

    static public double getOrganizador(){
        return mOrganizador.getEncoder().getPosition();
    }

    static public void setSpeedOrganizador(double speed){
        mOrganizador.set(speed);
    }

    static public void stopOrganizador(){
        mOrganizador.set(0);
    }

    static public double getOrganizadorVelocity(){
        return mOrganizador.getEncoder().getVelocity();
    }

    static public double getOrganizadorCurrent(){
        return mOrganizador.getOutputCurrent();
    }

    static void configEsteira(double P, double OutMin, double OutMax){
        cfgEsteira.inverted(false).idleMode(IdleMode.kBrake);
        cfgEsteira.openLoopRampRate(0.25);
        cfgEsteira.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgEsteira.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(OutMin, OutMax);

        mEsteira.configure(cfgEsteira, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrEsteira = mEsteira.getClosedLoopController();
    }

    static public double getEsteira(){
        return mEsteira.getEncoder().getPosition();
    }
    static public double getSpeedEsteira(){
        return mEsteira.getEncoder().getVelocity();
    }

    static public void setSpeedEsteira(double speed){
        mEsteira.set(speed);
    }

    static public void stopEsteira(){
        mEsteira.set(0);
    }
}
