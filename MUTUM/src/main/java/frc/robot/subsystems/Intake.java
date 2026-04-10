package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;


public class Intake {

    /* Intake INIT */
    public static TalonFX mIntakeL = new TalonFX(14);
    public static TalonFX mIntakeR = new TalonFX(15);

    public static TalonFX mArticulated = new TalonFX(16);
    public static PositionDutyCycle pidCtrArticulated = new PositionDutyCycle(0);
    /* Intake END */

    /* Belt INIT */
    public static final SparkMax mBelt = new SparkMax(18, MotorType.kBrushless);
    public static SparkClosedLoopController pidCtrBelt;
    /* Belt END */

    public Intake(){
        configArticulated(0.1, -0.1, 0.1, NeutralModeValue.Brake);
        configBelt(0.3, -1, 1);
    }

    /**
    * Configura a posição da articulação do intake.
    * @motor @param type 2 x Kraken X60
    * @param KP Ganho proporcional do sistema.
    * @param OutMin Valor de saida minimo do motor [LIMITE = -1].
    * @param OutMax Valor de saida maximo do motor [LIMITE = 1].
    */
    static void configIntake(double KP, double OutMin, double OutMax){
        TalonFXConfiguration cfgIntakeL = new TalonFXConfiguration();
        TalonFXConfiguration cfgIntakeR = new TalonFXConfiguration();

        cfgIntakeL.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgIntakeL.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgIntakeL.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfgIntakeL.CurrentLimits.SupplyCurrentLimit = 80;
        cfgIntakeL.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIntakeL.Feedback.SensorToMechanismRatio = 1.0;
        cfgIntakeL.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIntakeL.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIntakeL.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgIntakeL.Voltage.PeakForwardVoltage = 12;
        cfgIntakeL.Voltage.PeakReverseVoltage = -12;

        cfgIntakeR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgIntakeR.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgIntakeR.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfgIntakeR.CurrentLimits.SupplyCurrentLimit = 80;
        cfgIntakeR.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIntakeR.Feedback.SensorToMechanismRatio = 1.0;
        cfgIntakeR.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIntakeR.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIntakeR.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgIntakeR.Voltage.PeakForwardVoltage = 12;
        cfgIntakeR.Voltage.PeakReverseVoltage = -12;

        mIntakeL.getConfigurator().apply(cfgIntakeL);
        mIntakeR.getConfigurator().apply(cfgIntakeR);
    }

    /**
    * Retorna a posição dos coletores do intake.
    */
    static public double[] getIntakePosition(){
        return new double[] {
            mIntakeL.getPosition().getValueAsDouble(),
            mIntakeR.getPosition().getValueAsDouble()};
    }
    
    /**
    * Retorna a velocidade dos coletores do intake.
    */
    static public double[] getIntakeVelocity(){
        return new double[] {
            mIntakeL.getVelocity().getValueAsDouble(),
            mIntakeR.getVelocity().getValueAsDouble()};
    }
    
    /**
    * Retorna a velocidade dos coletores do intake.
    * @param speed Valor da velocidade do coletor [LIMITE = 1 a -1]
    */
    static public void setIntakeSpeed(double speed){
        mIntakeL.set(speed);
        mIntakeR.set(speed);
    }
    
    /**
    * Retorna a velocidade dos coletores do intake.
    * @param speed Valor da velocidade do coletor [LIMITE = 1 a -1]
    */
    static public void intakeStop(){
        mIntakeL.stopMotor();
        mIntakeR.stopMotor();
    }

    /**
    * Configura a posição da articulação do intake.
    * @motor @param type 1 x Kraken X60
    * @param KP Ganho proporcional do sistema.
    * @param OutMin Valor de saida minimo do motor [LIMITE = -1].
    * @param OutMax Valor de saida maximo do motor [LIMITE = 1].
    */
    public static void configArticulated(double KP, double OutMin, double OutMax, NeutralModeValue kMode) {
       TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimit = 80;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.MotorOutput.PeakForwardDutyCycle = OutMax;
        cfg.MotorOutput.PeakReverseDutyCycle = OutMin;

        cfg.Slot0.kP = KP;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
        cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;

        mArticulated.getConfigurator().apply(cfg);

    }

    /**
    * Retorna a posição da articulação do intake.
    */
    public static double getArticulatedPosition() {
        return mArticulated.getPosition().getValueAsDouble();
    }

    /**
    * Retorna a velocidade da articulação do intake.
    */
    public static double getArticulatedVelocity() {
        return mArticulated.getVelocity().getValueAsDouble();
    }

    /**
    * Configura a posição da articulação do intake.
    * @param kP Ganho proporcional do sistema.
    * @param position Valor da posição desejada.
    * @param speed Valor de velocidade maxima e minima do motor [LIMITE = 1 a -1].
    */
    public static void setArticulated(double kP, double position, double speed) {
        configArticulated(kP, -speed, speed, NeutralModeValue.Brake);
        mArticulated.setControl(pidCtrArticulated.withPosition(position));
    }

    /**
    * Deixa o motor livre.
    */
    static public void stopArticulated(){
        configArticulated(0, 0, 0, NeutralModeValue.Coast);
        mArticulated.set(0);
    }

    /**
    * Configura a posição da articulação do intake.
    * @motor @param type 1 x NEO
    * @param kP Ganho proporcional do sistema.
    * @param OutMin Valor de velocidade minima do motor [LIMITE = -1].
    * @param OutMax Valor de velocidade maxima do motor [LIMITE = 1].
    */
    static void configBelt(double kP, double OutMin, double OutMax){
        SparkMaxConfig cfgBelt = new SparkMaxConfig();

        cfgBelt.inverted(false).idleMode(IdleMode.kBrake);
        cfgBelt.openLoopRampRate(0.25);
        cfgBelt.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgBelt.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, 0.0, 0.0)
        .outputRange(OutMin, OutMax);

        mBelt.configure(cfgBelt, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrBelt = mBelt.getClosedLoopController();
    }

    /**
    * Retorna a posição do Belt
    */
    static public double getBeltPosition(){
        return mBelt.getEncoder().getPosition();
    }

    /**
    * Retorna a velocidade de Belt.
    */
    static public double getBeltVelocity(){
        return mBelt.getEncoder().getVelocity();
    }

    /**
    * Retorna a velocidade de Belt.
    * @param speed Velocidade do Belt [LIMITE = 1 a -1]
    */
    static public void setBeltSpeed(double speed){
        mBelt.set(speed);
    }

    /**
    * Desliga o Belt
    */
    static public void stopBelt(){
        mBelt.set(0);
    }
}
