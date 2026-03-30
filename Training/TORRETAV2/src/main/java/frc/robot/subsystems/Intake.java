package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake {

    public static final SparkMax mIntake = new SparkMax(16, MotorType.kBrushless);
    public static final SparkMaxConfig cfgIntake = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrIntake;

    public static final SparkMax mMasterInclina = new SparkMax(22, MotorType.kBrushless);
    public static final SparkMaxConfig cfgMasterInclina = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrMasterInclina;

    public static final SparkMax mSlaveInclina = new SparkMax(15, MotorType.kBrushless);
    public static final SparkMaxConfig cfgSlaveInclina = new SparkMaxConfig();
    public static SparkClosedLoopController pidCtrSlaveInclina;



    public Intake(){

    }

    static void configIntake(double P, double OutMin, double OutMax){
        cfgIntake.inverted(false).idleMode(IdleMode.kBrake);
        cfgIntake.openLoopRampRate(2);
        cfgIntake.closedLoopRampRate(2);
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

    static public void setIntake(double position){
        mIntake.getEncoder().setPosition(0);
        configIntake(0.25, -0.3, 0.3);
        pidCtrIntake.setSetpoint(position, ControlType.kPosition);
    }

    static public void IntakeStop(){
        mIntake.getEncoder().setPosition(0);
        configIntake(0.25, -0.15, 0.15);
        pidCtrIntake.setSetpoint(0, ControlType.kPosition);
        mIntake.getEncoder().setPosition(0);
    }

    static public void coletar(){
        mIntake.getEncoder().setPosition(0);
        configIntake(0.25, -0.4, 0.4);
        setIntake(-20000);
    }

    static void configInclina(double P, double OutMin, double OutMax){
        cfgMasterInclina.inverted(false).idleMode(IdleMode.kBrake);
        cfgMasterInclina.closedLoopRampRate(1);
        cfgMasterInclina.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgMasterInclina.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(OutMin, OutMax);

        mMasterInclina.configure(cfgMasterInclina, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrMasterInclina = mMasterInclina.getClosedLoopController();

        cfgSlaveInclina.inverted(true).idleMode(IdleMode.kBrake);
        cfgSlaveInclina.closedLoopRampRate(1);
        cfgSlaveInclina.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgSlaveInclina.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(OutMin, OutMax);

        mSlaveInclina.configure(cfgSlaveInclina, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidCtrSlaveInclina = mSlaveInclina.getClosedLoopController();
    }

    static public double getMasterInclina(){
        return mMasterInclina.getEncoder().getPosition();
    }

    static public double getSlaveInclina(){
        return mSlaveInclina.getEncoder().getPosition();
    }

    static public void setInclina(double position){
        configInclina(0.3, -0.1, 0.35);
        pidCtrMasterInclina.setSetpoint(position, ControlType.kPosition);
        pidCtrSlaveInclina.setSetpoint(position, ControlType.kPosition);
    }



}
