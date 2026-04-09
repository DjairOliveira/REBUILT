package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Climber  extends Command {

    public static TalonFXS mclimber = new TalonFXS(22);
    public static TalonFXSConfiguration config = new TalonFXSConfiguration();
    public static PositionDutyCycle PID = new PositionDutyCycle(0);

    private final SwerveSubsystem swerve;

    public Climber(SwerveSubsystem swerve){
        this.swerve = swerve;
    }

    static void configClimber(double KP, double OutMin, double OutMax, NeutralModeValue kMode) {
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;

        config.MotorOutput.NeutralMode = kMode;
        config.CurrentLimits.SupplyCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        // config.Feedback.SensorToMechanismRatio = 1.0;
        config.MotorOutput.PeakForwardDutyCycle = OutMax;
        config.MotorOutput.PeakReverseDutyCycle = OutMin;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = KP;
        slot0.kI = 0.0;
        slot0.kD = 0;

        mclimber.getConfigurator().apply(config);
    }

    public static double getPosition() {
        return mclimber.getPosition().getValueAsDouble();
    }

    public void setPosition(double position, double speed) {
        double blueX = 4.298; // Aliança
        double redX = 12.41; // Aliança

        Pose2d robotPose = swerve.getPose();

        if((!isRedAlliance() && robotPose.getX() <= blueX) || (isRedAlliance() && robotPose.getX() >= redX)){
            configClimber(2, -speed, speed, NeutralModeValue.Brake);
            mclimber.setControl(PID.withPosition(position));
        }
    }

    static public void stop(){
        configClimber(0, 0, 0, NeutralModeValue.Coast);
        mclimber.set(0);
    }

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

}
