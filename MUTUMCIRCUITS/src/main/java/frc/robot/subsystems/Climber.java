package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;


public class Climber  extends Command {

    public static TalonFXS mclimber = new TalonFXS(17);
    public static PositionDutyCycle PID = new PositionDutyCycle(0);

    private final CommandSwerveDrivetrain swerve;

    public Climber(CommandSwerveDrivetrain swerve){
        this.swerve = swerve;
    }

    /**
    * Configura o climber.
    * @motor @param Type 1 x Minion
    *
    * @param KP Ganho proporcional do sistema.
    * @param OutMin Velocidade maximo e minima do sistema [LIMITE = -1].
    * @param OutMax Velocidade maximo e minima do sistema [LIMITE = 1].
    * @param kMode Define o freio do motor, brake ou coast.
    */
    static void configClimber(double KP, double OutMin, double OutMax, NeutralModeValue kMode) {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode = kMode;
        config.CurrentLimits.SupplyCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.MotorOutput.PeakForwardDutyCycle = OutMax;
        config.MotorOutput.PeakReverseDutyCycle = OutMin;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = KP;
        slot0.kI = 0.0;
        slot0.kD = 0;

        mclimber.getConfigurator().apply(config);
    }

    /**
    * Retorna a posição do climber.
    *
    */
    public static double getPosition() {
        return mclimber.getPosition().getValueAsDouble();
    }

    /**
    * Define a posição do Climber.
    *
    * @param position Posição desejada do Climber.
    * @param speed Velocidade maximo e minima do sistema [LIMITES 1 a -1].
    */
    public void setPosition(double position, double speed) {
        double blueX = 4.298; // Aliança
        double redX = 12.41; // Aliança

        Pose2d robotPose = swerve.getPose();

        if((robotPose.getX() >= blueX-0.2 && robotPose.getX() <= (blueX-0.2)+1.2)
            || (robotPose.getX() >= (redX+0.2) - 1.2 && robotPose.getX() <= redX+0.2)){
            if(position < 20) mclimber.setControl(PID.withPosition(position));
        }
        else{
            if(position >= 20) mclimber.setControl(PID.withPosition(position));
        }
    }

    /**
    * Para o motor do Climber.
    */
    static public void stop(){
        configClimber(0, 0, 0, NeutralModeValue.Coast);
        mclimber.set(0);
    }
}
