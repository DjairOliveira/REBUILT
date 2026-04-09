package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import org.littletonrobotics.junction.Logger;

public class Hood extends Command {

    private static double startTime;

    /* Shooter INIT */
    public static final TalonFX mShooterR = new TalonFX(22);
    public static final TalonFX mShooterL = new TalonFX(21);
    public static final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private double targetRPM = 0.0;
    /* Shooter END */

    /* Hood INIT */
    public static TalonFXS mHood = new TalonFXS(23);
    public static PositionDutyCycle pidCtrHood = new PositionDutyCycle(0);
    /* Hood END */

    /* Index INIT */
    public static TalonFX mIndexL = new TalonFX(19);
    public static TalonFX mIndexR = new TalonFX(20);
    public static PositionDutyCycle pidCtrIndex = new PositionDutyCycle(0);
    /* Index END */

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

        double[] velocityShooter = getShooterVelocity();

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
            // poseHood = map(parabola(distanceHood), 45, 80, 6, 0);
            // speedShooter = Math.max(-0.7, Math.min(-0.52, interpolate(distanceHood, distances, speed) * Robot.velocityTiro.getDouble(0)));
            poseHood = Robot.setInclina.getDouble(0);
            speedShooter = Robot.velocityTiro.getDouble(0);
        } else{
            poseHood = 0;
            speedShooter = -0.2;
            speedIndex = 0;
            speedBelt = 0;
        }

        setHoodPosition(poseHood);
        setShooterRPM(speedShooter);  /// Max 6000
        if((targetRPM - velocityShooter[0]) < 1.5){
            dispOk = true;
        }else{
           dispOk = false; 
        }
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
        NetworkTableInstance.getDefault().getTable("HOOD").getEntry("Parabola").setDouble(parabola(distanceHood));
        setLogger();
    }

    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 1.25 ? true : false;
    }

    public void end() {
        setHoodPosition(0);
        mShooterL.set(-0.2);
        mShooterR.set(-0.2);
        setIndex(0);
        Intake.setBeltSpeed(0);
    }

    private void setLogger(){
        Logger.recordOutput("Turret/VerticalPosition", getHoodPositon());
        Logger.recordOutput("Turret/ShooterRight", mShooterL.getVelocity().getValueAsDouble());
        Logger.recordOutput("Turret/ShooterLeft", mShooterR.getVelocity().getValueAsDouble());
    }

    /**
    * Controla todos os motores necessarios para conduzir o fuel até o Shooter.
    *
    * @param index Speed do index.
    * @param belt Speed belt.
    */
    private void setFuel (double index, double belt){
        setIndex(index);
        Intake.setBeltSpeed(belt);

    }

    /**
    * Retorna um valor linear dentro do range estabelecido.
    *
    * @param x Variavel de leitura.
    * @param in_min Valor minimo de entrada da variavel x.
    * @param in_max Valor maximo de entrada da variavel x.
    * @param out_min Valor de saida minimo permitido (PODE TER B.Ozinhos).
    * @param out_max Valor de saida maximo permitido (PODE TER B.Ozinhos).
    */
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        double result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return result;
    }

    /**
    * Retorna o angulo de saida do objeto de jogo em graus.
    *
    * @param distHoodToHUB Distancia do Hood para o alvo.
    */
    public static double parabola(double distHoodToHUB) {
        double delta_T = 0, a_T = 0, b_T = 0, c_T = 1.112;
        double tan_angle_T = 0;

        a_T = (Math.pow(distHoodToHUB, 2) / 9.248);
        b_T = -distHoodToHUB;
        delta_T = (Math.pow(b_T, 2) - 4 * a_T * c_T);
        tan_angle_T = (distHoodToHUB + Math.sqrt(delta_T)) / (2 * a_T);
        return Math.toDegrees(Math.atan(tan_angle_T));
    }

    /**
    * Retorna a distancia do Hood até o HUB, e tbm gera o valor de Omega para o alinhamento di Chassi.
    *
    * @param Target_X Posição X do alvo a ser atingido.
    * @param Target_Y Posição Y do alvo a ser atingido.
    */
    private double hoodAling (double Target_X, double Target_Y){
        Pose2d robot_getValues = swerve.getPose();
        Rotation2d Robot_Yaw = robot_getValues.getRotation();
        Translation2d robot_pose = robot_getValues.getTranslation();

        Translation2d pose_Hub = new Translation2d(Target_X , Target_Y);
        Translation2d offsetHood = new Translation2d(-0.19719, 0);
        Translation2d poseHood = robot_pose.plus(offsetHood.rotateBy(Robot_Yaw));
        double distanceHood = poseHood.getDistance(pose_Hub);

        Translation2d angleRobot = pose_Hub.minus(robot_pose);
        Rotation2d targetAngle = angleRobot.getAngle();

        double error = MathUtil.angleModulus(targetAngle.minus(Robot_Yaw.plus(new Rotation2d(Math.PI))).getRadians());
        double kP = 1;
        OmegaCmd = kP * error;

        OmegaCmd = MathUtil.clamp(OmegaCmd, -3, 3);

        NetworkTableInstance.getDefault().getTable("ROBOT").getEntry("Hood").setDoubleArray(new double[] {
        poseHood.getX(), poseHood.getY(), Robot_Yaw.getRadians()});
        
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

    /**
    * Retorna valor do angulo que o robo deve atingir em Yaw para se alinhar com o HUB.
    */
    public double getOmega(){
        return OmegaCmd;
    }

    /**
    * Interpolação para os 
    */
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

    /**
    * Retorna a aliança da Drive Station
    */
    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
    * Configura o motor de inclinação do Hood.
    * @motor 2 x Kraken X60 opostos um ao outro.
    * @param kMode Define o freio do motor coast ou brake.
    */
    public static void configShooter(NeutralModeValue kMode) {
        
        TalonFXConfiguration cfgShooterL = new TalonFXConfiguration();
        TalonFXConfiguration cfgShooterR = new TalonFXConfiguration();

        cfgShooterL.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgShooterL.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgShooterL.MotorOutput.NeutralMode = kMode;
        cfgShooterL.CurrentLimits.SupplyCurrentLimit = 80;
        cfgShooterL.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgShooterL.Feedback.SensorToMechanismRatio = 1.0;
        cfgShooterL.MotorOutput.PeakForwardDutyCycle = 1;
        cfgShooterL.MotorOutput.PeakReverseDutyCycle = -1;
        cfgShooterL.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgShooterL.Voltage.PeakForwardVoltage = 12;
        cfgShooterL.Voltage.PeakReverseVoltage = -12;

        cfgShooterR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgShooterR.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgShooterR.MotorOutput.NeutralMode = kMode;
        cfgShooterR.CurrentLimits.SupplyCurrentLimit = 80;
        cfgShooterR.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgShooterR.Feedback.SensorToMechanismRatio = 1.0;
        cfgShooterR.MotorOutput.PeakForwardDutyCycle = 1;
        cfgShooterR.MotorOutput.PeakReverseDutyCycle = -1;
        cfgShooterR.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgShooterR.Voltage.PeakForwardVoltage = 12;
        cfgShooterR.Voltage.PeakReverseVoltage = -12;

        mShooterL.getConfigurator().apply(cfgShooterL);
        mShooterR.getConfigurator().apply(cfgShooterR);
    }
    
    /**
    * Retorna a posição dos motores do Shooter.
    *
    * @param null
    */
    static double[] getShooter() {
        return new double[] {
            mShooterL.getPosition().getValueAsDouble(),
            mShooterR.getPosition().getValueAsDouble()};
    }

    /**
    * Retorna a velocidade dos motores do Shooter.
    *
    * @param null
    */
    static double[] getShooterVelocity(){
        return new double[] {
            mShooterL.getVelocity().getValueAsDouble(),
            mShooterR.getVelocity().getValueAsDouble()};
    }

    /**
    * Para os motores do Shooter e configura o motor em modo coast.
    *
    * @param null
    */
    static public void stopShooterSpeed() {
        configShooter(NeutralModeValue.Coast);
        mShooterL.stopMotor();
        mShooterR.stopMotor();
    }

    /**
    * Define a velocidade dos motores do Shooter
    *
    * @param speedShooter Define a velocidade dos motores do Shotter Left e Right [Limites = 1 a -1].
    */
    static public void setShooterSpeed(double speedShooter){
        mShooterL.set(speedShooter);
        mShooterR.set(speedShooter);
    }
    
    /**
    * Para os motores do Shooter e configura o motor em modo coast.
    *
    * @param setpointRPM Define a velocidade dos motores do Shotter Left e Right.
    */
    public void setShooterRPM(double setpointRPM){
        this.targetRPM = setpointRPM / 60.0;
        double kV = 0.115;
        double VoltageFeedFoward = this.targetRPM * kV;
        mShooterL.setControl(velocityRequest.withVelocity(targetRPM).withFeedForward(VoltageFeedFoward));
        mShooterR.setControl(velocityRequest.withVelocity(targetRPM).withFeedForward(VoltageFeedFoward));
    }

    /**
    * Configura o motor de inclinação do Hood.
    * @motor Kraken X44.
    * @param KP Define o ganho proporcial do motor.
    * @param OutMin Saida minima aplicada no motor [Limite = -1].
    * @param OutMax Saida maxima aplicada no motor [Limite = 1].
    */
    public static void configHood(double KP, double OutMin, double OutMax) {
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

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

        mHood.getConfigurator().apply(cfg);
    }
    
    /**
    * Configura o motor de inclinação do Hood.
    * @motor Kraken X44.
    * @param KP Define o ganho proporcial do motor.
    * @param OutMin Saida minima aplicada no motor [Limite = -1].
    * @param OutMax Saida maxima aplicada no motor [Limite = 1].
    */
    static public double getHoodPositon() {
        return mHood.getPosition().getValueAsDouble();
    }

    /**
    * Define a posição do motor do Hood, considerando tambem não estar de baixo da Treanch
    * @param position Posição do Hood.
    */
    public void setHoodPosition(double position) {
        double blueX = 4.298;   // Aliança
        double redX = 12.41;    // Aliança

        Pose2d robotPose = swerve.getPose();

        if((!isRedAlliance() && robotPose.getX() <= blueX) || (isRedAlliance() && robotPose.getX() >= redX)){
            mHood.setControl(pidCtrHood.withPosition(position));
        }else{
            mHood.setControl(pidCtrHood.withPosition(0));
        }
    }

    /**
    * Configura o motor de inclinação do Hood.
    * @motor 2 x Kraken X60 opostos um ao outro.
    * @param kMode Define o freio do motor coast ou brake.
    */
    public static void configIndex(NeutralModeValue kMode) {
        
        TalonFXConfiguration cfgIndexL = new TalonFXConfiguration();
        TalonFXConfiguration cfgIndexR = new TalonFXConfiguration();
        
        cfgIndexL.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgIndexL.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgIndexL.MotorOutput.NeutralMode = kMode;
        cfgIndexL.CurrentLimits.SupplyCurrentLimit = 80;
        cfgIndexL.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIndexL.Feedback.SensorToMechanismRatio = 1.0;
        cfgIndexL.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIndexL.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIndexL.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgIndexL.Voltage.PeakForwardVoltage = 12;
        cfgIndexL.Voltage.PeakReverseVoltage = -12;

        cfgIndexR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgIndexR.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgIndexR.MotorOutput.NeutralMode = kMode;
        cfgIndexR.CurrentLimits.SupplyCurrentLimit = 80;
        cfgIndexR.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIndexR.Feedback.SensorToMechanismRatio = 1.0;
        cfgIndexR.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIndexR.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIndexR.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgIndexR.Voltage.PeakForwardVoltage = 12;
        cfgIndexR.Voltage.PeakReverseVoltage = -12;

        mIndexL.getConfigurator().apply(cfgIndexL);
        mIndexR.getConfigurator().apply(cfgIndexR);
    }
    
    /**
    * Retorna a posição dos motores do Indexer.
    *
    * @param null
    */
    static public double[] getIndexPosition() {
        return new double[] {
            mIndexL.getPosition().getValueAsDouble(),
            mIndexR.getPosition().getValueAsDouble()};
    }

    /**
    * Retorna a velocidade dos motores do Indexer.
    *
    * @param null
    */
    static public double[] getIndexVelocity() {
        return new double[] {
            mIndexL.getVelocity().getValueAsDouble(),
            mIndexR.getVelocity().getValueAsDouble()};
    }
    /**
    * Define a velocidade dos motores do Indexer
    *
    * @param speed Define a velocidade Left e Right [Limites = 1 a -1].
    */
    static public void setIndex(double speed){
        mIndexL.set(speed);
        mIndexR.set(speed);
    }

    /**
    * Para os motores do Indexer e configura o motor em modo coast.
    *
    * @param null
    */
    static public void stopIndexSpeed() {
        configShooter(NeutralModeValue.Coast);
        mIndexL.stopMotor();
        mIndexR.stopMotor();
    }
}