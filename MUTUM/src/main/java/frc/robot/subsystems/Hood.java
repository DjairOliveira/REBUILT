package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
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
    private boolean aligned = false;

    /* Shooter INIT */
    public static final TalonFX mShooterR = new TalonFX(22);
    public static final TalonFX mShooterL = new TalonFX(21);
    public static final VelocityVoltage shooterControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMShooter = 0.0;
    /* Shooter END */

    /* Hood INIT */
    public static TalonFXS mHood = new TalonFXS(23);
    public static PositionDutyCycle pidCtrHood = new PositionDutyCycle(0);
    /* Hood END */

    /* Index INIT */
    public static TalonFX mIndexL = new TalonFX(19);
    public static TalonFX mIndexR = new TalonFX(20);
    public static final VelocityVoltage indexControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMIndex = 0.0;
    /* Index END */

    public static double OmegaCmd = 0;
    public static double angleTurretSim = 0;
    public static double poseHood = 0;

    double[] distances = {0.7, 1.05, 1.35, 1.65, 2, 2.551, 3.089, 3.52, 4.027, 4.635, 5.19, 5.7};
    double[] speed = {-0.4,-0.42,-0.44, -0.46, -0.485, -0.5, -0.525, -0.54, -0.5625, -0.59, -0.63, -0.65};

    private final SwerveSubsystem swerve;
    
    public Field2d field = new Field2d();

    public Hood(SwerveSubsystem swerve) {
        this.swerve=swerve;
        configHood(0.05, -0.1, 0.1);
        configIndex(NeutralModeValue.Coast);
        configShooter(NeutralModeValue.Coast);
    }

    public void execute() {

        double blueX = 4.298;   // Aliança
        double redX = 12.41;    // Aliança
        double targetX = 0;
        double targetY = 0;
        double targetZ = 0;

        double RPMShooter = 0, speedBelt = 0, RPMIndex = 0;

        Pose2d robotPose = swerve.getPose();
        boolean dispOk = false;

        double[] velocityShooter = getShooterVelocity();
        double[] velocityIndex = getIndexVelocity();

        /*  Ajuste de target em função da arena - INIT */
        if((!isRedAlliance() && robotPose.getX() <= blueX) || (isRedAlliance() && robotPose.getX() >= redX)){
            targetX = isRedAlliance() ? 11.914 : 4.624;
            targetY = 4.044;
            targetZ = 1.63;
        }
        else if((!isRedAlliance() && robotPose.getX() > blueX + 1.4) || (isRedAlliance() && robotPose.getX() < redX - 1.4)){
            if((robotPose.getY() - 4.044) >= 0) targetY = 6;
            else targetY = 1.829;
            targetX = isRedAlliance() ? 14.32 : 2.331;
            targetZ = 0.5;
        }
        else{
            if((robotPose.getY() - 4.044) >= 0) targetY = 6;
            else targetY = 1.829;
            targetX = isRedAlliance() ? 14.32 : 2.331;
            targetZ = 0.5;
        }
        /*  Ajuste de target em função da arena - END */
        
        double distanceHood = hoodAling(targetX, targetY);

        /* Proteção da treanch - INIT */
        if((robotPose.getX() >= blueX-0.2 && robotPose.getX() <= (blueX-0.2)+1.2) || (robotPose.getX() >= (redX+0.2) - 1.2 && robotPose.getX() <= redX+0.2)){
            // poseHood = -70;
            poseHood = 0;
            RPMShooter = 0;
            RPMIndex = 0;
            speedBelt = 0;

        }else{
            // poseHood = map(parabola(distanceHood, targetX, targetY, targetZ), 40, 80, -110, -70); Simulação
            poseHood = Robot.auxiliar.getDouble(0);
            RPMShooter = Robot.velocityTiro.getDouble(0.05);
        }
        /* Proteção da treanch - END */

        setHoodPosition(poseHood);
        setShooterRPM(RPMShooter);
        SubSystemSIM.setShooterVelocity(3.65);

        boolean RPMShooterOK = (targetRPMShooter - velocityShooter[0]) < 1.5 ? true : false;
        boolean RPMIndexOK = (targetRPMIndex - velocityIndex[0]) < 1.5 ? true : false;

        if(aligned){
            if (RPMShooterOK) RPMIndex = 0.05; //Colocar valor para Indexar
            else RPMIndex = 0;

            if(RPMShooterOK && RPMIndexOK) speedBelt = 0.05;
            else speedBelt = 0;
        }else{
            RPMIndex = 0;
            speedBelt = 0;
        }

        setIndexer(RPMIndex, speedBelt);

        if((targetRPMShooter - velocityShooter[0]) > 0.5){
            startTime = Timer.getFPGATimestamp();
        }

        NetworkTableInstance.getDefault().getTable("HOOD").getEntry("Distance").setDouble(distanceHood);
        NetworkTableInstance.getDefault().getTable("HOOD").getEntry("Aling").setBoolean(aligned);
        NetworkTableInstance.getDefault().getTable("HOOD").getEntry("Interpolation speed").setDouble(interpolate(distanceHood, distances, speed));
        NetworkTableInstance.getDefault().getTable("HOOD").getEntry("Parabola").setDouble(parabola(distanceHood, targetX, targetY, targetZ));
        
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

        SubSystemSIM.setShooterVelocity(0);
    }

    private void setLogger(){
        Logger.recordOutput("Turret/VerticalPosition", getHoodPositon());
        Logger.recordOutput("Turret/ShooterRight", mShooterL.getVelocity().getValueAsDouble());
        Logger.recordOutput("Turret/ShooterLeft", mShooterR.getVelocity().getValueAsDouble());
    }

    public static double getAngleHood(){
        return poseHood;
    }

    /**
    * Controla todos os motores necessarios para conduzir o fuel até o Shooter.
    *
    * @param index Speed do index.
    * @param belt Speed belt.
    */
    private void setIndexer (double RPMIndex, double belt){
        setIndexRPM(RPMIndex);
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
    * @param targetH Altura alvo do Fuel (Altura do Fuel no centro do HUB);
    * @param initH Altura inicial da Fuel (Momento de saida do shooter do robô);
    * @param maxH Alura maxima do Fuel durante a trajetória.
    */
    public double parabola(double distHoodToHUB, double Target_X, double Target_Y, double Target_Z) {
        Pose2d robot_getValues = swerve.getPose();
        Translation2d robot_pose = robot_getValues.getTranslation();
        Translation2d pose_Target = new Translation2d(Target_X , Target_Y);
        Translation2d offsetHood = new Translation2d(0.19719, 0);
        Rotation2d Robot_Yaw = robot_getValues.getRotation();
        Translation2d poseHood = robot_pose.plus(offsetHood.rotateBy(Robot_Yaw));

        double hHUB = 1.83;

        double targetH = Target_Z;//1.63
        double initH = 0.518;
        double maxH = hHUB + Robot.setHmax.getDouble(1);

        double delta_T = 0, a_T = 0, b_T = 0, c_T = targetH - initH;  //1.112
        double tan_angle_T = 0;

        a_T = (Math.pow(distHoodToHUB, 2) / (4 * (maxH - initH)));
        b_T = -distHoodToHUB;
        delta_T = (Math.pow(b_T, 2) - 4 * a_T * c_T);
        tan_angle_T = (distHoodToHUB + Math.sqrt(delta_T)) / (2 * a_T);

        Pose3d[] traj = trajectoryFuel(
            poseHood,
            pose_Target,
            initH,   // altura inicial (saída do shooter)
            maxH,   // altura máxima da curva
            targetH    // altura do alvo
        );

        Logger.recordOutput("ShotTrajectory", traj);

        return Math.toDegrees(Math.atan(tan_angle_T));
    }

    /**
    * Gera a parabola com base nos parametros desejados.
    *
    * @param robotPos Pose do Hood
    * @param targetPos Pose do alvo a ser mirado.
    * @param startHeight Altura de inicio de saida da parabola.
    * @param peakHeight Altura maxima desejada da parabola.
    * @param endHeight Altura do alvo.
    */
    public Pose3d[] trajectoryFuel(Translation2d robotPos, Translation2d targetPos,
        double startHeight, double peakHeight, double endHeight) {

        List<Pose3d> points = new ArrayList<>();
        int steps = 20;

        for (int i = 0; i <= steps; i++) {
            double t = i / (double) steps;

            double x = robotPos.getX() + t * (targetPos.getX() - robotPos.getX());
            double y = robotPos.getY() + t * (targetPos.getY() - robotPos.getY());

            // Base linear entre início e fim
            double linearZ = startHeight + t * (endHeight - startHeight);

            // "Arco" parabólico com pico real em peakHeight
            double arc = 4.0 * t * (1.0 - t) * (peakHeight - ((startHeight + endHeight) / 2.0));

            double z = linearZ + arc;

            points.add(new Pose3d(x, y, z, new Rotation3d()));
        }
        return points.toArray(new Pose3d[0]);
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

        Translation2d pose_Target = new Translation2d(Target_X , Target_Y);
        Translation2d offsetHood = new Translation2d(0.19719, 0);
        Translation2d poseHood = robot_pose.plus(offsetHood.rotateBy(Robot_Yaw));
        double distanceHood = poseHood.getDistance(pose_Target);

        Translation2d angleRobot = pose_Target.minus(robot_pose);
        Rotation2d targetAngle = angleRobot.getAngle();

        double error = MathUtil.angleModulus(targetAngle.minus(Robot_Yaw.plus(new Rotation2d(0))).getRadians());
        double kP = 1;
        OmegaCmd = kP * error;

        OmegaCmd = MathUtil.clamp(OmegaCmd, -3, 3);

        aligned = Math.abs(error) < Math.toRadians(3);


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
    static public double[] getShooterPosition() {
        return new double[] {
            mShooterL.getPosition().getValueAsDouble(),
            mShooterR.getPosition().getValueAsDouble()};
    }

    /**
    * Retorna a velocidade dos motores do Shooter.
    *
    * @param null
    */
    static public double[] getShooterVelocity(){
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
    * @param maxRPMKraken 6000.
    */
    public void setShooterRPM(double setpointRPM){
        this.targetRPMShooter = setpointRPM / 60.0;
        double kV = 0.115;
        double VoltageFeedFoward = this.targetRPMShooter * kV;
        mShooterL.setControl(shooterControl.withVelocity(targetRPMShooter).withFeedForward(VoltageFeedFoward));
        mShooterR.setControl(shooterControl.withVelocity(targetRPMShooter).withFeedForward(VoltageFeedFoward));
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
        mHood.setControl(pidCtrHood.withPosition(position));
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
    * Controla os motores do indexer por kV.
    *
    * @param setpointRPM Define a velocidade dos motores do Shotter Left e Right.
    */
    public void setIndexRPM(double setpointRPM){
        this.targetRPMIndex = setpointRPM / 60.0;
        double kV = 0.115;
        double VoltageFeedFoward = this.targetRPMIndex * kV;
        mShooterL.setControl(shooterControl.withVelocity(targetRPMIndex).withFeedForward(VoltageFeedFoward));
        mShooterR.setControl(shooterControl.withVelocity(targetRPMIndex).withFeedForward(VoltageFeedFoward));
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