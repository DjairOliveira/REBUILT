package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    public SubSystemSIM m_SubSystemSIM = new SubSystemSIM();

    private XboxController m_Control = new XboxController(0);

    String frontCAM = "limelight1";
    String leftCAM  = "limelight2";
    String rightCAM = "limelight3";

    public LimelightHelpers.PoseEstimate mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(frontCAM);     /* TEMOS PERGUNTAS */ //SetRobotOrientation
    public LimelightHelpers.PoseEstimate mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(leftCAM);       /* TEMOS PERGUNTAS */ //SetRobotOrientation
    public LimelightHelpers.PoseEstimate mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rightCAM);     /* TEMOS PERGUNTAS */ //SetRobotOrientation 

    private double colisionProtect = 1;
    private static double OmegaCmd = 0;

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // O PathPlanner vai usar esta variável para injetar velocidades (X, Y e Giro) nos motores durante o Autônomo.
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null, state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null, state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> {
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
        }, null, this));

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) { return run(() -> this.setControl(request.get())); }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { return m_sysIdRoutineToApply.quasistatic(direction); }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) { return m_sysIdRoutineToApply.dynamic(direction); }

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(0.05)
        .withRotationalDeadband(0.05)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    public Command driveFieldOriented(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return applyRequest(() ->
            fieldCentricDrive
                .withVelocityX(x.getAsDouble())
                .withVelocityY(y.getAsDouble())
                .withRotationalRate(rot.getAsDouble())
        );
    }

    private double deadband(double value, double db) {
        return Math.abs(value) < db ? 0.0 : value;
    }

    public Command driveFieldOrientedScaled( DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, double db, double scale) {
        return applyRequest(() -> {
            double rawX = x.getAsDouble();
            double rawY = y.getAsDouble();
            double rawRot = rot.getAsDouble();

            rawX = deadband(rawX, db);
            rawY = deadband(rawY, db);
            rawRot = deadband(rawRot, db);

            double vx = rawX * scale;
            double vy = rawY * scale;
            double omega = rawRot * scale;

            return fieldCentricDrive .withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);
        });
    }

    public void zeroGyro() {
        this.getPigeon2().setYaw(0);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Define o ângulo inicial do robô baseado na aliança.
     * Como o TunerConstants já tem MountPoseYaw(180), aqui usamos a lógica padrão:
     * Red = 180 graus (olhando para o fundo do campo Azul)
     * Blue = 0 graus (olhando para o fundo do campo Vermelho)
     */
    public void configAngleInit() {
        var alliance = DriverStation.getAlliance();
        
        // O X = 0 e Y = 0 está na quina da parede AZUL. Logo, Apontar para a Parede Vermelha (Ser time azul) = 0°, Apontar para a Parede Azul (Ser time vermelho) = 180°
        // O código abaixo faz exatamente o oposto do padrão oficial devido a inversão do Pigeon.
        double novoAngulo = (alliance.isPresent() && alliance.get() == Alliance.Red) ? 0.0 : 180.0;

        this.getPigeon2().setYaw(novoAngulo);
        try { Thread.sleep(20); } catch (Exception e) {}

        Pose2d poseAtual = this.getState().Pose;
        this.resetPose(new Pose2d(poseAtual.getTranslation(), Rotation2d.fromDegrees(novoAngulo)));
        
        System.out.println("Giroscópio Resetado fisicamente para: " + novoAngulo + " graus");
    }

    @Override
    public void periodic() {

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation   
                        : kBlueAlliancePerspectiveRotation  
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        double YawRaw = this.getPigeon2().getYaw().getValueAsDouble();
        double YawReal = YawRaw * Constants.FATOR_ESCALA_PIGEON;
        double YawWrapping = MathUtil.inputModulus(YawReal, -180, 180);
        updateLimelightAngle(frontCAM, YawWrapping); /* TEMOS PERGUNTAS */

        if (megaTagUpdateOdometry(mt2Front, 4.0, 720.0)) {
            double xyStdDev;
            
            if (mt2Front.tagCount >= 2) xyStdDev = 0.1; 
            else xyStdDev = 0.1 + (mt2Front.avgTagDist * 0.2); 

            this.addVisionMeasurement(
                mt2Front.pose, 
                mt2Front.timestampSeconds,
                edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0) //999999 pois o piegon é o responsavel pelo ângulo.
            );
        }

        Pose2d currentPose = getState().Pose;

        Rotation2d heading = currentPose.getRotation();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, heading);

        double speedX = fieldSpeeds.vxMetersPerSecond;
        double speedY = fieldSpeeds.vyMetersPerSecond;

        if(m_SubSystemSIM.getSubClimber() >= 30){
            if(currentPose.getX() >= 4.628 - 1 && currentPose.getX() <= 4.628 + 1){
                if((currentPose.getY() >= 0 && currentPose.getY() <= 1.4)){
                    if((currentPose.getX() - 4.628) < 0 && speedX > 0.5 || (currentPose.getX() - 4.628) > 0 && speedX < -0.5) colisionProtect = 0.25; 
                    if((currentPose.getX() - 4.628) < 0 && speedX < -0.5 || (currentPose.getX() - 4.628) > 0 && speedX > 0.5) colisionProtect = 1;
                }
                else if((currentPose.getY() >= 6.393 && currentPose.getY() <= 8.2)){
                    if((currentPose.getX() - 4.628) < 0 && speedX > 0.5 || (currentPose.getX() - 4.628) > 0 && speedX < -0.5) colisionProtect = 0.25; 
                    if((currentPose.getX() - 4.628) < 0 && speedX < -0.5 || (currentPose.getX() - 4.628) > 0 && speedX > 0.5) colisionProtect = 1;
                }
                else{
                    colisionProtect = 1;
                }
            }
            else if(currentPose.getX() >= 11.927 - 1 && currentPose.getX() <= 11.927 + 1){
                if((currentPose.getY() >= 0 && currentPose.getY() <= 1.4)){
                    if((currentPose.getX() - 11.927) < 0 && speedX > 0.5 || (currentPose.getX() - 11.927) > 0 && speedX < -0.5) colisionProtect = 0.25; 
                    if((currentPose.getX() - 11.927) < 0 && speedX < -0.5 || (currentPose.getX() - 11.927) > 0 && speedX > 0.5) colisionProtect = 1;
                }
                else if((currentPose.getY() >= 6.393 && currentPose.getY() <= 8.2)){
                    if((currentPose.getX() - 11.927) < 0 && speedX > 0.5 || (currentPose.getX() - 11.927) > 0 && speedX < -0.5) colisionProtect = 0.25; 
                    if((currentPose.getX() - 11.927) < 0 && speedX < -0.5 || (currentPose.getX() - 11.927) > 0 && speedX > 0.5) colisionProtect = 1;
                }
                else{
                    colisionProtect = 1;
                }
            }
            else colisionProtect = 1;
        }
        else{
            colisionProtect = 1;
        }

        if(m_Control.getYButton()){
            double currentDeg = getHeading().getDegrees();
            double targetDeg = 0;

            if(currentDeg > 0 && currentDeg <= 90) targetDeg = 45;
            else if(currentDeg > 90 && currentDeg <= 180) targetDeg = 135;
            else if(currentDeg >= -90 && currentDeg < 0) targetDeg = -45;
            else if(currentDeg >= -180 && currentDeg < -90) targetDeg = -135;

            Rotation2d targetAngle = Rotation2d.fromDegrees(targetDeg);
            double error = MathUtil.angleModulus(targetAngle.minus(getHeading()).getRadians());

            double kP = 10;
            OmegaCmd = -kP * error;

            OmegaCmd = MathUtil.clamp(OmegaCmd, -3, 3);

        }
        else{
            OmegaCmd = m_Control.getRightX();
        }



        Logger.recordOutput("ROBOT/fieldSpeed", fieldSpeeds);
        Logger.recordOutput("ROBOT/getclimber", m_SubSystemSIM.getSubClimber());

        double YawLiar = MathUtil.inputModulus(YawRaw, -180, 180);
        Logger.recordOutput("PIGEON/YawLiar", YawLiar);
        Logger.recordOutput("PIGEON/YawWrapping", YawWrapping);
        
        Logger.recordOutput("POSE/Odometry/Real", new double[] {currentPose.getX(), currentPose.getY(), Math.toRadians(YawWrapping)});
        Logger.recordOutput("POSE/Odometry/Estimate", new double[] {currentPose.getX(), currentPose.getY(), getPose().getRotation().getRadians()});

        Logger.recordOutput("VISION/mt2Front", mt2Front != null ? mt2Front.pose.getX() : 0.0);
        Logger.recordOutput("VISION/mt2Left", mt2Left != null ? mt2Left.pose.getX() : 0.0);
        Logger.recordOutput("VISION/mt2Right", mt2Right != null ? mt2Right.pose.getX() : 0.0);
        

        // // Verifica qual a sua aliança para definir se o X e Y alvo do HUB é um valor para o lado azul e um outro valor para o lado vermelho
        // boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        // double alvoX = isRed ? Constants.HUB_X_Red : Constants.HUB_X_Blue;
        // double alvoY = isRed ? Constants.HUB_Y_Red : Constants.HUB_Y_Blue;

        // // Calcula a distância de uma linha reta que liga o X/Y do Robô e o X/Y do HUB
        // double distanciaHubMetros = Math.hypot(alvoX - poseAtual.getX(), alvoY - poseAtual.getY());
        // SmartDashboard.putNumber("Distancia HUB (m)", distanciaHubMetros);
    }

    public double getColision(){
        return colisionProtect;
    }

    public double getOmegaCmd(){
        return OmegaCmd;
    }

    // Configura o PathPlanner 
    private void configurePathPlanner() {
        try {
            //pega o arquivo .json com as configs do robo.
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                //ja pega a pose com a odometria, limelight etc.
                () -> getState().Pose, 
                //consumer
                this::resetPose, 
                // controle de feedforward
                () -> getState().Speeds, 
                //consumer do ff
                (speeds, feedforwards) -> setControl(autoRequest.withSpeeds(speeds)),
                // PID que o robô utilizará para translação e para rotação durante o Autônomo
                new PPHolonomicDriveController(new PIDConstants(5, 0.0, 0.0), new PIDConstants(5, 0.0, 0.0)),
                // Configurações do PathPlanner do robô
                config, 
                // Decide se o caminho precisa ser espelhado. 
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, 
                this
            ); 
        } catch (Exception e) {
            DriverStation.reportError("Falha ao configurar PathPlanner: " + e.getMessage(), true);
        }
    }

    
    /**
    * Metodo utilizado para atualizar a odometria do robo de forma precisa quando o mesmo se encontra parado.
    * @param mt2 MegaTAG2 relacionado a camera alvo.
    */
    public void odometryUpdateAutonomo(LimelightHelpers.PoseEstimate mt2) {
        
        if (megaTagUpdateOdometry(mt2, 3.5, 45.0)) {
            Pose2d currentPose = this.getState().Pose;
            Pose2d newPose = new Pose2d(mt2.pose.getTranslation(), currentPose.getRotation());

            this.resetPose(newPose);
            System.out.println("[VISÃO] Checkpoint Autônomo: Erro das rodas zerado!");
        }
    }

    /**
    * Atualiza o angulo de yaw verdadeiro para a limelight alvo
    */
    public void updateLimelightAngle(String limelightName, double newAngle){
        LimelightHelpers.SetRobotOrientation(limelightName, newAngle, 0, 0, 0, 0, 0);
    }

    /**
    * @return true se estiver tudo ok para atualizar a odometria do robo.
    *
    * @param mt2 MegaTAG2 relacionado a camera alvo.
    * @param maxDistanceTAG m - Distancia maxima permitida para atualizar a odometria.
    * @param maxRotationdeg deg/s - Velocidade de rotação maxima permitida para atualizar a odometria.
    */
    public boolean megaTagUpdateOdometry(LimelightHelpers.PoseEstimate mt2, double maxDistanceTAG, double maxRotation){
        double omega = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
        return mt2 != null && mt2.tagCount > 0 && mt2.avgTagDist < maxDistanceTAG && omega < maxRotation ? true : false;
    }

    /**
    * Num sei
    */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
    
    /**
    * Num sei
    */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}