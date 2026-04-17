package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // public SubSystemSIM m_SubSystemSIM = new SubSystemSIM();
    private XboxController m_Control = new XboxController(0);

    String frontCAM = "limelight-front";
    String leftCAM  = "limelight-left";
    String rightCAM = "limelight-right";

    public LimelightHelpers.PoseEstimate mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(frontCAM);     /* TEMOS PERGUNTAS */ //SetRobotOrientation
    public LimelightHelpers.PoseEstimate mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(leftCAM);       /* TEMOS PERGUNTAS */ //SetRobotOrientation
    public LimelightHelpers.PoseEstimate mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rightCAM);     /* TEMOS PERGUNTAS */ //SetRobotOrientation 

    private final PIDController headingPid = new PIDController(1.0, 0.0, 0.0);

    private Rotation2d lockedHeading = new Rotation2d();
    private boolean isHeadingLocked = false;

    private double colisionProtect = 1;
    private static double OmegaCmd = 0;

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

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

        updateLimelightAngle(frontCAM, YawWrapping);
        updateLimelightAngle(leftCAM, YawWrapping);
        updateLimelightAngle(rightCAM, YawWrapping);

        var front = LimelightHelpers.getBotPoseEstimate_wpiBlue(frontCAM);
        var left = LimelightHelpers.getBotPoseEstimate_wpiBlue(leftCAM);
        var right = LimelightHelpers.getBotPoseEstimate_wpiBlue(rightCAM);

        if (isValid(front)) {
            megaTagUpdateOdometry(mt2Front, 4, 30);
            addVisionMeasurement(front.pose, front.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(0.1, 0.1, 999999.0));
        }

        if (isValid(left)) {
            megaTagUpdateOdometry(mt2Left, 4, 30);
            addVisionMeasurement(left.pose, left.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(0.1, 0.1, 999999.0));
        }

        if (isValid(right)) {
            megaTagUpdateOdometry(mt2Right, 4, 30);
            addVisionMeasurement(right.pose, right.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(0.1, 0.1, 999999.0));
        }

        Pose2d currentPose = this.getState().Pose;
        Rotation2d heading = currentPose.getRotation();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, heading);

        double speedX = fieldSpeeds.vxMetersPerSecond;
        double speedY = fieldSpeeds.vyMetersPerSecond;

        // if(m_SubSystemSIM.getSubClimber() >= 30){
        //     if(currentPose.getX() >= 4.628 - 1 && currentPose.getX() <= 4.628 + 1){
        //         if((currentPose.getY() >= 0 && currentPose.getY() <= 1.4)){
        //             if((currentPose.getX() - 4.628) < 0 && speedX > 0.5 || (currentPose.getX() - 4.628) > 0 && speedX < -0.5) colisionProtect = 0.25; 
        //             if((currentPose.getX() - 4.628) < 0 && speedX < -0.5 || (currentPose.getX() - 4.628) > 0 && speedX > 0.5) colisionProtect = 1;
        //         }
        //         else if((currentPose.getY() >= 6.393 && currentPose.getY() <= 8.2)){
        //             if((currentPose.getX() - 4.628) < 0 && speedX > 0.5 || (currentPose.getX() - 4.628) > 0 && speedX < -0.5) colisionProtect = 0.25; 
        //             if((currentPose.getX() - 4.628) < 0 && speedX < -0.5 || (currentPose.getX() - 4.628) > 0 && speedX > 0.5) colisionProtect = 1;
        //         }
        //         else{
        //             colisionProtect = 1;
        //         }
        //     }
        //     else if(currentPose.getX() >= 11.927 - 1 && currentPose.getX() <= 11.927 + 1){
        //         if((currentPose.getY() >= 0 && currentPose.getY() <= 1.4)){
        //             if((currentPose.getX() - 11.927) < 0 && speedX > 0.5 || (currentPose.getX() - 11.927) > 0 && speedX < -0.5) colisionProtect = 0.25; 
        //             if((currentPose.getX() - 11.927) < 0 && speedX < -0.5 || (currentPose.getX() - 11.927) > 0 && speedX > 0.5) colisionProtect = 1;
        //         }
        //         else if((currentPose.getY() >= 6.393 && currentPose.getY() <= 8.2)){
        //             if((currentPose.getX() - 11.927) < 0 && speedX > 0.5 || (currentPose.getX() - 11.927) > 0 && speedX < -0.5) colisionProtect = 0.25; 
        //             if((currentPose.getX() - 11.927) < 0 && speedX < -0.5 || (currentPose.getX() - 11.927) > 0 && speedX > 0.5) colisionProtect = 1;
        //         }
        //         else{
        //             colisionProtect = 1;
        //         }
        //     }
        //     else colisionProtect = 1;
        // }
        // else{
        //     colisionProtect = 1;
        // }

        double anguloAcumuladoBruto = getPigeon2().getYaw().getValueAsDouble();
        double anguloRealModulado = MathUtil.inputModulus(anguloAcumuladoBruto * Constants.FATOR_ESCALA_PIGEON, -180, 180);
        
        if(m_Control.getYButton()){
            double currentDeg = getHeading().getDegrees();
            double targetDeg = 0;

            if(currentDeg > 0 && currentDeg <= 90) targetDeg = 45;
            else if(currentDeg > 90 && currentDeg <= 180) targetDeg = 135;
            else if(currentDeg >= -90 && currentDeg < 0) targetDeg = -45;
            else if(currentDeg >= -180 && currentDeg < -90) targetDeg = -135;

            Rotation2d targetAngle = Rotation2d.fromDegrees(targetDeg);
            double error = MathUtil.angleModulus(targetAngle.minus(getHeading()).getRadians());

            double kP = 1;
            OmegaCmd = kP * error;

            OmegaCmd = MathUtil.clamp(OmegaCmd, -3, 3);

            isHeadingLocked=false;

            /*  analizar a possibilidade de mudar isso juntando os dois */
        }
        else if(Math.abs(m_Control.getRightX()) < 0.1){
            if(m_Control.getLeftBumperButton()){  ///// talvez tenha que deixar independente
                OmegaCmd = Hood.getOmega();
                isHeadingLocked = false;
            }
            else{
                if (!isHeadingLocked) {
                    lockedHeading = Rotation2d.fromDegrees(anguloRealModulado);
                    headingPid.reset(); 
                    isHeadingLocked = true;
                }
                double calcTrava = headingPid.calculate(
                    Math.toRadians(anguloRealModulado),
                    lockedHeading.getRadians());
                    
                OmegaCmd = headingPid.atSetpoint() ? 0.0 : calcTrava;
                OmegaCmd = MathUtil.clamp(OmegaCmd, -Constants.LIMITE_ROTACAO, Constants.LIMITE_ROTACAO);
            }
        }
        else{
            OmegaCmd = -m_Control.getRightX();
            isHeadingLocked = false;
        }

        double pigeonMentirosoTela = MathUtil.inputModulus(YawRaw, -180, 180);
        SmartDashboard.putNumber("PIGEON/Raw", YawWrapping);
        SmartDashboard.putNumber("PIGOEN/Liar", pigeonMentirosoTela);
        SmartDashboard.putNumber("PIGEON/Real", YawReal);

        Logger.recordOutput("ODOMETRIA", currentPose);

        Logger.recordOutput("POSE/Odometry/Real", new double[] {currentPose.getX(), currentPose.getY(), Math.toRadians(YawWrapping)});
        Logger.recordOutput("POSE/Odometry/Estimate", new double[] {currentPose.getX(), currentPose.getY(), getPose().getRotation().getRadians()});

        Logger.recordOutput("VISION/mt2Front", mt2Front != null ? mt2Front.pose.getX() : 0.0);
        Logger.recordOutput("VISION/mt2Left", mt2Left != null ? mt2Left.pose.getX() : 0.0);
        Logger.recordOutput("VISION/mt2Right", mt2Right != null ? mt2Right.pose.getX() : 0.0);
        
        /* VISÃO MUTUM
        // boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        // double alvoX = isRed ? Constants.HUB_X_Red : Constants.HUB_X_Blue;
        // double alvoY = isRed ? Constants.HUB_Y_Red : Constants.HUB_Y_Blue;

        // double distanciaHubMetros = Math.hypot(alvoX - poseAtual.getX(), alvoY - poseAtual.getY());
        // SmartDashboard.putNumber("Distancia HUB (m)", distanciaHubMetros);

        // 4. Envia o ÂNGULO VERDADEIRO para Limelight (MegaTag 2). Ela precisa saber qual o ângulo do robô para definir de onde
        // o robô está olhando para a TAG quando a limelight ver uma TAG.
        
        // LimelightHelpers.SetRobotOrientation("limelight", anguloCorrigidoLL, 0, 0, 0, 0, 0);

        // // 5. Obtém as informações completas que a limelight está coletando: coordenada X, Y, quantas tags está vendo, etc.
        // // Ela só obterá as informações corretamente se você enviar o ângulo correto anteriormente no SetRobotOrientation 
        // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        // // 6. Pega a velocidade de giro do robô (para não confiar na câmera se estivermos girando muito rápido).
        // double velocidadeGiro = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

        // // Executa a atualização da odometria do robô no addVisionMesurement caso as seguintes condições sejam atendidas:
        // // i.    Existir alguma informação de bot pose da MT2;
        // // ii.   Alguma tag está sendo vista;
        // // iii.  Velocidade de giro for menor que 720 dps (graus por segundo);
        // // iiii. Distância média da TAG vista é inferior a 4 metros;
        // if (mt2 != null && mt2.tagCount > 0 && velocidadeGiro < 720.0 && mt2.avgTagDist < 4.0) {
            
        //     // --- CÁLCULO DINÂMICO DE CONFIANÇA ---
        //     double xyStdDev;
            
        //     if (mt2.tagCount >= 2) {
        //         // Se ver 2 ou mais tags, a MegaTag é absurdamente precisa. Confiança máxima!
        //         xyStdDev = 0.1; 
        //     } else {
        //         // Se ver 1 tag, a confiança diminui conforme a distância aumenta.
        //         // Ex: A 1 metro = 0.3 (Confia muito). A 3.5 metros = 0.8 (Confia pouco)
        //         xyStdDev = 0.1 + (mt2.avgTagDist * 0.2); 
        //     }

        //     // Atualiza a informação da odometria do robô com base nas informações da câmera, pegando o X e Y com base em uma
        //     // confiança que varia dependendo do número de TAGs vistas e a distância média delas. Já o ângulo não confia nem um
        //     // pouco na câmera, logo, só acreditará no ângulo do pigeon.
        //     this.addVisionMeasurement(
        //         mt2.pose, 
        //         mt2.timestampSeconds,
        //         edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0)
        //         //999999 pois o piegon é o responsavel pelo ângulo.
        //     );
        // }
         */
    }

    public boolean isValid(LimelightHelpers.PoseEstimate est) {
    return est != null
        && est.tagCount > 1
        && est.avgTagDist < 4.0;   // distância máxima
    }

    public boolean megaTagUpdateOdometry(LimelightHelpers.PoseEstimate mt2, double maxDistanceTAG, double maxRotation){
        double omega = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
        return mt2 != null && mt2.tagCount > 0 && mt2.avgTagDist < maxDistanceTAG && omega < maxRotation ? true : false;
    }

    public void updateLimelightAngle(String limelightName, double newAngle){
        LimelightHelpers.SetRobotOrientation(limelightName, newAngle, 0, 0, 0, 0, 0);
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
    public void odometryUpdateAutonomo() {
        double velocidadeGiro = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

        if (mt2Front != null && mt2Front.tagCount > 0 && mt2Front.avgTagDist < 3.5 && velocidadeGiro < 45.0) {
            Pose2d currentPose = this.getState().Pose;
            Pose2d newPose = new Pose2d(mt2Front.pose.getTranslation(), currentPose.getRotation());
            this.resetPose(newPose);
            System.out.println("[VISÃO FRONT] Checkpoint Autônomo: Erro das rodas zerado!");
        }
        if (mt2Left != null && mt2Left.tagCount > 0 && mt2Left.avgTagDist < 3.5 && velocidadeGiro < 45.0) {
            Pose2d currentPose = this.getState().Pose;
            Pose2d newPose = new Pose2d(mt2Left.pose.getTranslation(), currentPose.getRotation());
            this.resetPose(newPose);
            System.out.println("[VISÃO LEFT] Checkpoint Autônomo: Erro das rodas zerado!");
        }
        if (mt2Right != null && mt2Right.tagCount > 0 && mt2Right.avgTagDist < 3.5 && velocidadeGiro < 45.0) {
            Pose2d currentPose = this.getState().Pose;
            Pose2d newPose = new Pose2d(mt2Right.pose.getTranslation(), currentPose.getRotation());
            this.resetPose(newPose);
            System.out.println("[VISÃO RIGHT] Checkpoint Autônomo: Erro das rodas zerado!");
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

    public void configAngleInit() {
        var alliance = DriverStation.getAlliance();
        double novoAngulo = (alliance.isPresent() && alliance.get() == Alliance.Red) ? 180.0 : 0;

        this.getPigeon2().setYaw(novoAngulo);
        try { Thread.sleep(20); } catch (Exception e) {}

        Pose2d poseAtual = this.getState().Pose;
        this.resetPose(new Pose2d(poseAtual.getTranslation(), Rotation2d.fromDegrees(novoAngulo)));
        
        System.out.println("Giroscópio Resetado fisicamente para: " + novoAngulo + " graus");
    }

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

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

}