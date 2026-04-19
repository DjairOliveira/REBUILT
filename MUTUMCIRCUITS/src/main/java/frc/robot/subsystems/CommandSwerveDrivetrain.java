package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private XboxController m_Control = new XboxController(0);

    String frontCAM = "limelight-front";
    String leftCAM  = "limelight-left";
    String rightCAM = "limelight-right";

    // public LimelightHelpers.PoseEstimate mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(frontCAM);     /* TEMOS PERGUNTAS */ //SetRobotOrientation
    // public LimelightHelpers.PoseEstimate mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(leftCAM);       /* TEMOS PERGUNTAS */ //SetRobotOrientation
    // public LimelightHelpers.PoseEstimate mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rightCAM);     /* TEMOS PERGUNTAS */ //SetRobotOrientation 

    // public boolean isHeadingLocked = false;

    private double fixedAngle = 0;

    private double colisionProtect = 1;
    private static double OmegaCmd = 0;
    private boolean ctrInit = false;

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SwerveDriveBrake brakeX = new SwerveRequest.SwerveDriveBrake();

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

    public Command brakeX() {
        return applyRequest(() -> brakeX);
    }

    @Override
    public void periodic() {

        var mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue(frontCAM);
        var mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue(leftCAM);
        var mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue(rightCAM);

        if(ctrInit){
            if(mt2Front != null && mt2Front.tagCount > 1 && mt2Front.avgTagDist < 4){
                Pose2d pose = mt2Front.pose;
                configAngleInit(pose.getRotation().getDegrees());
                fixedAngle = -pose.getRotation().getDegrees();
            }

            if(mt2Left != null && mt2Left.tagCount > 1 && mt2Left.avgTagDist < 4){
                Pose2d pose = mt2Left.pose;
                configAngleInit(pose.getRotation().getDegrees());
                fixedAngle = -pose.getRotation().getDegrees();
            }

            if(mt2Right != null && mt2Right.tagCount > 1 && mt2Right.avgTagDist < 4){
                Pose2d pose = mt2Right.pose;
                configAngleInit(pose.getRotation().getDegrees());
                fixedAngle = -pose.getRotation().getDegrees();
            }
            
            ctrInit = false;
        }

        // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        //     DriverStation.getAlliance().ifPresent(allianceColor -> {
        //         setOperatorPerspectiveForward(
        //             allianceColor == Alliance.Red
        //                 ? kRedAlliancePerspectiveRotation   
        //                 : kBlueAlliancePerspectiveRotation  
        //         );
        //         m_hasAppliedOperatorPerspective = true;
        //     });
        // }

        double YawRaw = this.getPigeon2().getYaw().getValueAsDouble();
        double YawReal = YawRaw * Constants.FATOR_ESCALA_PIGEON;
        double YawWrapping = MathUtil.inputModulus(YawReal, -180, 180);

        // updateLimelightAngle(frontCAM, YawWrapping);
        // updateLimelightAngle(leftCAM, YawWrapping);
        // updateLimelightAngle(rightCAM, YawWrapping);

        double omega = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

        if (megaTagUpdateOdometry(mt2Front, 4, 30, omega)) {
            double xyStdDev = mt2Front.tagCount > 2 ? 0.1 : 0.1 + (mt2Front.avgTagDist * 0.2);
            addVisionMeasurement(mt2Front.pose, mt2Front.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, 999999.0));
            updateLimelightAngle(frontCAM, YawWrapping);
            Logger.recordOutput("VISION/Front", mt2Front.pose);
        }
        
        if (megaTagUpdateOdometry(mt2Left, 4, 30, omega)) {
            double xyStdDev = mt2Left.tagCount > 2 ? 0.1 : 0.1 + (mt2Left.avgTagDist * 0.2);
            addVisionMeasurement(mt2Left.pose, mt2Left.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, 999999.0));
            Logger.recordOutput("VISION/Left", mt2Left.pose);
        }

        if (megaTagUpdateOdometry(mt2Right, 4, 30, omega)) {
            double xyStdDev = mt2Right.tagCount > 2 ? 0.1 : 0.1 + (mt2Right.avgTagDist * 0.2);
            addVisionMeasurement(mt2Right.pose, mt2Right.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, 999999.0));
            Logger.recordOutput("VISION/Right", mt2Right.pose);
        }

        Pose2d currentPose = this.getState().Pose;
        Rotation2d heading = currentPose.getRotation();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, heading);

        double speedX = fieldSpeeds.vxMetersPerSecond;
        // double speedY = fieldSpeeds.vyMetersPerSecond;

        if(Climber.getPosition() >= 1){
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

            fixedAngle = anguloRealModulado;

            /*  analizar a possibilidade de mudar isso juntando os dois */
        }
        else if(Math.abs(m_Control.getRightX()) < 0.1){
            if(m_Control.getRightBumperButton()){       ///// talvez tenha que deixar independente
                OmegaCmd = Hood.getOmega();
                fixedAngle = anguloRealModulado;
            }
            else{
                Pose2d robot_getValues = getPose();
                Rotation2d Robot_Yaw = robot_getValues.getRotation();

                Rotation2d targetAngle = Rotation2d.fromDegrees(fixedAngle);
                double error = MathUtil.angleModulus(targetAngle.minus(Robot_Yaw).getRadians());

                double kP = 1.15;
                OmegaCmd = kP * error;
                
                OmegaCmd = MathUtil.clamp(OmegaCmd, -3, 3);
            }
        }
        else{
            fixedAngle = anguloRealModulado;
            OmegaCmd = -m_Control.getRightX();
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

    }

    public boolean isValid(LimelightHelpers.PoseEstimate est) {
    return est != null
        && est.tagCount > 1
        && est.avgTagDist < 4.0;   // distância máxima
    }

    // public boolean megaTagUpdateOdometry(LimelightHelpers.PoseEstimate mt2, double maxDistanceTAG, double maxRotation) {
    //     double omega = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
    //     return mt2 != null && mt2.tagCount > 1 && mt2.avgTagDist < maxDistanceTAG && omega < maxRotation ? true : false;
    // }

    public boolean megaTagUpdateOdometry(LimelightHelpers.PoseEstimate mt2, double maxDistanceTAG, double maxRotation, double omega) {
        return mt2 != null && mt2.tagCount > 1 && mt2.avgTagDist < maxDistanceTAG && omega < maxRotation ? true : false;
    }

    public void updateLimelightAngle(String limelightName, double newAngle) {
        LimelightHelpers.SetRobotOrientation(limelightName, newAngle, 0, 0, 0, 0, 0);
    }

    public void zeroGyroPigeon() {
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

        var mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue(frontCAM);
        var mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue(leftCAM);
        var mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue(rightCAM);

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
        double omega = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
        if (megaTagUpdateOdometry(mt2, 3.5, 45.0, omega)) {
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

    public void configAngleInit(double newAngle) {
        double novoAngulo = newAngle;

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