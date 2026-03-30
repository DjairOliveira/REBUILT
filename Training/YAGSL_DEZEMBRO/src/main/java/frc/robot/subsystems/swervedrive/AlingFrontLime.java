package frc.robot.subsystems.swervedrive;
import java.security.PrivateKey;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

public class AlingFrontLime {
    private final SwerveSubsystem swerve;
    private final String limelightName;
    private final double kPangle = 1;
    private final double kPx = 0.07;
    private double kPy;

    private final double offSetTX;
    private final double OffSetTY; //2
    private final double offSetRotation;

    private final int pipelineIndex;

    private final PIDController txController = new PIDController(0.07, 0.0, 0.001);

    public AlingFrontLime(SwerveSubsystem swerve, String limelightName, int pipelineIndex, double offSetTX, double offSetTY, double offSetRotation) {
        this.swerve = swerve;
        this.limelightName = limelightName;
        this.offSetTX = offSetTX;
        this.OffSetTY = offSetTY;
        this.offSetRotation = offSetRotation;
        this.pipelineIndex = pipelineIndex;
    }

    public void execute() {
        Alliance alliance = DriverStation.getAlliance().get();

        
        LimelightHelpers.PoseEstimate limelightPose;

        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);

        if (alliance == Alliance.Red) {
            limelightPose = LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
        } else if (alliance == Alliance.Blue) {
            limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        } else {
            limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }

        double TagTX = LimelightHelpers.getTX(limelightName);
        double TagYaw = limelightPose.pose.getRotation().getDegrees();
        double voltage = RobotController.getBatteryVoltage();

        if((LimelightHelpers.getTA(limelightName) - OffSetTY) <= 0) kPy=1;  // Avançar na TAG
        else kPy = 0.2;                                                     // Recuar da TAG

        /* 1º MÉTODO */
        // double vAngleTX   = kPangle * -TagTX;
        // double correctionTx = kPx * (TagTX - offSetTX);
        // double correctionTy = kPy * (LimelightHelpers.getTA(limelightName) - OffSetTY);

        /* 2º MÉTODO - Bateria GOOOD */  
        // double vAngleTX   = kPangle * -TagTX;
        // double correctionTx = map(voltage, 11.5, 12.8, 0.07, 0.022) * (TagTX - offSetTX);
        // double correctionTy = kPy * (LimelightHelpers.getTA(limelightName) - OffSetTY);

        /* 3º MÉTODO - PID  -  good tbm */
        double vAngleTX   = kPangle * -TagTX;
        double correctionTx = -txController.calculate(TagTX - offSetTX);
        double correctionTy = kPy * (LimelightHelpers.getTA(limelightName) - OffSetTY);

        if (TagTX != 0){
            swerve.drive(new edu.wpi.first.math.geometry.Translation2d(-correctionTy, -correctionTx), Math.toRadians(vAngleTX - ((TagYaw- offSetRotation)*1.5)) , false);
        }
        else {
            swerve.stop();
        }

        SmartDashboard.putNumber("vAngleTX", vAngleTX);
        SmartDashboard.putNumber("correctionTx", correctionTx);
        SmartDashboard.putNumber("correctionTy", correctionTy);
        SmartDashboard.putNumber("Voltage", voltage);
    }

    public boolean isFinished() {
        return false; //Math.abs(LimelightHelpers.getTY(limelightName)) <= OffSetTY
    }

    public void end() {
        swerve.stop();
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}