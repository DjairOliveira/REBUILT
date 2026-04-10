package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class SubSystemSIM extends SubsystemBase {

    private XboxController mControl = new XboxController(0);

    private double subGavetaPositon = 0.0;
    private double subIntakeAngle = 120;
    private double subClimberPositon = 0.0;

    private double intakeAngleSim = 0.0;
    private double climberPositionSim = 0.0;

    private PIDController intakePID = new PIDController(1, 0, 0.0);
    private PIDController climberPID = new PIDController(1, 0, 0.0);

    public SubSystemSIM() {
    }

    public void setSubIntake(double angle, double min, double max) {
        subIntakeAngle = Hood.map(angle, min, max, 120, 0);
    }

    public void setSubClimber(double position, double min, double max) {
        subClimberPositon = Hood.map(position, min, max, -0.1, 0.2);
    }

    @Override
    public void simulationPeriodic() {

        double intakeOutput = intakePID.calculate(intakeAngleSim, subIntakeAngle);
        intakeAngleSim += intakeOutput * 0.02;
        intakeAngleSim = MathUtil.clamp(intakeAngleSim, 0, 120);

        subGavetaPositon = Hood.map(intakeAngleSim, 0, 120, 0.28, 0);

        double climberOutput = climberPID.calculate(climberPositionSim, subClimberPositon);
        climberPositionSim += climberOutput * 0.02;

        Logger.recordOutput("RobotPose", new Pose2d());

        // subShooterAngle = Hood.map(mControl.getRightTriggerAxis(), 0, 1, -70, -110);

        Logger.recordOutput("SubSystemHood", new Pose3d[] {new Pose3d(
            0.345, 0, 0.43, new Rotation3d(0.0, Math.toRadians(Hood.getAngleHood()), Math.PI))});
        
        Logger.recordOutput("SubSystemIntake", new Pose3d[] {new Pose3d(
            -0.24, 0, 0.178, new Rotation3d(0.0, Math.toRadians(intakeAngleSim), 0))});

        Logger.recordOutput("SubSystemGaveta", new Pose3d[] {new Pose3d(
            0.0552 - subGavetaPositon, 0, 0.179, new Rotation3d(0.0, 0, 0))});

        Logger.recordOutput("SubSystemClimber", new Pose3d[] {new Pose3d(
           0.356, 0, climberPositionSim, new Rotation3d(0.0, 0, 0))});
    }
}
