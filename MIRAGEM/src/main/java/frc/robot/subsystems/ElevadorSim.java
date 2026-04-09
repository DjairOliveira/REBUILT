package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevadorSim extends SubsystemBase {

    // =========================
    // SIMULAÇÃO
    // =========================
    private ElevatorSim elevatorSim;

    // =========================
    // VISUALIZAÇÃO
    // =========================
    private Mechanism2d mech;
    private MechanismLigament2d elevatorLigament;

    // =========================
    // CONTROLE
    // =========================
    private double motorVoltage = 0.0;

    private PIDController controller = new PIDController(8.0, 0.0, 0.0);
    private PIDController angleController = new PIDController(0.75, 0.0, 0.0);

    private double targetHeight = 0.0;
    private double targetAngle = 0.0;

    private TrapezoidProfile.Constraints constraints =
    new TrapezoidProfile.Constraints(
        1.5, // velocidade máxima (m/s)
        2.0  // aceleração máxima (m/s²)
    );

    private XboxController control = new XboxController(0);
    // private Turret mTurret = new Turret(null);

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public ElevadorSim() {

        // Cria simulação do elevador
        elevatorSim = new ElevatorSim(
            DCMotor.getNEO(2), // 2 motores NEO
            10.0,              // redução
            5.0,               // massa (kg)
            0.05,              // raio do tambor (m)
            0.0,               // altura mínima
            2,               // altura máxima (1.5m)
            true,              // simula gravidade
            0.0
        );

        goal = new TrapezoidProfile.State(0, 0);
        setpoint = new TrapezoidProfile.State(0, 0);
        

        // Cria visual
        mech = new Mechanism2d(3, 2);
        MechanismRoot2d root = mech.getRoot("base", 0.25, 0);

        elevatorLigament = root.append(
            new MechanismLigament2d("elevator", 0.1, 70,6, new Color8Bit(0, 255, 255)));

        SmartDashboard.putData("ElevatorSim", mech);
    }

    // =========================
    // CONTROLE SIMPLES
    // =========================
    public void setVoltage(double voltage) {
        this.motorVoltage = voltage;
    }

    // public void setTargetHeight(double height) {
    //     targetHeight = height;
    // }
    public void setTargetHeight(double height) {
        goal = new TrapezoidProfile.State(height, 0);
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }
    // =========================
    // LOOP DE SIMULAÇÃO
    // =========================
    @Override
    public void simulationPeriodic() {

        double currentHeight = elevatorSim.getPositionMeters();

        // Atualiza perfil
        TrapezoidProfile profile =
            new TrapezoidProfile(constraints);

        setpoint = profile.calculate(0.02, setpoint, goal);

        // PID com setpoint suave
        double output = controller.calculate(currentHeight, setpoint.position);

        // Clamp correto
        output = Math.max(-12, Math.min(12, output));

        elevatorSim.setInput(output);
        elevatorSim.update(0.2);

        elevatorLigament.setLength(currentHeight + 0.1);

        double currentAngle = elevatorLigament.getAngle();

        double outputAngle = angleController.calculate(currentAngle, targetAngle);

        // opcional: limitar velocidade do movimento visual
        outputAngle = MathUtil.clamp(outputAngle, -100, 100);

        // integra no ângulo
        double newAngle = currentAngle + outputAngle * 0.2;

        elevatorLigament.setAngle(newAngle);


        Logger.recordOutput("RobotPose", new Pose2d());
        Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
        Logger.recordOutput("FinalComponentPoses", new Pose3d[] {new Pose3d(
            -0.1535, -0.1425-0.085, 0.335, new Rotation3d(0.0, 0, Turret.getAngleTurretSim()))});



        SmartDashboard.putNumber("ElevatorHeight", currentHeight);
        SmartDashboard.putNumber("ElevatorAngle", newAngle);

    }
}
