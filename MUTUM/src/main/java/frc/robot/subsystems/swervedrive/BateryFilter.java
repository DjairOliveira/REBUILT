package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.RobotController;

public class BateryFilter {
    private double filteredVoltage = 12.5;
    private final double alpha = 0.08;      // filtro
    private final double referenceVoltage = 12.6;

    public double updateAndGetGain() {
        double batteryVoltage = RobotController.getBatteryVoltage();

        filteredVoltage = alpha * batteryVoltage + (1.0 - alpha) * filteredVoltage;

        double gain = referenceVoltage / filteredVoltage;

        gain = Math.max(0.95, Math.min(1.15, gain));

        return gain;
    }

    public double getFilteredVoltage() {
        return filteredVoltage;
    }
}
