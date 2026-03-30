// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  public final double inclinationMax = 6.2; // 6.3
  public final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    driver.a().onTrue(Commands.runOnce(() -> Turret.disparar(0.705)));
    driver.a().onTrue(Commands.runOnce(() -> Turret.engatilhar()));
    driver.x().onTrue(Commands.runOnce(() -> Turret.disparar(0.7)));
    driver.x().onTrue(Commands.runOnce(() -> Turret.engatilhar()));
    // driver.x().onTrue(Commands.runOnce(()-> Turret.disparar(0.7)));
    driver.y().onTrue(Commands.runOnce(() -> Turret.disparar(0.71)));
    driver.y().onTrue(Commands.runOnce(() -> Turret.engatilhar()));
    driver.b().onTrue(Commands.runOnce(() -> Turret.stop()));
    driver.b().onTrue(Commands.runOnce(() -> Turret.stopEngatilhar()));

    driver.povDown().onTrue(Commands.runOnce(() -> Turret.setInclinaVertical(0)));
    driver.povLeft().onTrue(Commands.runOnce(() -> Turret.setInclinaVertical(inclinationMax * 0.3)));
    driver.povRight().onTrue(Commands.runOnce(() -> Turret.setInclinaVertical(inclinationMax * 0.6)));
    driver.povUp().onTrue(Commands.runOnce(() -> Turret.setInclinaVertical(inclinationMax)));
    //
    driver.back().onTrue(Commands.runOnce(() -> Turret.setInclinaHorizontal(0)));
    driver.leftBumper().onTrue(Commands.runOnce(() -> Turret.setInclinaHorizontal(180)));
    driver.rightBumper().onTrue(Commands.runOnce(() -> Turret.setInclinaHorizontal(-180)));
    // driver.y().onTrue(Commands.runOnce(()-> Turret.setInclinaHorizontal(-180)));
    // driver.x().onTrue(Commands.runOnce(()-> Turret.setInclinaHorizontal(180)));

    // driver.x().whileTrue(Commands.runOnce(() -> turret.execute()).repeatedly());
    driver.x().whileTrue(new Turret().repeatedly());
    driver.x().onFalse(Commands.runOnce(() -> Turret.end()));

    // driver.x().whileTrue(Commands.runOnce(() -> Turret.alingTurret()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
