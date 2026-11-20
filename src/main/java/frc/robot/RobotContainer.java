// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;

public class RobotContainer {
  private static final CommandXboxController xbox = new CommandXboxController(0);
  private Elevator elevator;

  public RobotContainer() {
    if (RobotBase.isSimulation()) {
      this.elevator = new Elevator(new ElevatorIOSim());
    } else {
      this.elevator = new Elevator(new ElevatorIOSparkMax());
    }
    configureBindings();
  }

  private void configureBindings() {
    xbox.a().onTrue(elevator.setPosition(0.5));
    //xbox.b().onTrue(elevator.setVoltageCommand(0.5));
    //xbox.b().onFalse(elevator.stopCommand());
    xbox.x().onTrue(elevator.setPosition(0.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
