package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.runOnce(() -> io.setVoltage(voltage), this);
    }

    public Command setPosition(double height) {
        return Commands.runOnce(() -> io.setPosition(height), this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> io.stop(), this);
    }
}