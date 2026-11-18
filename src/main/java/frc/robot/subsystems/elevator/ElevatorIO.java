package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    public class ElevatorInputs {
        public double position = 0.0;
        public double setpoint = 0.0;
        public double velocity = 0.0;
        public double acceleration = 0.0;

        public double leftVoltage = 0.0;
        public double rightVoltage = 0.0;

        public double leftTemperature = 0.0;
        public double rightTemperature = 0.0;

        public double leftSupplyCurrent = 0.0;
        public double rightSupplyCurrent = 0.0;

        public double leftStatorCurrent = 0.0;
        public double rightStatorCurrent = 0.0;
    }

    public default void updateInputs(ElevatorInputs inputs) {

    }

    public default void setVoltage(double voltage) {

    }

    public default void setPosition(double height) {

    }

    public default void stop() {

    }
}
