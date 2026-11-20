package frc.robot.subsystems.elevator;


import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorConstants {
    public static final double gearing = 11.2521;
    public static final double drumRadius = (5.0 / 1000.0 * 30) / (2 * Math.PI);
    public static final double maxHeight = 1.0;
    public static final double minHeight = 0;
    public static final double maxVelocity = 200;
    public static final double maxAcceleration = 10.0;
    public static final double momentOfInertia = 0.021;
    public static final DCMotor gearbox = DCMotor.getNEO(2);
    public static final double minVoltage = -12.0;
    public static final double maxVoltage = 12.0;
    public static final double positionConversionFactor = drumRadius * 2 * Math.PI / gearing;

    public static final double kP = 8.0;
    public static final double kI = 0.0;
    public static final double kD = 0.001;
    public static final double kG = 0.001;
    public static final double kS = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.0;


    public static int supplyCurrent = 80;
    public static int statorCurrent = 120;
    public static int supplyCurrentLow = 60;

    public static int elevatorLeftMotorID = 11;
    public static int elevatorRightMotorID = 12;

}