package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorConstants {
    public static final double gearing = 11.2521;
    public static final double drumRadius = 0.15;
    public static final double maxHeight = 1.2;
    public static final double minHeight = 0;
    public static final double maxVelocity = 200;
    public static final double maxAcceleration = 10.0;
    public static final double momentOfInertia = 0.021;
    public static final DCMotor gearbox = DCMotor.getNEO(2);
    public static final double minVoltage = -12.0;
    public static final double maxVoltage = 12.0;

    //public static final double kP = 6.0;
    //public static final double kI = 0.0;
    //public static final double kD = 0.001;
    //public statis final double kG = 0.00633;


    public static int supplyCurrent = 80;
    public static int statorCurrent = 120;
    public static int supplyCurrentLow = 60;
    
}