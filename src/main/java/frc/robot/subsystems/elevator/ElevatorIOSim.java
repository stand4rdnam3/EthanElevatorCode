package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorIOSim implements ElevatorIO {

    private double appliedVolts = 0.0;
    private double directVolts = 0.0;
    private double setpoint = 0.0;
    private double previousVelocity = 0.0;
    private double currentTime = Timer.getFPGATimestamp();
    private double previousTime = 0.0;
    private double deltaTime = 0.0;
    private boolean isVoltageMode = false;

    private final ElevatorSim sim;
    private final ProfiledPIDController pidController;

    public ElevatorIOSim() {
        sim = new ElevatorSim(LinearSystemId.createElevatorSystem(DCMotor.getNEO(2), ElevatorConstants.momentOfInertia, ElevatorConstants.drumRadius, ElevatorConstants.gearing), ElevatorConstants.gearbox, ElevatorConstants.minHeight, ElevatorConstants.maxHeight, true, ElevatorConstants.minHeight);
        pidController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        sim.update(0.02);

        inputs.position = sim.getPositionMeters();
        inputs.velocity = sim.getVelocityMetersPerSecond() / ElevatorConstants.drumRadius;

        currentTime = Timer.getFPGATimestamp();

        inputs.setpoint = this.setpoint;

        inputs.leftVoltage = appliedVolts;
        inputs.rightVoltage = appliedVolts;

        inputs.leftSupplyCurrent = sim.getCurrentDrawAmps();
        inputs.rightSupplyCurrent = sim.getCurrentDrawAmps();

        pidController.setPID(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

        if (isVoltageMode) {
            appliedVolts = MathUtil.clamp(directVolts, ElevatorConstants.minVoltage, ElevatorConstants.maxVoltage);
        } else {
            appliedVolts = MathUtil.clamp(pidController.calculate(inputs.position, setpoint) + ElevatorConstants.kG, ElevatorConstants.minVoltage, ElevatorConstants.maxVoltage);
        }

        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void setVoltage(double voltage) {
        isVoltageMode = true;
        this.directVolts = voltage;
    }

    @Override
    public void setPosition(double height) {
        if (isVoltageMode) {
            pidController.reset(sim.getPositionMeters(), sim.getVelocityMetersPerSecond());
            isVoltageMode = false;
        }
        this.setpoint = height;
    }

    @Override
    public void stop() {
        isVoltageMode = true;
        this.directVolts = 0.0;
    }
}