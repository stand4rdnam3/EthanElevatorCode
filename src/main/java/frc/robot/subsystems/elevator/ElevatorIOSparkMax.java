package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {

    private double setpoint = 0.0;
    private double previousVelocity = 0.0;
    private double currentTime = Timer.getFPGATimestamp();
    private double previousTime = 0.0;
    private double deltaTime = 0.0;

    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private SparkMaxConfig config;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    public ElevatorIOSparkMax() {
        this.leftMotor = new SparkMax(ElevatorConstants.elevatorLeftMotorID, MotorType.kBrushless);
        this.rightMotor = new SparkMax(ElevatorConstants.elevatorRightMotorID, MotorType.kBrushless);

        this.config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast).inverted(false).voltageCompensation(12.0).smartCurrentLimit(95);

        config.encoder.positionConversionFactor(ElevatorConstants.positionConversionFactor).velocityConversionFactor(ElevatorConstants.positionConversionFactor);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(leftMotor, true);

        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.controller = leftMotor.getClosedLoopController();

        this.encoder = leftMotor.getEncoder();

        this.encoder.setPosition(0.0);
    }

    public void updateInputs(ElevatorInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();

        currentTime = Timer.getFPGATimestamp();
        deltaTime = currentTime - previousTime;
        inputs.acceleration = (inputs.velocity - previousVelocity) / deltaTime;
        previousTime = currentTime;
        previousVelocity = inputs.velocity;

        inputs.setpoint = this.setpoint;

        inputs.leftVoltage = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
        inputs.rightVoltage = rightMotor.getBusVoltage() * rightMotor.getAppliedOutput();

        inputs.leftSupplyCurrent = leftMotor.getOutputCurrent();
        inputs.rightSupplyCurrent = rightMotor.getOutputCurrent();

        //inputs.leftStatorCurrent = leftMotor.getStatorCurrent();
        //inputs.rightStatorCurrent = rightMotor.getStatorCurrent();

}
@Override
public void setPosition(double height) {
    this.setpoint = MathUtil.clamp(height, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
    controller.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(0.0), ArbFFUnits.kVoltage);

}

@Override
public void setVoltage(double voltage) {
    leftMotor.setVoltage(MathUtil.clamp(voltage, ElevatorConstants.minVoltage, ElevatorConstants.maxVoltage));
}

@Override
public void stop() {
    leftMotor.setVoltage(0.0);
    rightMotor.setVoltage(0.0);
}
}
