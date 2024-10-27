// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;


public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private CANcoder encoder;
  //private CANcoderConfiguration configuration;
  private PIDController pidController;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public SwerveModule(int angleMotor, int speedMotor, int encoder) {
    this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
    this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
    this.speedMotor.setInverted(false);
    this.angleMotor.setInverted(true);

    this.encoder = new CANcoder(encoder, "rio");
    //configuration.MagnetSensor.MagnetOffset = 0.0;
    //this.encoder.getConfigurator().apply(configuration);

    pidController = new PIDController(kP, kI, kD);
    //0.15e-2
    kP = 5e-1; 
    kI = 0;
    kD = 0; 
    kIz = 0.02; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.enableContinuousInput(0, 360);
  }

  public void drive(double speed, double angle){
    speedMotor.set(speed);
    //aqui es donde se realciona PID motor con encoder
    angleMotor.set(pidController.calculate(encoder.getAbsolutePosition().getValue(), angle));
    SmartDashboard.putNumber("cancoder", encoder.getAbsolutePosition().getValue());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
