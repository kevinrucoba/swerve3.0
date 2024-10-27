// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveChassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  // private SwerveModule backRight;
  private SwerveModule backLeft;
  // private SwerveModule frontRight;
  // private SwerveModule frontLeft;

  //private AHRS NaVX;

  // public SwerveChassis(SwerveModule backRight, SwerveModule backLeft, SwerveModule frontRight, SwerveModule frontLeft) {
  //   this.backRight = backRight;
  //   this.backLeft = backLeft;
  //   this.frontRight = frontRight;
  //   this.frontLeft = frontLeft;

  //   NaVX = new AHRS(SPI.Port.kMXP);
  // }
  public SwerveChassis() {
    //backRight = new SwerveModule(Constants.SwerveConstants.ID_backRight_Angle_Motor, Constants.SwerveConstants.ID_backRight_Speed_Motor, Constants.SwerveConstants.ID_backRight_Encoder);
    backLeft = new SwerveModule(Constants.SwerveConstants.ID_backLeft_Angle_Motor, Constants.SwerveConstants.ID_backLeft_Speed_Motor, Constants.SwerveConstants.ID_backLeft_Encoder);
    //frontRight = new SwerveModule(Constants.SwerveConstants.ID_frontRight_Angle_Motor, Constants.SwerveConstants.ID_frontRight_Speed_Motor, Constants.SwerveConstants.ID_frontRight_Encoder);
    //frontLeft = new SwerveModule(Constants.SwerveConstants.ID_frontLeft_Angle_Motor, Constants.SwerveConstants.ID_frontLeft_Speed_Motor, Constants.SwerveConstants.ID_frontLeft_Encoder);

    //NaVX = new AHRS(SPI.Port.kMXP);
  }

  // public final double L = 22.7;
  // public final double W = 27.8;
  public final double L = 1;
  public final double W = 1;

  public void drive(double x1, double y1, double x2){
    double r = Math.sqrt((L*L)+(W*W));
    y1 *= -1;

    //double angleNaVX = (NaVX.getAngle()%360);
    //double angleRadians = Math.toRadians(angleNaVX);
    
    double matrizR[][] = new double[2][2];
    double ejesR[][] = new double[1][2];

    // matrizR[0][0] = Math.cos(angleRadians);
    // matrizR[0][1] = -Math.sin(angleRadians);
    // matrizR[1][0] = Math.sin(angleRadians);
    // matrizR[1][1] = Math.cos(angleRadians);

    if (Math.abs(x1)<0.1&&Math.abs(y1)<0.1&&Math.abs(x2)<0.1) {
      x1 = 0; y1 = 0; x2 = 0;
    }
    
    

    // ejesR[0][0] = ((matrizR[0][0]*x1)+(matrizR[0][1]*y1));     //X
    // ejesR[0][1] = ((matrizR[1][0]*x1)+(matrizR[1][1]*y1));    //Y
    
    // double a = ejesR[0][0] - x2 * (L / r);
    // double b = ejesR[0][0] + x2 * (L / r);
    // double c = ejesR[0][1] - x2 * (W / r);
    // double d = ejesR[0][1] + x2 * (W / r);

    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    // Cálculo de las velocidades
    double backLeftSpeed = Math.sqrt((a * a) + (d * d));
    double backRightSpeed = Math.sqrt((a * a) + (c * c));
    double frontLeftSpeed = Math.sqrt((b * b) + (d * d));
    double frontRightSpeed = Math.sqrt((b * b) + (c * c));
    // Cálculo de los ángulos normalizados (entre 0 y 1)
    double backLeftAngle = Math.atan2(a, d) / (2 * Math.PI);
    double backRightAngle = Math.atan2(a, c) / (2 * Math.PI);
    double frontLeftAngle = Math.atan2(b, d) / (2 * Math.PI);
    double frontRightAngle = Math.atan2(b, c) / (2 * Math.PI);

    // Normalización de ángulos entre 0 y 1
    backRightAngle = (backRightAngle + 1) % 1;
    backLeftAngle = (backLeftAngle + 1) % 1;
    frontRightAngle = (frontRightAngle + 1) % 1;
    frontLeftAngle = (frontLeftAngle + 1) % 1;

    // Optimización de ángulos y ajuste de velocidades
    if (backRightAngle > 0.25 && backRightAngle < 0.75) {
        backRightSpeed = -backRightSpeed;
        backRightAngle = (backRightAngle + 0.5) % 1;
    }
    if (backLeftAngle > 0.25 && backLeftAngle < 0.75) {
        backLeftSpeed = -backLeftSpeed;
        backLeftAngle = (backLeftAngle + 0.5) % 1;
    }
    if (frontRightAngle > 0.25 && frontRightAngle < 0.75) {
        frontRightSpeed = -frontRightSpeed;
        frontRightAngle = (frontRightAngle + 0.5) % 1;
    }
    if (frontLeftAngle > 0.25 && frontLeftAngle < 0.75) {
        frontLeftSpeed = -frontLeftSpeed;
        frontLeftAngle = (frontLeftAngle + 0.5) % 1;
    }

    //backRight.drive(backRightSpeed, backRightAngle);
    backLeft.drive(backLeftSpeed, backLeftAngle);
    //frontRight.drive(frontRightSpeed, frontRightAngle);
    //frontLeft.drive(frontLeftSpeed, frontLeftAngle);
    SmartDashboard.putNumber("backLeftSpeed", backLeftSpeed);
    SmartDashboard.putNumber("backLeftAngle", backLeftAngle);
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
