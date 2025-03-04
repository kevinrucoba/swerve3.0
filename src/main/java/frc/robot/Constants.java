// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class SwerveConstants {

    // public static final int ID_backRight_Angle_Motor = 0;
    // public static final int ID_backRight_Speed_Motor = 0;
    // public static final int ID_backRight_Encoder = 0;
    
    public static final int ID_backLeft_Angle_Motor = 7;
    public static final int ID_backLeft_Speed_Motor = 22;
    public static final int ID_backLeft_Encoder = 0;
    
    // public static final int ID_frontRight_Angle_Motor = 0;
    // public static final int ID_frontRight_Speed_Motor = 0;
    // public static final int ID_frontRight_Encoder = 0;
    
    // public static final int ID_frontLeft_Angle_Motor = 0;
    // public static final int ID_frontLeft_Speed_Motor = 0;
    // public static final int ID_frontLeft_Encoder = 0;

  }
}
