// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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

  public static class SparkMaxCANID {
    public static final int kFrontRightDrive = 1;
    public static final int kFrontRightTurn = 2;
  }

    public static class ModuleConstants{ 
      public static double DriveGearing = 45.0*22/15/13;
      public static double TurnPositionConversionFactor = 2*Math.PI;
      public static double TurnVelocityConversionFactor = 2*Math.PI/60;
      public static double DrivePositionConversionFactor = 2*Math.PI * Units.inchesToMeters(2)/DriveGearing;
      public static double DriveVelocityConversionFactor = 2*Math.PI/60*Units.inchesToMeters(2)/DriveGearing;
      public static double DriveWheelFreeSpeed = 5676.0 * 2*Math.PI * Units.inchesToMeters(2) /DriveGearing/60;

      public static double DriveP = 0.4;
      public static double DriveI = 0;
      public static double DriveD = 0;
      public static double Driveff = 1/DriveWheelFreeSpeed;
      public static double DriveMin = -1;
      public static double DriveMax = 1;
      
      public static double TurnP = 1;
      public static double TurnI = 0;
      public static double TurnD = 0;
      public static double Turnff = 0;
      public static double TurnMin = -1;
      public static double TurnMax = 1;

      public static double TrackWidth = Units.inchesToMeters(27);
      public static double WheelBase = Units.inchesToMeters(26.5);

      public static double DriveCurrentLimit = 50;
      public static double TurnCurrentLimit = 20;

      public static double MaxDriveMetersPerSecond = 4.46;
      public static double MaxTurnRadiansPerSecond = 2*Math.PI;
    }
    
}
