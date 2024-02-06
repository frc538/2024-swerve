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
    }
    
}
