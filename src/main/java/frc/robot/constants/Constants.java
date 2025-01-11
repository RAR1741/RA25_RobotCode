package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class Constants {
  public RobotConstants Robot = new RobotConstants();
  public FieldConstants Field = new FieldConstants();
  public SwerveDriveConstants SwerveDrive = new SwerveDriveConstants();

  public static class RobotConstants {
    public double k_width = 27.0; // Inches
    public double k_length = 30.0; // Inches

    public double k_bumperStart = 1.0; // Inches
    public double k_bumperHeight = 5.0; // Inches
  }

  public static class FieldConstants {
    public double k_width = Units.feetToMeters(54.0);
    public double k_length = Units.feetToMeters(27.0);
  }

  public static class SwerveDriveConstants {
    // Drivetrain wheel offsets
    public double k_xDistance = 0.762; // 30 inches Forward/Backward
    public double k_yDistance = 0.762; // in meters! Side-to-Side

    public double k_xCenterDistance = k_xDistance / 2.0;
    public double k_yCenterDistance = k_yDistance / 2.0;

    // Max speeds
    public double k_maxSpeed = 2.5; // Meters per second
    public double k_maxBoostSpeed; // Meters per second
    public double k_maxAngularSpeed = Math.PI * 2.0; // Radians per second

    public double k_maxDemoSpeed = k_maxSpeed / 2.0;
    public double k_maxDemoAngularSpeed = k_maxAngularSpeed / 2.0;
    public double k_maxDemoBoostSpeed = 4.5;

    // Max acceleration
    public double k_maxLinearAcceleration = 12.0; // Meters per second^2
    public double k_maxAngularAcceleration = Math.PI * 8.0; // Radians per second^2

    public double k_slowScaler; // % reduction in speed
    public double k_boostScaler; // % increase in speed

    public double k_wheelRadiusIn; // inches
    public double k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public double k_turnGearRatio = 7.0 / 150.0;

    public DriveConstants Drive = new DriveConstants();

    // Drivetrain drive motor constants
    public class DriveConstants {
      public final int k_FLMotorId = 5;
      public final int k_FRMotorId = 6;
      public final int k_BLMotorId = 7;
      public final int k_BRMotorId = 8;

      public int k_currentLimit = 40;

      public double k_P = 1.5;
      public double k_I = 0.0;
      public double k_D = 0.0;
      public double k_IZone = 0.0;

      public double k_FFS;
      public double k_FFV;
      public double k_FFA;
    }

    public TurnConstants Turn = new TurnConstants();

    // Drivetrain (turn) constants
    public class TurnConstants {
      // Drivetrain turning offset constants
      public double k_FLOffset;
      public double k_FROffset;
      public double k_BROffset;
      public double k_BLOffset;

      public int k_FLAbsId = 0;
      public int k_FRAbsId = 1;
      public int k_BRAbsId = 2;
      public int k_BLAbsId = 3;

      public int k_currentLimit = 25;

      public final int k_FLMotorId = 9;
      public final int k_FRMotorId = 10;
      public final int k_BLMotorId = 11;
      public final int k_BRMotorId = 12;

      public double k_P = 1.0;
      public double k_I = 0.0;
      public double k_D = 0.0;
      public double k_IZone = 0.0;

      // We only use FF and are too scared to delete the others
      public double k_FF = 0.0;

      public double k_FFS = 0.29745;
      public double k_FFV = 0.43892; // Not used (fear)
      public double k_FFA = 0.048573; // Not used (fear)

      public double k_minOutput = -1.0;
      public double k_maxOutput = 1.0;
    }
  }
}
