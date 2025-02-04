package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.wrappers.PIDConstants;
import frc.robot.wrappers.ProfiledPIDConstants;

public class Constants {
  public RobotConstants Robot = new RobotConstants();
  public FieldConstants Field = new FieldConstants();
  public SwerveDriveConstants SwerveDrive = new SwerveDriveConstants();
  public OdometryConstants Odometry = new OdometryConstants();
  public AutoConstants Auto = new AutoConstants();

  public static class RobotConstants {
    public String k_canBus = "rio"; // this is the default, but it helps differentiate between this and the
                                    // Drivetrain CANivore

    public double k_width = 28.0; // Inches
    public double k_length = 32.0; // Inches

    public double k_bumperStart = 1.0; // Inches
    public double k_bumperHeight = 5.0; // Inches

    public double k_period = 1.0 / 50.0; // the robot runs at 50Hz
  }

  public static class FieldConstants {
    public double k_width = Units.feetToMeters(26.0) + Units.inchesToMeters(5);
    public double k_length = Units.feetToMeters(57.0) + Units.inchesToMeters(6.0 + (7.0 / 8.0));

    public double k_reefFaceToFaceWidth = Units.inchesToMeters((5 * 12) + 5.5); // 5'5.5" ("REEF ZONE": section 5.3)

    public double k_blueReefX = Units.inchesToMeters(144) + k_reefFaceToFaceWidth / 2; // 144 inches + half the
                                                                                       // width
    public double k_blueReefY = Units.inchesToMeters(((26 * 12) + 5) / 2); // 26'5" / 2

    public Pose3d k_blueReefPose = new Pose3d(k_blueReefX, k_blueReefY, 2.0, new Rotation3d());
    public Pose3d k_redReefPose = new Pose3d(k_length - k_blueReefX, k_blueReefY, 2.0, new Rotation3d());
  }

  public static class OdometryConstants {
    public int k_threadUpdateFrequency = 250; // Hz
  }

  public static class SwerveDriveConstants {
    public String k_canBus = "Drivetrain";

    // Drivetrain wheel offsets
    public double k_xDistance = Units.inchesToMeters(26.75); // 30 inches Forward/Backward
    public double k_yDistance = Units.inchesToMeters(22.75); // in meters! Side-to-Side

    public double k_xCenterDistance = k_xDistance / 2.0;
    public double k_yCenterDistance = k_yDistance / 2.0;

    // Max speeds
    public double k_maxSpeed = 1.5; // Meters per second
    public double k_maxBoostSpeed = 4.5; // Meters per second
    public double k_maxAngularSpeed = Math.PI * 2.0; // Radians per second

    // Max acceleration
    public double k_maxLinearAcceleration = 12.0; // Meters per second^2
    public double k_maxAngularAcceleration = Math.PI * 8.0; // Radians per second^2

    public double k_slowScaler = 0; // % reduction in speed
    public double k_boostScaler = 2; // % increase in speed

    public double k_wheelRadiusIn = 2.0; // inches
    public double k_wheelCircumference = Units.inchesToMeters(k_wheelRadiusIn * 2.0 * Math.PI); // meters
    public double k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public double k_turnGearRatio = 150.0 / 7.0;

    public DriveConstants Drive = new DriveConstants();

    // Drivetrain drive motor constants
    public class DriveConstants {
      public final int k_FLMotorId = 5;
      public final int k_FRMotorId = 6;
      public final int k_BLMotorId = 7;
      public final int k_BRMotorId = 8;

      public int k_currentLimit = 40;

      public double k_P = 0.84992; // setCory("goated");
      public double k_I = 0.0;
      public double k_D = 0.0;
      public double k_IZone = 0.0;

      public double k_FFS = 0.2368;
      public double k_FFV = 0.67229;
      public double k_FFA = 0.080151;
    }

    public TurnConstants Turn = new TurnConstants();

    // Drivetrain (turn) constants
    public class TurnConstants {
      // Drivetrain turning offset constants
      public double k_FLOffset = -0.1521;
      public double k_FROffset = 0.02417;
      public double k_BLOffset = 0.594889;
      public double k_BROffset = 0.419678;

      public int k_FLAbsId = 13;
      public int k_FRAbsId = 14;
      public int k_BLAbsId = 15;
      public int k_BRAbsId = 16;

      public int k_currentLimit = 25;

      public final int k_FLMotorId = 9;
      public final int k_FRMotorId = 10;
      public final int k_BLMotorId = 11;
      public final int k_BRMotorId = 12;

      public double k_P = 70.0;
      public double k_I = 0.0;
      public double k_D = 1.0;
      public double k_IZone = 0.0;

      // We only use FF and are too scared to delete the others
      public double k_FF = 0.0;

      public double k_S = 0.0; // 0.29745;
      public double k_V = 0.0; // 0.43892; // Not used (fear)
      public double k_A = 0.0; // 0.048573; // Not used (fear)

      public double k_minOutput = -1.0;
      public double k_maxOutput = 1.0;
    }

    public ChassisConstants Chassis = new ChassisConstants();

    public class ChassisConstants {
      public DriveConstants Drive = new DriveConstants();

      public class DriveConstants {
        public double k_P = 0.0;
        public double k_I = 0.0;
        public double k_D = 0.0;
      }

      public TurnConstants Turn = new TurnConstants();

      public class TurnConstants {
        public double k_P = 5.0;
        public double k_I = 0.0;
        public double k_D = 0.0;
      }
    }
  }

  public PoseAlignerConstants PoseAligner = new PoseAlignerConstants();

  public class PoseAlignerConstants {

  }

  public static class AutoConstants {
    // Needs to be more than the max robot speed, to allow for turning
    public double k_maxVelocity = 0.0; // Meters per second
    public double k_maxAcceleration = 0.0; // Meters per second
    public PIDConstants k_translationConstants = new PIDConstants(0.0, 0.0, 0.0);
    public ProfiledPIDConstants k_rotationConstants = new ProfiledPIDConstants(0.0, 0.0, 0.0, k_maxVelocity, k_maxAcceleration);

    public TimingConstants Timing = new TimingConstants();

    public class TimingConstants {
      
    }
  }
}
