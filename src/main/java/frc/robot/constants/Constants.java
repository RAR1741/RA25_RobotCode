package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.wrappers.PIDConstants;
import frc.robot.wrappers.ProfiledPIDConstants;

public class Constants {
  public final RobotConstants Robot = new RobotConstants();
  public final FieldConstants Field = new FieldConstants();
  public final SwerveDriveConstants SwerveDrive = new SwerveDriveConstants();
  public final IntakeConstants Intake = new IntakeConstants();
  public final ArmConstants Arm = new ArmConstants();
  public final AutoConstants Auto = new AutoConstants();
  public final ElevatorConstants Elevator = new ElevatorConstants();
  public final LaserCanConstants LaserCan = new LaserCanConstants();
  public final EndEffectorConstants EndEffector = new EndEffectorConstants();
  public final OdometryConstants Odometry = new OdometryConstants();
  public final HopperConstants Hopper = new HopperConstants();
  public final AutoAlignConstants AutoAlign = new AutoAlignConstants();

  public static class RobotConstants {
    public final String k_canBus = "rio"; // this is the default, but it helps differentiate between this and the
    // Drivetrain CANivore

    public final double k_width = 28.0; // Inches
    public final double k_length = 32.0; // Inches

    public double k_bumperStart = 1.0; // Inches
    public double k_bumperHeight = 5.0; // Inches

    public double k_period = 1.0 / 50.0; // the robot runs at 50Hz
  }

  public static class FieldConstants {
    public final double k_width = Units.feetToMeters(26.0) + Units.inchesToMeters(5);
    public final double k_length = Units.feetToMeters(57.0) + Units.inchesToMeters(6.0 + (7.0 / 8.0));

    public final double k_reefFaceToFaceWidth = Units.inchesToMeters((5 * 12) + 5.5); // 5'5.5" ("REEF ZONE": section
                                                                                      // 5.3)

    public final double k_blueReefX = Units.inchesToMeters(144) + k_reefFaceToFaceWidth / 2; // 144 inches + half the
    // width
    public final double k_blueReefY = Units.inchesToMeters(((26 * 12) + 5) / 2); // 26'5" / 2

    public final Pose3d k_blueReefPose = new Pose3d(k_blueReefX, k_blueReefY, 2.0, new Rotation3d());
    public final Pose3d k_redReefPose = new Pose3d(k_length - k_blueReefX, k_blueReefY, 2.0, new Rotation3d());
  }

  public static class OdometryConstants {
    public final int k_threadUpdateFrequency = 250; // Hz
  }

  public static class SwerveDriveConstants {
    public final String k_canBus = "Drivetrain";

    // Drivetrain wheel offsets
    public final double k_xDistance = Units.inchesToMeters(26.75); // 30 inches Forward/Backward
    public final double k_yDistance = Units.inchesToMeters(22.75); // in meters! Side-to-Side

    public final double k_xCenterDistance = k_xDistance / 2.0;
    public final double k_yCenterDistance = k_yDistance / 2.0;

    // Max speeds
    public final double k_maxSpeed = 1.5; // Meters per second
    public final double k_maxBoostSpeed = 4.5; // Meters per second
    public final double k_maxAngularSpeed = Math.PI * 1.5; // Radians per second

    // Max acceleration
    public final double k_maxLinearAcceleration = 12.0; // Meters per second^2
    public final double k_maxAngularAcceleration = Math.PI * 8.0; // Radians per second^2

    public final double k_slowScaler = 0; // % reduction in speed
    public final double k_boostScaler = 2; // % increase in speed

    public final double k_wheelRadiusIn = 2.0; // inches
    public final double k_wheelCircumference = Units.inchesToMeters(k_wheelRadiusIn * 2.0 * Math.PI); // meters
    public final double k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public final double k_turnGearRatio = 150.0 / 7.0;

    public final DriveConstants Drive = new DriveConstants();

    // Drivetrain drive motor constants
    public class DriveConstants {
      public final int k_FLMotorId = 5;
      public final int k_FRMotorId = 6;
      public final int k_BLMotorId = 7;
      public final int k_BRMotorId = 8;

      public final int k_currentLimit = 40;

      public double k_P;
      public double k_I;
      public double k_D;
      public double k_IZone;

      public double k_FFS;
      public double k_FFV;
      public double k_FFA;
    }

    public final TurnConstants Turn = new TurnConstants();

    // Drivetrain (turn) constants
    public class TurnConstants {
      // Drivetrain turning offset constants
      public double k_FLOffset;
      public double k_FROffset;
      public double k_BLOffset;
      public double k_BROffset;

      public final int k_FLAbsId = 13;
      public final int k_FRAbsId = 14;
      public final int k_BLAbsId = 15;
      public final int k_BRAbsId = 16;

      public final int k_currentLimit = 25;

      public final int k_FLMotorId = 9;
      public final int k_FRMotorId = 10;
      public final int k_BLMotorId = 11;
      public final int k_BRMotorId = 12;

      public double k_P;
      public double k_I;
      public double k_D;
      public double k_IZone;

      // We only use FF and are too scared to delete the others
      public final double k_FF = 0.0;

      public final double k_S = 0.0; // 0.29745;
      public final double k_V = 0.0; // 0.43892; // Not used (fear)
      public final double k_A = 0.0; // 0.048573; // Not used (fear)

      public final double k_minOutput = -1.0;
      public final double k_maxOutput = 1.0;
    }

    public final ChassisConstants Chassis = new ChassisConstants();

    public class ChassisConstants {
      public final DriveConstants Drive = new DriveConstants();

      public class DriveConstants {
        public final double k_P = 0.0;
        public final double k_I = 0.0;
        public final double k_D = 0.0;
      }

      public final TurnConstants Turn = new TurnConstants();

      public class TurnConstants {
        public final double k_P = 5.0;
        public final double k_I = 0.0;
        public final double k_D = 0.0;
      }
    }
  }

  public static class AutoConstants {
    // Needs to be more than the max robot speed, to allow for turning
    public double k_maxVelocity = 0.0; // Meters per second
    public double k_maxAcceleration = 0.0; // Meters per second
    public PIDConstants k_translationConstants = new PIDConstants(0.0, 0.0, 0.0);
    public ProfiledPIDConstants k_rotationConstants = new ProfiledPIDConstants(0.0, 0.0, 0.0, k_maxVelocity,
        k_maxAcceleration);

    public TimingConstants Timing = new TimingConstants();

    public class TimingConstants {

    }
  }

  public static class ElevatorConstants {
    public final int k_elevatorLeftMotorId = 20;
    public final int k_elevatorRightMotorId = 21;

    public final double k_P = 0.15;
    public final double k_I = 0.0;
    public final double k_D = 0.0;
    public final double k_IZone = 0.0;
    public final double k_FF = 0.50;

    public final double k_maxVelocity = 65;
    public final double k_maxAcceleration = 200;

    public final int k_maxCurrent = 30;

    public final double k_stowHeight = 0.0;
    public final double k_L1Height = 16.0;
    public final double k_L2Height = 25.64;
    public final double k_L3Height = 42.7;
    public final double k_L4Height = 60.5; // 59.6;
    public final double k_maxHeight = 60.5;
    // public final double k_groundAlgaeHeight = 0.0;
    // public final double k_lowAlgaeHeight = 24.8;
    // public final double k_highAlgaeHeight = 42.5;

    public final double k_allowedError = 0.2; // TODO: Change this please 👁️👄👁️
  }

  public static class ArmConstants {
    public final int k_motorId = 30;

    public final double k_P = 10.8;
    public final double k_I = 0.0;
    public final double k_D = 0.0;
    public final double k_IZone = 0.0;

    public final double k_FFS = 0.0;
    public final double k_FFV = 0.0;
    public final double k_FFA = 0.0;
    public final double k_FFG = 0.35;

    public final double k_constantVoltage = -0.4;
    public final double k_stowThreshold = 0.02;

    public final int k_maxCurrent = 10; // TODO: maybe change this?

    public double k_stowAngle;
    public double k_L4Angle;
    public double k_horizontalAngle;

    public final double k_maxAcceleration = 0.8;
    public final double k_maxVelocity = 0.4;

    public final double k_allowedError = 0.02; // TODO: Change this please 🥺
  }

  public static class EndEffectorConstants {
    public final int k_leftMotorId = 31;
    public final int k_rightMotorId = 32;

    public final double[] k_stopSpeeds = new double[] { 0.0, 0.0 };
    public final double[] k_forwardIndexSpeeds = new double[] { 0.15, 0.15 };
    public final double[] k_reverseIndexSpeeds = new double[] { -0.05, -0.05 };
    public final double[] k_reverseSpeeds = new double[] { -0.1, -0.1 };
    public final double[] k_branchSpeeds = new double[] { 0.5, 0.4 };
    public final double[] k_troughSpeeds = new double[] { 0.3, 0.5 };
  }

  public static class LaserCanConstants {
    public final int k_indexId = 33;
    public final int k_entranceId = 34;
    public final int k_exitId = 35;
  }

  public static class IntakeConstants {
    public final int k_pivotMotorIdLeft = 40;
    public final int k_pivotMotorIdRight = 41;

    public final int k_rollerMotorIdLeft = 42;
    public final int k_rollerMotorIdRight = 43;

    public final int k_pivotCurrentLimit = 30;
    public final int k_rollerCurrentLimit = 40;

    public final double k_rollerGearRatio = (1.0 / 4.0);

    // TODO: maybe tune more for higher speed, needs to match with drive train speed
    public final double k_pivotMotorP = 15.0;
    public final double k_pivotMotorI = 0.0;
    public final double k_pivotMotorD = 0.0007; // TODO: Please don't do this

    public final double k_pivotMotorKS = 0.0;
    public final double k_pivotMotorKG = 0.0;
    public final double k_pivotMotorKV = 0.0;
    public final double k_pivotMotorKA = 0.0;

    // TODO: maybe tune P and FF further
    public final double k_rollerMotorP = 0.000425; // 0.0017;
    public final double k_rollerMotorI = 0.0;
    public final double k_rollerMotorD = 0.0; // 0.1;
    public final double k_rollerMotorFF = 0.0006; // 0.25

    public final double k_maxAcceleration = 3.0;
    public final double k_maxVelocity = 1.0;

    public final double k_maxIntakeSpeed = 600.0;

    public final LeftConstants Left = new LeftConstants();
    public final RightConstants Right = new RightConstants();

    public class LeftConstants {
      public double k_stowPosition;
      public double k_groundPosition;
      public double k_ejectPosition;
      public double k_horizontalPosition;
    }

    public class RightConstants {
      public double k_stowPosition;
      public double k_groundPosition;
      public double k_ejectPosition;
      public double k_horizontalPosition;
    }
  }

  public static class HopperConstants {
    public final int k_hopperMotorId = 50;
    public final double k_hopperSpeed = 0.2;
  }

  public static class AutoAlignConstants {
    // Distances to the reef
    public final double k_minSafeArmDistance = 1.5;
    public final double k_minSafeTargetDistance = 1.6;
    public final double k_minSafeElevatorDistance = 2.0;

    // Scoring offsets
    public final double k_scoringDistance = 0.235;
    public final double k_scoringHorizontalOffset = 0.175;

    public final double k_maxApproachSpeed = 5.0;
    public final double k_fallOffDistance = 1.5;
  }

  // TODO add Gamepiece class for Coral- and Algae-related constants
}
