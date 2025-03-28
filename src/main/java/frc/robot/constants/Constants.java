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
  public final LEDConstants LEDs = new LEDConstants();

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
    public final double k_xDistance = Units.inchesToMeters(26.75); // 32 inches Forward/Backward
    public final double k_yDistance = Units.inchesToMeters(22.75); // in meters! Side-to-Side

    public final double k_xCenterDistance = k_xDistance / 2.0;
    public final double k_yCenterDistance = k_yDistance / 2.0;
    public final double k_wheelBaseRadius = Math.hypot(k_xCenterDistance, k_yCenterDistance);

    // Max speeds
    public final double k_maxDriverSpeed = 2.5; // Meters per second
    public final double k_maxDriverBoostSpeed = 4.5; // Meters per second
    public final double k_maxPossibleSpeed = 4.574; // Meters per second

    public final double k_maxDriverAngularSpeed = Math.PI * 1.5; // Radians per second
    public final double k_maxPossibleAngularSpeed = 10.256; // Radians per second, pulled from Choreo

    // Max acceleration
    public final double k_maxLinearAcceleration = 10.791; // Meters per second^2, pulled from Choreo
    public final double k_maxAngularAcceleration = 46.304; // Radians per second^2, pulled from Choreo

    public final double k_slowScaler = 0.5; // % reduction in speed
    public final double k_boostScaler = 4.5 / 2.5; // % increase in speed (k_maxDriverBoostSpeed/k_maxDriverSpeed)

    public final double k_wheelRadiusIn = 2.0; // inches
    public final double k_wheelCircumference = Units.inchesToMeters(k_wheelRadiusIn * 2.0 * Math.PI); // meters
    public final double k_driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public final double k_turnGearRatio = 150.0 / 7.0;

    public final DriveConstants Drive = new DriveConstants();

    // Drivetrain drive motor constants
    public class DriveConstants {
      public final int k_FLMotorId = 5;
      public final int k_FRMotorId = 6;
      public final int k_BLMotorId = 7;
      public final int k_BRMotorId = 8;

      public final int k_statorCurrentLimit = 120;
      public final int k_supplyCurrentLimit = 70;

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

      public final int k_statorCurrentLimit = 40;

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
    public double k_maxAngularVelocity = 10.256; // rads per second
    public double k_maxAngularAcceleration = 46.304; // rads per second^2
    public PIDConstants k_translationConstants = new PIDConstants(6.0, 0.0, 0.0);
    public ProfiledPIDConstants k_rotationConstants = new ProfiledPIDConstants(6.0, 0.0, 0.0, k_maxAngularVelocity,
        k_maxAngularAcceleration);

    public final double k_lowSpeed = 0.2;

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

    public final double k_maxVelocity = 150; // 130;
    public final double k_maxAcceleration = 600; // 400;

    public final int k_maxCurrent = 50;

    public final double k_stowHeight = 0.0;
    public final double k_L1Height = 16.0;
    public final double k_L2Height = 26.0;
    public final double k_L3Height = 42.7;
    public final double k_L4Height = 60.5; // 59.6;
    public final double k_maxHeight = 60.5;
    public final double k_feederHeight = 29.57;
    // public final double k_groundAlgaeHeight = 0.0;
    public final double k_lowAlgaeHeight = 27.643;
    public final double k_highAlgaeHeight = 44.571;

    public final double k_allowedError = 0.4;
  }

  public static class ArmConstants {
    public final int k_motorId = 30;

    public final double k_P = 5.0;
    public final double k_I = 0.0;
    public final double k_D = 0.0;
    public final double k_IZone = 0.0;

    public final double k_FFS = 0.0;
    public final double k_FFV = 0.0;
    public final double k_FFA = 0.0;
    public final double k_FFG = 0.35;

    public final double k_constantVoltage = -0.4;
    public final double k_stowThreshold = 0.02;

    public final int k_maxCurrent = 30;

    public double k_stowAngle;
    public double k_L4Angle;
    public double k_horizontalAngle;
    public double k_sourceAngle;

    public final double k_maxAcceleration = 8.0;
    public final double k_maxVelocity = 2.0;

    public final double k_allowedError = 0.02;
  }

  public static class EndEffectorConstants {
    public final int k_leftMotorId = 31;
    public final int k_rightMotorId = 32;

    public final double k_rollerGearRatio = 1.0 / 10.0;

    public final double k_rollerP = 0.0004;
    public final double k_rollerI = 0.000;
    public final double k_rollerD = 0.0;
    public final double k_rollerFF = 0.001;

    public final int k_maxCurrent = 20;

    // public final double[] k_stopSpeeds = new double[] { 0.0, 0.0 };
    // public final double[] k_forwardIndexFastSpeeds = new double[] { 0.20, 0.20 };
    // public final double[] k_forwardIndexSlowSpeeds = new double[] { 0.10, 0.10 };
    // public final double[] k_reverseIndexSpeeds = new double[] { -0.05, -0.05 };
    // public final double[] k_branchSpeeds = new double[] { 0.6, 0.45 };
    // public final double[] k_troughSpeeds = new double[] { 0.3, 0.5 };

    public final double k_stopSpeed = 0.0;
    public final double k_forwardIndexFastSpeed = 300.0;
    public final double k_forwardIndexSlowSpeed = 80.0;
    public final double k_reverseIndexSpeed = -60.0;
    public final double k_branchSpeed = 300.0;
    public final double k_troughSpeed = 300.0;
  }

  public static class LaserCanConstants {
    public final int k_indexId = 33;
    public final int k_entranceId = 34;
    public final int k_exitId = 35;

    public final double k_entranceThreshold = 110.0;
    public final double k_exitThreshold = 40.0;
  }

  public static class IntakeConstants {
    public final int k_pivotMotorIdLeft = 40;
    public final int k_pivotMotorIdRight = 41;

    public final int k_rollerMotorIdLeft = 42;
    public final int k_rollerMotorIdRight = 43;

    public final int k_pivotCurrentLimit = 30;
    public final int k_rollerCurrentLimit = 60;

    public final double k_rollerGearRatio = (1.0 / 4.0);

    public final double k_pivotMotorP = 4.0; // 15.0;
    public final double k_pivotMotorI = 0.0;
    public final double k_pivotMotorD = 0.0;

    public final double k_pivotMotorKS = 0.0;
    public final double k_pivotMotorKG = -0.25;
    public final double k_pivotMotorKV = 0.0;
    public final double k_pivotMotorKA = 0.0;

    // TODO: maybe tune P and FF further
    public final double k_rollerMotorP = 0.001; // 0.0015;
    public final double k_rollerMotorI = 0.0;
    public final double k_rollerMotorD = 0.0;
    public final double k_rollerMotorFF = 0.0006;

    public final double k_maxAcceleration = 4.0;
    public final double k_maxVelocity = 2.0;

    public final double k_maxIntakeSpeed = 300.0;

    public final double k_allowedPivotError = 0.01;
    public final double k_lowestRollerSpeed = -1000.0; // TODO: Make this work
    public final int k_debounceLimit = 5;

    public final LeftConstants Left = new LeftConstants();
    public final RightConstants Right = new RightConstants();

    public class LeftConstants {
      public double k_stowPosition;
      public double k_groundPosition;
      public double k_ejectPosition;
      public double k_horizontalPosition;
      public double k_stuckPosition;
    }

    public class RightConstants {
      public double k_stowPosition;
      public double k_groundPosition;
      public double k_ejectPosition;
      public double k_horizontalPosition;
      public double k_stuckPosition;
    }
  }

  public static class HopperConstants {
    public final int k_hopperMotorId = 50;
    public final double k_hopperSpeed = 0.3;

    public final int k_maxCurrent = 20;
  }

  public static class AutoAlignConstants {
    // Distances to the reef
    public final double k_minSafeArmDistance = 1.5;
    public final double k_minSafeTargetDistance = 1.6;
    public final double k_minSafeElevatorDistance = 2.0;

    // Scoring offsets
    // public final double k_l4ScoringDistance = 0.384;
    public final double k_l4ScoringDistance = 0.254;
    public final double k_otherScoringOffset = 0.0;
    public final double k_scoringHorizontalOffset = 0.175;
    public final double k_scoringTroughHorizontalOffset = 0.46;
    public final double k_algaeHorizontalOffset = 0.0;
    public final double k_algaeReverseExtraDistance = -0.5;

    public final double k_maxApproachSpeed = 5.0;
    public final double k_maxIndexApproachSpeed = 2.0;
    public final double k_fallOffDistance = 1.5;

    // Base feeder scoring pose
    public final double k_feederStationXOffset = 1.2;
    public final double k_feederStationYOffset = 0.92;
    public final double k_feederStationRotationOffset = 54.0;
  }

  public static class LEDConstants {
    public int k_PWMId = 0;
    public boolean k_isEnabled = true;
    public int k_drivetrainUnusedLEDCount = 38;

    public RightElevator Right = new RightElevator();

    public class RightElevator {
      public int k_start = 0;
      // public int k_sections = 8;
      public int k_sectionLength = 15;

      public int k_length = 116;
    }

    public LeftElevator Left = new LeftElevator();

    public class LeftElevator {
      public int k_start = k_drivetrainUnusedLEDCount + Right.k_start + Right.k_length;
      public int k_length = 116;
    }

    public int k_totalLength = 300;
  }
}
