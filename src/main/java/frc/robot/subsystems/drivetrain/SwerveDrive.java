package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.wrappers.PIDConstants;
import frc.robot.wrappers.ProfiledPIDConstants;

public class SwerveDrive extends Subsystem {
  private static SwerveDrive m_swerve = null;

  private SwerveDrive() {
    super("SwerveDrive");
  }

  // Robot "forward" is +x
  // Robot "left" is +y
  // Robot "clockwise" is -z
  private final Translation2d m_frontLeftLocation = new Translation2d(
      RobotConstants.robotConfig.SwerveDrive.k_xCenterDistance,
      RobotConstants.robotConfig.SwerveDrive.k_yCenterDistance);
  private final Translation2d m_frontRightLocation = new Translation2d(
      RobotConstants.robotConfig.SwerveDrive.k_xCenterDistance,
      -RobotConstants.robotConfig.SwerveDrive.k_yCenterDistance);
  private final Translation2d m_backLeftLocation = new Translation2d(
      -RobotConstants.robotConfig.SwerveDrive.k_xCenterDistance,
      RobotConstants.robotConfig.SwerveDrive.k_yCenterDistance);
  private final Translation2d m_backRightLocation = new Translation2d(
      -RobotConstants.robotConfig.SwerveDrive.k_xCenterDistance,
      -RobotConstants.robotConfig.SwerveDrive.k_yCenterDistance);

  private static final SwerveModule[] m_modules = {
      new SwerveModule(
          "FL",
          RobotConstants.robotConfig.SwerveDrive.Drive.k_FLMotorId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_FLMotorId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_FLAbsId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_FLOffset), // 0

      new SwerveModule(
          "FR",
          RobotConstants.robotConfig.SwerveDrive.Drive.k_FRMotorId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_FRMotorId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_FRAbsId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_FROffset), // 1

      new SwerveModule(
          "BL",
          RobotConstants.robotConfig.SwerveDrive.Drive.k_BLMotorId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_BLMotorId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_BLAbsId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_BLOffset), // 2

      new SwerveModule(
          "BR",
          RobotConstants.robotConfig.SwerveDrive.Drive.k_BRMotorId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_BRMotorId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_BRAbsId,
          RobotConstants.robotConfig.SwerveDrive.Turn.k_BROffset) // 3
  };

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final RARHolonomicDriveController m_driveController = new RARHolonomicDriveController(
      new PIDConstants(
          RobotConstants.robotConfig.SwerveDrive.Chassis.Drive.k_P,
          RobotConstants.robotConfig.SwerveDrive.Chassis.Drive.k_I,
          RobotConstants.robotConfig.SwerveDrive.Chassis.Drive.k_D),
      new ProfiledPIDConstants(
          RobotConstants.robotConfig.SwerveDrive.Chassis.Turn.k_P,
          RobotConstants.robotConfig.SwerveDrive.Chassis.Turn.k_I,
          RobotConstants.robotConfig.SwerveDrive.Chassis.Turn.k_D,
          RobotConstants.robotConfig.SwerveDrive.k_maxAngularSpeed,
          RobotConstants.robotConfig.SwerveDrive.k_maxAngularAcceleration),
      0.02);

  public static SwerveDrive getInstance() {
    if (m_swerve == null) {
      m_swerve = new SwerveDrive();
    }
    return m_swerve;
  }

  public void setBrake(boolean isBrake) {
    for (SwerveModule module : m_modules) {
      if (isBrake) {
        module.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
      } else {
        module.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
      }
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                RAROdometry.getInstance().getGyro().getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    double maxBoostSpeed = RobotConstants.robotConfig.SwerveDrive.k_maxSpeed
        * RobotConstants.robotConfig.SwerveDrive.k_boostScaler;

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxBoostSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        RobotConstants.robotConfig.SwerveDrive.k_maxBoostSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Pose2d currentPose,
      Pose2d targetPose) {
    ChassisSpeeds driverChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
            RAROdometry.getInstance().getGyro().getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);
    // TODO be able to call RAROdometry from Swerve Drive (maybe pass modules as
    // parameters)

    ChassisSpeeds targetPoseChassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(
        currentPose,
        targetPose,
        RobotConstants.robotConfig.AutoAlign.k_maxApproachSpeed);

    drive(driverChassisSpeeds.plus(targetPoseChassisSpeeds));
  }

  public void drive(Pose2d currentPose, Pose2d goalPose) {
    ChassisSpeeds targetPoseChassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(
        currentPose,
        goalPose,
        RobotConstants.robotConfig.AutoAlign.k_maxApproachSpeed);

    drive(targetPoseChassisSpeeds);
  }

  public void resetDriveController() {
    Pose2d currentPose = RAROdometry.getInstance().getPose();
    m_driveController.reset(currentPose, m_kinematics.toChassisSpeeds(getCurrentStates())); // this is fine 🔥
  }

  public void resetDriveEncoders() {
    for (SwerveModule module : m_modules) {
      module.resetDriveEncoder();
    }
  }

  @Override
  public void periodic() {
    for (SwerveModule module : m_modules) {
      module.periodic();
    }
  }

  @Override
  public void stop() {
    setBrake(true);
    drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public void writePeriodicOutputs() {
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public SwerveModule getModule(int module) {
    switch (module) {
      case Module.FRONT_LEFT -> {
        return m_modules[Module.FRONT_LEFT];
      }
      case Module.FRONT_RIGHT -> {
        return m_modules[Module.FRONT_RIGHT];
      }
      case Module.BACK_LEFT -> {
        return m_modules[Module.BACK_LEFT];
      }
      case Module.BACK_RIGHT -> {
        return m_modules[Module.BACK_RIGHT];
      }
    }
    return null;
  }

  public SwerveModule[] getSwerveModules() {
    return m_modules;
  }

  @AutoLogOutput
  private SwerveModuleState[] getCurrentStates() {
    SwerveModuleState[] currentStates = {
        m_modules[Module.FRONT_LEFT].getState(),
        m_modules[Module.FRONT_RIGHT].getState(),
        m_modules[Module.BACK_LEFT].getState(),
        m_modules[Module.BACK_RIGHT].getState()
    };

    return currentStates;
  }

  @AutoLogOutput
  private SwerveModuleState[] getDesiredStates() {
    SwerveModuleState[] desiredStates = {
        m_modules[Module.FRONT_LEFT].getDesiredState(),
        m_modules[Module.FRONT_RIGHT].getDesiredState(),
        m_modules[Module.BACK_LEFT].getDesiredState(),
        m_modules[Module.BACK_RIGHT].getDesiredState()
    };

    return desiredStates;
  }

  @AutoLogOutput
  private ChassisSpeeds getCurrentChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getCurrentStates());
  }

  public interface Module {
    int FRONT_LEFT = 0;
    int FRONT_RIGHT = 1;
    int BACK_LEFT = 2;
    int BACK_RIGHT = 3;
  }

  @Override
  public void reset() {
  }
}
