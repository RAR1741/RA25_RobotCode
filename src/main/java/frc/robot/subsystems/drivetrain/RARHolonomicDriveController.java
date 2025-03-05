package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.PIDConstants;
import frc.robot.wrappers.ProfiledPIDConstants;

/** Pose targeter for holonomic drive trains */
public class RARHolonomicDriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController rotationController;

  private boolean m_isEnabled = true;
  private double mpsToRps = 1.0 / RobotConstants.robotConfig.SwerveDrive.k_wheelBaseRadius;

  // private boolean firstTimeForEverything = false;

  /**
   * Constructs a HolonomicDriveController
   *
   * @param translationConstants PID constants for the translation PID controllers
   * @param rotationConstants    PID constants for the rotation controller
   * @param period               Period of the control loop in seconds
   */
  public RARHolonomicDriveController(
      PIDConstants translationConstants, ProfiledPIDConstants rotationConstants, double period) {
    this.xController = new PIDController(
        translationConstants.k_P, translationConstants.k_I, translationConstants.k_D, period);
    this.xController.setIntegratorRange(-translationConstants.k_iZone, translationConstants.k_iZone);

    this.yController = new PIDController(
        translationConstants.k_P, translationConstants.k_I, translationConstants.k_D, period);
    this.yController.setIntegratorRange(-translationConstants.k_iZone, translationConstants.k_iZone);

    // Temp rate limit of 0, will be changed in calculate
    this.rotationController = new ProfiledPIDController(
        rotationConstants.k_P,
        rotationConstants.k_I,
        rotationConstants.k_D,
        new Constraints(rotationConstants.k_maxVel, rotationConstants.k_maxAcc),
        period);
    this.rotationController.setIntegratorRange(-rotationConstants.k_iZone, rotationConstants.k_iZone);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Constructs a HolonomicDriveController
   *
   * @param translationConstants PID constants for the translation PID controllers
   * @param rotationConstants    PID constants for the rotation controller
   */
  public RARHolonomicDriveController(
      PIDConstants translationConstants, ProfiledPIDConstants rotationConstants) {
    this(translationConstants, rotationConstants, 0.02);
  }

  /**
   * Enables and disables the controller for troubleshooting. When calculate() is
   * called on a
   * disabled controller, no values are returned.
   *
   * @param enabled If the controller is enabled or not
   */
  public void setEnabled(boolean enabled) {
    m_isEnabled = enabled;
  }

  /**
   * Resets the controller based on the current state of the robot
   *
   * @param currentPose   Current robot pose
   * @param currentSpeeds Current robot relative chassis speeds
   */
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    xController.reset();
    yController.reset();
    rotationController.reset(new State(currentPose.getRotation().getRadians(),
        currentSpeeds.omegaRadiansPerSecond));
  }

  /**
   * Calculates the next output of the path following controller
   *
   * @param currentPose      The current robot pose
   * @param targetState      The desired trajectory state
   * @param goalPose         The pose to end at
   * @param maxApproachSpeed the speed in m/s to run at
   * @return The next robot relative output of the path following controller
   */
  public ChassisSpeeds calculatePoseSpeeds(Pose2d currentPose, Pose2d goalPose, double maxApproachSpeed) {

    Logger.recordOutput("Odometry/GoalPose", goalPose);

    if (!m_isEnabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, currentPose.getRotation());
    }

    Rotation2d targetHeading = goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double translationError = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    double rotationError = Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees());

    // As we get closer to the target, we should be ramping down our x/y FF
    double targetSpeed = maxApproachSpeed;
    double falloffDistance = RobotConstants.robotConfig.AutoAlign.k_fallOffDistance;

    if (translationError < falloffDistance) {
      targetSpeed = (maxApproachSpeed / falloffDistance) * translationError;
    }

    Logger.recordOutput("AutoAligner/translationError", translationError);
    Logger.recordOutput("AutoAligner/rotationError", rotationError);

    Logger.recordOutput("AutoAligner/TargetSpeed", targetSpeed);

    double xFF = targetSpeed * targetHeading.getCos();
    double yFF = targetSpeed * targetHeading.getSin();

    double xFeedback = this.xController.calculate(currentPose.getX(), goalPose.getX());
    double yFeedback = this.yController.calculate(currentPose.getY(), goalPose.getY());

    double rotationFeedback = rotationController.calculate(
        currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFeedback, currentPose.getRotation());
  }

  /**
   * Calculates the next output of the path following controller
   *
   * @param currentPose      The current robot pose
   * @param targetState      The desired trajectory state
   * @param goalPose         The pose to end at
   * @param maxApproachSpeed the speed in m/s to run at
   * @return The next robot relative output of the path following controller
   */
  public ChassisSpeeds calculateTrajectorySpeeds(Pose2d currentPose, SwerveSample targetSample) {
    Pose2d goalPose = targetSample.getPose();

    Logger.recordOutput("Odometry/GoalPose", goalPose);

    if (!m_isEnabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, currentPose.getRotation());
    }

    double translationError = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    double rotationError = Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees());
    double targetSpeed = Math.hypot(targetSample.vx, targetSample.vy);

    Logger.recordOutput("Auto/DriveTrajectory/translationError", translationError);
    Logger.recordOutput("Auto/DriveTrajectory/rotationError", rotationError);
    Logger.recordOutput("Auto/DriveTrajectory/TargetSpeed", targetSpeed);

    double xFeedback = this.xController.calculate(currentPose.getX(), goalPose.getX());
    double yFeedback = this.yController.calculate(currentPose.getY(), goalPose.getY());

    double rotationFeedback = rotationController.calculate(
        currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        targetSample.vx + xFeedback, targetSample.vy + yFeedback, rotationFeedback, currentPose.getRotation());
  }

  // public ChassisSpeeds calculateTrajectorySpeeds(Pose2d currentPose,
  // SwerveSample targetState) {
  // // This is the only thing we actually changed
  // // if (firstTimeForEverything) {
  // // firstTimeForEverything = false;
  // // rotationController.reset(currentPose.getRotation().getRadians());
  // // }

  // double xFF = targetState.vx * Math.cos(targetState.heading);
  // double yFF = targetState.vy * Math.sin(targetState.heading);

  // double translationError =
  // currentPose.getTranslation().getDistance(targetState.getPose().getTranslation());
  // double rotationError =
  // Math.abs(currentPose.getRotation().minus(targetState.getPose().getRotation()).getDegrees());
  // double targetSpeed = Math.hypot(targetState.vx, targetState.vy);

  // if (!m_isEnabled) {
  // return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, 0,
  // currentPose.getRotation());
  // }

  // double xFeedback = this.xController.calculate(currentPose.getX(),
  // targetState.x);
  // double yFeedback = this.yController.calculate(currentPose.getY(),
  // targetState.y);

  // double angVelConstraint = targetState.omega;
  // double maxAngVel = angVelConstraint;

  // if (Double.isFinite(maxAngVel)) {
  // // Approximation of available module speed to do rotation with
  // double maxAngVelModule = Math.max(0,
  // RobotConstants.robotConfig.SwerveDrive.k_maxPossibleSpeed
  // - Math.hypot(targetState.vx, targetState.vy))
  // * mpsToRps;
  // maxAngVel = Math.min(angVelConstraint, maxAngVelModule);
  // }

  // TrapezoidProfile.Constraints rotationConstraints = new
  // TrapezoidProfile.Constraints(maxAngVel,
  // Math.hypot(targetState.ax, targetState.ay));

  // Rotation2d targetRotation = targetState.getPose().getRotation();

  // // if (rotationTargetOverride != null) {
  // // targetRotation = rotationTargetOverride.get().orElse(targetRotation);
  // // }

  // double rotationFeedback = rotationController.calculate(
  // currentPose.getRotation().getRadians(),
  // new TrapezoidProfile.State(targetRotation.getRadians(), 0),
  // rotationConstraints);
  // double rotationFF = targetState.omega; //
  // .orElse(rotationController.getSetpoint().velocity);

  // Logger.recordOutput("Auto/DriveTrajectory/translationError",
  // translationError);
  // Logger.recordOutput("Auto/DriveTrajectory/rotationError", rotationError);
  // Logger.recordOutput("Auto/DriveTrajectory/TargetSpeed", targetSpeed);

  // return ChassisSpeeds.fromFieldRelativeSpeeds(
  // xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback,
  // currentPose.getRotation());
  // }
}
