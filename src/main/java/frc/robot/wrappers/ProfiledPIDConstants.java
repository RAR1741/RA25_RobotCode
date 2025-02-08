package frc.robot.wrappers;

import com.pathplanner.lib.config.PIDConstants;

public class ProfiledPIDConstants extends PIDConstants {
  /** Max Vel */
  public final double maxVel;
  /** Max Acc */
  public final double maxAcc;

  /**
   * Create a new ProfiledPIDConstants object
   *
   * @param kP P
   * @param kI I
   * @param kD D
   * @param iZone Integral range
   * @param maxVel Maximum Velocity
   * @param maxAcc Maximum Acceleration
   */
  public ProfiledPIDConstants(
    double kP,
    double kI,
    double kD,
    double iZone,
    double maxVel,
    double maxAcc) {
    super(kP, kI, kD, iZone);
    this.maxVel = maxVel;
    this.maxAcc = maxAcc;
  }

  /**
   * Create a new ProfiledPIDConstants object
   *
   * @param kP P
   * @param kI I
   * @param kD D
   * @param maxVel Maximum Velocity
   * @param maxAcc Maximum Acceleration
   */
  public ProfiledPIDConstants(
    double kP,
    double kI,
    double kD,
    double maxVel,
    double maxAcc) {
    super(kP, kI, kD);
    this.maxVel = maxVel;
    this.maxAcc = maxAcc;
  }
}
