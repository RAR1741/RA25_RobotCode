package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.LaserCanHandler;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Arm.ArmState;

public class EndEffector extends Subsystem {
  private static EndEffector m_instance;

  private PeriodicIO m_periodicIO;

  SparkMax m_leftMotor;
  SparkMax m_rightMotor;

  private LaserCanHandler m_laserCan;

  public static EndEffector getInstance() {
    if (m_instance == null) {
      m_instance = new EndEffector();
    }
    return m_instance;
  }

  private EndEffector() {
    super("EndEffector");

    m_periodicIO = new PeriodicIO();

    m_leftMotor = new SparkMax(RobotConstants.robotConfig.EndEffector.k_leftMotorId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(RobotConstants.robotConfig.EndEffector.k_rightMotorId, MotorType.kBrushless);

    // m_laserCan = LaserCanHandler.getInstance();

    SparkBaseConfig endEffectorConfig = new SparkFlexConfig().idleMode(IdleMode.kCoast);

    m_rightMotor.configure(
        endEffectorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_leftMotor.configure(
        endEffectorConfig.inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private static class PeriodicIO {
    EndEffectorState state = EndEffectorState.OFF;
  }

  public enum EndEffectorState {
    OFF,
    INDEX,
    REVERSE,
    SCORE_BRANCHES,
    SCORE_TROUGH
  }

  public void setState(EndEffectorState state) {
    m_periodicIO.state = state;
  }

  private void off() {
    setState(EndEffectorState.OFF);
  }

  private void index() {
    setState(EndEffectorState.INDEX);
  }

  private void reverse() {
    setState(EndEffectorState.REVERSE);
  }

  private void branches() {
    setState(EndEffectorState.SCORE_BRANCHES);
  }

  private void trough() {
    setState(EndEffectorState.SCORE_TROUGH);
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void periodic() {
    // checkAutoTasks();
  }

  @Override
  public void writePeriodicOutputs() {
    double[] speeds = getIntakeSpeeds();

    if (Arm.getInstance().getArmState() == ArmState.EXTEND) {
      speeds[0] *= -1;
      speeds[1] *= -1;
    }

    m_leftMotor.set(speeds[0]);
    m_rightMotor.set(speeds[1]);
  }

  @AutoLogOutput(key = "EndEffector/State")
  public EndEffectorState getEndEffectorState() {
    return m_periodicIO.state;
  }

  @AutoLogOutput(key = "EndEffector/Position/Target")
  private double[] getIntakeSpeeds() {
    switch (m_periodicIO.state) {
      case OFF:
        return RobotConstants.robotConfig.EndEffector.k_stopSpeeds;
      case INDEX:
        return RobotConstants.robotConfig.EndEffector.k_indexSpeeds;
      case REVERSE:
        return RobotConstants.robotConfig.EndEffector.k_reverseSpeeds;
      case SCORE_BRANCHES:
        return RobotConstants.robotConfig.EndEffector.k_branchSpeeds;
      case SCORE_TROUGH:
        return RobotConstants.robotConfig.EndEffector.k_troughSpeeds;
      default:
        return RobotConstants.robotConfig.EndEffector.k_stopSpeeds;
    }
  }

  @Override
  public void stop() {
    // we might want more here later, but this is enough for testing
    off();
  }

  private void checkAutoTasks() {
    switch (m_periodicIO.state) {
      case INDEX:
        // if (!m_laserCan.getEntranceSeesCoral()) {
        off();
        // }
        break;
      case OFF:
        // if (!(m_laserCan.getEntranceSeesCoral() || m_laserCan.getIndexSeesCoral())
        // && m_laserCan.getExitSeesCoral()) {
        // reverse();
        // } else if (m_laserCan.getEntranceSeesCoral()) {
        // index();
        // }
        break;
      case REVERSE:
        // if (m_laserCan.getIndexSeesCoral()) {
        off();
        // }
        break;
      case SCORE_BRANCHES:
        // if (!m_laserCan.getExitSeesCoral()) {
        // off();
        // } else {
        branches();
        // }
        break;
      case SCORE_TROUGH:
        // if (!m_laserCan.getExitSeesCoral()) {
        // off();
        // } else {
        trough();
        // }
        break;
      default:
        break;
    }
  }
}
