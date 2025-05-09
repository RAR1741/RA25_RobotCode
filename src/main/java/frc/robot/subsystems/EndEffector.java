package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Helpers;
import frc.robot.LaserCanHandler;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.leds.LEDs;

public class EndEffector extends Subsystem {
  private static EndEffector m_instance;

  private final PeriodicIO m_periodicIO;

  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;

  private final SparkClosedLoopController m_rightRollerPIDController;
  private final SparkClosedLoopController m_leftRollerPIDController;

  private LaserCanHandler m_laserCan;
  private Arm m_arm;
  private Elevator m_elevator;
  // private LEDs m_leds;

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

    m_rightRollerPIDController = m_rightMotor.getClosedLoopController();
    m_leftRollerPIDController = m_leftMotor.getClosedLoopController();

    m_laserCan = LaserCanHandler.getInstance();
    m_arm = Arm.getInstance();
    m_elevator = Elevator.getInstance();
    // m_leds = LEDs.getInstance();

    SparkBaseConfig rollerConfig = new SparkFlexConfig();

    rollerConfig
        .smartCurrentLimit(RobotConstants.robotConfig.EndEffector.k_maxCurrent)
        .idleMode(IdleMode.kBrake);

    rollerConfig.encoder.velocityConversionFactor(RobotConstants.robotConfig.EndEffector.k_rollerGearRatio);

    rollerConfig.closedLoop.pidf(
        RobotConstants.robotConfig.EndEffector.k_rollerP,
        RobotConstants.robotConfig.EndEffector.k_rollerI,
        RobotConstants.robotConfig.EndEffector.k_rollerD,
        RobotConstants.robotConfig.EndEffector.k_rollerFF);

    m_rightMotor.configure(
        rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_leftMotor.configure(
        rollerConfig.inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private static class PeriodicIO {
    EndEffectorState state = EndEffectorState.INDEXED;
    boolean shouldBeIndexingCoral = false;
  }

  public boolean getShouldBeIndexingCoral() {
    return m_periodicIO.shouldBeIndexingCoral;
  }

  public void setShouldBeIndexingCoral(boolean shouldBeIndexingCoral) {
    m_periodicIO.shouldBeIndexingCoral = shouldBeIndexingCoral;
  }

  public enum EndEffectorState {
    // Indexing
    OFF,
    FORWARD_INDEX_FAST,
    FORWARD_INDEX_SLOW,
    REVERSE_INDEX,
    INDEXED,

    // 💩
    REINDEX,
    L4_REVERSE,

    // Scoring
    SCORE_BRANCHES,
    SCORE_TROUGH,

    // Algae
    ALGAE_GRAB,
    ALGAE_SCORE
  }

  public void setState(EndEffectorState state) {
    m_periodicIO.state = state;

    // switch (state) {
    // case OFF -> {
    // m_leds.setAllColor(Color.kRed);
    // }
    // case FORWARD_INDEX_FAST -> {
    // m_leds.setAllColor(Color.kYellow);
    // }
    // case INDEXED -> {
    // m_leds.setAllColor(Color.kGreen);
    // }
    // }
  }

  private void off() {
    setState(EndEffectorState.OFF);
  }

  private void indexFast() {
    setState(EndEffectorState.FORWARD_INDEX_FAST);
  }

  private void branches() {
    setState(EndEffectorState.SCORE_BRANCHES);
  }

  private void trough() {
    setState(EndEffectorState.SCORE_TROUGH);
  }

  public void algaeGrab() {
    setState(EndEffectorState.ALGAE_GRAB);
  }

  public void algaeScore() {
    setState(EndEffectorState.ALGAE_SCORE);
  }

  @Override
  public void reset() {
  }

  @Override
  public void periodic() {
    if (isAlgae()) {
      if(m_laserCan.getEntranceSeesCoral()) {
        setState(EndEffectorState.FORWARD_INDEX_FAST);
      }
    } else {
      updateState();
    }
  }

  @Override
  public void writePeriodicOutputs() {
    double desiredSpeed = getDesiredRollerSpeed();

    if(isAlgae()) {
      m_rightMotor.set(desiredSpeed);
      m_leftMotor.set(desiredSpeed);
    } else {
      m_rightRollerPIDController.setReference(
          desiredSpeed,
          ControlType.kVelocity);
      m_leftRollerPIDController.setReference(
          desiredSpeed,
          ControlType.kVelocity);
    }
  }

  public boolean shouldDriveSlow() {
    return m_periodicIO.shouldBeIndexingCoral
        && (m_periodicIO.state == EndEffectorState.OFF || m_periodicIO.state == EndEffectorState.FORWARD_INDEX_FAST);
  }

  public boolean isAlgae() {
    EndEffectorState state = getEndEffectorState();
    return state == EndEffectorState.ALGAE_GRAB || state == EndEffectorState.ALGAE_SCORE;
  }

  @AutoLogOutput(key = "EndEffector/RightMotorVoltage")
  public double getRightMotorVoltage() {
    return Helpers.getVoltage(m_rightMotor);
  }

  @AutoLogOutput(key = "EndEffector/LeftMotorVoltage")
  public double getLeftMotorVoltage() {
    return Helpers.getVoltage(m_leftMotor);
  }

  @AutoLogOutput(key = "EndEffector/RightMotorCurrent")
  public double getRightMotorCurrent() {
    return m_rightMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "EndEffector/LeftMotorCurrent")
  public double getLeftMotorCurrent() {
    return m_leftMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "EndEffector/RightMotorVelocityRPM")
  public double getRightMotorVelocity() {
    return m_rightMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "EndEffector/LeftMotorVelocityRPM")
  public double getLeftMotorVelocity() {
    return m_leftMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "EndEffector/IsSafeToScore")
  public boolean isSafeToScore() {
    return m_periodicIO.state == EndEffectorState.INDEXED;
  }

  @AutoLogOutput(key = "EndEffector/State")
  public EndEffectorState getEndEffectorState() {
    return m_periodicIO.state;
  }

  @AutoLogOutput(key = "EndEffector/Position/Target")
  private double getDesiredRollerSpeed() {
    switch (m_periodicIO.state) {
      case OFF -> {
        return RobotConstants.robotConfig.EndEffector.k_stopSpeed;
      }

      case FORWARD_INDEX_SLOW -> {
        return RobotConstants.robotConfig.EndEffector.k_forwardIndexSlowSpeed;
      }

      case FORWARD_INDEX_FAST -> {
        return RobotConstants.robotConfig.EndEffector.k_forwardIndexFastSpeed;
      }

      case REVERSE_INDEX -> {
        return RobotConstants.robotConfig.EndEffector.k_reverseIndexSpeed;
      }

      case SCORE_BRANCHES -> {
        if (m_arm.getArmState() == ArmState.EXTEND || m_elevator.getTargetState() == ElevatorState.L4) {
          return RobotConstants.robotConfig.EndEffector.k_l4Speed;
        }

        return RobotConstants.robotConfig.EndEffector.k_branchSpeed;
      }

      case SCORE_TROUGH -> {
        return RobotConstants.robotConfig.EndEffector.k_troughSpeed;
      }

      case L4_REVERSE -> {
        return -RobotConstants.robotConfig.EndEffector.k_reverseIndexSpeed;
      }

      case REINDEX -> {
        return RobotConstants.robotConfig.EndEffector.k_reverseIndexSpeed;
      }

      case ALGAE_GRAB -> {
        return RobotConstants.robotConfig.EndEffector.k_algaeGrabPower;
      }

      case ALGAE_SCORE -> {
        return RobotConstants.robotConfig.EndEffector.k_algaeScorePower;
      }

      default -> {
        return RobotConstants.robotConfig.EndEffector.k_stopSpeed;
      }
    }
  }

  @Override
  public void stop() {
    off();
  }

  private void updateState() {
    if (!m_laserCan.getExitSeesCoral() && getEndEffectorState() != EndEffectorState.FORWARD_INDEX_FAST
        && getEndEffectorState() != EndEffectorState.OFF) {
      setState(EndEffectorState.OFF);
      return;
    }

    switch (m_periodicIO.state) {
      case FORWARD_INDEX_FAST -> {
        if (m_laserCan.getExitSeesCoral()) {
          setState(EndEffectorState.FORWARD_INDEX_SLOW);
        }
      }

      case FORWARD_INDEX_SLOW -> {
        if (!m_laserCan.getEntranceSeesCoral()) {
          // setState(EndEffectorState.INDEXED);
          setState(EndEffectorState.REVERSE_INDEX);
        }
      }

      case REVERSE_INDEX -> {
        if (m_laserCan.getEntranceSeesCoral()) {
          setState(EndEffectorState.INDEXED);
        }
      }

      case INDEXED -> {
        if (!m_laserCan.getExitSeesCoral()) {
          setState(EndEffectorState.OFF);
        }

        if (!m_laserCan.getMiddleSeesCoral()) {
          setState(EndEffectorState.REINDEX);
        }
      }

      case REINDEX -> {
        if (m_laserCan.getMiddleSeesCoral()) {
          setState(EndEffectorState.INDEXED);
        }
      }

      case OFF -> {
        if (m_laserCan.getEntranceSeesCoral()) {
          indexFast();
        }
      }

      case SCORE_BRANCHES -> {
        if (!m_laserCan.getExitSeesCoral()) {
          off();
        } else {
          branches();
        }
      }

      case SCORE_TROUGH -> {
        if (!m_laserCan.getExitSeesCoral()) {
          off();
        } else {
          trough();
        }
      }

      default -> {
      }
    }
  }
}
