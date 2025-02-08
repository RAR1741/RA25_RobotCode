package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Helpers;
import frc.robot.constants.RobotConstants;

public class Arm extends Subsystem {
  private static Arm m_arm = null;

  private PeriodicIO m_periodicIO;

  private SparkMax m_motor;
  private SparkAbsoluteEncoder m_encoder;
  private SparkClosedLoopController m_pidController;

  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_currentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_goalState = new TrapezoidProfile.State();
  private double m_previousUpdateTime = Timer.getFPGATimestamp();

  private Arm() {
    super("Arm");

    m_periodicIO = new PeriodicIO();

    m_motor = new SparkMax(RobotConstants.robotConstants.Arm.k_motorId, MotorType.kBrushless);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_pidController = m_motor.getClosedLoopController();

    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.closedLoop
        .pid(RobotConstants.robotConstants.Arm.k_P,
            RobotConstants.robotConstants.Arm.k_I,
            RobotConstants.robotConstants.Arm.k_D)
        .iZone(RobotConstants.robotConstants.Arm.k_IZone);

    armConfig.absoluteEncoder
        .positionConversionFactor(360.0) // [0, 1) to [0, 360)
        .zeroOffset(0.0);

    armConfig.smartCurrentLimit(RobotConstants.robotConstants.Arm.k_maxCurrent);

    m_motor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            RobotConstants.robotConstants.Arm.k_maxVelocity,
            RobotConstants.robotConstants.Arm.k_maxAcceleration));
  }

  private static class PeriodicIO {
    ArmState arm_state = ArmState.STOW;
  }

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
    }
    return m_arm;
  }

  public void setArmState(ArmState state) {
    m_periodicIO.arm_state = state;
  }

  @Override
  public void reset() {
  }

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    double currentTime = Timer.getFPGATimestamp();
    double deltaTime = currentTime - m_previousUpdateTime;

    m_previousUpdateTime = currentTime;

    // Update goal
    m_goalState.position = getArmTarget();

    // Calculate new state
    m_currentState = m_profile.calculate(deltaTime, m_currentState, m_goalState);

    // Set PID controller to new state
    m_pidController.setReference(
        m_currentState.position,
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        RobotConstants.robotConstants.Arm.k_FF,
        ArbFFUnits.kVoltage);
  }

  public enum ArmState {
    STOW, SCORE
  }

  @Override
  public void stop() {
    m_pidController.setReference(0.0, SparkBase.ControlType.kVoltage);
  }

  @AutoLogOutput(key = "Arm/Position/Current")
  public double getArmPosition() {
    return m_encoder.getPosition();
  }

  @AutoLogOutput(key = "Arm/RelEncoder/Position")
  public double getArmRelativePosition() {
    return m_motor.getEncoder().getPosition();
  }

  @AutoLogOutput(key = "Arm/Position/Target")
  public double getArmTarget() {
    switch (m_periodicIO.arm_state) {
      case STOW -> {
        return RobotConstants.robotConstants.Arm.k_stowAngle;
      }
      case SCORE -> {
        return RobotConstants.robotConstants.Arm.k_L4Angle;
      }
      default -> {
        return RobotConstants.robotConstants.Arm.k_stowAngle;
      }
    }
  }

  @AutoLogOutput(key = "Arm/Position/Setpoint")
  private double getElevatorPosSetpoint() {
    return m_currentState.position;
  }

  @AutoLogOutput(key = "Arm/Voltage")
  public double getArmVoltage() {
    return Helpers.getVoltage(m_motor);
  }

  @AutoLogOutput(key = "Arm/CurrentAmps")
  public double getArmCurrent() {
    return m_motor.getOutputCurrent();
  }
}
