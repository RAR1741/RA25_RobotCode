package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.RARSparkMax;

public class Arm extends Subsystem {
  private static Arm m_arm = null;

  private PeriodicIO m_periodicIO;

  private RARSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkClosedLoopController m_pidController;

  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_currentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_goalState = new TrapezoidProfile.State();
  private double m_previousUpdateTime = Timer.getFPGATimestamp();

  private Arm() {
    super("Arm");

    m_periodicIO = new PeriodicIO();

    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.closedLoop
        .pid(RobotConstants.robotConfig.Arm.k_P, RobotConstants.robotConfig.Arm.k_I, RobotConstants.robotConfig.Arm.k_D)
        .iZone(RobotConstants.robotConfig.Arm.k_IZone)
        .minOutput(RobotConstants.robotConfig.Arm.k_minOutput)
        .maxOutput(RobotConstants.robotConfig.Arm.k_maxOutput);

    armConfig.smartCurrentLimit(RobotConstants.robotConfig.Arm.k_maxCurrent);

    armConfig.idleMode(IdleMode.kBrake);

    m_motor = new RARSparkMax(RobotConstants.robotConfig.Arm.k_motorId, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getClosedLoopController();
    m_motor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            RobotConstants.robotConfig.Arm.k_maxVelocity,
            RobotConstants.robotConfig.Arm.k_maxAcceleration));
  }

  private static class PeriodicIO {
    double arm_target = 0.0;
    double arm_power = 0.0;

    boolean is_arm_pos_control = false;
  }

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
    }
    return m_arm;
  }

  public void setAngleTarget(double angle) {
    m_periodicIO.is_arm_pos_control = true;
    m_periodicIO.arm_target = angle;
  }

  public void setArmPower(double power) {
    m_periodicIO.is_arm_pos_control = false;
    m_periodicIO.arm_power = power;
  }

  @Override
  public void reset() {
    m_encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    if (m_periodicIO.is_arm_pos_control) {
      double currentTime = Timer.getFPGATimestamp();
      double deltaTime = currentTime - m_previousUpdateTime;

      m_previousUpdateTime = currentTime;

      // Update goal
      m_goalState.position = m_periodicIO.arm_target;

      // Calculate new state
      m_currentState = m_profile.calculate(deltaTime, m_currentState, m_goalState);
    } else {
      m_currentState.position = m_encoder.getPosition();
      m_currentState.velocity = 0;
    }
  }

  @Override
  public void writePeriodicOutputs() {
    if (m_periodicIO.is_arm_pos_control) {
      // Set PID controller to new state
      m_pidController.setReference(
          m_currentState.position,
          SparkBase.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          RobotConstants.robotConfig.Arm.k_FF,
          ArbFFUnits.kVoltage);
    } else {
      m_motor.set(m_periodicIO.arm_power);
    }
  }

  @Override
  public void stop() {
    m_periodicIO.is_arm_pos_control = false;
    m_periodicIO.arm_power = 0.0;

    m_motor.set(0.0);
  }

}
