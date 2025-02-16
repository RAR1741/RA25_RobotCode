package frc.robot.subsystems.intakes;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import frc.robot.constants.RobotConstants;

public class Intake {
  private final String m_intakeName;
  
  private final SparkMax m_pivotMotor;
  private final SparkFlex m_rollerMotor;
  private final PeriodicIO m_periodicIO;
  private final SparkClosedLoopController m_pivotPIDController;
  private final SparkClosedLoopController m_rollerPIDController;
  private final ArmFeedforward m_pivotFeedforward;

  //TODO Use SmartMotion or MAXMotion
  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_currentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_goalState = new TrapezoidProfile.State();
  private double m_previousUpdateTime = Timer.getFPGATimestamp();

  private static class PeriodicIO {
    IntakeState desiredIntakeState = IntakeState.NONE;
  }

  Intake(String intakeName, int pivotMotorId, int rollerMotorId, boolean isInverted) {
    m_intakeName = intakeName;
    AutoLogOutputManager.addObject(this);

    m_pivotFeedforward = new ArmFeedforward(
        RobotConstants.robotConfig.Arm.k_FFS,
        RobotConstants.robotConfig.Arm.k_FFG,
        RobotConstants.robotConfig.Arm.k_FFV,
        RobotConstants.robotConfig.Arm.k_FFA);

    m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            RobotConstants.robotConfig.Intake.k_maxVelocity,
            RobotConstants.robotConfig.Intake.k_maxAcceleration));

    m_pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
    m_rollerMotor = new SparkFlex(rollerMotorId, MotorType.kBrushless);
    m_pivotPIDController = m_pivotMotor.getClosedLoopController();
    m_rollerPIDController = m_rollerMotor.getClosedLoopController();

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    SparkFlexConfig rollerConfig = new SparkFlexConfig();

    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(true);

    //TODO Due to how REV handles their rollovers, we can't do this anymore. Figure out a solution?
    pivotConfig.absoluteEncoder.positionConversionFactor(1.0); //360.0 // stinky
    // if (pivotMotorId == RobotConstants.robotConfig.Intake.k_pivotMotorIdLeft) {
    //   pivotConfig.absoluteEncoder.zeroOffset(RobotConstants.robotConfig.Intake.k_leftPivotOffset);
    // } else if (pivotMotorId == RobotConstants.robotConfig.Intake.k_pivotMotorIdRight) {
    //   pivotConfig.absoluteEncoder.zeroOffset(RobotConstants.robotConfig.Intake.k_rightPivotOffset);
    // }

    pivotConfig.inverted(isInverted);
    pivotConfig.absoluteEncoder.inverted(isInverted);
    rollerConfig.inverted(isInverted);

    pivotConfig.closedLoop.pid(
      RobotConstants.robotConfig.Intake.k_pivotMotorP,
      RobotConstants.robotConfig.Intake.k_pivotMotorI,
      RobotConstants.robotConfig.Intake.k_pivotMotorD);

    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfig.idleMode(IdleMode.kCoast);

    rollerConfig.closedLoop
      .pidf(
        RobotConstants.robotConfig.Intake.k_rollerMotorP,
        RobotConstants.robotConfig.Intake.k_rollerMotorI,
        RobotConstants.robotConfig.Intake.k_rollerMotorD,
        RobotConstants.robotConfig.Intake.k_rollerMotorFF
      );

    m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_periodicIO = new PeriodicIO();
  }

  public void setIntakeState(IntakeState target) {
    m_periodicIO.desiredIntakeState = target;
  }

  public void periodic() {
  }

  public void writePeriodicOutputs() {
    double currentTime = Timer.getFPGATimestamp();
    double deltaTime = currentTime - m_previousUpdateTime;

    m_previousUpdateTime = currentTime;

    // Update goal
    m_goalState.position = getTargetPivotAngle();

    // Calculate new state
    m_currentState = m_profile.calculate(deltaTime, m_currentState, m_goalState);

    double ff = m_pivotFeedforward.calculate(getPivotReferenceToHorizontal(), m_currentState.velocity);

    // Set PID controller to new state
    m_pivotPIDController.setReference(
        m_currentState.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ff,
        ArbFFUnits.kVoltage);
        
    m_rollerPIDController.setReference(getDesiredRollerSpeed(), ControlType.kVelocity);
  }

  public void stop() {
    m_periodicIO.desiredIntakeState = IntakeState.NONE;
    m_pivotPIDController.setReference(0.0, ControlType.kVoltage);
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/IntakeState")
  public String getDesiredIntakeState() {
    return m_periodicIO.desiredIntakeState.toString();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/RollerSpeed")
  public double getDesiredRollerSpeed() {
    switch(m_periodicIO.desiredIntakeState) {
      case NONE -> {
        return 0.0;
      }
      case INTAKE -> {
        return RobotConstants.robotConfig.Intake.k_maxIntakeSpeed;
      }
      case EJECT -> {
        return -RobotConstants.robotConfig.Intake.k_maxIntakeSpeed;
      }
      default -> {
        return 0.0;
      }
    }
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/PivotAngleFromTarget")
  public double getTargetPivotAngle() {
    switch(m_periodicIO.desiredIntakeState) {
      case NONE -> {
        return RobotConstants.robotConfig.Intake.k_stowAngle;
      }
      case INTAKE -> {
        return RobotConstants.robotConfig.Intake.k_groundAngle;
      }
      case EJECT -> {
        return RobotConstants.robotConfig.Intake.k_ejectAngle;
      }
      default -> {
        return RobotConstants.robotConfig.Intake.k_stowAngle;
      }
    }
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Current/PivotAngle")
  public double getPivotAngle() {
    return m_pivotMotor.getAbsoluteEncoder().getPosition();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Current/RollerSpeed")
  public double getRollerSpeed() {
    return m_rollerMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Current/PivotReferenceToHorizontal")
  public double getPivotReferenceToHorizontal() {
    if (m_intakeName.equalsIgnoreCase("Left")) {
      return (getPivotAngle() - RobotConstants.robotConfig.Intake.k_ejectAngle) * (2.0 * Math.PI);
    } else if (m_intakeName.equalsIgnoreCase("Right")) {
      return (getPivotAngle() - RobotConstants.robotConfig.Intake.k_ejectAngle) * (2.0 * Math.PI);
    }

    return 0.0;
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT
  }
}