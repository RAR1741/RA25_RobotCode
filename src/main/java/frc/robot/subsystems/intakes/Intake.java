package frc.robot.subsystems.intakes;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import frc.robot.constants.RobotConstants;

public class Intake {
  private final String m_intakeName;
  
  private final SparkMax m_pivotMotor;
  private final SparkFlex m_intakeMotor;
  private final PeriodicIO m_periodicIO;
  private final SparkClosedLoopController m_pivotPIDController;

  private static class PeriodicIO {
    IntakeState desiredIntakeState = IntakeState.NONE;
    IntakePivotTarget desiredPivotTarget = IntakePivotTarget.NONE;
    double intakeSpeed = 0.0;
  }

  Intake(String intakeName, int pivotMotorId, int intakeMotorId, boolean isInverted) {
    m_intakeName = intakeName;
    AutoLogOutputManager.addObject(this);

    m_pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
    m_intakeMotor = new SparkFlex(intakeMotorId, MotorType.kBrushless);
    m_pivotPIDController = m_pivotMotor.getClosedLoopController();

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    SparkFlexConfig intakeConfig = new SparkFlexConfig();

    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(true);

    //TODO: Due to how REV handles their rollovers, we can't do this anymore. Figure out a solution?
    pivotConfig.absoluteEncoder.positionConversionFactor(1.0); //360.0 // stinky
    // if (pivotMotorId == RobotConstants.robotConfig.Intake.k_pivotMotorIdLeft) {
    //   pivotConfig.absoluteEncoder.zeroOffset(RobotConstants.robotConfig.Intake.k_leftPivotOffset);
    // } else if (pivotMotorId == RobotConstants.robotConfig.Intake.k_pivotMotorIdRight) {
    //   pivotConfig.absoluteEncoder.zeroOffset(RobotConstants.robotConfig.Intake.k_rightPivotOffset);
    // }

    pivotConfig.inverted(isInverted);
    pivotConfig.absoluteEncoder.inverted(isInverted);
    intakeConfig.inverted(isInverted);

    pivotConfig.closedLoop.pidf(
      RobotConstants.robotConfig.Intake.k_pivotMotorP,
      RobotConstants.robotConfig.Intake.k_pivotMotorI,
      RobotConstants.robotConfig.Intake.k_pivotMotorD,
      RobotConstants.robotConfig.Intake.k_pivotMotorFF);

    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeConfig.idleMode(IdleMode.kCoast);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_periodicIO = new PeriodicIO();
  }

  public void setPivotTarget(IntakePivotTarget target) {
    m_periodicIO.desiredPivotTarget = target;
  }

  public void setIntakeState(IntakeState target) {
    m_periodicIO.desiredIntakeState = target;
  }

  public void periodic() {
    m_periodicIO.intakeSpeed = getDesiredIntakeSpeed();
  }

  public void writePeriodicOutputs() {
    m_pivotPIDController.setReference(getTargetPivotAngle(), ControlType.kPosition);
    m_intakeMotor.set(m_periodicIO.intakeSpeed);
  }

  public void stop() {
    m_periodicIO.intakeSpeed = 0.0;
    m_periodicIO.desiredIntakeState = IntakeState.NONE;
    m_periodicIO.desiredPivotTarget = IntakePivotTarget.NONE;
    m_pivotPIDController.setReference(0.0, ControlType.kVoltage);
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/IntakeState")
  public String getDesiredIntakeState() {
    return m_periodicIO.desiredIntakeState.toString();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/PivotTarget")
  public String getDesiredPivotTarget() {
    return m_periodicIO.desiredPivotTarget.toString();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/IntakeSpeed")
  public double getDesiredIntakeSpeed() {
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
    switch(m_periodicIO.desiredPivotTarget) {
      case NONE -> {
        return 0.0;
      }
      case GROUND -> {
        return RobotConstants.robotConfig.Intake.k_groundAngle;
      }
      case EJECT -> {
        return RobotConstants.robotConfig.Intake.k_ejectAngle;
      }
      case STOW -> {
        return RobotConstants.robotConfig.Intake.k_stowAngle;
      }
      default -> {
        return 0.0;
      }
    }
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Current/PivotAngle")
  public double getPivotAngle() {
    return m_pivotMotor.getAbsoluteEncoder().getPosition();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Current/IntakeSpeed")
  public double getIntakeSpeed() {
    return m_intakeMotor.getEncoder().getVelocity();
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

  public enum IntakePivotTarget {
    NONE,
    GROUND,
    EJECT,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT
  }
}