package frc.robot.subsystems.intakes;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Helpers;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Intake {
  private final String m_intakeName;

  private final PeriodicIO m_periodicIO;

  private final SparkMax m_pivotMotor;
  private final SparkFlex m_rollerMotor;

  private final SparkClosedLoopController m_pivotPIDController;
  private final SparkClosedLoopController m_rollerPIDController;

  private final ArmFeedforward m_pivotFeedforward;

  // TODO: Use SmartMotion or MAXMotion
  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_currentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_goalState = new TrapezoidProfile.State();
  private double m_previousUpdateTime = Timer.getFPGATimestamp();

  private static class PeriodicIO {
    IntakeState desiredIntakeState = IntakeState.STOW;
  }

  Intake(String intakeName, int pivotMotorId, int rollerMotorId, boolean isInverted) {
    m_intakeName = intakeName;
    AutoLogOutputManager.addObject(this);

    m_pivotFeedforward = new ArmFeedforward(
        RobotConstants.robotConfig.Intake.k_pivotMotorKS,
        RobotConstants.robotConfig.Intake.k_pivotMotorKG,
        RobotConstants.robotConfig.Intake.k_pivotMotorKV,
        RobotConstants.robotConfig.Intake.k_pivotMotorKA);

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

    // TODO Due to how REV handles their rollovers, we can't do this anymore. Figure
    // out a solution?
    pivotConfig.absoluteEncoder.positionConversionFactor(1.0); // 360.0 // stinky
    pivotConfig.encoder.positionConversionFactor(1.0);
    // if (pivotMotorId == RobotConstants.robotConfig.Intake.k_pivotMotorIdLeft) {
    // pivotConfig.absoluteEncoder.zeroOffset(RobotConstants.robotConfig.Intake.k_leftPivotOffset);
    // } else if (pivotMotorId ==
    // RobotConstants.robotConfig.Intake.k_pivotMotorIdRight) {
    // pivotConfig.absoluteEncoder.zeroOffset(RobotConstants.robotConfig.Intake.k_rightPivotOffset);
    // }

    // rollerConfig.smartCurrentLimit(RobotConstants.robotConfig.Intake.k_rollerCurrentLimit);
    pivotConfig.smartCurrentLimit(RobotConstants.robotConfig.Intake.k_pivotCurrentLimit);

    pivotConfig.inverted(true);
    pivotConfig.absoluteEncoder.inverted(true);
    rollerConfig.inverted(isInverted);

    pivotConfig.closedLoop.pid(
        RobotConstants.robotConfig.Intake.k_pivotMotorP,
        RobotConstants.robotConfig.Intake.k_pivotMotorI,
        RobotConstants.robotConfig.Intake.k_pivotMotorD);

    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfig.idleMode(IdleMode.kCoast);

    rollerConfig.encoder.velocityConversionFactor(RobotConstants.robotConfig.Intake.k_rollerGearRatio);

    rollerConfig.closedLoop
        .pidf(
            RobotConstants.robotConfig.Intake.k_rollerMotorP,
            RobotConstants.robotConfig.Intake.k_rollerMotorI,
            RobotConstants.robotConfig.Intake.k_rollerMotorD,
            RobotConstants.robotConfig.Intake.k_rollerMotorFF);

    m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_periodicIO = new PeriodicIO();
  }

  public void setIntakeState(IntakeState target) {
    m_periodicIO.desiredIntakeState = target;
  }

  int counter = 0;

  public void periodic() {
    getRollerVolts();
    if (getRollerSpeed() <= RobotConstants.robotConfig.Intake.k_lowestRollerSpeed
        && getPivotAngle() > getTargetPivotAngle(IntakeState.EJECT) && getDesiredIntakeState() != "STOW") {
      counter++;
      if (counter > RobotConstants.robotConfig.Intake.k_debounceLimit) {
        setIntakeState(IntakeState.STUCK);
        counter = 0;
      }
    } else {
      counter = 0;
    }

    Logger.recordOutput("Intakes/" + m_intakeName + "/DebounceCounter", counter);
    Logger.recordOutput("Intakes/" + m_intakeName + "/IsRollerBelowLowest",
        getRollerSpeed() <= RobotConstants.robotConfig.Intake.k_lowestRollerSpeed);
    Logger.recordOutput("Intakes/" + m_intakeName + "/IsPivotAboveHorz",
        getPivotAngle() > getTargetPivotAngle(IntakeState.EJECT));
  }

  public void reset() {
    m_currentState.position = getPivotAngle();
    m_currentState.velocity = 0.0;
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

    Logger.recordOutput("Intakes/" + m_intakeName + "/Feedforward", ff);

    // Set PID controller to new state
    m_pivotPIDController.setReference(
        m_currentState.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ff,
        ArbFFUnits.kVoltage);

        
    if (getDesiredRollerSpeed() != 0.0) {
      if(m_periodicIO.desiredIntakeState == IntakeState.ALGAE || m_periodicIO.desiredIntakeState == IntakeState.SCORE_ALGAE) {
        m_rollerMotor.setVoltage(getDesiredRollerSpeed());
      } else {
        m_rollerPIDController.setReference(getDesiredRollerSpeed(), ControlType.kVelocity);
      }
    } else {
      if (isAtState()) {
        m_rollerPIDController.setReference(0.0, ControlType.kVoltage);
      } else {
        m_rollerPIDController.setReference(RobotConstants.robotConfig.Intake.k_stowingIntakeSpeed, ControlType.kVelocity);
      }
    }
  }

  public void stop() {
    m_periodicIO.desiredIntakeState = IntakeState.STOW;
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/IsAtState")
  public boolean isAtState() {
    if (RobotBase.isSimulation()) {
      return true;
    }

    double currentPos = getPivotAngle();
    double targetPos = getTargetPivotAngle();
    double allowedPivotError = RobotConstants.robotConfig.Intake.k_allowedPivotError;

    return Math.abs(currentPos - targetPos) <= allowedPivotError;
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/IntakeState")
  public String getDesiredIntakeState() {
    return m_periodicIO.desiredIntakeState.toString();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/RollerSpeed")
  public double getDesiredRollerSpeed() {
    if (!(Elevator.getInstance().getIsAtState() && Elevator.getInstance().getTargetState() == ElevatorState.STOW)) {
      return 0.0;
    }

    switch (m_periodicIO.desiredIntakeState) {
      case STOW -> {
        return 0.0;
      }
      case INTAKE -> {
        return RobotConstants.robotConfig.Intake.k_maxIntakeSpeed;
      }
      case STUCK -> {
        return RobotConstants.robotConfig.Intake.k_maxIntakeSpeed;
      }
      case EJECT -> {
        return -RobotConstants.robotConfig.Intake.k_maxIntakeSpeed;
      }
      case END_EJECT -> {
        return 0.0;
      }
      case ALGAE -> {
        return -3.5; //Volts
      }
      case SCORE_ALGAE -> {
        return 3.5; // Volts
      }
      default -> {
        return 0.0;
      }
    }
  }

  public double getTargetPivotAngle(IntakeState state) {
    switch (state) {
      case STOW -> {
        if (m_intakeName.equalsIgnoreCase("Left")) {
          return RobotConstants.robotConfig.Intake.Left.k_stowPosition;
        } else {
          return RobotConstants.robotConfig.Intake.Right.k_stowPosition;
        }
      }
      case INTAKE -> {
        if (m_intakeName.equalsIgnoreCase("Left")) {
          return RobotConstants.robotConfig.Intake.Left.k_groundPosition;
        } else {
          return RobotConstants.robotConfig.Intake.Right.k_groundPosition;
        }
      }
      case EJECT -> {
        return getPivotAngle();
      }
      case END_EJECT -> {
        return getPivotAngle();
      }
      case ALGAE -> {
        if (m_intakeName.equalsIgnoreCase("Left")) {
          return RobotConstants.robotConfig.Intake.Left.k_algaePosition;
        } else {
          return RobotConstants.robotConfig.Intake.Right.k_algaePosition;
        }
      }
      case SCORE_ALGAE -> {
        if (m_intakeName.equalsIgnoreCase("Left")) {
          return RobotConstants.robotConfig.Intake.Left.k_algaePosition;
        } else {
          return RobotConstants.robotConfig.Intake.Right.k_algaePosition;
        }
      }
      case STUCK -> {
        if (m_intakeName.equalsIgnoreCase("Left")) {
          return RobotConstants.robotConfig.Intake.Left.k_stuckPosition;
        } else {
          return RobotConstants.robotConfig.Intake.Right.k_stuckPosition;
        }
      }
      default -> {
        if (m_intakeName.equalsIgnoreCase("Left")) {
          return RobotConstants.robotConfig.Intake.Left.k_stowPosition;
        } else {
          return RobotConstants.robotConfig.Intake.Right.k_stowPosition;
        }
      }
    }
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/PivotAngleFromTarget")
  public double getTargetPivotAngle() {
    return getTargetPivotAngle(m_periodicIO.desiredIntakeState);
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
      return (getPivotAngle() - RobotConstants.robotConfig.Intake.Left.k_horizontalPosition) * (2.0 * Math.PI);
    } else if (m_intakeName.equalsIgnoreCase("Right")) {
      return (getPivotAngle() - RobotConstants.robotConfig.Intake.Right.k_horizontalPosition) * (2.0 * Math.PI);
    }

    return 0.0;
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/Desired/PivotSetpoint")
  private double getArmSetpoint() {
    return m_currentState.position;
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/PivotVoltage")
  public double getPivotVoltage() {
    return Helpers.getVoltage(m_pivotMotor);
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/PivotAmps")
  public double getPivotCurrentAmps() {
    return m_pivotMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/RollerAmps")
  public double getRollerCurrentAmps() {
    return m_rollerMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "Intakes/{m_intakeName}/RollerVolts")
  public double getRollerVolts() {
    return Helpers.getVoltage(m_rollerMotor);
  }

  public enum IntakeState {
    STOW,
    INTAKE,
    STUCK,
    EJECT,
    END_EJECT,
    ALGAE,
    SCORE_ALGAE
  }
}
