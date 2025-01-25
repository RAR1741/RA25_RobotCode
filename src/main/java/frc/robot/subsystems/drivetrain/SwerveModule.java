package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Helpers;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.SignalManager;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turnMotor;
  
  private final CANcoder m_turningCANcoder;

  private final PeriodicIO m_periodicIO;

  private final SignalManager m_signalManager;

  private double m_turningOffset;

  @SuppressWarnings("unused")
  private final String m_moduleName;

  private static class PeriodicIO {
    SwerveModuleState desiredState = new SwerveModuleState();
    // boolean shouldChangeState = false; // TODO: maybe add this back?
  }

  private boolean m_moduleDisabled = false;

  public SwerveModule(String moduleName, int driveMotorID, int turningMotorID, int turningCANcoderID, double turningOffset) {
    m_periodicIO = new PeriodicIO();
    m_signalManager = SignalManager.getInstance();

    m_moduleName = moduleName;
    m_turningOffset = turningOffset;

    // START DRIVE MOTOR INIT
    m_driveMotor = new TalonFX(driveMotorID, RobotConstants.robotConfig.SwerveDrive.k_canBus);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    m_driveMotor.setNeutralMode(NeutralModeValue.Coast);

    driveConfig.Feedback.SensorToMechanismRatio = RobotConstants.robotConfig.SwerveDrive.k_driveGearRatio;
    // driveConfig.Feedback.RotorToSensorRatio = 0.0f; // TODO: Add this when we have CANCoders

    // the sound of silence
    driveConfig.Audio.BeepOnBoot = false;
    driveConfig.Audio.BeepOnConfig = false;

    driveConfig.Slot0.kP = RobotConstants.robotConfig.SwerveDrive.Drive.k_P;
    driveConfig.Slot0.kI = RobotConstants.robotConfig.SwerveDrive.Drive.k_I;
    driveConfig.Slot0.kD = RobotConstants.robotConfig.SwerveDrive.Drive.k_D;

    driveConfig.Slot0.kS = RobotConstants.robotConfig.SwerveDrive.Drive.k_FFS;
    driveConfig.Slot0.kV = RobotConstants.robotConfig.SwerveDrive.Drive.k_FFV;
    driveConfig.Slot0.kA = RobotConstants.robotConfig.SwerveDrive.Drive.k_FFA;

    // driveConfig.CurrentLimits.StatorCurrentLimit = 15.0;
    // driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    m_driveMotor.getConfigurator().apply(driveConfig);
    // END DRIVE MOTOR INIT

    // START TURN MOTOR INIT
    m_turnMotor = new TalonFX(turningMotorID, RobotConstants.robotConfig.SwerveDrive.k_canBus);
    
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();

    m_turnMotor.setNeutralMode(NeutralModeValue.Brake);

    turnConfig.Audio.BeepOnBoot = false;
    turnConfig.Audio.BeepOnConfig = false;

    turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;// TODO: maybe come back to this later
    turnConfig.Feedback.RotorToSensorRatio = RobotConstants.robotConfig.SwerveDrive.k_turnGearRatio; // TODO: verify
                                                                                                         // this works
                                                                                                         // for both
                                                                                                         // position and
                                                                                                         // velocity

    turnConfig.Slot0.kP = RobotConstants.robotConfig.SwerveDrive.Turn.k_P;
    turnConfig.Slot0.kI = RobotConstants.robotConfig.SwerveDrive.Turn.k_I;
    turnConfig.Slot0.kD = RobotConstants.robotConfig.SwerveDrive.Turn.k_D;

    turnConfig.Slot0.kS = RobotConstants.robotConfig.SwerveDrive.Turn.k_S;
    turnConfig.Slot0.kV = RobotConstants.robotConfig.SwerveDrive.Turn.k_V;
    turnConfig.Slot0.kA = RobotConstants.robotConfig.SwerveDrive.Turn.k_A;

    turnConfig.MotionMagic.MotionMagicAcceleration = 1000;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100;

    // turnConfig.CurrentLimits.StatorCurrentLimit = 10.0;
    // turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    m_turningOffset = turningOffset;

    m_turningCANcoder = new CANcoder(turningCANcoderID, RobotConstants.robotConfig.SwerveDrive.k_canBus);
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    // TODO: check that this doesnt interfere with the inversion of the turn motor
    // output
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = m_turningOffset;

    m_turningCANcoder.getConfigurator().apply(canCoderConfig);
    turnConfig.Feedback.FeedbackRemoteSensorID = m_turningCANcoder.getDeviceID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    m_turnMotor.getConfigurator().apply(turnConfig);
    // END TURN MOTOR INIT

    BaseStatusSignal.setUpdateFrequencyForAll(250,
        m_driveMotor.getPosition(), m_driveMotor.getVelocity(), m_driveMotor.getAcceleration(), m_driveMotor.getMotorVoltage(),
        m_turnMotor.getPosition(), m_turnMotor.getVelocity(), m_turnMotor.getAcceleration(), m_turnMotor.getMotorVoltage());

    // register all signals with the SignalManager so that any downstream callers
    // get updated signals
    m_signalManager.register(
        m_driveMotor.getPosition(), m_driveMotor.getVelocity(), m_driveMotor.getAcceleration(), m_driveMotor.getMotorVoltage(),
        m_turnMotor.getPosition(), m_turnMotor.getVelocity(), m_turnMotor.getAcceleration(), m_turnMotor.getMotorVoltage());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getTurnPosition()));
  }

  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveMotor.getPosition().getValueAsDouble();

    return new SwerveModulePosition(drivePosition, Rotation2d.fromRadians(getTurnPosition()));
  }

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public TalonFX getTurnMotor() {
    return m_turnMotor;
  }

  /**
   * Returns an array of the required signals for odometry.
   * 
   * @return the required signals for odometry
   */
  public BaseStatusSignal[] getSignals() {
      return new BaseStatusSignal[] {
        m_driveMotor.getPosition(), m_driveMotor.getVelocity(), m_turnMotor.getPosition(), m_turnMotor.getVelocity() 
      };
  }

  public void resetDriveEncoder() {
    m_driveMotor.setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(Rotation2d.fromRadians(getTurnPosition()));

    desiredState.angle = new Rotation2d(Helpers.modRadians(desiredState.angle.getRadians()));
    // m_periodicIO.shouldChangeState =
    // !desiredState.equals(m_periodicIO.desiredState);

    m_periodicIO.desiredState = desiredState;
  }

  public SwerveModuleState getDesiredState() {
    return m_periodicIO.desiredState;
  }

  public void periodic() {
    double driveVelocity = Helpers.MPSToRPS(getDriveTargetVelocity(), RobotConstants.robotConfig.SwerveDrive.k_wheelCircumference);
    double turnPosition = Units.radiansToRotations(getTurnTargetAngleRadians());

    VelocityVoltage driveRequest = new VelocityVoltage(driveVelocity).withSlot(0);
    m_driveMotor.setControl(driveRequest);

    MotionMagicVoltage turnRequest = new MotionMagicVoltage(turnPosition).withSlot(0);
    turnRequest.EnableFOC = true;
    m_turnMotor.setControl(turnRequest);
  }

  // Logged
  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/targetAngleRads")
  private double getTurnTargetAngleRadians() {
    return m_periodicIO.desiredState.angle.getRadians();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/speedMPS")
  private double getDriveTargetVelocity() {
    return m_periodicIO.desiredState.speedMetersPerSecond;
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/VoltageV")
  public double getTurnMotorVoltage() {
    return m_turnMotor.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/CurrentA")
  public double getTurnMotorCurrent() {
    return m_turnMotor.getStatorCurrent().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/VoltageV")
  public double getDriveMotorVoltage() {
    return m_driveMotor.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/CurrentA")
  public double getDriveMotorCurrent() {
    return m_driveMotor.getTorqueCurrent().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/AbsoluteEncoder/AbsolutePosition")
  public double getTurnAbsEncoderPosition() {
    return m_turningCANcoder.getAbsolutePosition().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/TemperatureC")
  public double getDriveTemp() {
    return m_driveMotor.getDeviceTemp().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/TemperatureC")
  public double getTurnTemp() {
    return m_turnMotor.getDeviceTemp().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/VelocityMPS")
  public double getDriveVelocity() {
    return Helpers.RPSToMPS(
      m_driveMotor.getVelocity().getValueAsDouble(),
      RobotConstants.robotConfig.SwerveDrive.k_wheelCircumference);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/PositionRot")
  public double getDrivePositionRot() {
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/PositionMet")
  public double getDrivePositionMet() {
    return Helpers.RPSToMPS(
      m_driveMotor.getPosition().getValueAsDouble(),
      RobotConstants.robotConfig.SwerveDrive.k_wheelCircumference);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/PositionRads")
  public double getTurnPosition() {
    // return Helpers.modRadians(m_turningRelEncoder.getPosition());
    return Units.rotationsToRadians(m_turnMotor.getPosition().getValueAsDouble());
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/VelocityRPS")
  public double getTurnVelocity() {
    return m_turnMotor.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/ErrorDeg")
  public double getTurnError() {
    return getState().angle.minus(m_periodicIO.desiredState.angle).getDegrees();
  }

  // @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/errorRelToAbs")
  // public double getTurnErrorRelToAbs() {
  // return
  // getState().angle.minus(Rotation2d.fromRotations(getTurnAbsPosition())).getDegrees();
  // }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/ModuleDisabled")
  public boolean isModuleDisabled() {
    return m_moduleDisabled;
  }
}
