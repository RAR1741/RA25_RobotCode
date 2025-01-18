package frc.robot.subsystems.intakes;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.wrappers.RARSparkMax;
import frc.robot.wrappers.REVThroughBoreEncoder;

//* dear all programmers seeing this, this is my first year as a SPEC member, expect this file of java code to be a dumpster file
//* i am sorry

public class Intakes extends Subsystem {
  private static Intakes m_instance;

  //~ pivots, for joints of forklift
  private RARSparkMax m_pivotMotorLeft;
  private RARSparkMax m_pivotMotorRight;

  //~ intakes, for mouth of robot
  private RARSparkMax m_intakeMotorLeft;
  private RARSparkMax m_intakeMotorRight;

  //~ PID's
  private final ProfiledPIDController m_pivotMotorPIDRight;
  private final ProfiledPIDController m_pivotMotorPIDLeft;

  //~ Feeders
  private final ArmFeedforward m_pivotFeedForwardRight;
  private final ArmFeedforward m_pivotFeedForwardLeft;

  private final double k_pivotThreshold = 3.0;
  private final double k_intakeSpeedThreshold = 0.1;

  private final REVThroughBoreEncoder m_pivotAbsEncoderRight = new REVThroughBoreEncoder(RobotConstants.robotConfig.Intake.k_pivotEncoderIdRight);
  private final REVThroughBoreEncoder m_pivotAbsEncoderLeft = new REVThroughBoreEncoder(RobotConstants.robotConfig.Intake.k_pivotEncoderIdLeft);

  private Intakes() {
    super("Intakes");
  }

  public static Intakes getInstance() {
    if(m_instance == null) {
      m_instance = new Intakes();
    }
    return m_instance;
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'writePeriodicOutputs'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

}
