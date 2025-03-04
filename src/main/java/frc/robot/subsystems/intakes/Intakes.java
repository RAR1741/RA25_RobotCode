package frc.robot.subsystems.intakes;

import java.util.ArrayList;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.intakes.Intake.IntakeState;

public class Intakes extends Subsystem {
  private static Intakes m_instance;
  private final ArrayList<Intake> m_intakes;

  private Intakes() {
    super("Intakes");

    m_intakes = new ArrayList<>();

    m_intakes.add(new Intake("Left",
        RobotConstants.robotConfig.Intake.k_pivotMotorIdLeft,
        RobotConstants.robotConfig.Intake.k_rollerMotorIdLeft, false));

    m_intakes.add(new Intake("Right",
        RobotConstants.robotConfig.Intake.k_pivotMotorIdRight,
        RobotConstants.robotConfig.Intake.k_rollerMotorIdRight, true));
  }

  public static Intakes getInstance() {
    if(m_instance == null) {
      m_instance = new Intakes();
    }
    return m_instance;
  }

  public Intake getIntake(IntakeVariant intakeVariant) {
    return m_intakes.get(intakeVariant.ordinal());
  }

  public boolean isAtState(IntakeVariant intakeVariant) {
    return getIntake(intakeVariant).isAtState();
  }

  public void setIntakeState(IntakeVariant intakeVariant, IntakeState intakeState) {
    getIntake(intakeVariant).setIntakeState(intakeState);
  }

  @Override
  public void reset() {
    m_intakes.forEach(intake -> intake.reset());
  }

  @Override
  public void periodic() {
    m_intakes.forEach(intake -> intake.periodic());
  }

  @Override
  public void writePeriodicOutputs() {
    m_intakes.forEach(intake -> intake.writePeriodicOutputs());
  }

  @Override
  public void stop() {
    m_intakes.forEach(intake -> intake.stop());
  }

  public enum IntakeVariant {
    LEFT, RIGHT
  }
}
