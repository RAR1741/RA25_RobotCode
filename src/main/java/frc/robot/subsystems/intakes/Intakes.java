package frc.robot.subsystems.intakes;

import java.util.ArrayList;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.intakes.Intake.IntakePivotTarget;
import frc.robot.subsystems.intakes.Intake.IntakeState;

public class Intakes extends Subsystem {
  private static Intakes m_instance;
  private final ArrayList<Intake> m_intakes;

  private Intakes() {
    super("Intakes");

    m_intakes = new ArrayList<>();

    m_intakes.add(new Intake("Left", 
        RobotConstants.robotConfig.Intake.k_pivotMotorIdLeft, 
        RobotConstants.robotConfig.Intake.k_intakeMotorIdRight, false));
    
    m_intakes.add(new Intake("Right", 
        RobotConstants.robotConfig.Intake.k_pivotMotorIdRight, 
        RobotConstants.robotConfig.Intake.k_intakeMotorIdRight, true));
  }

  public static Intakes getInstance() {
    if(m_instance == null) {
      m_instance = new Intakes();
    }
    return m_instance;
  }

  public Intake getIntake(IntakeVariant intake) {
    return m_intakes.get(intake.ordinal());
  }

  public void setIntakeState(IntakeVariant intakeVariant, IntakeState intakeState) {
    m_intakes.get(intakeVariant.ordinal()).setIntakeState(intakeState);
  }

  public void setPivotTarget(IntakeVariant intakeVariant,  IntakePivotTarget pivotTarget) {
    m_intakes.get(intakeVariant.ordinal()).setPivotTarget(pivotTarget);
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
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
