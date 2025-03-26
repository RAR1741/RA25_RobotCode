package frc.robot.controls.controllers;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Elevator.ElevatorState;

public class OperatorController extends FilteredController {
  private ElevatorState m_desiredElevatorState = ElevatorState.L4;

  public OperatorController(int port) {
    super(port, false, false, 0);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput, double triggerActivationThreshold) {
    super(port, useDeadband, useSquaredInput, triggerActivationThreshold);
  }

  private final int k_leftIntakeButton = Button.LEFT_BUMPER;

  public boolean getWantsLeftIntakeGround() {
    return getRawButtonPressed(k_leftIntakeButton);
  }

  public boolean getWantsLeftIntakeStow() {
    return getRawButtonReleased(k_leftIntakeButton);
  }

  private final int k_rightIntakeButton = Button.RIGHT_BUMPER;

  public boolean getWantsRightIntakeGround() {
    return getRawButtonPressed(k_rightIntakeButton);
  }

  public boolean getWantsRightIntakeStow() {
    return getRawButtonReleased(k_rightIntakeButton);
  }

  private final int k_ejectButton = Button.B;

  public boolean getWantsIntakeEject() {
    return getRawButtonPressed(k_ejectButton);
  }

  public boolean getWantsIntakeEjectStopped() {
    return getRawButtonReleased(k_ejectButton);
  }

  public ElevatorState getDesiredElevatorState() {
    if (getHatPressed(Direction.DOWN)) {
      m_desiredElevatorState = ElevatorState.L1;
    } else if (getHatPressed(Direction.RIGHT)) {
      m_desiredElevatorState = ElevatorState.L2;
    } else if (getHatPressed(Direction.LEFT)) {
      m_desiredElevatorState = ElevatorState.L3;
    } else if (getHatPressed(Direction.UP)) {
      m_desiredElevatorState = ElevatorState.L4;
    } else if (getRawButtonPressed(Button.LEFT_JOYSTICK)) {
      m_desiredElevatorState = ElevatorState.FEEDER_STATION;
    }

    Logger.recordOutput("OperatorController/DesiredElevatorState", m_desiredElevatorState.toString());

    return m_desiredElevatorState;
  }

  public boolean isDPadUsed() {
    return this.getHat(Direction.UP) || this.getHat(Direction.DOWN) || this.getHat(Direction.LEFT) || this.getHat(Direction.RIGHT);
  }

  public boolean getWantsStow() {
    return this.getRawButtonPressed(Button.A);
  }

  public boolean getWantsResetElevator() {
    return this.getRawButtonPressed(Button.START);
  }

  private final int k_scoreButton = Button.X;

  public boolean getWantsScore() {
    return this.getRawButton(k_scoreButton);
  }

  public boolean getWantsEndEffectorOff() {
    return this.getRawButtonReleased(k_scoreButton);
  }

  private final int k_hopperButton = Button.Y;

  public boolean getWantsReverseHopper() {
    return this.getRawButton(k_hopperButton);
  }

  public boolean getStoppedReverseHopper() {
    return this.getRawButtonReleased(k_hopperButton);
  }

  public boolean getWantsElevatorOverride(){
    return this.getRawButton(Button.START);
  }

  private final int k_reverseButton = Button.BACK;

  public boolean getWantsReverseEndEffector() {
    return this.getRawButton(k_reverseButton);
  }

  public boolean getWantsReverseEndEffectorStopped() {
    return this.getRawButtonReleased(k_reverseButton);
  }
}
