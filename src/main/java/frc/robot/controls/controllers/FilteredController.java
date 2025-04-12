package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.controls.Deadband;
import frc.robot.controls.SquaredInput;

public class FilteredController extends GenericHID {
  private static final double k_deadbandLimit = 0.06;

  private boolean m_useDeadband;
  private boolean m_useSquaredInput;
  private double m_triggerActivationThreshold;

  public double m_allianceMultiplier = -1.0;

  private Deadband m_deadband = new Deadband(k_deadbandLimit);
  private SquaredInput m_squaredInput = new SquaredInput(k_deadbandLimit);

  private final DPadButton[] hatButtons = { new DPadButton(this, DPadButton.Direction.UP),
      new DPadButton(this, DPadButton.Direction.DOWN), new DPadButton(this, DPadButton.Direction.LEFT),
      new DPadButton(this, DPadButton.Direction.RIGHT) };

  public FilteredController(int port) {
    super(port);
    m_useDeadband = false;
    m_useSquaredInput = false;
  }

  public FilteredController(int port, boolean useDeadband, boolean useSquaredInput, double triggerActivationThreshold) {
    this(port);
    this.m_useDeadband = useDeadband;
    this.m_useSquaredInput = useSquaredInput;
    this.m_triggerActivationThreshold = triggerActivationThreshold;
  }

  public void setAllianceMultiplier() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      m_allianceMultiplier = 1.0;
    } else {
      m_allianceMultiplier = -1.0;
    }
  }

  public double getFilteredAxis(int axis) {
    double value = this.getRawAxis(axis);

    if (axis == Axis.LEFT_TRIGGER || axis == Axis.RIGHT_TRIGGER) {
      value = value >= m_triggerActivationThreshold ? value : 0;
    } else {
      // Apply squared input, if requested
      if (m_useSquaredInput) {
        value = m_squaredInput.scale(value);
      }

      // Apply deadband, if requested
      if (m_useDeadband) {
        value = m_deadband.scale(value);
      }
    }

    return value;
  }

  public boolean getHatPressed(int direction) {
    return hatButtons[direction].getPressed();
  }

  public boolean getHatReleased(int direction) {
    return hatButtons[direction].getReleased();
  }

  public boolean getHat(int direction) {
    return hatButtons[direction].get();
  }

  public boolean getAnyHatReleased() {
    return getHatReleased(Direction.DOWN) || getHatReleased(Direction.UP) || getHatReleased(Direction.LEFT) || getHatReleased(Direction.RIGHT);
  }

  public interface Button {
    int A = 1;
    int B = 2;
    int X = 3;
    int Y = 4;
    int LEFT_BUMPER = 5;
    int RIGHT_BUMPER = 6;
    int BACK = 7;
    int START = 8;
    int LEFT_JOYSTICK = 9;
    int RIGHT_JOYSTICK = 10;
  }

  public interface Axis {
    int LEFT_X_AXIS = 0;
    int LEFT_Y_AXIS = 1;
    int LEFT_TRIGGER = 2;
    int RIGHT_TRIGGER = 3;
    int RIGHT_X_AXIS = 4;
    int RIGHT_Y_AXIS = 5;
  }

  public interface Direction {
    int UP = 0;
    int DOWN = 1;
    int LEFT = 2;
    int RIGHT = 3;
  }
}
