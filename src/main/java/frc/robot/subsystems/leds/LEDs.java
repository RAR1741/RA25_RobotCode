package frc.robot.subsystems.leds;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Subsystem;

public class LEDs extends Subsystem {
  private static LEDs m_instance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = RobotConstants.robotConfig.LEDs.k_totalLength;

  // Main sections
  private ArrayList<Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>>> m_leftColors = new ArrayList<Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>>>();
  private ArrayList<Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>>> m_rightColors = new ArrayList<Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>>>();

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  private LEDs() {
    super("LEDs");

    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      m_leftColors.add(LEDModes.rainbowChase);
      m_rightColors.add(LEDModes.rainbowChase);

      m_led = new AddressableLED(RobotConstants.robotConfig.LEDs.k_PWMId);
      m_led.setLength(m_ledTotalLength);
      m_buffer = new AddressableLEDBuffer(m_ledTotalLength);
      m_led.start();
    }
  }

  @Override
  public void periodic() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setColorModes();

      m_led.setData(m_buffer);
      m_leftColors.clear();
      m_rightColors.clear();
    }
  }

  public void setAllColorMode(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> mode) {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      m_leftColors.add(mode);
      m_rightColors.add(mode);
    }
  }

  public void setAllColor(Color color) {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setLeftColor(color);
      setRightColor(color);
    }
  }

  public void setLeftColor(Color color) {
    m_leftColors.clear();
    m_leftColors.add(LEDModes.setColor(color));
  }

  public void setLeftColor(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> colorMode) {
    m_leftColors.clear();
    m_leftColors.add(colorMode);
  }

  public void setLeftColors(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>>... colors) {
    m_leftColors = LEDModes.makeAL(colors);
  }

  public void setRightColor(Color color) {
    m_rightColors.clear();
    m_rightColors.add(LEDModes.setColor(color));
  }

  public void setRightColors(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>>... colors) {
    m_rightColors = LEDModes.makeAL(colors);
  }

  public void setColorFromElevatorState(ElevatorState state) {
    switch (state) {
      case L2 ->
        setAllColorModes(
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kBlack),
            LEDModes.setColor(Color.kBlack),
            LEDModes.setColor(Color.kBlack),
            LEDModes.setColor(Color.kBlack),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed));
      case L3 ->
        setAllColorModes(
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kBlack),
            LEDModes.setColor(Color.kBlack),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed));
      case L4 ->
        setAllColorModes(
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed),
            LEDModes.setColor(Color.kRed));
      default -> {
      }
    }
  }

  public void chase() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setAllColorMode(LEDModes.redChase);
    }
  }

  public void breathe() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setAllColorMode(LEDModes.redBreathe);
    }
  }

  public void rainbowChase() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setAllColorMode(LEDModes.rainbowChase);
    }
  }

  public void rainbowBreatheSlow() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setAllColorMode(LEDModes.rainbowBreatheSlow);
    }
  }

  public void rainbowBreatheFast() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setAllColorMode(LEDModes.rainbowBreatheFast);
    }
  }

  public void redTwinkleSlow() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setAllColorMode(LEDModes.redTwinkleSlow);
    }
  }

  public void redTwinkleFast() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setAllColorMode(LEDModes.redTwinkleFast);
    }
  }

  public void off() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setAllColorMode(LEDModes.setColor(Color.kBlack));
    }
  }

  public void setAllColorModes(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>>... colors) {
    setLeftColors(colors);
    setRightColors(colors);
  }

  public void setColorModes() {
    for (int i = 0; i < m_rightColors.size(); i++) {
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> tempMode = m_rightColors
          .get(i);

      int sections = m_rightColors.size();
      int ledsPerSection = RobotConstants.robotConfig.LEDs.Right.k_length / sections;

      m_buffer = tempMode
          .apply(RobotConstants.robotConfig.LEDs.Right.k_start + (i * ledsPerSection))
          .apply(ledsPerSection)
          .apply(m_buffer);
    }

    for (int i = 0; i < m_leftColors.size(); i++) {
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> tempMode = m_leftColors
          .get(i);

      int sections = m_leftColors.size();
      int ledsPerSection = RobotConstants.robotConfig.LEDs.Left.k_length / sections;

      m_buffer = tempMode
          .apply(RobotConstants.robotConfig.LEDs.Left.k_start + (i * ledsPerSection))
          .apply(ledsPerSection)
          .apply(m_buffer);
    }

    // m_buffer = m_right1Color.apply(RobotConstants.robotConfig.LEDs.Left.k_start)
    // .apply(RobotConstants.robotConfig.LEDs.Left.k_length)
    // .apply(m_buffer);

    // m_buffer = m_right2Color.apply(RobotConstants.robotConfig.LEDs.Left.k_start)
    // .apply(RobotConstants.robotConfig.LEDs.Left.k_length)
    // .apply(m_buffer);

    // m_buffer = m_right3Color.apply(RobotConstants.robotConfig.LEDs.Left.k_start)
    // .apply(RobotConstants.robotConfig.LEDs.Left.k_length)
    // .apply(m_buffer);

    // m_buffer = m_right4Color.apply(RobotConstants.robotConfig.LEDs.Left.k_start)
    // .apply(RobotConstants.robotConfig.LEDs.Left.k_length)
    // .apply(m_buffer);
  }

  @Override
  public void stop() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void reset() {
  }
}
