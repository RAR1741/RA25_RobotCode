package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.RobotConstants;

public class LEDs implements Runnable {
  private static LEDs m_instance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = RobotConstants.robotConfig.LEDs.k_totalLength;

  // Main sections
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_leftColor = LEDModes
      .setColor(Color.kRed);
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_rightColor = LEDModes.rainbowChase;

  private Thread m_thread;

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  private LEDs() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      m_led = new AddressableLED(RobotConstants.robotConfig.LEDs.k_PWMId);
      m_led.setLength(m_ledTotalLength);
      m_buffer = new AddressableLEDBuffer(m_ledTotalLength);
      m_led.start();

      m_thread = new Thread(this);
      m_thread.setDaemon(true);
    }
  }

  @Override
  public void run() {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setRightColorMode();
      setLeftColorMode();

      m_led.setData(m_buffer);
    }
  }

  public void setAllColorMode(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> mode) {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      m_leftColor = mode;
      m_rightColor = mode;
    }
  }

  public void setAllColor(Color color) {
    if (RobotConstants.robotConfig.LEDs.k_isEnabled) {
      setLeftColor(color);
      setRightColor(color);
    }
  }

  public void setLeftColor(Color color) {
    m_leftColor = LEDModes.setColor(color);
  }

  public void setLeftColor(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> colorMode) {
    m_leftColor = colorMode;
  }

  public void setRightColor(Color color) {
    m_rightColor = LEDModes.setColor(color);
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

  public void setLeftColorMode() {
    m_buffer = m_leftColor.apply(RobotConstants.robotConfig.LEDs.Left.k_start)
        .apply(RobotConstants.robotConfig.LEDs.Left.k_length)
        .apply(m_buffer);
  }

  public void setRightColorMode() {
    m_buffer = m_rightColor.apply(RobotConstants.robotConfig.LEDs.Right.k_start)
        .apply(RobotConstants.robotConfig.LEDs.Right.k_length)
        .apply(m_buffer);
  }
}