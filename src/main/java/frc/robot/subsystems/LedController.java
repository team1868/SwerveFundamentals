package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedController extends SubsystemBase {
  private AddressableLED _ledStrip;
  private AddressableLEDBuffer _ledBuffer;
  public final int LED_LENGTH;

  public static final Color8Bit DEFAULT_COLOR = new Color8Bit(Color.kLightBlue);

  public LedController(int length) {
    LED_LENGTH = length;
    _ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
    _ledStrip = new AddressableLED(9);

    for (int i = 0; i < LED_LENGTH; i++) {
      _ledBuffer.setRGB(i, DEFAULT_COLOR.red, DEFAULT_COLOR.green, DEFAULT_COLOR.blue);
    }

    _ledStrip.setLength(LED_LENGTH);
    _ledStrip.setData(_ledBuffer);
    _ledStrip.start();
  }

  public void setSolidColor(Color c) {
    setSolidColor(new Color8Bit(c));
  }

  public void setSolidColor(Color8Bit c) {
    for (int i = 0; i < LED_LENGTH; i++) {
      _ledBuffer.setRGB(i, c.red, c.green, c.blue);
    }
    _ledStrip.setData(_ledBuffer);
  }

  public void update() {
    update(_ledBuffer);
  }

  public void update(AddressableLEDBuffer buffer) {
    _ledStrip.setData(buffer);
  }
}
