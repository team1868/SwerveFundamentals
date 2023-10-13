
package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedController;

public class SolidLedCommand extends CommandBase {
  private LedController _leds;
  private Color8Bit _color;
  private AddressableLEDBuffer _ledBuffer;

  public SolidLedCommand(LedController leds, Color8Bit color) {
    _leds = leds;
    _color = color;
    _ledBuffer = new AddressableLEDBuffer(leds.LED_LENGTH);

    for (int i = 0; i < _leds.LED_LENGTH; i++) {
      _ledBuffer.setRGB(i, _color.red, _color.green, _color.blue);
    }

    addRequirements(_leds);
  }

  public SolidLedCommand(LedController leds, Color color) {
    this(leds, new Color8Bit(color));
  }

  @Override
  public void initialize() {
    _leds.update(_ledBuffer);
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _leds.update();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
