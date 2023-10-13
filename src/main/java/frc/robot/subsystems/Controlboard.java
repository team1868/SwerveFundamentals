package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Ports;

public class Controlboard {
  public CommandXboxController _xboxDrive = new CommandXboxController(Ports.DRIVER_XBOX_USB_PORT);

  private Alliance _alliance;

  public Controlboard(Field2d field) {}

  public void configShuffleboard() {}

  public void updateShuffleboard() {}

  // Left is positive X in terms of field, negative so our controller aligns
  public double getDriveX() {
    return _alliance == Alliance.Red ? _xboxDrive.getLeftY() : -_xboxDrive.getLeftY();
  }

  public double getDriveY() {
    return _alliance == Alliance.Red ? _xboxDrive.getLeftX() : -_xboxDrive.getLeftX();
  }

  public double getRotX() {
    return -_xboxDrive.getRightX();
  }

  public double getRotY() {
    return _xboxDrive.getRightY();
  }

  public boolean getBackButtonPressed() {
    return _xboxDrive.getHID().getBackButtonPressed();
  }

  public void updateAlliance(Alliance alliance) {
    _alliance = alliance;
  }

  public boolean isRedAlliance() {
    return _alliance == Alliance.Red;
  }

  public void driverRumble() {
    _xboxDrive.getHID().setRumble(RumbleType.kBothRumble, 1.0);
  }

  public void driverResetRumble() {
    _xboxDrive.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }
}
