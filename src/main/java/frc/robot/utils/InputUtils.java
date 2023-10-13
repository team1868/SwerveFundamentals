package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants.Control;

public class InputUtils {
  public static double scaleJoystickX_MPS(double rawAxis, double maxSpeed_mps) {
    double translationX = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 : Math.pow(rawAxis, 3);
    return translationX * maxSpeed_mps;
  }
  public static double scaleJoystickY_MPS(double rawAxis, double maxSpeed_mps) {
    double translationY = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 : Math.pow(rawAxis, 3);
    return translationY * maxSpeed_mps;
  }

  public static double ScaleJoystickTheta_RADPS(double rawAxis, Rotation2d maxAngularSpeed) {
    return ScaleJoystickTheta_RADPS(rawAxis, maxAngularSpeed.getRadians());
  }

  public static double ScaleJoystickTheta_RADPS(double rawAxis, double maxAngularSpeed_radps) {
    double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 : Math.pow(rawAxis, 3);
    return angular * maxAngularSpeed_radps;
  }
}
