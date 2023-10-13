package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Ports {
  public static final int DRIVER_XBOX_USB_PORT = 0;

  public enum DrivetrainPorts {
    COMP_PORTS(
        "Default Name",
        new int[] {10, 16, 12, 14},
        new int[] {11, 17, 13, 15},
        new int[] {1, 3, 0, 2},
        "Default Name",
        35
    ),
    PRACTICE_PORTS(
        "Default Name",
        new int[] {10, 16, 12, 14},
        new int[] {11, 17, 13, 15},
        new int[] {1, 2, 0, 3},
        "Default Name",
        35
    ),
    SWERVE_BASE_PORTS(
        "rio",
        new int[] {11, 16, 12, 14},
        new int[] {10, 17, 13, 15},
        new int[] {30, 33, 31, 32},
        "rio",
        35
    );

    public final String moduleCanBus;
    public final int[] drive;
    public final int[] steer;
    public final int[] encoder;
    public final String pigeonCanBus;
    public final int pigeonID;

    DrivetrainPorts(
        String moduleCanBus,
        int[] drive,
        int[] steer,
        int[] encoder,
        String pigeonCanBus,
        int pigeonID
    ) {
      this.moduleCanBus = moduleCanBus;
      this.drive = drive;
      this.steer = steer;
      this.encoder = encoder;
      this.pigeonCanBus = pigeonCanBus;
      this.pigeonID = pigeonID;
    }
  }
}
