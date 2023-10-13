package frc.robot.constants;

public enum RobotVersions {
  CompBot(DrivetrainConfs.COMP_BOT_CONFS),
  PracticeBot(DrivetrainConfs.PRACTICE_BOT_CONFS),
  SwerveBase(DrivetrainConfs.SWERVE_BASE_CONFS),
  TestBoard(null);

  public final DrivetrainConfs drive;

  RobotVersions(DrivetrainConfs drive) {
    this.drive = drive;
  }
}
