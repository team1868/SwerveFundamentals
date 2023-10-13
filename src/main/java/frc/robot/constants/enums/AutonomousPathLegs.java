package frc.robot.constants.enums;

import frc.robot.constants.enums.AutonomousRoutines.PPPConstraints;

public enum AutonomousPathLegs {
  INTAKE_1_TO_WIRESHOT("EXAMPLE", PPPConstraints.AUTO_DEFAULT);

  public final boolean currentlyExists;
  public final String name;
  public final PPPConstraints constraint;

  AutonomousPathLegs(String name, PPPConstraints constraint) {
    this(true, name, constraint);
  }

  AutonomousPathLegs(boolean exists, String name, PPPConstraints constraint) {
    this.currentlyExists = exists;
    this.name = name;
    this.constraint = constraint;
  }
}
