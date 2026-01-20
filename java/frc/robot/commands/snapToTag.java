// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;
public class snapToTag extends Command {
  private SwerveSub swerve;
  public snapToTag(SwerveSub swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

 
  @Override
  public void execute() {

      double rot = swerve.getSnapRotation();

    swerve.drive(
        new Translation2d(0, 0), // trava X/Y
        rot,
        false,
        true
    );
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(
        new Translation2d(0, 0),
        0,
        false,
        true
    );

    swerve.cancelSnap();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(swerve.getSnapRotation()) < 0.15;
  }
}
