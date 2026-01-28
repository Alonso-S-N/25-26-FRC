package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;

public class snapToTag extends Command {

  private final SwerveSub swerve;
  private final ShooterSub shooter;

  public snapToTag(SwerveSub swerve, ShooterSub shooter) {
    this.swerve = swerve;
    this.shooter = shooter;
    addRequirements(swerve);
  }

  @Override
  public void execute() {

    if (!swerve.HasTarget()) {
      swerve.drive(new Translation2d(), 0, false, true);
      return;
    }

    boolean robotMoving = !swerve.robotStopped();

    double rot;

    if (robotMoving) {
      double distance = shooter.getDistanceToTag();

      Rotation2d aim = shooter.getCompensatedAim(distance);
      
      rot = swerve.getSnapRotationCompensated(aim);
    } else {
      rot = swerve.getSnapRotation();
    }

    swerve.drive(
        new Translation2d(0, 0), 
        rot,
        false,
        true
    );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false, true);
    swerve.cancelSnap();
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}
