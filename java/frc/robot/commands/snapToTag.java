package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import java.util.function.DoubleSupplier;

public class snapToTag extends Command {

  private final SwerveSub swerve;
  private final ShooterSub shooter;


private final DoubleSupplier xSupplier;
private final DoubleSupplier ySupplier;

  public snapToTag(SwerveSub swerve, ShooterSub shooter,  DoubleSupplier xSupplier,
    DoubleSupplier ySupplier) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
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
      double distance = shooter.getDistanceToCenter();

      Rotation2d aim = shooter.getCompensatedAim(distance);
      
      rot = swerve.getSnapRotationCompensated(aim);
    } else {
      rot = swerve.getSnapRotation();
    }

    swerve.drive(
        new Translation2d(  xSupplier.getAsDouble(),
        ySupplier.getAsDouble()), 
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
