package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

public class TagFollower extends Command {

  private final SwerveSub swerve;

  private final PIDController xPID = new PIDController(2.0, 0, 0);
  private final PIDController yPID = new PIDController(2.0, 0, 0);
  private final PIDController rotPID = new PIDController(4.0, 0, 0);

  private static final Rotation2d CAMERA_YAW =
    Rotation2d.fromDegrees(-15); 

  private static final double TARGET_DISTANCE = 0.60; // metros



  public TagFollower(SwerveSub swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    rotPID.enableContinuousInput(-Math.PI, Math.PI);

    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);
    rotPID.setTolerance(Math.toRadians(2));
  }

  @Override
  public void execute() {
    Optional<Transform3d> tagOpt = swerve.getCameraToTag();
    if (tagOpt.isEmpty()) {
      swerve.setChassisSpeeds(new ChassisSpeeds());
      return;
    }
  
    Transform3d camToTag = tagOpt.get();
  
    Translation2d camError = new Translation2d(
        camToTag.getX() - TARGET_DISTANCE,
        camToTag.getY()
    );
  
    Translation2d robotError =
        camError.rotateBy(CAMERA_YAW);
  
    double xSpeed = xPID.calculate(robotError.getX());
    double ySpeed = yPID.calculate(robotError.getY());
  
    double rotError = Math.atan2(
        robotError.getY(),
        robotError.getX()
    );
  
    double rotSpeed = rotPID.calculate(rotError);
  
    swerve.setChassisSpeeds(
        new ChassisSpeeds(
            MathUtil.clamp(xSpeed, -1.5, 1.5),
            MathUtil.clamp(ySpeed, -1.5, 1.5),
            MathUtil.clamp(rotSpeed, -2.0, 2.0)
        )
    );
  }


  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() &&
           yPID.atSetpoint() &&
           rotPID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds());
  }
}
