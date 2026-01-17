package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

public class TagFollower extends Command {

  private final SwerveSub swerve;

  // PIDs
  private final PIDController xPID  = new PIDController(2.0, 0.0, 0.15);
  private final PIDController yPID  = new PIDController(1.8, 0.0, 0.10);
  private final PIDController rotPID = new PIDController(2.2, 0.0, 0.08);

  private static final double TARGET_DISTANCE = 0.20; // metros
  private static final double STABLE_TIME = 0.25;

  private double stableStart = -1;

  public TagFollower(SwerveSub swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    rotPID.disableContinuousInput();

    xPID.setTolerance(0.03);
    yPID.setTolerance(0.03);
    rotPID.setTolerance(2.5);
  }

  @Override
  public void execute() {

    Optional<Transform3d> tagOpt = swerve.getCameraToTag();

    if (tagOpt.isEmpty()) {
      swerve.setChassisSpeeds(new ChassisSpeeds());
      stableStart = -1;
      return;
    }

    Transform3d camToTag = tagOpt.get();

    // Medidas da câmera
    double x = camToTag.getX(); // frente
    double y = camToTag.getY(); // esquerda

    double xError = x - TARGET_DISTANCE;
    double yError = y; 

    double yawError = Math.atan2(y, x);

    // PID
    double rotSpeed = rotPID.calculate(yawError, 0.0);
    double xSpeed   = xPID.calculate(xError, 0.0);
    double ySpeed   = yPID.calculate(yError, 0.0);

    // DEADZONES
    if (Math.abs(yawError) < Math.toRadians(2.5)) rotSpeed = 0.0;
    if (Math.abs(xError) < 0.03) xSpeed = 0.0;
    if (Math.abs(yError) < 0.03) ySpeed = 0.0;
    
    // não anda antes de alinhar
    boolean yawAligned = Math.abs(yawError) < Math.toRadians(4.0);
    if (!yawAligned) {
      xSpeed = 0.0; 
    } 
    double xScale = MathUtil.clamp(Math.abs(yawError) / Math.toRadians(10), 0.2, 1.0);
xSpeed *= xScale;

    // APLICA VELOCIDADES
    swerve.setChassisSpeeds(
        new ChassisSpeeds(
            MathUtil.clamp(-ySpeed, -2.0, 2.0),
            MathUtil.clamp( -xSpeed, -1.0, 1.0),
            MathUtil.clamp( rotSpeed, -1.2, 1.2)
        )
    );

    boolean errorOK =
        Math.abs(xError)   < 0.05 &&
        Math.abs(yError)   < 0.05 &&
        Math.abs(yawError) < Math.toRadians(3.0);

    if (errorOK && swerve.robotStopped()) {
      if (stableStart < 0) {
        stableStart = Timer.getFPGATimestamp();
      }
    } else {
      stableStart = -1;
    }
  }

  @Override
  public boolean isFinished() {
    return stableStart > 0 &&
           Timer.getFPGATimestamp() - stableStart > STABLE_TIME;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds());
  }
}
