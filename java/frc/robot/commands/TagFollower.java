package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

public class TagFollower extends Command {

  private final SwerveSub swerve;

  /* ================= PID ================= */

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3.0, 2.0);

  private final ProfiledPIDController xPID   = new ProfiledPIDController(2.0, 0.0, 0.15,constraints);
  private final ProfiledPIDController yPID   = new ProfiledPIDController(1.8, 0.0, 0.10,constraints);
  private final ProfiledPIDController rotPID = new ProfiledPIDController(2.2, 0.0, 0.08,constraints);

  /* ================= CONSTANTES ================= */
  private static final double TARGET_DISTANCE = 0.25; // metros
  private static final double STABLE_TIME     = 0.25; // segundos

  /* ================= ESTADO ================= */
  private boolean reachedDistance = false;
  private double stableStart = -1;

  public TagFollower(SwerveSub swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    xPID.setTolerance(0.05,0.10);   // 5 cm
    yPID.setTolerance(0.05,0.10);
    rotPID.setTolerance(1.5);  // graus
  }

  @Override
  public void execute() {
    Optional<Transform3d> tagOpt = swerve.getCameraToTag();

    if (tagOpt.isEmpty()) {
      stopAndReset();
      return;
    }

    Transform3d camToTag = tagOpt.get();

    double x = camToTag.getX(); // positivo = tag à esquerda/direita
    double y = camToTag.getY(); // positivo = tag à frente

    double txDeg = swerve.getTx();

    double rotSpeed = -rotPID.calculate(Math.toRadians(txDeg),0.0);
    if (Math.abs(txDeg) < 2.0) rotSpeed = 0.0;

    double xSpeed = xPID.calculate(x, 0.0);
    if (Math.abs(swerve.getTx()) < 2.0 ) xSpeed = 0.0;
    
    double ySpeed = yPID.calculate(y, TARGET_DISTANCE); 

    boolean distanceOK = Math.abs(y - TARGET_DISTANCE) < 0.05;

    if (distanceOK) {
      ySpeed = 0.0;
    }
  
    reachedDistance = yPID.atSetpoint();

if (xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint()) {
  swerve.setChassisSpeeds(new ChassisSpeeds());
  reachedDistance = true;
  return;
}
 
ySpeed = MathUtil.applyDeadband(ySpeed, 0.04);
xSpeed = MathUtil.applyDeadband(xSpeed, 0.03);
rotSpeed = MathUtil.applyDeadband(rotSpeed, 0.02);

    swerve.setChassisSpeeds(
        new ChassisSpeeds(
            MathUtil.clamp(ySpeed,  -2.0, 2.0),
            MathUtil.clamp(xSpeed,  -0.6, 0.6),
            MathUtil.clamp(rotSpeed,-1.2, 1.2)
        )
    );

    /* ===== FINALIZAÇÃO ===== */
    boolean errorOK =
        reachedDistance &&
        Math.abs(txDeg) < 1.5;

    if (errorOK && swerve.robotStopped()) {
      if (stableStart < 0) stableStart = Timer.getFPGATimestamp();
    } else {
      stableStart = -1;
    }
  }

  private void stopAndReset() {
    swerve.setChassisSpeeds(new ChassisSpeeds());
    stableStart = -1;
    reachedDistance = false;
  }

  @Override
  public boolean isFinished() {
    return stableStart > 0 &&
           Timer.getFPGATimestamp() - stableStart > STABLE_TIME;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds());
    reachedDistance = false;
    stableStart = -1;
  }
}