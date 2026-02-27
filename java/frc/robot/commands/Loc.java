package frc.robot.commands;

import frc.robot.subsystems.SwerveSub;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;



public class Loc extends Command {
  private SwerveSub swerve;
  
      private PS5Controller ps5;
      private double y,x;
      private double MAX_SPEED = 3; // meters per second
      

      private final PIDController headingPID =
      new PIDController(2.0, 0.0, 0.2);
  
    public Loc(SwerveSub swerve, PS5Controller ps5) {
       headingPID.setTolerance(Math.toRadians(2.0));
       this.swerve = swerve;
       this.ps5 = ps5;

       headingPID.enableContinuousInput(-Math.PI, Math.PI);
    
      addRequirements(swerve);
    }
  
    @Override
    public void initialize() {

  }

  public void MainControll(){
    this.y = applyDeadband(ps5.getLeftY());
    this.x = applyDeadband(ps5.getLeftX());
    double rot;

    int pov = ps5.getPOV();
     if (pov != -1) {
      rot = rotateToAngle(pov);

    } else  if (ps5.getL1Button()) {
      rot = swerve.getSnapRotation();
      headingPID.reset();
    } else {
      rot = applyDeadband(-ps5.getRightX()) * 2;
      swerve.cancelSnap();
      headingPID.reset();
    }
    
    swerve.drive(
    new Translation2d(y * MAX_SPEED, x * MAX_SPEED),
    rot,
    true,
    true
);
}
      
private double rotateToAngle(double targetDegrees) {
  Rotation2d current = swerve.getHeading();

  double output = headingPID.calculate(
      current.getRadians(),
      Math.toRadians(targetDegrees)
  );

  if (headingPID.atSetpoint()) {
    return 0.0;
  }

  return MathUtil.clamp(output, -3.0, 3.0);
}
 
        @Override
  public void execute() {
    MainControll();
    SmartdashBoard();
  }

    public void SmartdashBoard() {
    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);

    swerve.getCameraToTag().ifPresent(tag ->
    SmartDashboard.putNumber(
        "Snap/YawErrorDeg",
        Math.toDegrees(tag.getRotation().getZ())
    )
);
  }

  @Override
  public void end(boolean interrupted) {}

 
  @Override
  public boolean isFinished() {
    return false;
  }

  private double applyDeadband(double value) {
    return Math.abs(value) < 0.05 ? 0.0 : value;
}
}
