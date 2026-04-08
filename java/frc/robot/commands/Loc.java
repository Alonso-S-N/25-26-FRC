package frc.robot.commands;

import frc.robot.subsystems.ShooterSub;
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
      private boolean snapMode = false;
      private double rot;


      private ShooterSub shooter;
      

      private final PIDController headingPID =
      new PIDController(2.0, 0.0, 0.2);
  
   public Loc(SwerveSub swerve, ShooterSub shooter, PS5Controller ps5) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.ps5 = ps5;
    
    headingPID.setTolerance(Math.toRadians(2.0));
    headingPID.enableContinuousInput(Math.toRadians(-90), Math.toRadians(90));

    addRequirements(swerve);
}
    @Override
    public void initialize() {

  }

  public void setSnapMode(boolean enabled) {
  snapMode = enabled;
}

  public void MainControll(){
    this.y = applyDeadband(ps5.getLeftY());
    this.x = applyDeadband(ps5.getLeftX());

    int pov = ps5.getPOV();
     if (pov != -1) {
      rot = rotateToAngle(pov);

    } else if (snapMode && shooter.HasTarget()) {

    boolean robotMoving =
    Math.abs(swerve.getRobotVelocity().vxMetersPerSecond) > 0.05 ||
    Math.abs(swerve.getRobotVelocity().vyMetersPerSecond) > 0.05;

  if (robotMoving) {
    double distance = shooter.getDistanceToCenter();
   double tx = swerve.getTx();

double movingComp =
shooter.getMovingShotTxComp();

rot =
swerve.getSnapRotationFromTx(
    tx + movingComp
);

  } else {
    rot = swerve.getSnapRotation();
  }

} else {
      rot = applyDeadband(-ps5.getRightX()) * 2;
      swerve.cancelSnap();
      headingPID.reset();
    }
    
    swerve.drive(
    new Translation2d(x * MAX_SPEED, y * MAX_SPEED),
    rot,
    false,
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
    SmartDashboard.putNumber("Snap Rot", rot);

    swerve.getCameraToTag().ifPresent(tag ->
    SmartDashboard.putNumber(
        "Snap/YawErrorDeg",
        Math.toDegrees(tag.getRotation().getZ())
    )
);
   swerve.printOffsets();
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
