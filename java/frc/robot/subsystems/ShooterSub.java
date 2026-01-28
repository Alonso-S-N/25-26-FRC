package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSub extends SubsystemBase {

  private final SparkMax shooterMotor = new SparkMax(15, MotorType.kBrushless);
  private final SparkMax feederMotor  = new SparkMax(16, MotorType.kBrushless);

  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
  private final NetworkTable limelight =
      NetworkTableInstance.getDefault().getTable("limelight2");

  private static final double RpmTolerance = 75;
  private static final double STABLE_TIME = 0.2;

  private static final double G = 9.81;
  private static final double shotinhoAngDeg = 45.0;
  private static final double WHEEL_RADIUS = 0.05;

  private final Timer rpmStableTimer = new Timer();
  private boolean wasAtSpeed = false;

  private final SwerveSub swerve;

  public ShooterSub(SwerveSub swerve) {
    this.swerve = swerve;

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.idleMode(IdleMode.kCoast)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0005, 0, 0);

    shooterMotor.configure(
        shooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );
  }


  public boolean HasTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1.0;
  }

  public void StopShooter() {
    shooterMotor.set(0);
    feederMotor.set(0);
    rpmStableTimer.stop();
    rpmStableTimer.reset();
    wasAtSpeed = false;
  }
 // ================== RPM CHECK ================== //
  private boolean atTargetRPM(double targetRPM) {
    double error = Math.abs(shooterEncoder.getVelocity() - targetRPM);

    boolean atSpeed = error <= RpmTolerance;
    if (atSpeed) {
      if (!wasAtSpeed) {
        rpmStableTimer.restart();
        wasAtSpeed = true;
      }
    } else {
      rpmStableTimer.reset();
      wasAtSpeed = false;
    }
    return wasAtSpeed && rpmStableTimer.hasElapsed(STABLE_TIME);
  }

 //================== RPM FROM DISTANCE ================== //
  public double getRPMFromDistance(double distanceMeters) {

    double shooterHeight = 0.67;
    double targetHeight  = 1.82;
    double deltaH = targetHeight - shooterHeight;

    double angleRad = Math.toRadians(shotinhoAngDeg);

    double cos2 = Math.cos(angleRad) * Math.cos(angleRad);
    double inner = distanceMeters * Math.tan(angleRad) - deltaH;

    if (inner <= 0) return 0;

    double v = Math.sqrt(
        (G * distanceMeters * distanceMeters) /
        (2.0 * cos2 * inner)
    );

    double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS;
    return MathUtil.clamp((v / wheelCircumference) * 60.0, 0, 5000); // se necessario multiplicar pelo arrasto aero (1.25 ou 25%) :p
  }

  private double rpmToVelocity(double rpm) {
    return (rpm / 60.0) * (2.0 * Math.PI * WHEEL_RADIUS);
  }

  private double getTimeOfFlight(double distance, double rpm) {
    if (rpm <= 0) return 0.0;

    double v = rpmToVelocity(rpm);
    return distance /
        (v * Math.cos(Math.toRadians(shotinhoAngDeg)));
  }
  
  //=============== COMPENSATED AIM =============== //

  public Rotation2d getCompensatedAim(double distanceMeters) {

    double rpm = getRPMFromDistance(distanceMeters);
    if (rpm <= 0) {
      return swerve.getHeading();
    }

    double timeOfFlight = getTimeOfFlight(distanceMeters, rpm);

    ChassisSpeeds speeds = swerve.getRobotVelocity();
    Translation2d robotVelocity =
        new Translation2d(
            speeds.vyMetersPerSecond,
            speeds.vxMetersPerSecond
        );

    Translation2d offset = robotVelocity.times(timeOfFlight);

    double txRad = Math.toRadians(swerve.getTx());
    Rotation2d targetDirection =
        swerve.getHeading().plus(new Rotation2d(txRad));

    Translation2d targetVector =
        new Translation2d(distanceMeters, targetDirection);

    Translation2d compensatedVector =
        targetVector.minus(offset);

    return compensatedVector.getAngle();
  }

 // ===================== SHOOT ===================== //
  public void shoot(double distanceMeters) {

    if (!HasTarget()) {
      StopShooter();
      return;
    }

    double targetRPM = getRPMFromDistance(distanceMeters) ;
    if (targetRPM <= 0) {
      StopShooter();
      return;
    } 

    shooterMotor.getClosedLoopController()
        .setReference(targetRPM, ControlType.kVelocity);

    if (atTargetRPM(targetRPM)) {
      feederMotor.set(0.6);
    } else {
      feederMotor.set(0.0);
    }
  }

// ================= DISTANCE TO TAG ================= //
  public double getDistanceToTag() {
    double cameraHeight = 0.67;
    double targetHeight = 1.82;
    double cameraAngle = 20;
    double ty = swerve.getTy();

    return (targetHeight - cameraHeight) /
           Math.tan(Math.toRadians(cameraAngle + ty));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/RPM", shooterEncoder.getVelocity());
  }
}
