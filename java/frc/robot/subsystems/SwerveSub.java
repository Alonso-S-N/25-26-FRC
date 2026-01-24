package frc.robot.subsystems;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import java.io.File;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.VisionValidator;

public class SwerveSub extends SubsystemBase {

  private final SwerveDrive swerve;
  private final NetworkTable limelight;

  private RobotConfig config;
  private VisionValidator visionValidator = new VisionValidator();
  private final NetworkTable motorTable = NetworkTableInstance.getDefault().getTable("Motors");
  
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3.0, 2.0);

      private final ProfiledPIDController snapPID =
    new ProfiledPIDController(2.5, 0.0, 0.0,constraints);


  public SwerveSub() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

    try {
      File json = new File(Filesystem.getDeployDirectory(), "yagsl");
      swerve = new SwerveParser(json).createSwerveDrive(3.0);
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Erro ao criar Swerve", e);
    }

    
    swerve.setGyroOffset(
      new Rotation3d(
          0.0,                     
          0.0,                     
          Math.toRadians(180) // yaw (0°)
      )
  );

    // Confiança da visão (MegaTag)
    swerve.setVisionMeasurementStdDevs(
        VecBuilder.fill(
            0.05, // X
            0.05, // Y
            9999  // Gyro manda
        )
    );

    swerve.setCosineCompensator(false);

    snapPID.setTolerance(Math.toRadians(2.5));
    snapPID.enableContinuousInput(Math.toRadians(-90), Math.toRadians(90));
  }

  /* =================== DRIVE =================== */

  public void drive(
      Translation2d translation,  
      double rotation,
      boolean fieldRelative,
      boolean openLoop
  ) { 
    swerve.drive(translation, rotation, fieldRelative, openLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }

  /* =================== POSE =================== */

  public Pose2d getPose() {
    return swerve.getPose();
  }

  public Rotation2d getHeading() {
    return swerve.getYaw();
  }

  public void resetOdometry(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  public double getTy(){
    return limelight.getEntry("ty").getDouble(0);
  }

  public double getTx(){
    return limelight.getEntry("tx").getDouble(0);
  }
  
  public double getTA(){
    return limelight.getEntry("tx").getDouble(0);
  }
  /* =================== LIMELIGHT =================== */
  public Optional<Transform3d> getCameraToTag() {
    if (limelight.getEntry("tv").getDouble(0) != 1) {
      return Optional.empty();
    }
  
    double[] pose = limelight
        .getEntry("targetpose_cameraspace")
        .getDoubleArray(new double[6]);
  
    if (pose.length < 6) return Optional.empty();
  
    return Optional.of(
        new Transform3d(
            new Translation3d(
                pose[0], // X frente
                pose[1], // Y esquerda
                pose[2]  // Z cima
            ),
            new Rotation3d(
                Math.toRadians(pose[3]),
                Math.toRadians(pose[4]),
                Math.toRadians(pose[5])
            )
        )
    );
  }
  
  
  public Optional<Pose2d> getMegaTagPose() {
    if (limelight.getEntry("tv").getDouble(0) != 1) {
      return Optional.empty();
    }
  
    double[] pose = limelight
        .getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);
  
    if (pose.length < 6) return Optional.empty();
  
    return Optional.of(
        new Pose2d(
            pose[0],
            pose[1],
            Rotation2d.fromDegrees(pose[5])
        )
    );
  }
  
 
  public boolean robotStopped() {
    ChassisSpeeds speeds = swerve.getRobotVelocity();
    return Math.abs(speeds.vxMetersPerSecond) < 0.05 &&
           Math.abs(speeds.vyMetersPerSecond) < 0.05 &&
           Math.abs(speeds.omegaRadiansPerSecond) < 0.05;
  }
 
public double getSnapRotation() {

  if (!HasTarget()) {
    snapPID.reset(0.0,0.0);
    return 0.0;
  }

  double txDeg = getTx();

  if (Math.abs(txDeg) < 1.0) {
    snapPID.reset(0.0,0.0);
    return 0.0;
  }

  double txRad = Math.toRadians(txDeg);

  double rot = snapPID.calculate(txRad, 0.0);

  return MathUtil.clamp(-rot, -2.0, 2.0);
}



public void cancelSnap() {
  snapPID.reset(0.0,0.0);
}



  /* =================== PERIODIC =================== */

  @Override
  public void periodic() {
     getMegaTagPose().ifPresent(visionPose -> { 
        if (!visionValidator.isValid(visionPose)){
          return;
        }
      double error =
          visionPose.getTranslation()
              .getDistance(getPose().getTranslation());
 
      // Se erro grande e robô parado -> RESET
      if (error > 0.75 && robotStopped()) {
        swerve.resetOdometry(visionPose);
        visionValidator.reset();
      }
      // Senão -> correção suave (MegaTag)
      else if (error > 0.15){
        swerve.addVisionMeasurement(
            visionPose,
            Timer.getFPGATimestamp()
        );
      }

      ChassisSpeeds speeds = swerve.getRobotVelocity();
       if (Math.abs(speeds.omegaRadiansPerSecond)>2.0){
        return;
       }
    });
    SmartDashboard.putBoolean("LL/HasTarget",
    limelight.getEntry("tv").getDouble(0) == 1);

getMegaTagPose().ifPresent(p ->
    SmartDashboard.putString("LL/Pose", p.toString()));
  }

  /* =================== PATHPLANNER =================== */

  public void configureAutoBuilder() {
    if (config == null) {
      throw new RuntimeException("RobotConfig null");
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        this::setChassisSpeeds,
        new PPHolonomicDriveController(
            new PIDConstants(3.5, 0.0, 0.1),
            new PIDConstants(2.5, 0.0, 0.2)
        ),
        config,
        DriverStation::isAutonomous,
        this
    );
  }
  public void logDriveMotorInfo(int moduleIndex, String name) {
    var motor = swerve.getModules()[moduleIndex].getDriveMotor();
    Object realMotor = motor.getMotor();
  
    if (RobotBase.isReal() && realMotor instanceof SparkMax spark) {
  
      NetworkTable moduleTable = motorTable.getSubTable(name);
  
      double temp = spark.getMotorTemperature();
      double current = spark.getOutputCurrent();
      double voltage = spark.getBusVoltage() * spark.getAppliedOutput();
      double rpm = spark.getEncoder().getVelocity();
  
      boolean overTemp = temp > 80;
      boolean overCurrent = current >= 40;
  
      String status =
          overTemp ? "OVER TEMP" :
          overCurrent ? "OVER CURRENT" :
          "OK";
  
      moduleTable.getEntry("Temp").setDouble(temp);
      moduleTable.getEntry("TempUnit").setString("C");
      moduleTable.getEntry("Current").setDouble(current);
      moduleTable.getEntry("Voltage").setDouble(voltage);
      moduleTable.getEntry("RPM").setDouble(rpm);
  
      moduleTable.getEntry("OverTemp").setBoolean(overTemp);
      moduleTable.getEntry("OverCurrent").setBoolean(overCurrent);
      moduleTable.getEntry("Status").setString(status);
    }
  }
   public SwerveModuleState[] getModules() {
    return swerve.getStates();
}

public void testDriveMotor(int moduleIndex, double speed, String name) { 
  swerve.getModules()[moduleIndex].getDriveMotor().set(speed);
 }

 public void testSteerMotor(int moduleIndex, double speed, String name) { 
  swerve.getModules()[moduleIndex].getAngleMotor().set(speed); 
}
 public void stopModules() {
    swerve.setChassisSpeeds(new ChassisSpeeds());
    }

 public ChassisSpeeds getRobotVelocity() {
  return swerve.getRobotVelocity();
 } 

 public boolean HasTarget(){
  return limelight.getEntry("tv").getDouble(0) == 1;
 }
}
