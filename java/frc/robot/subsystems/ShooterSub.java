// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {

  SparkMax ShooterMotor = new SparkMax(15,MotorType.kBrushless);
  SparkMax impulMotor = new SparkMax(16,MotorType.kBrushless);

  private final RelativeEncoder shooterEncoder = ShooterMotor.getEncoder();
  // Comentar se o Neo For Utilizado (Deixar Controle ClosedLoop Interno) //
  private PIDController shooterPID = new PIDController(0.0005, 0, 0);
  private SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(0.2, 0.12);
  
  private static final double RPM_TOLERANCE = 75; // ±75 RPM
private static final double STABLE_TIME = 0.2; // segundos

  private final double g = 9.81; // gravidade

  private final Timer rpmStableTimer = new Timer();
private boolean wasAtSpeed = false;

  private SwerveSub swerve;

  public ShooterSub(SwerveSub swerve) {
    this.swerve = swerve;

    // MotorConfig//
    
    final SparkMaxConfig Kcoast = new SparkMaxConfig();
  /* ///////////// Se o NEO for Utilizado /////////////////
    SparkMaxConfig shooterConfig = new SparkMaxConfig();

    shooterConfig
      .idleMode(IdleMode.kCoast)
      .closedLoop
        .feedbackSensor(FeedbackSensor.kRelativeEncoder)
        .pid(0.0005, 0.0, 0.0)
        .velocityFF(0.00018); 
    
    ShooterMotor.configure(
      shooterConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    */
    
    
      Kcoast.idleMode(SparkBaseConfig.IdleMode.kCoast);

      Kcoast.openLoopRampRate(0);

      ShooterMotor.configure(Kcoast, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      impulMotor.configure(Kcoast, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }

  public void DescobrirKV(){
    ShooterMotor.set(0.4);
  }

  public void StopShooter(){
    ShooterMotor.set(0);
    impulMotor.set(0);

    rpmStableTimer.stop();
    rpmStableTimer.reset();
    wasAtSpeed = false;
  }

  public boolean atTargetRPM(double targetRPM) {
    double currentRPM = shooterEncoder.getVelocity();
    double error = Math.abs(currentRPM - targetRPM);
  
    boolean atSpeed = error <= RPM_TOLERANCE;
  
    if (atSpeed) {
      if (!wasAtSpeed) {
        rpmStableTimer.restart();
        wasAtSpeed = true;
      }
    } else {
      rpmStableTimer.stop();
      rpmStableTimer.reset();
      wasAtSpeed = false;
    }
  
    return wasAtSpeed && rpmStableTimer.hasElapsed(STABLE_TIME);
  }
  
// Lembrar de Comentar este metodo se for utilizar o NEO com controle de velocidade interno //
  private double rpmToRadPerSec(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0;
  }

  public boolean HasTarget(){
   return  swerve.HasTarget();
  }

  public void shoot(double targetRPM) {
    if (targetRPM <= 0) {
      StopShooter();
      return;
    }

    double currentRPM = shooterEncoder.getVelocity();
  
    double pidOutput = shooterPID.calculate(currentRPM, targetRPM);
  
    double targetRadPerSec = rpmToRadPerSec(targetRPM);
    double feedforwardVolts = shooterFeedforward.calculate(targetRadPerSec);
  
    double outputVolts = MathUtil.clamp(pidOutput + feedforwardVolts, -12.0, 12.0);
  
    ShooterMotor.setVoltage(outputVolts);
   if (atTargetRPM(targetRPM)){
    impulMotor.set(0.6);
   } else {
    impulMotor.set(0.0);
   }

     /*  ///////////////// SE O NEO FOR UTILIZADO /////////////////
   * public void shoot(double targetRPM) {
  if (targetRPM <= 0) {
    StopShooter();
    return;
  }

  ShooterMotor.getClosedLoopController()
      .setReference(targetRPM, ControlType.kVelocity);

  if (atTargetRPM(targetRPM)) {
    impulMotor.set(0.6);
  } else {
    impulMotor.set(0.0);
  }
}

   */
  }

  public double getRPMFromDistance(double distanceMeters) {
    double shooterHeight = 0.54; // altura do shooter (m)
    double targetHeight = 1.82;  // m (hub 2022)
    double deltaH = targetHeight - shooterHeight; // 2.00 m

    double angleDeg = 45.0; // ângulo fixo (exemplo)
    double angleRad = Math.toRadians(angleDeg);

    double r = 0.05; // raio da roda do shooter (m)
    double x = distanceMeters;

    // Denominador: 2 * cos^2(angle) * ( x * tan(angle) - deltaH )
    double cos2 = Math.cos(angleRad) * Math.cos(angleRad);
    double inner = x * Math.tan(angleRad) - deltaH;
    double denom = 2.0 * cos2 * inner;

    // Se denom <= 0, não é possivel o arremessar com esse ângulo.
    if (denom <= 0.0) {
        return 0.0;
    }

    double numerator = g * x * x;
    double v = Math.sqrt(numerator / denom); // velocidade inicial necessária (m/s)

    double wheelCircumference = 2.0 * Math.PI * r; // m/rev
    double revPerSec = v / wheelCircumference;
    double rpm = revPerSec * 60.0;

    // limites práticos
    rpm = MathUtil.clamp(rpm, 0.0, 5000);

    return rpm;

  }  

  public double getDistanceToTag() {
    double cameraHeight = 0.6;
    double targetHeight = 2.64;   //2.64 metros (hub 2022)
    double cameraAngle = 20;  
    double ty = swerve.getTy();

    return (targetHeight - cameraHeight) /
          Math.tan(Math.toRadians(cameraAngle + ty));
}
  
  @Override
  public void periodic() {
  }
}
