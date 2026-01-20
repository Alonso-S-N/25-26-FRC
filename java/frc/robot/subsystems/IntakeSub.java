// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  private SparkMax IntakeAng = new SparkMax(17,MotorType.kBrushless);
  private SparkMax IntakeRot = new SparkMax(18,MotorType.kBrushless);
   
  private final DutyCycleEncoder intakeAngleEncoder = new DutyCycleEncoder(0);

  private PIDController IntakeAngPid = new PIDController(0.02, 0, 0);

  private static final double MIN_ANGLE = 0.0;    // graus
  private static final double MAX_ANGLE = 110.0;  // graus

  public IntakeSub() {
    SparkMaxConfig intakeAngConfig = new SparkMaxConfig();
    SparkMaxConfig intakeRotConfig = new SparkMaxConfig();

    intakeRotConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    intakeAngConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    intakeAngConfig.openLoopRampRate(0);
    intakeRotConfig.openLoopRampRate(0);

    IntakeAng.configure(intakeAngConfig,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    ); 

    IntakeRot.configure(intakeRotConfig,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    );

    IntakeAngPid.setTolerance(2.0);

   intakeAngleEncoder.setDutyCycleRange(0.0, 1.0);
  }

  public void setIntakeAngle(double angleSetpoint){
    angleSetpoint = MathUtil.clamp(angleSetpoint, MIN_ANGLE, MAX_ANGLE);

    double absolutePos = intakeAngleEncoder.get();
    double currentAngle =
        MIN_ANGLE + absolutePos * (MAX_ANGLE - MIN_ANGLE);
    
    double output = IntakeAngPid.calculate(currentAngle, angleSetpoint);
    
    output = MathUtil.clamp(output, -0.5, 0.5);
    IntakeAng.set(output);
  }

  public boolean atAngle() {
    return IntakeAngPid.atSetpoint();
  }

  public void setIntakeRotSpeed(double Speed){
    IntakeRot.set(Speed);
  }

  @Override
  public void periodic() {
  }
}
