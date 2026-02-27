package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  private SparkMax IntakeAng = new SparkMax(10,MotorType.kBrushless);
  private SparkMax IntakeRot = new SparkMax(9,MotorType.kBrushed);
   
  private final DutyCycleEncoder intakeAngleEncoder = new DutyCycleEncoder(0);

  private PIDController IntakeAngPid = new PIDController(0.17, 0.0, 0.0);

  private static final double MIN_POS = 0.071;
  private static final double MAX_POS = 0.75; //exemplo :p
   
  public IntakeSub() {
    SparkMaxConfig intakeAngConfig = new SparkMaxConfig();
    SparkMaxConfig intakeRotConfig = new SparkMaxConfig();

    intakeRotConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    intakeAngConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    intakeAngConfig.openLoopRampRate(0);
    intakeRotConfig.openLoopRampRate(0);

    IntakeAng.configure(intakeAngConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    ); 

    IntakeRot.configure(intakeRotConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

   intakeAngleEncoder.setDutyCycleRange(0.0, 1.0);

   IntakeAngPid.enableContinuousInput(0, 1);

   IntakeAngPid.setTolerance(0.01);
  }

  public void setIntakeAngle(double setpoint){
    setpoint = MathUtil.clamp(setpoint, MIN_POS, MAX_POS);

    double currentPos = intakeAngleEncoder.get();
    double output = IntakeAngPid.calculate(currentPos, setpoint);

    if ((currentPos < MIN_POS && output < 0) || (currentPos > MAX_POS && output > 0)) {
      output = 0;
    }
    
    output = MathUtil.clamp(output, -0.1, 0.1);
    IntakeAng.set(output);
    
  }

  public boolean atAngle() {
    return IntakeAngPid.atSetpoint();
  }
  
  public void stopPID() {
    IntakeAngPid.reset();
    IntakeAng.set(0);
  }

  public void setIntakeRotSpeed(double Speed){
    IntakeRot.set(Speed);
  }

  public void setSpeeds(){
    IntakeAng.set(0.2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DutyCycle",intakeAngleEncoder.get());
  }
}
