package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;
import org.littletonrobotics.junction.Logger;

public class TesteSwerveMotors extends Command {
  SwerveSub swerve;
  private int step = 0;
  private double stepStartTime;

  private static final double Test_Speed = 0.2;
  private static final double Test_Time = 1.0;


  public TesteSwerveMotors(SwerveSub swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    step = 0;
    stepStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double elapsed = Timer.getFPGATimestamp() - stepStartTime;

    if (elapsed >= Test_Time) {
      step++;
      stepStartTime = Timer.getFPGATimestamp();
    }
    swerve.stopModules();

    switch (step) {
      case 0 -> swerve.testDriveMotor(0, Test_Speed, "FL Drive");
      case 1 -> swerve.testSteerMotor(0, Test_Speed, "FL Steer");

      case 2 -> swerve.testDriveMotor(1, Test_Speed, "FR Drive");
      case 3 -> swerve.testSteerMotor(1, Test_Speed, "FR Steer");

      case 4 -> swerve.testDriveMotor(2, Test_Speed, "BL Drive");
      case 5 -> swerve.testSteerMotor(2, Test_Speed, "BL Steer");

      case 6 -> swerve.testDriveMotor(3, Test_Speed, "BR Drive");
      case 7 -> swerve.testSteerMotor(3, Test_Speed, "BR Steer");
    }

    String name = switch (step) {
      case 0 -> "FL Drive";
      case 1 -> "FL Steer";
      case 2 -> "FR Drive";
      case 3 -> "FR Steer";
      case 4 -> "BL Drive";
      case 5 -> "BL Steer";
      case 6 -> "BR Drive";
      case 7 -> "BR Steer";
      default -> "Unknown";
    };

    int moduleIndex = step/2;
    if (moduleIndex < 0 || moduleIndex >= 4) {
      return;
    }
    swerve.logDriveMotorInfo(moduleIndex, name);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return step > 7;
  }
}
