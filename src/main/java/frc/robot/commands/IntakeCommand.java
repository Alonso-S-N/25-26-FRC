
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;
public class IntakeCommand extends Command {
  private IntakeSub intake;
  private double setpoint;
  
    public IntakeCommand(IntakeSub intake,double setpoint) {
    this.intake = intake;
    this.setpoint = setpoint;

    addRequirements(intake);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intake.setIntakeAngle(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopPID();
  }

  @Override
  public boolean isFinished() {
    return intake.atAngle();
  }
}
