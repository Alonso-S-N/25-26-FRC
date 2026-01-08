
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSub;

public class ClimbCommand extends Command {
  private ClimbSub climb;
  private PIDController climbPID = new PIDController(0.001, 0, 0.0001); // Exemplo de valores de ganho (Ajustar)
  private ElevatorFeedforward ClimbFF = new ElevatorFeedforward(0.21, 0.81, 2.15,0); // Exemplo de valores de FF (Ajustar)
  private final double target;

  public ClimbCommand(ClimbSub climb, double target) {
    this.target = target;
    this.climb = climb;
    climbPID.reset();
    climbPID.setTolerance(0.09);
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbPID.reset();
  }

  @Override
  public void execute() {
    double output = climbPID.calculate(climb.getPosition(), target);

    double ffOutput = ClimbFF.calculate(getVelocityMetersPerSecond());

    output += ffOutput;

    if ((output > 0 && climb.atUpperLimit()) ||
    (output < 0 && climb.atLowerLimit())) {
    output = 0;
    }

    climb.setMotor(MathUtil.clamp(output, -0.5, 0.5)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.STOP();
  }
  public void safeButton(){
    climb.STOP();
    climbPID.reset();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbPID.atSetpoint();
  }

  public double getVelocityMetersPerSecond() {
    double motorRPM = climb.getVelocity();
    double motorRPS = motorRPM / 60.0;
    double shaftRPS = motorRPS / 100; // gear ratio
    return shaftRPS * 2 * Math.PI * 0.02; // raio do tambor
  }

  public double getPositionMeters() {
    double rotations = climb.getPosition();
    double shaftRotations = rotations / 100;
    return shaftRotations * 2 * Math.PI * 0.02;
  }
  
  
}
