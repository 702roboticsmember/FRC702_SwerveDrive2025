package frc.robot.commands;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class CalculateCommand extends Command{
    private ShooterSubsystem s_ShooterSubsystem;
    private double distance;
    private double angle = 45;
    private double hieght = 1.2;//feet

    public CalculateCommand(ShooterSubsystem s_ShooterSubsystem, double distance){
        addRequirements(s_ShooterSubsystem);
        this.distance = distance;//feet
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      
      double Velocity = Math.sqrt((16 * distance * distance)/ (hieght - distance));
      
      new SequentialCommandGroup(
            new InstantCommand(() -> s_ShooterSubsystem.setVelocity(Velocity)),
            new WaitCommand(2),
            new InstantCommand(() -> s_ShooterSubsystem.setVelocity(0))            
        );
    }

    @Override
    public void end(boolean interrupted) {
        s_ShooterSubsystem.setVelocity(0);
    }

    @Override
    public boolean isFinished() {
    return false;
    }

}
