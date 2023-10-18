package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveDrive;

// field oriented
public class SwerveGoCartesianF extends CommandBase {
    private SwerveDrive m_swerve;

    public SwerveGoCartesianF(SwerveDrive swerve) {
        addRequirements(m_swerve);
    }

    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public void end(boolean interrupted) {

    }
}
