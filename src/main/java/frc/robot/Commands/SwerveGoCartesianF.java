package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveDrive;

// field oriented, move relative to current position, forward should be -y, left +x
public class SwerveGoCartesianF extends CommandBase {
    private SwerveDrive m_swerve;
    private Translation2d target;
    private double Pvalue = 0;
    private double vLimit = 0.7;

    public SwerveGoCartesianF(SwerveDrive swerve, Translation2d trans) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        target = m_swerve.getPose().getTranslation().plus(trans);
    }
    // speedlimit 0.0-1.0 i think
    public SwerveGoCartesianF(SwerveDrive swerve, Translation2d trans, double speedLimit) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        target = m_swerve.getPose().getTranslation().plus(trans);
        vLimit = speedLimit;
    }

    
    @Override
    public void initialize() {
        
    }


    @Override
    public void execute() {
        Translation2d curTrans = m_swerve.getPose().getTranslation();
        // adjust pid off units
        Pvalue = Math.min(target.getDistance(curTrans)*0.25, 0.2);

        //correct forwards -y and left +y to 
        double strafeGo = -Pvalue*(target.getX()-curTrans.getX());
        double speedGo = -Pvalue*(target.getY()-curTrans.getY()); //forwards -y

        double hypoSpeed = Math.sqrt(strafeGo*strafeGo+speedGo*speedGo);
        if (hypoSpeed>vLimit) {
            strafeGo = strafeGo/hypoSpeed*vLimit;
            speedGo = speedGo/hypoSpeed*vLimit;
        }


        m_swerve.swerveDriveF(strafeGo, speedGo, 0);    
    }
    
    @Override
    public boolean isFinished() {
        if(Pvalue<0.01) return true;
        else return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerve.swerveDriveF(0, 0, 0);
    }
}
