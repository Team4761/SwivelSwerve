package frc.robot.Subsystems;

import frc.robot.Subsystems.SwerveModuleTalon;
import frc.robot.Subsystems.SwerveModuleNeo;

import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveDrive extends SubsystemBase {
    // inches rn
    private static final SwerveDrive INSTANCE = new SwerveDrive(new Translation2d(-12.25, 12.25), new Translation2d(12.25, 12.25), new Translation2d(-12.25, -12.25), new Translation2d(12.25, -12.25));

    SwerveModuleState[] targetStates = new SwerveModuleState[4];

    // motors offset in degrees
    private SwerveModuleNeo m_frontLeftModule  = new SwerveModuleNeo(3 , 1 , 2 , -88, -1.0,  1.0);
    private SwerveModuleNeo m_frontRightModule = new SwerveModuleNeo(4 , 6 , 4 , -9, -1.0, -1.0);
    private SwerveModuleNeo m_backLeftModule   = new SwerveModuleNeo(7 , 8 ,3 , 115, -1.0, -1.0);
    private SwerveModuleNeo m_backRightModule  = new SwerveModuleNeo(5 , 2 , 1 , -47, -1.0,  1.0);

    //private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private SwerveDriveKinematics m_kinematics;

    private SwerveDriveOdometry m_odometry;
    private Pose2d m_pose;


    // positions of the wheels relative to center (meters?)
    public SwerveDrive (Translation2d fL, Translation2d fR, Translation2d bL, Translation2d bR) {
        m_kinematics = new SwerveDriveKinematics(fL, fR, bL, bR);
    }
    
    public static SwerveDrive getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {

        
        // update pose
        //crashes code because of course it does
        /*m_pose = m_odometry.update(
            //m_gyro.getRotation2d(),
            new Rotation2d(),
            new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(), 
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(), 
                m_backRightModule.getPosition()
        });*/
        
        SmartDashboard.putNumber("Front Left Rot", m_frontLeftModule.getRotation().getDegrees());
        SmartDashboard.putNumber("Front Right Rot", m_frontRightModule.getRotation().getDegrees());
        SmartDashboard.putNumber("Back Left Rot", m_backLeftModule.getRotation().getDegrees());
        SmartDashboard.putNumber("Back Right Rot", m_backRightModule.getRotation().getDegrees());

        
        SmartDashboard.putNumber("Front Left Target", targetStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Front Right Target", targetStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Back Left Target", targetStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Back Right Target", targetStates[3].angle.getDegrees());

        // set the stuff
        m_frontLeftModule .setTargetState(targetStates[0]);
        m_frontRightModule.setTargetState(targetStates[1]);
        m_backLeftModule  .setTargetState(targetStates[2]);
        m_backRightModule .setTargetState(targetStates[3]);

        
        // temp debug stuff
        //m_frontLeftModule .setSpeeds(0, 0.2);
        //m_frontRightModule.setSpeeds(0, 0.2);
        //m_backLeftModule  .setSpeeds(0, 0.2);
        //m_backRightModule .setSpeeds(0, 0.2);


        m_frontLeftModule.go();
        m_frontRightModule.go();
        m_backLeftModule.go();
        m_backRightModule.go();



        //System.out.println("Back Left target/measure: "+targetStates[2].angle.getDegrees()+" | "+m_backLeftModule.getRotation().getDegrees());
        //System.out.println("Ecoders: "+m_frontLeftModule.getRotation().getDegrees()+", "+m_frontRightModule.getRotation().getDegrees()+", "+m_backLeftModule.getRotation().getDegrees()+", "+m_backRightModule.getRotation().getDegrees());
    }
    
    // drives
    // try applying acceleration cap to inputs in drives instead of on wheels

    // field oriented, m/s, m/s, rad/s or something, i think x is forward
    public void swerveDriveF(double speedX, double speedY, double speedRot) {
        targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRot, /*m_gyro.getRotation2d()*/new Rotation2d()));

    }
    // robot oriented, m/s, m/s, rad/s or something
    public void swerveDriveR(double speed, double strafe, double speedRot) {

        targetStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(speed, strafe, 0/*speedRot 0 cause broke */));
    }
    // car, m/s, degrees    
    public void carDrive(double speed, double turn) {
        turn = MathUtil.clamp(turn, -90, 90);

        // do the optimize thing
        
        targetStates[0] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[1] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[2] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[3] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));

    }

    // stuff
    // degrees
    public double getGyroAngle() {
        return 0;//m_gyro.getAngle();
    }

    public void stop() {
        m_frontLeftModule.setSpeeds(0, 0);
        m_frontRightModule.setSpeeds(0, 0);
        m_backLeftModule.setSpeeds(0, 0);
        m_backRightModule.setSpeeds(0, 0);
    }
}
