package frc.robot.Subsystems;

import frc.robot.MathStuff;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveModuleNeo extends SubsystemBase{
    private CANSparkMax drive;
    private CANSparkMax steer;
    private CANCoder encoder;
    private double offset;

    private double dM;
    private double sM;

    // m/s, rotation2d
    private SwerveModuleState targetState = new SwerveModuleState();

    // pass in a ff+pid object or something, o is offset in radians
    public SwerveModuleNeo(int driveID, int steerID, int encoderID, double o, double driveMult, double steerMult) {
        drive = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
        steer = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new CANCoder(encoderID);

        offset = o;


        // multiplier prob just for reversing
        dM = driveMult;
        sM = steerMult;
    }

    public void go() {
        //System.out.println("speed: "+targetState.speedMetersPerSecond);
        // get to the set positions 
        drive.set(targetState.speedMetersPerSecond/2.0*dM);
            
        //System.out.println(targetState.angle.getDegrees()+", "+getRotation().getDegrees()+", "+sM);
        double steerA = MathStuff.subtract(targetState.angle, getRotation()).getRotations()*sM*0.8;
        double steerB = Math.signum(steerA)*0.01;
        steer.set(steerA+steerB);

        //if()
    }

    public void setSpeeds(double d, double s) {
        drive.set(d*dM);
        steer.set(s*sM);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drive.getEncoder().getPosition(), 
            new Rotation2d(encoder.getAbsolutePosition() * 0.0174533) // 2048 ticks to radians is 2pi/2048
        );
    }
    public Rotation2d getRotation() {
        return new Rotation2d((encoder.getAbsolutePosition() + offset) * 0.0174533);
    }

    // -1 to 1
    /*public void setSpeeds(double driveSpeed, double turnSpeed) {

        // check controlmodes for maybe useful or cool stuff
        drive.set(TalonFXControlMode.PercentOutput, driveSpeed);
        drive.set(TalonFXControlMode.PercentOutput, turnSpeed);
    }*/
    
    // default use optimization
    public void setTargetState(double metersPerSecond, Rotation2d angle) {
        targetState = new SwerveModuleState(metersPerSecond, angle);
        SwerveModuleState.optimize(targetState, getRotation());
    }
    public void setTargetState(double metersPerSecond, Rotation2d angle, boolean optimize) {
        targetState = new SwerveModuleState(metersPerSecond, angle);
        if(optimize) SwerveModuleState.optimize(targetState, getRotation());
    }
    public void setTargetState(SwerveModuleState target) {
        targetState = target;
    }
}
