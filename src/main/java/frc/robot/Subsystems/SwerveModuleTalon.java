package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleTalon extends SubsystemBase{
    private TalonFX drive;
    private TalonFX steer;

    // m/s, rotation2d
    private SwerveModuleState targetState = new SwerveModuleState();

    // pass in a ff+pid object or something
    public SwerveModuleTalon(int driveID, int steerID) {
        drive = new TalonFX(driveID);
        steer = new TalonFX(steerID);
    }

    @Override
    public void periodic() {
        // get to the set positions 
        drive.set(TalonFXControlMode.Velocity, targetState.speedMetersPerSecond);


        steer.set(TalonFXControlMode.Velocity, targetState.angle.minus(getRotation()).getRotations() * 5);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drive.getSensorCollection().getIntegratedSensorPosition(), 
            new Rotation2d(steer.getSensorCollection().getIntegratedSensorAbsolutePosition()*0.00306796157) // 2048 ticks to radians is 2pi/2048
        );
    }
    public Rotation2d getRotation() {
        return new Rotation2d(steer.getSensorCollection().getIntegratedSensorAbsolutePosition()*0.00306796157);
    }

    // -1 to 1
    /*public void setSpeeds(double driveSpeed, double turnSpeed) {

        // check controlmodes for maybe useful or cool stuff
        drive.set(TalonFXControlMode.PercentOutput, driveSpeed);
        drive.set(TalonFXControlMode.PercentOutput, turnSpeed);
    }*/
    public void setTargetState(double metersPerSecond, Rotation2d angle) {
        targetState = new SwerveModuleState(metersPerSecond, angle);
    }
    public void setTargetState(SwerveModuleState target) {
        targetState = target;
    }
}
