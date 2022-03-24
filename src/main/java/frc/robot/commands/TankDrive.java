// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.



package frc.robot.commands;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
    

/**
 *
 */
public class TankDrive extends CommandBase {

    
        private final DriveTrain m_driveTrain;
        private Joystick leftJoystick;
        private Joystick rightJoystick;
        private double driveScale;
    

    public TankDrive(DriveTrain subsystem, Joystick left, Joystick right) {
        
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);
        leftJoystick = left;
        rightJoystick = right;
      
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveScale = ((1+ leftJoystick.getRawAxis(3)) * .5);
        //m_driveTrain.getDifferentialDrive().tankDrive(leftJoystick.getY() * -Constants.DriveSpeed * driveScale, rightJoystick.getY() * Constants.DriveSpeed * driveScale);
     //double turningValue = (Constants.kAngleSetPoint -RobotContainer.gyro.getAngle()) * Constants.kP;
       // turningValue = Math.copySign(turningValue, leftJoystick.getY(), rightJoystick.getY());
       m_driveTrain.getDifferentialDrive().tankDrive(leftJoystick.getY() * -Constants.DriveSpeed * Constants.driveDirection , rightJoystick.getY() * Constants.DriveSpeed * Constants.driveDirection);
    System.out.println(RobotContainer.frontLeftDriveMotor2.getSelectedSensorPosition());
    }

  

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    m_driveTrain.getDifferentialDrive().tankDrive(0, 0);

}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        
        return false;
    }
}
