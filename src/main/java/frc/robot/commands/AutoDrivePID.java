// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDrivePID extends PIDCommand {
  /** Creates a new AutoDrivePID. */
  
  
  public AutoDrivePID(DriveTrain m_DriveTrain) {
    
    super(
        // The controller that the command will use
        new PIDController(Constants.DrivePIDkd, Constants.DrivePIDki, Constants.DrivePIDkp),
        // This should return the measurement
        () ->  (GetPosition.GettingMotorPosition()), 
        // This should return the setpoint (can also be a constant)
        Constants.autoDriveDistance,
        // This uses the output
        output -> {
          m_DriveTrain.getDifferentialDrive().tankDrive(output* 0.01, -(output * 0.01));
          System.out.println(GetPosition.GettingMotorPosition());
        },  
        m_DriveTrain);
        
          // Use the output here
        getController().setTolerance(Constants.DrivePIDTolerance);

        
        
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  
  
  
  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / Constants.kCountsPerRev;
    double wheelRotations = motorRotations / Constants.kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
    return positionMeters;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (GetPosition.GettingMotorPosition() == Constants.autoDriveDistance ) {
    //   return true;
    // } else return false;
    return getController().atSetpoint();
  }
}

