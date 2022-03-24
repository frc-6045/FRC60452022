// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class AutoDriveGyro extends CommandBase {
  /** Creates a new AutoDrive. */
  private final DriveTrain m_driveTrain;
  private double speed;

  
  public AutoDriveGyro(DriveTrain subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
    this.speed = speed;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.getDifferentialDrive().arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //m_driveTrain.getDifferentialDrive().arcadeDrive(speed, 0);  
  double turningValue = (Constants.kAngleSetPoint - RobotContainer.gyro.getAngle()) * Constants.kP;
  //turningValue = Math.copySign(turningValue, speed);
  m_driveTrain.getDifferentialDrive().arcadeDrive(speed, turningValue); 
  System.out.println((RobotContainer.gyro.getAngle())); 

 }
   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.getDifferentialDrive().arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
