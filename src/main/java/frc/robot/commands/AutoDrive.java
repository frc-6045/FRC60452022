// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;


public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  private final DriveTrain m_driveTrain;
  double leftSpeed;
  double rightSpeed;
  double time;
  double runTime;
  
  public AutoDrive(DriveTrain subsystem, double leftSpeed, double rightSpeed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    this.time = time;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runTime = Timer.getMatchTime();
  m_driveTrain.getDifferentialDrive().tankDrive(leftSpeed, rightSpeed);
 }
   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.getDifferentialDrive().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  runTime >= time;
  }
}