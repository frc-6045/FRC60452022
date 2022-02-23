// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoTurn extends CommandBase {
  private final DriveTrain m_driveTrain;
  private double speed;

  
  /** Creates a new AutoTurn. */
  public AutoTurn(DriveTrain subsystem) {
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turningValue = (Constants.kAngleSetPoint -RobotContainer.gyro.getAngle()) * Constants.kP;
        turningValue = Math.copySign(turningValue, speed);
      m_driveTrain.getDifferentialDrive().arcadeDrive(speed, turningValue);
      System.out.println(RobotContainer.gyro.getAngle()); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}