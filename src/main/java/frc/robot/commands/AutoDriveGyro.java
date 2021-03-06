// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class AutoDriveGyro extends CommandBase {
  /** Creates a new AutoDrive. */
  private final DriveTrain m_driveTrain;
  private double m_speed;
  private double m_heading;
private final PIDController m_PIDHeading;
private double m_DistanceSetPoint;

  
  public AutoDriveGyro(DriveTrain subsystem, double speed, double distanceSetPoint, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_PIDHeading = new PIDController(0 , 0, 0);
    m_PIDHeading.setTolerance(1);
    m_PIDHeading.enableContinuousInput(-180, 180);
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
    m_speed = speed;
    m_heading = heading;
    m_DistanceSetPoint = distanceSetPoint;
  
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.getDifferentialDrive().arcadeDrive(0, 0);
    m_PIDHeading.setSetpoint(m_heading);
    m_driveTrain.resetGyro();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //m_driveTrain.getDifferentialDrive().arcadeDrive(speed, 0);  

  double turningValue = m_driveTrain.getHeading();
  double PIDOut = m_PIDHeading.calculate(turningValue);
  // turningValue = Math.copySign(turningValue, speed);


  m_driveTrain.getDifferentialDrive().arcadeDrive(m_speed, PIDOut);  
  System.out.println("Gyro value: " + m_driveTrain.getHeading());
  System.out.println();
  

 }
   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.getDifferentialDrive().arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return /*m_PIDHeading.atSetpoint() && */ Math.abs(m_driveTrain.get_Right_Encoder_inch()) >= m_DistanceSetPoint;
  }
}
