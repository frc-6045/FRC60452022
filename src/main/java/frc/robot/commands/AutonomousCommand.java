// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class AutonomousCommand extends SequentialCommandGroup {
     static  double timer = Timer.getMatchTime();

    public AutonomousCommand(DriveTrain drive, Intake intake, FlyWheel fly) {
        super ( //new AutoIntake(intake, Constants.intakeSpinSpeed, timer, 13, 12),
                new AutoDriveDistance(drive)
               /* new AutoIntake(intake, Constants.intakeSpinSpeed, 8.5, 7.5), 
                new AutoDrive(drive, Constants.autoDriveSpeed * -1, timer,7, 4.5), 
                new AutoScore(fly, Constants.dumpSpeed, timer, 4,2)*/);
    
    }
/*
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    } */
}
