package frc.robot.commands;

import frc.robot.RobotContainer;

public class GetPosition {
    public static double GettingMotorPosition () {
        return RobotContainer.frontLeftDriveMotor2.getSelectedSensorPosition();
    }
    
}
