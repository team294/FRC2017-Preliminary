package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;



/**
 *
 */
public class TurnToAngle extends Command {

	private double angle;
	private double speed;
	final private double MIN_POWER = 0.2;
	final private double MAX_POWER = 0.7;
	
    public TurnToAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	this.angle = angle;
    	this.speed = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (speed >= 1.0) speed = 1.0;
    	if (speed <= -1.0) speed = -1.0;
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }
    
    private double calculateSpeed(double angle) {
    	double power = 0;
    	if (angle >= 90) {
    		power = MAX_POWER;
    	} else {
    		power = MIN_POWER + angle*(MAX_POWER-MIN_POWER)/90;
    	}
    	if (angle <= 0) {
    		power = MIN_POWER*-1;
    	} else {
    		power = MAX_POWER*-1;
    	}
    	return power;
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}