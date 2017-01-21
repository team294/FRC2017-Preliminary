package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
//import org.usfirst.frc.team294.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team294.robot.commands.DriveWithJoysticks;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Teh Drivetrain
 */
public class DriveTrain extends Subsystem {

    // Drive Train hardware
	private final CANTalon leftMotor1 = new CANTalon(RobotMap.driveTrainLeftMotor1);
    private final CANTalon leftMotor2 = new CANTalon(RobotMap.driveTrainLeftMotor2);
    //private final CANTalon leftMotor3 = new CANTalon(RobotMap.driveTrainLeftMotor3);
    private final CANTalon rightMotor1 = new CANTalon(RobotMap.driveTrainRightMotor1);
    private final CANTalon rightMotor2 = new CANTalon(RobotMap.driveTrainRightMotor2);
    //private final CANTalon rightMotor3 = new CANTalon(RobotMap.driveTrainRightMotor3);
    private final RobotDrive robotDrive = new RobotDrive(leftMotor2, rightMotor2);

    public DriveTrain() {
    	super();
    	
    	leftMotor2.reverseSensor(true);
    	
    	leftMotor1.changeControlMode(TalonControlMode.Follower);
    	//leftMotor3.changeControlMode(TalonControlMode.Follower);
        rightMotor1.changeControlMode(TalonControlMode.Follower);
        //rightMotor3.changeControlMode(TalonControlMode.Follower);
        leftMotor1.set(leftMotor2.getDeviceID());
        //leftMotor3.set(leftMotor2.getDeviceID());
        rightMotor1.set(rightMotor2.getDeviceID());
        //rightMotor3.set(rightMotor2.getDeviceID());
    }

    /**
     * Set the drive controller to use power settings, instead of using the
     * encoder PID controller.
     */
    public void setDriveControlByPower() {
        leftMotor2.changeControlMode(TalonControlMode.PercentVbus);    	
        rightMotor2.changeControlMode(TalonControlMode.PercentVbus);
        leftMotor2.configPeakOutputVoltage(+12.0f, -12.0f);
        rightMotor2.configPeakOutputVoltage(+12.0f, -12.0f);
        robotDrive.setSafetyEnabled(true);
    }
    
    /**
     * Use this in the execute method of a DriveWithJoysticks command.  <p>
     * <b>NOTE:</b> Be sure to call setDriveControlByPower() in the initialize method.
     * @param leftStick Left joystick
     * @param rightStick Right joystick
     */
    public void driveWithJoystick(Joystick leftStick, Joystick rightStick) {
        leftMotor2.clearStickyFaults();
        rightMotor2.clearStickyFaults();
    	robotDrive.tankDrive(leftStick, rightStick);
    }
    
    /**
     * Stop the drive train motors
     */
	public void stop() {
		setDriveControlByPower();
		robotDrive.drive(0, 0);
	}
    
	/**
	 * Drive the robot straight forward
	 * @param speed +1 to -1, + = forward, - = backward
	 */
	public void driveForward(double speed) {
		setDriveControlByPower();
		robotDrive.drive(-speed, 0);
	}

	/**
	 * Drive the robot straight backward
	 * @param speed +1 to -1, + = backward, - = forward
	 */
	public void driveBackward(double speed) {
		setDriveControlByPower();
		robotDrive.drive(speed, 0);
	}
	
	/**
	 * Drive the robot at an angle
	 * @param speed +1 to -1, + = backward, - = forward
	 * @param curve +1 to -1, >0 = left, <0 = right
	 */
	public void driveAtAngle(double speed, double curve) {
		setDriveControlByPower();
		robotDrive.drive(-speed, curve);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithJoysticks());
    }
    
    /**
     * Logs the talon output to a file
     */
	public void logTalonStatus() {
		Robot.log.writeLog(
				"Motors:  Left Motor 2 (Main)-- TempC " + leftMotor2.getTemperature() +
				" Set " + leftMotor2.getSetpoint() + 
				" BusVolt " + leftMotor2.getBusVoltage() + 
				" OutVolt " + leftMotor2.getOutputVoltage() + 
				" OutCur " + leftMotor2.getOutputCurrent() + 
				" PulsePos " + leftMotor2.getPulseWidthPosition() + 
				" PulseVel " + leftMotor2.getPulseWidthVelocity() + 
				" PulseRF " + leftMotor2.getPulseWidthRiseToFallUs() + 
				" PulseRR " + leftMotor2.getPulseWidthRiseToRiseUs() + 
				" Get " + leftMotor2.get() +
				
				" Left Motor 1 (Follower)-- TempC " + leftMotor1.getTemperature() + 
				" Set " + leftMotor1.getSetpoint() + 
				" BusVolt " + leftMotor1.getBusVoltage() + 
				" OutVolt " + leftMotor1.getOutputVoltage() + 
				" OutCur " + leftMotor1.getOutputCurrent() + 
				" PulsePos " + leftMotor1.getPulseWidthPosition() + 
				" PulseVel " + leftMotor1.getPulseWidthVelocity() + 
				" PulseRF " + leftMotor1.getPulseWidthRiseToFallUs() + 
				" PulseRR " + leftMotor1.getPulseWidthRiseToRiseUs() + 
				" Get " + leftMotor1.get() +
				
				/* " Left Motor 3 (Follower)-- TempC " + leftMotor1.getTemperature() + 
				" Set " + leftMotor3.getSetpoint() + 
				" BusVolt " + leftMotor3.getBusVoltage() + 
				" OutVolt " + leftMotor3.getOutputVoltage() + 
				" OutCur " + leftMotor3.getOutputCurrent() + 
				" PulsePos " + leftMotor3.getPulseWidthPosition() + 
				" PulseVel " + leftMotor3.getPulseWidthVelocity() + 
				" PulseRF " + leftMotor3.getPulseWidthRiseToFallUs() + 
				" PulseRR " + leftMotor3.getPulseWidthRiseToRiseUs() + 
				" Get " + leftMotor3.get() +
				*/
				
				" Right Motor 2 (Main)-- TempC " + rightMotor2.getTemperature() + 
				" Set " + rightMotor2.getSetpoint() + 
				" BusVolt " + rightMotor2.getBusVoltage() + 
				" OutVolt " + rightMotor2.getOutputVoltage() + 
				" OutCur " + rightMotor2.getOutputCurrent() + 
				" PulsePos " + rightMotor2.getPulseWidthPosition() + 
				" PulseVel " + rightMotor2.getPulseWidthVelocity() + 
				" PulseRF " + rightMotor2.getPulseWidthRiseToFallUs() + 
				" PulseRR " + rightMotor2.getPulseWidthRiseToRiseUs() + 
				" Get " + rightMotor2.get() +
				
				" Right Motor 1 (Follower)-- TempC " + rightMotor1.getTemperature() + 
				" Set " + rightMotor1.getSetpoint() + 
				" BusVolt " + rightMotor1.getBusVoltage() + 
				" OutVolt " + rightMotor1.getOutputVoltage() + 
				" OutCur " + rightMotor1.getOutputCurrent() + 
				" PulsePos " + rightMotor1.getPulseWidthPosition() + 
				" PulseVel " + rightMotor1.getPulseWidthVelocity() + 
				" PulseRF " + rightMotor1.getPulseWidthRiseToFallUs() + 
				" PulseRR " + rightMotor1.getPulseWidthRiseToRiseUs() + 
				" Get " + rightMotor1.get()
				
				/*+ " Right Motor 3 (Follower)-- TempC " + rightMotor3.getTemperature() + 
				" Set " + rightMotor3.getSetpoint() + 
				" BusVolt " + rightMotor3.getBusVoltage() + 
				" OutVolt " + rightMotor3.getOutputVoltage() + 
				" OutCur " + rightMotor3.getOutputCurrent() + 
				" PulsePos " + rightMotor3.getPulseWidthPosition() + 
				" PulseVel " + rightMotor3.getPulseWidthVelocity() + 
				" PulseRF " + rightMotor3.getPulseWidthRiseToFallUs() + 
				" PulseRR " + rightMotor3.getPulseWidthRiseToRiseUs() + 
				" Get " + rightMotor3.get()
				*/
				);
	}
}
