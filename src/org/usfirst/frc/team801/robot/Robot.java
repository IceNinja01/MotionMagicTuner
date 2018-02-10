/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the setSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.  
 *
 * Tweak the PID gains accordingly.
 */
package org.usfirst.frc.team801.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	TalonSRX _talon = new TalonSRX(0);
	XboxController _joy = new XboxController(0);
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	boolean _lastButton1 = false;
	double targetPositionRotations;
	private Preferences prefs;
	private double kf;
	private double kp;
	private double ki;
	private double kd;
	private double maxPosition;
	private double minPosition;
	private boolean _lastButton2;
	private double maxVel;
	private double accel;

	public void robotInit() {
		prefs = Preferences.getInstance();
		 /* set closed loop gains in slot0 */
        kf = prefs.getDouble("Kf", 0.2);
        kp = prefs.getDouble("Kp", 0.5);
        ki = prefs.getDouble("Ki", 0.00);
        kd = prefs.getDouble("Kd", 0.2);
    	//Setup for Motion Magic
    	maxVel = prefs.getInt("CrusieVelocity", 1); //RPSec input
    	maxVel *= (7.5*4096)/125;; //convert to native units
    	accel = prefs.getInt("Acceleration", 1); //RPSec input
    	accel *= (7.5*4096)/125;; //convert to native units
    	minPosition = prefs.getDouble("MinPosition", 0.00);
    	maxPosition = prefs.getDouble("MaxPosition", 0.0); //Rotations 12.5in per wheel rotation; 7.5 is equal to 1 full rotation
    	minPosition *= 7.5*4096/12.5;
    	maxPosition *= 7.5*4096/12.5;	               
		SmartDashboard.putData(Scheduler.getInstance()); //Displaying the Scheduler status
		/* choose the sensor and sensor direction */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* choose to ensure sensor is positive when output is positive */
		_talon.setSensorPhase(Constants.kSensorPhase);
		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
		_talon.setInverted(Constants.kMotorInvert);
		_talon.selectProfileSlot(0, 0);
		/* set the peak and nominal outputs, 12V means full */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(11, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-11, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);	

    	_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
    	
		
	}
	public void disabledPeriodic() {

	}
	public void teleopInit() {
        /* set closed loop gains in slot0 */
        kf = prefs.getDouble("Kf", 0.2);
        kp = prefs.getDouble("Kp", 0.2);
        ki = prefs.getDouble("Ki", 0.00);
        kd = prefs.getDouble("Kd", 0.2);
        _talon.config_kF(0, kf, 10);
        _talon.config_kP(0, kp, 10);
        _talon.config_kI(0, ki, 10);
        _talon.config_kD(0, kd, 10);
       	//Setup for Motion Magic
    	maxVel = prefs.getDouble("Cruise Velocity", 1.0); //RPSec input
    	maxVel *= (7.5*4096)/125; //convert to native units
    	accel = prefs.getDouble("Acceleration", 1.0); //RPSec input
    	accel *= (7.5*4096)/125; //convert to native units
    	_talon.configMotionCruiseVelocity((int) maxVel, Constants.kTimeoutMs);
    	_talon.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
    	minPosition = prefs.getDouble("Min Position", 0.00);
    	maxPosition = prefs.getDouble("Max Position", 0.0); //Rotations
    	minPosition *= 7.5*4096/12.5;
    	maxPosition *= 7.5*4096/12.5;
    	_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 

	}
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
	
		/* get gamepad axis */
		double leftYstick = _joy.getY(Hand.kLeft);
		double motorOutput = _talon.getMotorOutputPercent();
		boolean button1 = _joy.getRawButton(1);
		boolean button2 = _joy.getRawButton(2);
		/* deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* within 10% of zero */
			leftYstick = 0;

		}
		/* prepare line to print */
		_sb.append("\tout:");
		/* cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%"); /* perc */

		_sb.append("\tpos:");
		_sb.append(_talon.getSelectedSensorPosition(0));
		_sb.append("u"); /* units */

		/* on button1 press enter closed-loop mode on target position */
		if (_lastButton1 && button1) {
			/* Position mode - button just pressed */
	    	_talon.configMotionCruiseVelocity((int) maxVel, Constants.kTimeoutMs);
	    	_talon.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
			targetPositionRotations = maxPosition;
			_talon.set(ControlMode.MotionMagic, targetPositionRotations);

		}
		/* on button1 press enter closed-loop mode on target position */
		if (_lastButton2 && button2) {
			/* Position mode - button just pressed */
	    	_talon.configMotionCruiseVelocity((int) maxVel, Constants.kTimeoutMs);
	    	_talon.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = minPosition;
			_talon.set(ControlMode.MotionMagic, targetPositionRotations);

		}
//		else {
//			/* Percent voltage mode */
//			_talon.set(ControlMode.PercentOutput, leftYstick);
//		}
		/* if Talon is in position closed-loop, print some more info */
		if (_talon.getControlMode() == ControlMode.MotionMagic) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(0));
			_sb.append("u"); /* units */

			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations);
			_sb.append("u"); /* units */
		}
		/*
		 * print every ten loops, printing too much too fast is generally bad
		 * for performance
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}
		_sb.setLength(0);
		/* save button state for on press detect */
		_lastButton1 = button1;
		_lastButton2 = button2;
		SmartDashboard.putNumber("Error", _talon.getClosedLoopError(0));
		SmartDashboard.putNumber("Position", _talon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Position", _talon.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Current", _talon.getOutputCurrent());
		SmartDashboard.putNumber("Voltage", _talon.getMotorOutputVoltage());
		SmartDashboard.putNumber("k_p", kp);
		SmartDashboard.putNumber("k_f", kf);
		SmartDashboard.putNumber("k_i", ki);
		SmartDashboard.putNumber("k_d", kd);
	 }
}
