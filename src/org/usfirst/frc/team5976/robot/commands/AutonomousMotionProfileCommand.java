package org.usfirst.frc.team5976.robot.commands;

import org.usfirst.frc.team5976.robot.RobotMap;
import org.usfirst.frc.team5976.robot.subsystems.DriveTrain;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;

public class AutonomousMotionProfileCommand extends Command {

	// CANTalons
	private CANTalon leftMaster, leftSlave, rightMaster, rightSlave;
	private MotionProfileController leftMasterController, rightMasterController;
	// Might matter if left is inverted or not
	private boolean isInverted = true;

	private int counter;

	public AutonomousMotionProfileCommand(DriveTrain driveTrain) {
		leftMaster = driveTrain.getLeftMaster();
		leftSlave = driveTrain.getLeftSlave();
		rightMaster = driveTrain.getRightMaster();
		rightSlave = driveTrain.getRightSlave();
		leftMasterController = new MotionProfileController(leftMaster, -1);
		rightMasterController = new MotionProfileController(rightMaster, 1);

	}

	protected void initialize() {
		initMaster(leftMasterController);
		//leftMaster.setInverted(isInverted);
		initMaster(rightMasterController);
		initSlave(leftSlave, RobotMap.LEFT_MASTER);
		initSlave(rightSlave, RobotMap.RIGHT_MASTER);

		counter = 0;

		ReportHelper.report(leftMaster, this);
		ReportHelper.report(rightMaster, this);
		ReportHelper.report(leftSlave, this);
		ReportHelper.report(rightSlave, this);
	}

	@Override
	protected void execute() {

		leftMasterController.control();
		rightMasterController.control();

		if (counter % 10 == 0) {
			//System.out.println(counter + " " + leftMasterController.getSetValue().value + " "
					//+ rightMasterController.getSetValue().value);
			
		}
		//System.out.println(counter + " Data for left master");
		Instrumentation.process("L", leftMasterController.getStatus());
		//System.out.println(counter + " Data for right master");
		Instrumentation.process("R", rightMasterController.getStatus());
		counter++;

	}

	@Override
	protected boolean isFinished() {
		if (leftMasterController.isFinished())
			System.out.println("Left master controller is finished");
		if (rightMasterController.isFinished())
			System.out.println("Right master controller is finished");
		return leftMasterController.isFinished() && rightMasterController.isFinished();
	}

	private void initMaster(MotionProfileController controller) {
		controller.reset();
		controller.getTalon().setPosition(0);
		controller.getTalon().setEncPosition(0);
		controller.getTalon().clearMotionProfileTrajectories();
		controller.getTalon().changeControlMode(TalonControlMode.MotionProfile);
		controller.getTalon().setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
		controller.getTalon().enable();
		controller.startMotionProfile();
	}

	private void initSlave(CANTalon talon, int masterID) {
		talon.changeControlMode(CANTalon.TalonControlMode.Follower);
		talon.set(masterID);
		talon.enable();
	}

}
