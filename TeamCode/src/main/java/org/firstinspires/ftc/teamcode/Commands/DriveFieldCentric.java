package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Sensor_IMU;

public class DriveFieldCentric extends CommandBase {

    private final MecanumDrive drive;
    private final Gamepad gamepad;
    private final Sensor_IMU imu;


    public DriveFieldCentric(MecanumDrive drive, Gamepad gamepad, Sensor_IMU imu) {
        this.drive = drive;
        this.gamepad = gamepad;
        this.imu = imu;
        addRequirements(drive);
    }


    @Override
    public void execute() {


        // Call subsystem drive method
        drive.driveFieldRelative(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, imu.currentHeadingDeg());
    }


    @Override
    public boolean isFinished() {
        return false; // TeleOp runs continuously
    }


    @Override
    public void end(boolean interrupted) {
        drive.drive(0,0,0); // stop motors
    }
}
