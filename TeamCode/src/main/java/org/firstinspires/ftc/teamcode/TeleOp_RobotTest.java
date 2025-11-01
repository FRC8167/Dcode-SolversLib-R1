package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.SetIntake;
import org.firstinspires.ftc.teamcode.Commands.ToggleIntakeCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


//@Disabled
@TeleOp(name="TeleOp_RobotTest")
public class TeleOp_RobotTest extends CommandOpMode {

    public GamepadEx driver;
    public GamepadEx operator;
    public ElapsedTime timer;

    private final Robot robot = Robot.getInstance();

    private final Pose startPose = new Pose(24, 24, Math.toRadians(90)); // Test
    private Pose autoEndPose = new Pose(0, 0, 0);


    @Override
    public void initialize() {
        // Must have for all opModes
        Robot.OP_MODE_TYPE = Robot.OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        //Initialize the robot
        try {
            robot.init(hardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        /* Create Button Bindings */
        Button intakeToggle  = new GamepadButton(driver, GamepadKeys.Button.A);
        Button intakeReverse = new GamepadButton(driver, GamepadKeys.Button.B);
        Button intakePassive = new GamepadButton(driver, GamepadKeys.Button.X);

        intakeToggle.whenPressed(new ToggleIntakeCommand(robot.intake));
        intakeReverse.whileHeld( new SetIntake(robot.intake, Intake.MotorState.REVERSE));
        intakePassive.whileHeld( new SetIntake(robot.intake, Intake.MotorState.PASSIVE));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed (new InstantCommand(robot.drive::enableSnailDrive))
                .whenReleased(new InstantCommand(robot.drive::disableSnailDrive));


        robot.follower = Constants.createFollower(hardwareMap);
        robot.follower.setStartingPose(startPose);


//        if (gamepad1.right_bumper) {
            schedule(new DriveCommand(robot.drive, gamepad1));
//        schedule(new SetIntake(robot.intake, Intake.MotorState.FORWARD));
//        }

        }


    @Override
    public void run() {
        super.run();
        robot.follower.update();
        autoEndPose = robot.follower.getPose();

        telemetry.addData("Intake State", robot.intake.motorState);
        telemetry.addData("autoEndPose", autoEndPose.toString());
        telemetry.addData("FollowerX", Math.round(robot.follower.getPose().getX()*100)/100.0);
        telemetry.addData("FollowerY", Math.round(robot.follower.getPose().getY()*100)/100.0);
        telemetry.addData("FollowerH", Math.round(Math.toDegrees(robot.follower.getPose().getHeading())*100)/100.0);

        telemetry.addLine();
        telemetry.addData("Cmd Auth", "%.2f", robot.drive.getControlAuthority());

        telemetry.update();
    }


    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }


    public Pose getAutoEndPose() {
        return autoEndPose;
    }


}

