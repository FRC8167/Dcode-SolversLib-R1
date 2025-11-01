package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.hardware.SensorColor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.SubSystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Sensor_IMU;

import java.util.List;

public class Robot extends com.seattlesolvers.solverslib.command.Robot {

    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    public Telemetry telemetry;

    public enum OpModeType {
        AUTO,
        TELEOP
    }
    public static OpModeType OP_MODE_TYPE;

    static List<LynxModule> ctrlHubs;

    public Follower follower;
    public GoBildaPinpointDriver pinpoint;

    /* ******************************** SubSystems ******************************** */
    MecanumDrive drive;
    Intake intake;
//    public Shooter shooter;
//    public Vision vision;

    /* ********************************** Sensors ********************************** */
    public SensorColor colorSensor;
    public WebcamName webCam1;
//    public Limelight3A limelight;
    public Sensor_IMU imu;



    /**
     * Function to Initialize the Robot.  Identifies Hardware and Creates SubSystem Objects
     * @param hardwareMap   Robot hardware map
     * @throws InterruptedException Exception
     */
    public void init(HardwareMap hardwareMap) throws InterruptedException {

        // Hardware
        MotorEx driveMotorRF = new MotorEx(hardwareMap, "RightFront").setCachingTolerance(0.01);
        MotorEx driveMotorLF = new MotorEx(hardwareMap, "LeftFront").setCachingTolerance(0.01);
        MotorEx driveMotorLR = new MotorEx(hardwareMap, "LeftRear").setCachingTolerance(0.01);
        MotorEx driveMotorRR = new MotorEx(hardwareMap, "RightRear").setCachingTolerance(0.01);

        MotorEx intakeMotor = new MotorEx(hardwareMap, "Intake").setCachingTolerance(0.01);

//        MotorGroup shooterMotors = new MotorGroup(new MotorEx(hardwareMap, "leftShooterMotor").setCachingTolerance(0.01),
//                new MotorEx(hardwareMap, "rightShooterMotor").setCachingTolerance(0.01)
//        );
//        MotorEx.Encoder shooterEncoder = new MotorEx(hardwareMap, "rightShooterMotor").encoder;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(6.5, 0.0, DistanceUnit.INCH);  //TODO measure this
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH , 0.0, 0.0, AngleUnit.DEGREES,0.0));

        webCam1 = hardwareMap.get(WebcamName.class, "Webcam1");
        //limelight = hwMap.get(Limelight3A.class, "limelight");

        IMU imuSensor = hardwareMap.get(IMU.class, "imu");


        //Instantiate Subsystems
        drive  = new MecanumDrive(driveMotorLF, driveMotorLR, driveMotorRF, driveMotorRR);
        intake = new Intake(intakeMotor);
        imu    = Sensor_IMU.getInstance(imuSensor);
//        shooter = new Shooter(shooterMotors, shooterEncoder);
//        vision  = new Vision(webCam1);

        // Register Subsystems with the Command Scheduler
        register(drive, intake);

        if (OP_MODE_TYPE.equals(OpModeType.AUTO)) {
            initHasMovement();
        }

        ctrlHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : ctrlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }


    public void initHasMovement() {
        //TODO what goes here??
    }


}
