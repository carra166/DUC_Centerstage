package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.processors.ducProcessorBlueLeft;
import org.firstinspires.ftc.teamcode.processors.ducProcessorRedLeft;
import org.firstinspires.ftc.vision.VisionPortal;

/*
10430 AUTONOMOUS PROGRAM
Uses encoders
*/
@Autonomous
//@Disabled
public class AutoBlueLeft<myIMUparameters> extends LinearOpMode {

    int step = 0;
    boolean runOnce = true;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    Servo servoWrist;
    CRServo servoIntake;

    private IMU imu;
    private YawPitchRollAngles robotOrientation;
    private ducProcessorBlueLeft ducProcessor;
    private double duckPosition;

    InterpLUT armAngles;
    double kCos = 0.25;
    double tgtArmPower = 0;
    double armPosition = -20;

    double kp = 0.005;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");
        initializeMotors();

        armAngles = new InterpLUT();
        armAngles.add(-100, -41); //safety 1
        armAngles.add(0, -40); //init position
        armAngles.add(100, 0); //straight out (forward)
        armAngles.add(400, 90); //straight up
        armAngles.add(666, 180); //straight back
        armAngles.add(1000, 181); //safety 2
        armAngles.createLUT();

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

        ducProcessor = new ducProcessorBlueLeft();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(ducProcessor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}
        servoWrist.setPosition(0);

        while (!isStarted() && !isStopRequested()) {
            duckPosition = ducProcessor.getDuckPosition();
            telemetry.addData("DUCK POSITION", duckPosition);
            telemetry.update();
        }

        //start auto

        while (isStarted() && !isStopRequested()) {
            tgtArmPower = calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos, kp, armPosition);
            armMotor.setPower(tgtArmPower);
            armMotor2.setPower(-tgtArmPower);

            switch (step) {
                case 0:
                    if (runOnce) {
                        if (duckPosition == 2) {
                            linearMovement(0.2, (535 * 2) + 50, "forward");
                        } else {
                            linearMovement(0.2, (535 * 2) + 50, "forward");
                        }
                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 1:
                    if (runOnce) {
                        if (duckPosition == 2) {
                            servoIntake.setPower(-1);
                            sleep(1000);
                        } else if (duckPosition == 3) {
                            linearMovement(0.2, (480 * 2), "turnRight");
                        } else {
                            linearMovement(0.2, (480 * 2), "turnLeft");
                        }
                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 2:
                    if (runOnce) {
                        if (duckPosition == 2) {
                            servoIntake.setPower(0);
                        } else if (duckPosition == 3) {
                            linearMovement(0.2, 300, "backward");
                        } else {
                            linearMovement(0.2, 100, "strafeRight");
                        }
                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 3:
                    if (runOnce) {
                        if (duckPosition == 2) {
                            //come back to this putting pixel on board
                        } else if (duckPosition == 3) {
                            //inearMovement(0.2, (480 * 4), "turnRight");
                        } else {
                            linearMovement(0.2, 100, "forward");
                        }
                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 4:
                    if (runOnce) {
                        if (duckPosition == 2) {
                            //come back to this putting pixel on board
                        } else if (duckPosition == 3) {
                            servoIntake.setPower(-1);
                            sleep(5000);
                        } else {
                            servoIntake.setPower(-1);
                            sleep(1000);
                        }
                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 5:
                    if (runOnce) {
                        if (duckPosition == 2) {
                            //come back to this putting pixel on board
                        } else if (duckPosition == 3) {
                            servoIntake.setPower(0);
                        } else {
                            servoIntake.setPower(0);
                        }
                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 6:
                    if (runOnce) {
                        if (duckPosition == 2) {
                            //come back to this putting pixel on board
                        } else if (duckPosition == 3) {
                            //linearMovement(0.2, 300, "backward");
                        } else {
                            servoIntake.setPower(0);
                        }
                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                    break;


                case 7:
                    if (runOnce) {
                        if (duckPosition == 2) {
                            //come back to this putting pixel on board
                        } else if (duckPosition == 3) {
                            //servoWrist.setPosition(1);
                            //armPosition = 90;
                        } else {
                            servoIntake.setPower(0);
                        }
                        runOnce = false;
                    }
                    if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                        step++;
                        runOnce = true;
                    }
                    break;
            }
        }

        //linearMovement(1.0, 1440, "Test");

        telemetry.addData("a", "Whaha");
        telemetry.update();

    }

    private double inchesToEncoders(double inches) {
        return inches;
    }

    private void initializeMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM1");
        armMotor2 = hardwareMap.get(DcMotor.class, "ARM2");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoIntake = hardwareMap.get(CRServo.class,"intake");
        servoWrist = hardwareMap.get(Servo.class, "wrist");

    }

    private void linearMovement(double power, int distance, String type) {
        //POSITIVE VALUES GO FORWARD, NEGATIVE VALUES GO BACKWARD

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch (type) {

            case "backward":
                frontRight.setTargetPosition(-distance);
                frontLeft.setTargetPosition(distance);
                backRight.setTargetPosition(-distance);
                backLeft.setTargetPosition(distance);
            break;

            case "strafeLeft":
                frontRight.setTargetPosition(distance);
                frontLeft.setTargetPosition(distance);
                backRight.setTargetPosition(-distance);
                backLeft.setTargetPosition(-distance);
            break;

            case "strafeRight":
                frontRight.setTargetPosition(-distance);
                frontLeft.setTargetPosition(-distance);
                backRight.setTargetPosition(distance);
                backLeft.setTargetPosition(distance);
            break;

            case "turnLeft":
                frontRight.setTargetPosition(distance);
                frontLeft.setTargetPosition(distance);
                backRight.setTargetPosition(distance);
                backLeft.setTargetPosition(distance);
            break;

            case "turnRight":
                frontRight.setTargetPosition(-distance);
                frontLeft.setTargetPosition(-distance);
                backRight.setTargetPosition(-distance);
                backLeft.setTargetPosition(-distance);
            break;

            default:
                frontRight.setTargetPosition(distance);
                frontLeft.setTargetPosition(-distance);
                backRight.setTargetPosition(distance);
                backLeft.setTargetPosition(-distance);
            break;
        } //this takes the type input variable and sets the direction accordingly
        //for example, the "forward" input would make all motors go forward (obviously)

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Drive(power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            //telemetry.addData("Type of movement", type);
        }

        //sets power to zero, therefore braking
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Drive(0);
        sleep(500); // Use this after setting power to 0 to give it time to brake

    }

    private void Drive(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    private double calculateArmPower(double armAngle, double kCos, double kp, double target) {
        return kCos * Math.cos(Math.toRadians(armAngle)) + (kp * (target-armAngle));
    }

}