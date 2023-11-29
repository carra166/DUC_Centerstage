package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
10430 TELEOP
Motor gamepad controls
*/
@TeleOp
public class TeleopMK2 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    IMU imu;
    YawPitchRollAngles robotOrientation;

    Servo servoDoor;
    Servo servoWrist;
    CRServo servoIntake;

    double tgtPowerForward = 0;
    double tgtPowerStrafe = 0;
    double tgtPowerTurn = 0;
    double tgtPowerArm = 0;

    double division = 1;

    boolean inputOn = false;
    boolean canInputOn = true;

    boolean canMove = true;

    @Override
    public void runOpMode() {

        //initialized motors
        initializeMotors();
        imu = hardwareMap.get(IMU.class, "imu");

        //init imu and variables
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            /*this cuts the target power in half if you're
            holding down the b button on the controller*/
            if (this.gamepad1.right_trigger > 0) {
                //slow
                division = 3;
            } else if (this.gamepad1.left_trigger > 0) {
                //fast
                division = 1;
            } else {
                //normal
                division = 2;
            }

            if(gamepad1.y) {
                // move to 0 degrees.
                servoWrist.setPosition(0);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                servoWrist.setPosition(1);
            }

            if (gamepad1.dpad_left) {
                servoDoor.setPosition(0.5);
                sleep(750);
                servoDoor.setPosition(0);
            }

            if (gamepad1.dpad_right) {
                if (canInputOn) {
                    if (inputOn) {
                        inputOn = false;
                        servoIntake.setPower(0);
                    } else {
                        inputOn = true;
                        servoIntake.setPower(0.5);
                    }
                    canInputOn = false;
                }
            } else {
                canInputOn = true;
            }

            if(gamepad1.b && canMove) {
                if (tagProcessor.getDetections().size() > 0) {
                    canMove = false;
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    telemetry.addData("AprilTags", "FOUND");
                    telemetry.addData("Range", tag.ftcPose.range);
                    sleep(5000);
                    canMove = true;
                } else {
                    telemetry.addData("AprilTags", "NONE FOUND");
                }
            }

            if (gamepad1.x) {
                while (gamepad1.x) {}
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setTargetPosition(10);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(-1);
                while (armMotor.isBusy()) {}
                servoWrist.setPosition(0);
                armMotor.setPower(0);
            }

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("AprilTags", "FOUND");
                telemetry.addData("Range", tag.ftcPose.range);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                //bearing to the right is positive, left is negative

            } else {
                telemetry.addData("AprilTags", "NONE FOUND");
            }

            tgtPowerForward = (-this.gamepad1.left_stick_y / division);
            tgtPowerStrafe = (-this.gamepad1.left_stick_x / division);
            tgtPowerTurn = (this.gamepad1.right_stick_x / division);
            tgtPowerArm = (this.gamepad1.right_stick_y);

            //sets motor powereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeew44444444444444    q1`1`
            //this comment corrupted but its just too funny to delete
            if (canMove) {
                setPowerAll(
                        -tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                        tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                        -tgtPowerForward - tgtPowerStrafe - tgtPowerTurn,
                        tgtPowerForward - tgtPowerStrafe - tgtPowerTurn
                );
            }
            armMotor.setPower(tgtPowerArm);
            armMotor2.setPower(-tgtPowerArm);

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("current wrist amount", servoWrist.getPosition());
            telemetry.update();
        }
    }


    private void setPowerAll(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private void initializeMotors() {
        armMotor = hardwareMap.get(DcMotor.class, "ARM1");
        armMotor2 = hardwareMap.get(DcMotor.class, "ARM2");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoDoor = hardwareMap.get(Servo.class, "door");
        servoWrist = hardwareMap.get(Servo.class, "wrist");
        servoIntake = hardwareMap.get(CRServo.class,"intake");

    }

}