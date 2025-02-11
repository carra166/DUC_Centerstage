package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Config
@Disabled
public class TeleopMK2 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    IMU imu;
    YawPitchRollAngles robotOrientation;

    Servo servoClaw;
    Servo servoWrist;
    Servo servoLauncher;
    Servo servoPooper;

    double tgtPowerForward = 0;
    double tgtPowerStrafe = 0;
    double tgtPowerTurn = 0;
    double tgtPowerArm;

    public static double kStrafing = 1;

    double division = 1;

    boolean inputOn = false;
    boolean canInputOn = true;

    boolean openClaw = true; //ignore the fact that it's inverted
    boolean wristPos = true; //ignore the fact that it's inverted

    boolean canMove = true;

    InterpLUT armAngles;
    double kCos = 0.12;
    double downPower = 0;

    @Override
    public void runOpMode() {

        //initialized motors
        initializeMotors();
        imu = hardwareMap.get(IMU.class, "imu");

        //init imu and variables
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        armAngles = new InterpLUT();
        armAngles.add(-10000, -41); //safety 1
        armAngles.add(0, -40); //init position
        armAngles.add(100, 0); //straight out (forward)
        armAngles.add(400, 90); //straight up
        armAngles.add(666, 180); //straight back
        armAngles.add(10000, 181); //safety 2
        armAngles.createLUT();

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servoWrist.setPosition(0.1);
        servoClaw.setPosition(0.5);
        servoLauncher.setPosition(0.2);
        servoPooper.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {

            //gamepad 1

            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                servoLauncher.setPosition(0);
            }

            /*this cuts the target power in half if you're
            holding down the b button on the controller*/
            if (this.gamepad1.right_trigger > 0) {
                //slow
                division = 4;
            } else {
                //normal
                division = 2;
            }


            //gamepad 2
            if (gamepad2.a) {
                while (gamepad2.a) {}
                if (openClaw) {
                    servoClaw.setPosition(0.675);
                    openClaw = false;
                } else {
                    servoClaw.setPosition(0.5);
                    openClaw = true;
                }
            }


            if (gamepad2.x) {
                while (gamepad2.x) {}
                if (wristPos) {
                    servoWrist.setPosition(0.3); //arm up position
                    wristPos = false;
                } else {
                    servoWrist.setPosition(0.9); //arm down position
                    wristPos = true;
                }
            }

            if (gamepad2.y) {
                downPower = -1;
            } else {
                downPower = 0;
            }

            tgtPowerForward = (-this.gamepad1.left_stick_y / division);
            tgtPowerStrafe = (-this.gamepad1.left_stick_x / division);
            tgtPowerTurn = (this.gamepad1.right_stick_x / division);
            tgtPowerArm = ((this.gamepad2.right_stick_y / -5) + calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos) + downPower);

            //sets motor powereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeew44444444444444    q1`1`
            //this comment corrupted but its just too funny to delete
            if (canMove) {
                setPowerAll(
                        -tgtPowerForward + (tgtPowerStrafe * kStrafing) - tgtPowerTurn,
                        tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                        -tgtPowerForward - (tgtPowerStrafe * kStrafing) - tgtPowerTurn,
                        tgtPowerForward - tgtPowerStrafe - tgtPowerTurn
                );
            }
            armMotor.setPower(tgtPowerArm);
            armMotor2.setPower(-tgtPowerArm);

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("current wrist amount", servoWrist.getPosition());
            telemetry.addData("right stick y", this.gamepad1.right_stick_y);

            telemetry.addData("current arm position", armMotor.getCurrentPosition());
            telemetry.addData("arm angle", armAngles.get(armMotor.getCurrentPosition()));
            telemetry.addData("gravity compensation", calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos));
            telemetry.addData("current tgt arm", tgtPowerArm);
            telemetry.addData("current arm motor power", armMotor.getPower());
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

        servoClaw = hardwareMap.get(Servo.class, "claw");
        servoWrist = hardwareMap.get(Servo.class, "wrist");
        servoLauncher = hardwareMap.get(Servo.class, "launcher");
        servoPooper = hardwareMap.get(Servo.class, "pooper");

    }

    private double calculateArmPower(double armAngle, double kCos) {
        return kCos * Math.cos(Math.toRadians(armAngle));
    }

}