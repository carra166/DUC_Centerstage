package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
10430 TELEOP
Motor gamepad controls
*/
@TeleOp
@Config
public class TeleopMK3 extends LinearOpMode {

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

    InterpLUT frontRightLUT;
    InterpLUT frontLeftLUT;
    InterpLUT backRightLUT;
    InterpLUT backLeftLUT;

    double tgtPowerForward = 0;
    double tgtPowerStrafe = 0;
    double tgtPowerTurn = 0;
    double tgtPowerArm;

    double initYaw;
    double adjustedYaw;
    public static double negative = -1;
    public static double negative2 = -1;

    public static double kStrafing = 1;

    double division = 1;

    boolean inputOn = false;
    boolean canInputOn = true;

    boolean openClaw = true; //ignore the fact that it's inverted
    boolean wristPos = true; //ignore the fact that it's inverted

    boolean canMove = true;
    boolean fieldCentricStrafing = true;

    InterpLUT armAngles;
    double kCos = 0.12;
    double downPower = 0;

    int soundID = hardwareMap.appContext.getResources().getIdentifier("Quack.wav", "raw", hardwareMap.appContext.getPackageName());

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

        initializeYaw();

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

        SoundPlayer.getInstance().startPlaying((hardwareMap.appContext), soundID);
        waitForStart();

        while (opModeIsActive()) {

            //gamepad 1


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

            if (gamepad1.left_trigger > 0) {
                if (gamepad1.y) {
                    initializeYaw();
                }

                if (gamepad1.a) {
                    while (gamepad1.a) {}
                    if (fieldCentricStrafing) {
                        fieldCentricStrafing = false;
                    } else {
                        fieldCentricStrafing = true;
                    }
                }
            } else {
                if (gamepad1.right_bumper && gamepad1.left_bumper) {
                    servoLauncher.setPosition(0);
                }

                if (gamepad1.right_trigger > 0) {
                    //slow
                    division = 4;
                } else {
                    //normal
                    division = 2;
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

            tgtPowerForward = (this.gamepad1.left_stick_y / division); //y
            tgtPowerStrafe = (-this.gamepad1.left_stick_x / division); //x
            tgtPowerTurn = (-this.gamepad1.right_stick_x / division); //turning
            tgtPowerArm = ((this.gamepad2.right_stick_y / -5) + calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos) + downPower);

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            adjustedYaw = orientation.getYaw(AngleUnit.DEGREES)-initYaw;
            double zeroedYaw = -initYaw+orientation.getYaw(AngleUnit.DEGREES);

            double theta = Math.atan2(tgtPowerForward, tgtPowerStrafe) * 180/Math.PI;
            double realTheta;
            realTheta = (360 - zeroedYaw) + theta;
            double power = Math.hypot(tgtPowerForward, tgtPowerStrafe);

            double sin = Math.sin((realTheta * (Math.PI/180)) - (Math.PI / 4));
            double cos = Math.cos((realTheta * (Math.PI/180)) - (Math.PI / 4));
            double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

            //sets motor powereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeew44444444444444    q1`1`
            //this comment corrupted but its just too funny to delete
            double[] motorPower;
            if (fieldCentricStrafing) {
                motorPower = new double[]{
                        (power * cos / maxSinCos + tgtPowerTurn), //front left
                        (power * sin / maxSinCos + tgtPowerTurn), //back left
                        -(power * sin / maxSinCos - tgtPowerTurn), //front right
                        -(power * cos / maxSinCos - tgtPowerTurn) //back right
                };
            } else {
                motorPower = new double[]{
                        tgtPowerForward + (tgtPowerStrafe * kStrafing) + tgtPowerTurn, //front left
                        -tgtPowerForward + tgtPowerStrafe + tgtPowerTurn, //back left
                        tgtPowerForward - (tgtPowerStrafe * kStrafing) + tgtPowerTurn, //front right
                        -tgtPowerForward - tgtPowerStrafe + tgtPowerTurn //back right
                };
            }
            if (motorPower.length == 0) {
                motorPower = new double[]{
                    0, 0, 0, 0
                };
                telemetry.addLine("HELP BAD THINGS ARE HAPPENING BAD CODING IM BAD AT CODING");
            }
            if ((power + Math.abs(tgtPowerTurn)) > 1)
                for (int i = 0; i < 4; i++) {
                    if (i < 1) {
                        motorPower[i] /= power + tgtPowerTurn;
                    } else {
                        motorPower[i] /= power - tgtPowerTurn;
                    }
                }
            if (canMove) {
                setPowerAll(motorPower);
            }
            armMotor.setPower(tgtPowerArm);
            armMotor2.setPower(-tgtPowerArm);

            telemetry.addData("Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("field centric multiplier frontright", 1);
            telemetry.update();
        }
    }


    private void setPowerAll(double[] motorPowers) {
        frontLeft.setPower(motorPowers[0]);
        backLeft.setPower(motorPowers[1]);
        frontRight.setPower(motorPowers[2]);
        backRight.setPower(motorPowers[3]);
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

    public void initializeYaw() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        initYaw = orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getFieldCentric(double theta, InterpLUT lut) {
        return lut.get(theta);
    }

}