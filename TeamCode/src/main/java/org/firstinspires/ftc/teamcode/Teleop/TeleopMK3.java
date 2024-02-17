package org.firstinspires.ftc.teamcode.Teleop;

import android.media.SoundPool;

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
    private DcMotor climbMotor;
    private DcMotor climbMotor2;
    IMU imu;
    YawPitchRollAngles robotOrientation;

    Servo servoClawLeft;
    Servo servoClawRight;
    Servo servoWrist;
    Servo servoLauncher;

    InterpLUT frontRightLUT;
    InterpLUT frontLeftLUT;
    InterpLUT backRightLUT;
    InterpLUT backLeftLUT;

    double tgtPowerForward = 0;
    double tgtPowerStrafe = 0;
    double tgtPowerTurn = 0;
    double tgtPowerArm = 0;
    double tgtPowerHang = 0;

    double initYaw;
    double adjustedYaw;
    public static double servoOpen = 0.1;
    public static double servoClosed = 0.3;

    public static double kStrafing = 1;

    double division = 1;

    boolean inputOn = false;
    boolean canInputOn = true;

    boolean leftClawOpen = true; //ignore the fact that it's inverted
    boolean rightClawOpen = true;
    boolean wristPos = true; //ignore the fact that it's inverted
    public static double wristAngle = 0.24;

    boolean canMove = true;
    boolean fieldCentricStrafing = true;

    InterpLUT armAngles;
    InterpLUT wristAngles;
    double kCos = 0.13;
    double downPower = 0;

    @Override
    public void runOpMode() {

        int[] soundIDs = new int[]{ getSoundId("quack"), getSoundId("superquack"), getSoundId("song") };

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

        wristAngles = new InterpLUT();
        wristAngles.add(-10000, 0.25);
        wristAngles.add(125, 0.24);
        wristAngles.add(180, 0.39);
        wristAngles.add(10000, 0.40);
        wristAngles.createLUT();



        initializeYaw();

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servoWrist.setPosition(0);
        servoClawLeft.setPosition(0.35);
        servoClawRight.setPosition(0.6);
        servoLauncher.setPosition(0.2);

        SoundPlayer.getInstance().startPlaying((hardwareMap.appContext), soundIDs[0]);
        waitForStart();

        while (opModeIsActive()) {

            //gamepad 2
            if (gamepad2.a) {
                while (gamepad2.a) {}
                    //OPEN
                    servoClawLeft.setPosition(0.18);
                    servoClawRight.setPosition(0.8);
                    leftClawOpen = false;
                    rightClawOpen = false;
            }

            if (gamepad2.y) {
                while (gamepad2.y) {}
                //CLOSE
                servoClawLeft.setPosition(0.35);
                servoClawRight.setPosition(0.6);
                leftClawOpen = true;
                rightClawOpen = true;
            }

            if (gamepad2.right_bumper) {
                while (gamepad2.right_bumper) {}
                if (rightClawOpen) {
                    servoClawRight.setPosition(0.8);
                    rightClawOpen = false;
                } else {
                    servoClawRight.setPosition(0.6);
                    rightClawOpen = true;
                }
                //servoClawRight.setPosition(rightClawOpen ? 0.6 : 0.8);
                telemetry.addData("right claw", rightClawOpen);
            }

            if (gamepad2.left_bumper) {
                while (gamepad2.left_bumper) {}
                if (leftClawOpen) {
                    servoClawLeft.setPosition(0.18);
                    leftClawOpen = false;
                } else {
                    servoClawLeft.setPosition(0.35);
                    leftClawOpen = true;
                }
                telemetry.addData("left claw", leftClawOpen);
            }

            if (gamepad2.dpad_up) {
                servoWrist.setPosition(wristAngle);
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
                    servoLauncher.setPosition(0.5);
                }

                if (gamepad1.right_trigger > 0) {
                    //slow
                    division = 4;
                } else if (gamepad1.left_bumper) {
                    //fast
                    division = 0.9f;
                } else {
                    //normal
                    division = 1.5;
                }
            }


            if (gamepad2.x) {
                while (gamepad2.x) {}
                if (wristPos) {
                    wristPos = false;
                } else {
                    servoWrist.setPosition(0.78); //arm down position
                    wristPos = true;
                }
            } //0.4 for auto btw

            if (gamepad2.y) {
                downPower = -1;
            } else {
                downPower = 0;
            }

            if (this.gamepad1.dpad_up) {
                tgtPowerHang = 1 / division;
            } else if (this.gamepad1.dpad_down) {
                tgtPowerHang = -1 / division;
            } else {
                tgtPowerHang = 0;
            }

            if (gamepad1.dpad_left) {
                while (gamepad1.dpad_left) {}
                SoundPlayer.getInstance().startPlaying((hardwareMap.appContext), soundIDs[0]);
            }

            tgtPowerForward = (this.gamepad1.left_stick_y / division); //y
            tgtPowerStrafe = (-this.gamepad1.left_stick_x / division); //x
            tgtPowerTurn = (-this.gamepad1.right_stick_x / division); //turning
            tgtPowerArm = ((this.gamepad2.right_stick_y / -3) + calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos) + downPower);

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
            climbMotor.setPower(tgtPowerHang);
            climbMotor2.setPower(-tgtPowerHang);
            if (!wristPos) {
                wristAngle = wristAngles.get(armAngles.get(armMotor.getCurrentPosition()));
                servoWrist.setPosition(wristAngle);
            }

            telemetry.addData("Wrist position", wristAngle);
            telemetry.addData("Arm encoder", armMotor.getCurrentPosition());
            telemetry.addData("Arm angle", armAngles.get(armMotor.getCurrentPosition()));
            telemetry.addData("Arm power", calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos));
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
        climbMotor = hardwareMap.get(DcMotor.class, "CLIMBL");
        climbMotor2 = hardwareMap.get(DcMotor.class, "CLIMBR");
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

        servoClawLeft = hardwareMap.get(Servo.class, "clawLeft");
        servoClawRight = hardwareMap.get(Servo.class, "clawRight");
        servoWrist = hardwareMap.get(Servo.class, "wrist");
        servoLauncher = hardwareMap.get(Servo.class, "launcher");

    }

    private double calculateArmPower(double armAngle, double kCos) {
        return kCos * Math.cos(Math.toRadians(armAngle));
    }

    public void initializeYaw() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        initYaw = orientation.getYaw(AngleUnit.DEGREES);
    }

    public int getSoundId(String soundName) {
        return hardwareMap.appContext.getResources().getIdentifier(soundName, "raw", hardwareMap.appContext.getPackageName());
    }

}