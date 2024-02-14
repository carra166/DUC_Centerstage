package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Environment;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.processors.ducProcessorBlueBackstage;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/*
10430 AUTONOMOUS PROGRAM
Uses encoders
*/
@Autonomous
@Config
public class Autonomous10430<myIMUparameters> extends LinearOpMode {

    int step = 0;
    int lastStep = 0;
    boolean runOnce = true;

    private static final String BASE_FOLDER_NAME = "autonomousTexts";
    public static final String AUTONOMOUS_DIRECTORY = "";
    static String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME+"/"+AUTONOMOUS_DIRECTORY;
    public static String textFileName = "test";
    public static double speed = 0.3;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    Servo servoClaw;
    Servo servoClaw2;
    Servo servoWrist;
    Servo servoLauncher;
    Servo servoPooper;

    private IMU imu;
    private YawPitchRollAngles robotOrientation;
    private ducProcessorBlueBackstage ducProcessor;
    private double duckPosition;

    InterpLUT armAngles;
    InterpLUT wristAngles;
    double kCos = 0.13;
    double tgtArmPower = 0;
    double armPosition = 0;
    double countdown = 100;
    double failsafeCountdown = 0;

    boolean wristPos = true;

    double kp = 0.0025;
    String highestPosition = "";

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");
        initializeMotors();

        armAngles = new InterpLUT();
        armAngles.add(-100, -41); //safety 1
        armAngles.add(0, -40); //init position
        armAngles.add(100, 0); //straight out (forward)
        armAngles.add(400, 90); //straight up
        armAngles.add(666, 180); //straight back SCARY!!!!!!!!!!!!!!!!!!!!!!!!!!
        armAngles.add(700, 190); //straight back SCARY!!!!!!!!!!!!!!!!!!!!!!!!!!
        armAngles.add(1000, 191); //safety 2
        armAngles.createLUT();

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

        ducProcessor = new ducProcessorBlueBackstage();

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
        servoClaw.setPosition(0.35);
        servoClaw2.setPosition(0.6);
        servoLauncher.setPosition(0);
        servoPooper.setPosition(0.5);
        while (!isStarted() && !isStopRequested()) {
            duckPosition = ducProcessor.getDuckPosition();
            telemetry.addData("DUCK POSITION", duckPosition);
            telemetry.addData("TARGET FILE", textFileName);
            telemetry.update();
        }

        //start auto

        List<double[]> parsedLines = readAndParseDoublesFromFile();

        /*
        wrist position: 0.35
        arm angle: 180
        (arm power: -0.12)
        */

        while (isStarted() && !isStopRequested()) {
            tgtArmPower = calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos, kp, armPosition);
            armMotor.setPower(tgtArmPower);
            armMotor2.setPower(-tgtArmPower);

            switch (step) {
                case 100:
                    if (runOnce) {
                        failsafeCountdown = 100;
                        servoWrist.setPosition(0.35);
                        runOnce = false;
                    }
                    failsafeCountdown--;
                    if (servoWrist.getPosition() == 0.35) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 101:
                    if (runOnce) {
                        failsafeCountdown = 100;
                        countdown = 100;
                        armPosition = 190;
                        runOnce = false;
                    }
                    failsafeCountdown--;
                    if (armAngles.get(armMotor.getCurrentPosition()) > 189 || failsafeCountdown<1) {
                        countdown--;
                        if (countdown<1) {
                            step++;
                            runOnce = true;
                        }
                    }
                    break;
                case 102:
                    if (runOnce) {
                        countdown = 50;
                        servoClaw.setPosition(0.18);
                        runOnce = false;
                    }
                    if (servoClaw.getPosition() == 0.18) {
                        countdown--;
                        if (countdown<1) {
                            step++;
                            runOnce = true;
                        }
                    }
                    break;
                case 103:
                    if (runOnce) {
                        servoWrist.setPosition(0);
                        runOnce = false;
                    }
                    if (servoWrist.getPosition() == 0) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 104:
                    if (runOnce) {
                        countdown = 50;
                        servoClaw.setPosition(0.35);
                        runOnce = false;
                    }
                    if (servoClaw.getPosition() == 0.35) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 105:
                    if (runOnce) {
                        armPosition = -30;
                        runOnce = false;
                    }
                    if (armAngles.get(armMotor.getCurrentPosition()) < -29) {
                        countdown--;
                        if (countdown<1) {
                            step++;
                            runOnce = true;
                        }
                    }
                    break;
                case 106:
                    step = lastStep;
                    break;
                default:
                    caseStatement(parsedLines);
                break;
            }
            telemetry.addData("Step:", step);
            telemetry.addData("Countdown:", countdown);
            telemetry.addData("Failsafe:", failsafeCountdown);
            telemetry.addData("Arm angle:", armAngles.get(armMotor.getCurrentPosition()));
            telemetry.update();
        }

        for (double[] line : parsedLines) {
            runToParsedPosition(line, 0.1);
        }

        telemetry.addLine(parsedLines.get(0)[0]+"");
        telemetry.update();

    }

    private void caseStatement(List<double[]> parsedLines) {
            if (step == parsedLines.size()) {
                return;
            }
            if (step == 0) {
                servoWrist.setPosition(0.743);
            }
            if (runOnce) {
                runToParsedPosition(parsedLines.get(step), 0.1);
                runOnce = false;
            }
            if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                step++;
                runOnce = true;
            }
    }

    private void runToParsedPosition(double[] myLine, double power) {

        if (myLine[0] == 69420) {
            //open pooper
            servoClaw2.setPosition(0.8);
            sleep(500);
            return;
        }
        if (myLine[0] == 6969) {
            lastStep = step + 1;
            step = 99;
            return;
        }

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setTargetPosition((int)myLine[0]);
        frontLeft.setTargetPosition((int)myLine[1]);
        backRight.setTargetPosition((int)myLine[2]);
        backLeft.setTargetPosition((int)myLine[3]);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Drive(speed, myLine, 0.3);

        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy()) {}

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

        servoClaw = hardwareMap.get(Servo.class, "clawLeft");
        servoClaw2 = hardwareMap.get(Servo.class, "clawRight");
        servoWrist = hardwareMap.get(Servo.class, "wrist");
        servoLauncher = hardwareMap.get(Servo.class, "launcher");
        servoPooper = hardwareMap.get(Servo.class, "pooper");

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
        } //this takes the type input variable and sets the direction accordingly
        //for example, the "forward" input would make all motors go forward (obviously)

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Drive(power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Distance left:", frontRight.getTargetPosition()-frontRight.getCurrentPosition());
            telemetry.update();
        }

        //sets power to zero, therefore braking
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(500); // Use this after setting power to 0 to give it time to brake

    }

    private void Drive(double power, double[] targetPositions, double kp) {

        double highestPositionIndex = findHighest(targetPositions);
        double highestPosition = targetPositions[(int)highestPositionIndex];
        double[] powerPercentagesArray = new double[]{ (Math.abs(targetPositions[0]/highestPosition) * 100), (Math.abs(targetPositions[1]/highestPosition) * 100), (Math.abs(targetPositions[2]/highestPosition) * 100), (Math.abs(targetPositions[3]/highestPosition) * 100) };
        DcMotor[] motors = new DcMotor[]{ frontRight, frontLeft, backRight, backLeft };

        for (int i = 0; i<4; i++) {
            if (targetPositions[i] > 0) {
                motors[i].setPower(power*(powerPercentagesArray[i]/100) /*+ (kp * (targetPositions[i]-motors[i].getCurrentPosition()))*/);
            } else {
                motors[i].setPower(-power*(powerPercentagesArray[i]/100) /*+ (kp * (targetPositions[i]-motors[i].getCurrentPosition()))*/);
            }
        }
    }

    private double calculateArmPower(double armAngle, double kCos, double kp, double target) {
        return kCos * Math.cos(Math.toRadians(armAngle)) + (kp * (target-armAngle));
    }

    private static List<double[]> readAndParseDoublesFromFile() {
        List<double[]> parsedLines = new ArrayList<>();
        try {
            FileReader reader = new FileReader(directoryPath+"/"+textFileName+".txt");
            BufferedReader bufferedReader = new BufferedReader(reader);

            String line;
            while ((line = bufferedReader.readLine()) != null) {
                String[] numbersAsString = line.split(" ");
                double[] parsedNumbers = new double[numbersAsString.length];
                for (int i = 0; i < numbersAsString.length; i++) {
                    //these stupid little functions basically set the value of array in place of the number because
                    //java arrays are stupid and can only do one kind of value in an array GRAHHHH!!!!!
                    if (stringCompare(numbersAsString[i], "pooper") == 0) {
                        parsedNumbers = new double[]{69420};
                    } else if (stringCompare(numbersAsString[i], "backboard") == 0) {
                        parsedNumbers = new double[]{6969};
                    } else {
                        parsedNumbers[i] = Double.parseDouble(numbersAsString[i]);
                    }
                }
                parsedLines.add(parsedNumbers);
            }

            bufferedReader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return parsedLines;
    }

    //i love chat-gpt! if this code doesnt work dont blame me
    public static double findHighest(double[] arr) {
        double highest = arr[0];
        double highestI = 0;
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] > highest) {
                highest = arr[i];
                highestI = i;
            }
        }
        return highestI;
    }

    public static int stringCompare(String str1, String str2)
    {

        int l1 = str1.length();
        int l2 = str2.length();
        int lmin = Math.min(l1, l2);

        for (int i = 0; i < lmin; i++) {
            int str1_ch = (int)str1.charAt(i);
            int str2_ch = (int)str2.charAt(i);

            if (str1_ch != str2_ch) {
                return str1_ch - str2_ch;
            }
        }

        // Edge case for strings like
        // String 1="Geeks" and String 2="Geeksforgeeks"
        if (l1 != l2) {
            return l1 - l2;
        }

        // If none of the above conditions is true,
        // it implies both the strings are equal
        else {
            return 0;
        }
    }

}