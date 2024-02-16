package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Environment;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.AutoLib;
import org.firstinspires.ftc.teamcode.processors.ducProcessorRedBackstage;
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
@Disabled
public class AutonomousRedBackstageOLD<myIMUparameters> extends LinearOpMode {

    int step = 0;
    boolean runOnce = true;

    public static final String PARK_POSITION = "left"; //LOWERCASE ONLY
    private static final String BASE_FOLDER_NAME = "autonomousTexts";
    public static final String AUTONOMOUS_DIRECTORY = "backstageRed";
    static String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME+"/"+AUTONOMOUS_DIRECTORY;
    public static String textFileName = "center";
    public static double speed = 0.3;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    Servo servoClaw;
    Servo servoWrist;
    Servo servoLauncher;
    Servo servoPooper;

    private IMU imu;
    private YawPitchRollAngles robotOrientation;
    private ducProcessorRedBackstage ducProcessor;
    private double duckPosition;

    InterpLUT armAngles;
    double kCos = 0;
    double tgtArmPower = 0;
    double armPosition = -20;
    double countdown = 100;

    double kp = 0.003;
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
        armAngles.add(666, 180); //straight back
        armAngles.add(1000, 181); //safety 2
        armAngles.createLUT();

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

        ducProcessor = new ducProcessorRedBackstage();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(ducProcessor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}
        servoWrist.setPosition(0.1);
        servoClaw.setPosition(0.5);
        servoLauncher.setPosition(0.2);
        servoPooper.setPosition(0.5);
        while (!isStarted() && !isStopRequested()) {
            duckPosition = ducProcessor.getDuckPosition();
            telemetry.addData("DUCK POSITION", duckPosition);
            switch((int)duckPosition) {
                case 1:
                    textFileName = "left";
                break;
                case 2:
                    textFileName = "center";
                break;
                case 3:
                    textFileName = "right";
                break;
            }
            telemetry.addData("TARGET POSITION", textFileName);
            telemetry.update();
        }

        //start auto

        List<double[]> parsedLines = readAndParseDoublesFromFile();

        while (isStarted() && !isStopRequested()) {
            tgtArmPower = AutoLib.calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos, kp, armPosition);
            armMotor.setPower(tgtArmPower);
            armMotor2.setPower(-tgtArmPower);

            switch (step) {
                case 0:
                    caseStatement(parsedLines);
                break;
                case 1:
                    caseStatement(parsedLines);
                    break;
                case 2:
                    caseStatement(parsedLines);
                    break;
                case 3:
                    caseStatement(parsedLines);
                    break;
                case 4:
                    caseStatement(parsedLines);
                    break;
                case 5:
                    caseStatement(parsedLines);
                    break;
                case 6:
                    caseStatement(parsedLines);
                    break;
                case 7:
                    caseStatement(parsedLines);
                    break;
                case 8:
                    caseStatement(parsedLines);
                    break;
                case 9:
                    caseStatement(parsedLines);
                    break;
                case 10:
                    if (runOnce) {
                        servoWrist.setPosition(0.3);
                        runOnce = false;
                    }
                    if (servoWrist.getPosition() == 0.3) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 11:
                    if (runOnce) {
                        armPosition = 130;
                        runOnce = false;
                    }
                    if (armAngles.get(armMotor.getCurrentPosition()) > 129) {
                        countdown--;
                        if (countdown<1) {
                            step++;
                            runOnce = true;
                        }
                    }
                    break;
                case 12:
                    if (runOnce) {
                        countdown = 100;
                        servoClaw.setPosition(0.675);
                        runOnce = false;
                    }
                    if (servoClaw.getPosition() == 0.675) {
                        countdown--;
                        if (countdown<1) {
                            step++;
                            runOnce = true;
                        }
                    }
                    break;
                case 13:
                    if (runOnce) {
                        servoWrist.setPosition(0.9);
                        runOnce = false;
                    }
                    if (servoWrist.getPosition() == 0.9) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 14:
                    if (runOnce) {
                        countdown = 50;
                        servoClaw.setPosition(0.5);
                        runOnce = false;
                    }
                    if (servoClaw.getPosition() == 0.5) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 15:
                    if (runOnce) {
                        armPosition = 30;
                        runOnce = false;
                    }
                    if (armAngles.get(armMotor.getCurrentPosition()) < 31) {
                        countdown--;
                        if (countdown<1) {
                            step++;
                            runOnce = true;
                        }
                    }
                    break;
                case 16:
                    if (runOnce) {
                        armPosition = -10;
                        runOnce = false;
                    }
                    if (armAngles.get(armMotor.getCurrentPosition()) < -9) {
                        step++;
                        runOnce = true;
                    }
                    break;
                case 17:
                        if (runOnce) {
                            runToParsedPosition(parsedLines.get(parsedLines.size() - 1), 0.1);
                            runOnce = false;
                        }
                        if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                            step++;
                            runOnce = true;
                        }
            }
            telemetry.addData("hello", step);
            telemetry.update();
        }



        for (double[] line : parsedLines) {
            runToParsedPosition(line, 0.1);
        }

        telemetry.addLine(parsedLines.get(0)[0]+"");
        telemetry.update();

    }

    private void caseStatement(List<double[]> parsedLines) {
        if (parsedLines.size() < step + 2) {

            DcMotor[] motors = new DcMotor[]{ frontRight, frontLeft, backRight, backLeft };
            for (int i = 0; i<4; i++) {
                motors[i].setPower(0);
            }

            step = 10;
            runOnce = true;
        } else {
            if (runOnce) {
                runToParsedPosition(parsedLines.get(step), 0.1);
                runOnce = false;
            }
            if (!frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                step++;
                runOnce = true;
            }
        }
    }

    private void runToParsedPosition(double[] myLine, double power) {

        if (myLine[0] == 69420) {
            //open pooper
            servoPooper.setPosition(0);
            sleep(500);
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

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoClaw = hardwareMap.get(Servo.class, "claw");
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

        DriveSimple(power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            //telemetry.addData("Type of movement", type);
        }

        //sets power to zero, therefore braking
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(100); // Use this after setting power to 0 to give it time to brake

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void DriveSimple(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    private void Drive(double power, double[] targetPositions, double kp) {

        double highestPositionIndex = findHighest(targetPositions);
        double highestPosition = targetPositions[(int)highestPositionIndex];
        double[] powerPercentagesArray = new double[]{ (Math.abs(targetPositions[0]/highestPosition) * 100), (Math.abs(targetPositions[1]/highestPosition) * 100), (Math.abs(targetPositions[2]/highestPosition) * 100), (Math.abs(targetPositions[3]/highestPosition) * 100) };
        DcMotor[] motors = new DcMotor[]{ frontRight, frontLeft, backRight, backLeft };

        for (int i = 0; i<4; i++) {
            if (targetPositions[i] > 0) {
                motors[i].setPower(power*(powerPercentagesArray[i]/100) + (kp * (targetPositions[i]-motors[i].getCurrentPosition())));
            } else {
                motors[i].setPower(-power*(powerPercentagesArray[i]/100) + (kp * (targetPositions[i]-motors[i].getCurrentPosition())));
            }
        }
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

            if (stringCompare("left", PARK_POSITION) == 0) {
                parsedLines.add(new double[]{ -844.0, -1004.0, 966.0, 845.0 });
            } else {
                parsedLines.add(new double[]{ 844.0, 1004.0, -966.0, -845.0 });
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