package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.exp;
import static java.lang.Thread.sleep;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Main")
public class Tel extends OpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight3A;
    boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();
    //PIDF
    private final double TkP = 0.008, TkI = 0.0, TkD = 0.08, SkP = 10.23, SkI = 0.0, SkF = 14.95, SkD = 10.58;
    private final int TURRET_MIN_POS = 309;
    private final int TURRET_MAX_POS = 647;
    private final int TURRET_INIT_POS = 490;
    public boolean taranie = false, abis = false, jay = false, Ungur = false, MiklosUngur = false, RichiUngur = false, Ungur2 = false, Invartitoare = false, SeInvarteMiklos = false;
    private double integral = 0, lastError = 0, tx = 0, imata;
    int pos, albastru = 0, rosu = 0, verde = 0, loculete;


    @Override
    public void init() {
        m.initsisteme(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(pos - TURRET_INIT_POS) > 3) {
            pos = m.turela.getCurrentPosition();
            double direction = Math.signum(TURRET_INIT_POS - pos);
            m.turela.setPower(0.09 * direction);
        }
        m.turela.setPower(0);


        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();

    }

    public void start() {
        Chassis.start();
        System.start();
        Turela.start();
    }

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                FL = y + x + rx;
                BL = y - x + rx;
                FR = y - x - rx;
                BR = y + x - rx;

                max = abs(FL);
                if (abs(FR) > max) {
                    max = abs(FR);
                }
                if (abs(BL) > max) {
                    max = abs(BL);
                }
                if (abs(BR) > max) {
                    max = abs(BR);
                }
                if (max > 1) {
                    FL /= max;
                    FR /= max;
                    BL /= max;
                    BR /= max;
                }
                if (gamepad1.left_trigger > 0) {
                    sm = 2;
                } else {
                    if (gamepad1.right_trigger > 0) {
                        sm = 5;
                    } else {
                        sm = 1;
                    }
                }

                POWER(FR / sm, BL / sm, BR / sm, FL / sm);

            }
        }
    });
    private final Thread System = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                imata = m.distanta.getDistance(DistanceUnit.CM);
                rosu = m.color.red();
                albastru = m.color.blue();
                verde = m.color.green();
                /// Turela toggle
                boolean mata = gamepad1.dpad_right;
                if (taranie != mata) {
                    if (mata) {
                        abis = !abis;
                    }
                    taranie = mata;
                }

                /// Intake
                boolean Richi = gamepad1.a;
                if (Ungur2 != Richi) {
                    if (gamepad1.a) {
                        RichiUngur = !RichiUngur;
                        if (RichiUngur && loculete != 3) {
                            m.intake.setPower(1);
                            if (imata >= 7.5 && loculete == 0) {
                                m.kdf(1000);
                                m.sortare.setPosition(Pozitii.luarea2);
                                loculete++;
                            } else if (imata >= 7.5 && loculete == 1) {
                                m.kdf(1000);
                                m.sortare.setPosition(Pozitii.luarea3);
                                loculete++;
                            }
                            else if (imata >= 7.5 && loculete == 2) {
                                m.kdf(1000);
                                loculete++;
                            }
                        } else {
                            m.sortare.setPosition(Pozitii.luarea1);
                            m.intake.setPower(0);
                        }
                    }
                    Ungur2 = Richi;
                }

                ///Servo Aruncare in shooter
                boolean african = gamepad1.y;
                if (jay != african) {
                    if (jay) {
                        m.Saruncare.setPosition(Pozitii.lansare);
                    } else {
                        //TODO: Schimbale daca nu sunt okay
                        m.Saruncare.setPosition(Pozitii.coborare);
                    }
                    jay = african;
                }

                ///Shooter
                PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
                m.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);

                boolean Miklos = gamepad1.x;
                if (Ungur != Miklos) {
                    if (gamepad1.x) {
                        MiklosUngur = !MiklosUngur;
                        if (MiklosUngur) {
                            loculete--;
                            m.shooter.setVelocity(2000);
                        } else {
                            m.shooter.setVelocity(0);
                        }
                    }
                    Ungur = Miklos;
                }

            }
        }
    });

    private final Thread Turela = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (abis) {

                    LLResult result = limelight3A.getLatestResult();
                    pos = m.turela.getCurrentPosition();

                    if (result.isValid() && pos >= TURRET_MIN_POS && pos <= TURRET_MAX_POS) {

                        double tx = result.getTx();
                        double error = tx;

                        integral += error;
                        double derivative = error - lastError;

                        double power = TkP * error + TkI * integral + TkD * derivative;
                        m.turela.setPower(power);

                        lastError = error;
//                    } else {
//                        if (pos >= TURRET_MAX_POS) {
//                            turela.setPower(-0.1);
//                        } else if (pos <= TURRET_MIN_POS) {
//                            turela.setPower(0.1);
//                        } else {
//                            turela.setPower(0.1);
//                        }

                    }
                } else {
                    m.turela.setPower(0);
                    lastError = 0;
                    integral = 0;
                }
            }
        }
    });

    public void stop() {
        stop = true;
    }

    @Override
    public void loop() {
        telemetry.addData("tx", tx);
        telemetry.addData("pos", pos);
        telemetry.addData("ab", abis);
        telemetry.addData("jay", jay);
        telemetry.addData("velocity", m.shooter.getVelocity());
        telemetry.addData("Red", rosu);
        telemetry.addData("Green", verde);
        telemetry.addData("Blue", albastru);
        telemetry.addData("Distanta", imata);
        telemetry.addData("loculete", loculete);
//        telemetry.addLine("==============");
//        telemetry.addData("fr", posFR);
//        telemetry.addData("fl", posFL);
//        telemetry.addData("br", posBR);
//        telemetry.addData("bl", posBL);
        telemetry.update();

    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}