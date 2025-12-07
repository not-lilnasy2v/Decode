package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
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
    private DcMotorEx frontRight, frontLeft, backRight, backLeft, turela, shooter, intake;
    private ServoImplEx Saruncare, sortare;
    private Limelight3A limelight3A;
    private ColorSensor color;
    private DistanceSensor distanta;
    boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();

    //PIDF
    private final double TkP = 0.008, TkI = 0.0, TkD = 0.08, SkP = 10.23, SkI = 0.0, SkF = 14.95, SkD = 10.58;
    public boolean turelaTracking = false, tracking = false, ShooterPornit = false, Spornit = false, IntakeFull = false, Ipornit = false, IntakePornit = false, ButonulRosu = false, ResetareISortare = false;
    private double integral = 0, lastError = 0, imata, tx = 0, power = 0;
    int pos, albastru = 0, rosu = 0, verde = 0, loculete = 0;


    @Override
    public void init() {
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
//
//        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPower(0);


        turela = hardwareMap.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Saruncare = hardwareMap.get(ServoImplEx.class, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = hardwareMap.get(ServoImplEx.class, "sortare");
        sortare.setPosition(Pozitii.luarea1);

        distanta = hardwareMap.get(DistanceSensor.class, "distanta");


        color = hardwareMap.get(ColorSensor.class, "color");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setPower(0);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(3);
        limelight3A.start();
    }


    public void start() {
        Chassis.start();
        Butoane.start();
        Turela.start();
        Sortare.start();
        Shooter.start();
    }


    private final Thread Butoane = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                imata = distanta.getDistance(DistanceUnit.CM);
                verde = color.green();

                /// Turela toggle
                boolean dpad_right1 = gamepad1.dpad_right;
                if (turelaTracking != dpad_right1) {
                    if (gamepad1.dpad_right) {
                        tracking = !tracking;
                    }
                    turelaTracking = dpad_right1;
                }


                /// Intake
                boolean gamepad1_a = gamepad1.a;
                if (IntakePornit != gamepad1_a) {
                    if (gamepad1.a) {
                        Ipornit = !Ipornit;
                    }
                    IntakePornit = gamepad1_a;
                }
                /// shooter
                boolean gamepad1_x = gamepad1.x;
                if (ShooterPornit != gamepad1_x) {
                    if (gamepad1.x) {
                        Spornit = !Spornit;
                    }
                    ShooterPornit = gamepad1_x;
                }

                /// Resetare Sortare Intake
                boolean gamepad2_a = gamepad2.a;
                if (ResetareISortare != gamepad2_a) {
                    if (gamepad2.a) {
                        ButonulRosu = !ButonulRosu;
                    }
                    ResetareISortare = gamepad2_a;
                }
            }
        }
    });

    private final Thread Turela = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (tracking) {
                    pos = turela.getCurrentPosition();
                    LLResult result = limelight3A.getLatestResult();
                    if (result.isValid() && pos >= Pozitii.TURRET_MIN_POS && pos <= Pozitii.TURRET_MAX_POS) {

                        tx = result.getTx();

                        integral += tx;
                        double derivative = tx - lastError;

                        power = TkP * tx + TkI * integral + TkD * derivative;
                        turela.setPower(power);

                        lastError = tx;

//                    }
//                    else {
//                        if (pos >= Pozitii.TURRET_MAX_POS) {
//                            turela.setPower(-0.1);
//                        } else if (pos <= Pozitii.TURRET_MIN_POS) {
//                            turela.setPower(0.1);
//                        } else {
//                            turela.setPower(0.1);
//                        }

                    }
                } else {
                    turela.setPower(0);
                    lastError = 0;
                    integral = 0;
                }
            }
        }
    });

    private enum SortareIntake {
        IDLE,
        ASTEPTARE,
        Luare1,
        Luare2,
        Luare3,
        FULL
    }

    private SortareIntake Istare = SortareIntake.IDLE;
    private final Thread Sortare = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (Ipornit && loculete < 3) {
                    intake.setPower(1);
                    switch (Istare) {
                        case IDLE:
                            Istare = SortareIntake.Luare1;
                            break;
                        case ASTEPTARE:
                            if (imata < 20 && intake.isBusy()) {
                                if (loculete == 0) Istare = SortareIntake.Luare1;
                                else if (loculete == 1) Istare = SortareIntake.Luare2;
                                else if (loculete == 2) Istare = SortareIntake.Luare3;
                                else if (loculete == 3) Istare = SortareIntake.FULL;
                            }
                            break;
                        case Luare1:
                            sortare.setPosition(Pozitii.luarea2);
                            m.kdf(950);
                            loculete = 1;
                            Istare = SortareIntake.ASTEPTARE;
                            break;

                        case Luare2:
                            sortare.setPosition(Pozitii.luarea3);
                            m.kdf(950);
                            loculete = 2;
                            Istare = SortareIntake.ASTEPTARE;
                            break;

                        case Luare3:
                            m.kdf(950);
                            loculete = 3;
                            Istare = SortareIntake.FULL;
                            break;
                        case FULL:
                            intake.setPower(0);
                            IntakeFull = true;

                            if (ButonulRosu) {
                                loculete = 0;
                                IntakeFull = false;
                                Istare = SortareIntake.IDLE;
                            }
                            break;
                    }
//                    while(loculete <= 3) {
//                        intake.setPower(1);
//                        if (imata < 20 && loculete == 0) {
//                            sortare.setPosition(Pozitii.luarea2);
//                            loculete++;
//                            m.kdf(950);
//                            break;
//                        }
//                        if (imata < 20 && loculete == 1) {
//                            sortare.setPosition(Pozitii.luarea3);
//                            loculete++;
//                            m.kdf(1000);
//                            break;
//                        }
//                        if (imata < 20 && loculete == 2) {
//                            richiChelioasa = true;
//                            loculete++;
//                        }
//                    }


                } else {
                    sortare.setPosition(Pozitii.luarea1);
                    intake.setPower(0);

                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                }
            }
        }
    });

    private final Thread Shooter = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (Spornit && IntakeFull) {
                    PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
                    shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
                    shooter.setVelocity(2000);
//                    do {
//                        shooter.setVelocity(2000);
//
//                        if (verde <= Pozitii.mov_verde) {
//                            m.kdf(980);
//                            Saruncare.setPosition(Pozitii.lansare);
//                            loculete--;
//                            m.kdf(2000);
//                            Saruncare.setPosition(Pozitii.coborare);
//
//
//                        } else if (verde != Pozitii.mov_verde) {
//                            sortare.setPosition(Pozitii.aruncare2);
//                        }
//                    } while (MiklosUngur && loculete >= 1 && loculete <= 3);
                    shooter.setVelocity(0);

                    sortare.setPosition(Pozitii.aruncare1);
                }

            }
        }
    });
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

    public void stop() {
        stop = true;
    }

    @Override
    public void loop() {
        telemetry.addData("Green", verde);
        telemetry.addData("Distanta", imata);
        telemetry.addData("loculete", loculete);
        telemetry.addData("Case", Istare);
        telemetry.update();

    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}