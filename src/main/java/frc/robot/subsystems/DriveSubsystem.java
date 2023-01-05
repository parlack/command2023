package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.PUERTOSCAN;

public class DriveSubsystem extends SubsystemBase {
 
  WPI_TalonSRX MCI1ENC = new WPI_TalonSRX(PUERTOSCAN.PuertMotIzq1yEncoder);
  WPI_TalonSRX MCI2 = new WPI_TalonSRX(PUERTOSCAN.PuertMotIzq2);
  WPI_TalonSRX MCI3 = new WPI_TalonSRX(PUERTOSCAN.PuertMotIzq3);

  WPI_TalonSRX MCD4ENC = new WPI_TalonSRX(PUERTOSCAN.PuertMotDer1yEncoder);
  WPI_TalonSRX MCD5 = new WPI_TalonSRX(PUERTOSCAN.PuertMotDer2);
  WPI_TalonSRX MCD6 = new WPI_TalonSRX(PUERTOSCAN.PuertMotDer3);

  MotorControllerGroup MCIMASTER = new MotorControllerGroup(MCI1ENC, MCI2, MCI3);
  MotorControllerGroup MCDMASTER = new MotorControllerGroup(MCD4ENC, MCD5, MCD6);

  DifferentialDrive chasis = new DifferentialDrive(MCIMASTER, MCDMASTER);

  // LIMELIGHT //////
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  double ajustdist;
  double ajutGi;

  // joystick pa que vibre
  Joystick joystick = new Joystick(0);

  // path
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  DifferentialDriveOdometry m_odometry;

  @Override
  public void periodic() {

    m_odometry.update(
        m_gyro.getRotation2d(), getRightEncoderdistance(), getLeftEncoderdistance());

    SmartDashboard.putNumber("encizq", MCI1ENC.getSelectedSensorPosition() / 4096 / 9.01);

    SmartDashboard.putNumber("encizq", getLeftEncoderdistance());
    SmartDashboard.putNumber("encder", getRightEncoderdistance());
    SmartDashboard.putNumber("gyro", getHeading());

    SmartDashboard.putNumber("velocity", -MCD4ENC.getSelectedSensorVelocity() / 4096 / 9.01 * 10);

  }

  public DriveSubsystem() {
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    MCIMASTER.setInverted(true);
    MCDMASTER.setInverted(false);

  }

  public void CHASIS(double velocidad, double giro) {

        chasis.arcadeDrive(-velocidad * 0.80, giro * 0.8);
    
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-MCI1ENC.getSelectedSensorVelocity() / 4096 / 9.01 * 10,
        MCD4ENC.getSelectedSensorVelocity() / 4096 / 9.01 * 10);
  }

  public void resetEncoders() {

    MCI1ENC.setSelectedSensorPosition(0);
    MCD4ENC.setSelectedSensorPosition(0);

  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void arcadeDrive(double fwd, double rot) {
    chasis.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    MCIMASTER.setVoltage(leftVolts);
    MCDMASTER.setVoltage(rightVolts);
    chasis.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderdistance() + getRightEncoderdistance()) / 2.0;
  }

  public double getLeftEncoderdistance() {
    return -MCI1ENC.getSelectedSensorPosition() / 4096 / 9.01;
  }

  public double getRightEncoderdistance() {
    return MCD4ENC.getSelectedSensorPosition() / 4096 / 9.01;
  }

  public void setMaxOutput(double maxOutput) {
    chasis.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

}

