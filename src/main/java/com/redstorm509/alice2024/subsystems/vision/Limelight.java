package com.redstorm509.alice2024.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.Optional;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

public class Limelight {

	private String name;

	public Limelight(String limelightName, Pose3d cameraPoseInRobotSpace) {
		name = limelightName;
		Translation3d trans = cameraPoseInRobotSpace.getTranslation();
		Rotation3d rot = cameraPoseInRobotSpace.getRotation();
		setCameraPose_RobotSpace(trans.getX(), trans.getY(), trans.getZ(),
				rot.getX(), rot.getY(), rot.getZ());
	}

	public class LimelightTarget_Retro {

		@JsonProperty("t6c_ts")
		private double[] cameraPose_TargetSpace;

		@JsonProperty("t6r_fs")
		private double[] robotPose_FieldSpace;

		@JsonProperty("t6r_ts")
		private double[] robotPose_TargetSpace;

		@JsonProperty("t6t_cs")
		private double[] targetPose_CameraSpace;

		@JsonProperty("t6t_rs")
		private double[] targetPose_RobotSpace;

		public Pose3d getCameraPose_TargetSpace() {
			return toPose3D(cameraPose_TargetSpace);
		}

		public Pose3d getRobotPose_FieldSpace() {
			return toPose3D(robotPose_FieldSpace);
		}

		public Pose3d getRobotPose_TargetSpace() {
			return toPose3D(robotPose_TargetSpace);
		}

		public Pose3d getTargetPose_CameraSpace() {
			return toPose3D(targetPose_CameraSpace);
		}

		public Pose3d getTargetPose_RobotSpace() {
			return toPose3D(targetPose_RobotSpace);
		}

		public Pose2d getCameraPose_TargetSpace2D() {
			return toPose2D(cameraPose_TargetSpace);
		}

		public Pose2d getRobotPose_FieldSpace2D() {
			return toPose2D(robotPose_FieldSpace);
		}

		public Pose2d getRobotPose_TargetSpace2D() {
			return toPose2D(robotPose_TargetSpace);
		}

		public Pose2d getTargetPose_CameraSpace2D() {
			return toPose2D(targetPose_CameraSpace);
		}

		public Pose2d getTargetPose_RobotSpace2D() {
			return toPose2D(targetPose_RobotSpace);
		}

		@JsonProperty("ta")
		public double ta;

		@JsonProperty("tx")
		public double tx;

		@JsonProperty("txp")
		public double tx_pixels;

		@JsonProperty("ty")
		public double ty;

		@JsonProperty("typ")
		public double ty_pixels;

		@JsonProperty("ts")
		public double ts;

		public LimelightTarget_Retro() {
			cameraPose_TargetSpace = new double[6];
			robotPose_FieldSpace = new double[6];
			robotPose_TargetSpace = new double[6];
			targetPose_CameraSpace = new double[6];
			targetPose_RobotSpace = new double[6];
		}

	}

	public class LimelightTarget_Fiducial {

		@JsonProperty("fID")
		public double fiducialID;

		@JsonProperty("fam")
		public String fiducialFamily;

		@JsonProperty("t6c_ts")
		private double[] cameraPose_TargetSpace;

		@JsonProperty("t6r_fs")
		private double[] robotPose_FieldSpace;

		@JsonProperty("t6r_ts")
		private double[] robotPose_TargetSpace;

		@JsonProperty("t6t_cs")
		private double[] targetPose_CameraSpace;

		@JsonProperty("t6t_rs")
		private double[] targetPose_RobotSpace;

		public Pose3d getCameraPose_TargetSpace() {
			return toPose3D(cameraPose_TargetSpace);
		}

		public Pose3d getRobotPose_FieldSpace() {
			return toPose3D(robotPose_FieldSpace);
		}

		public Pose3d getRobotPose_TargetSpace() {
			return toPose3D(robotPose_TargetSpace);
		}

		public Pose3d getTargetPose_CameraSpace() {
			return toPose3D(targetPose_CameraSpace);
		}

		public Pose3d getTargetPose_RobotSpace() {
			return toPose3D(targetPose_RobotSpace);
		}

		public Pose2d getCameraPose_TargetSpace2D() {
			return toPose2D(cameraPose_TargetSpace);
		}

		public Pose2d getRobotPose_FieldSpace2D() {
			return toPose2D(robotPose_FieldSpace);
		}

		public Pose2d getRobotPose_TargetSpace2D() {
			return toPose2D(robotPose_TargetSpace);
		}

		public Pose2d getTargetPose_CameraSpace2D() {
			return toPose2D(targetPose_CameraSpace);
		}

		public Pose2d getTargetPose_RobotSpace2D() {
			return toPose2D(targetPose_RobotSpace);
		}

		@JsonProperty("ta")
		public double ta;

		@JsonProperty("tx")
		public double tx;

		@JsonProperty("txp")
		public double tx_pixels;

		@JsonProperty("ty")
		public double ty;

		@JsonProperty("typ")
		public double ty_pixels;

		@JsonProperty("ts")
		public double ts;

		public LimelightTarget_Fiducial() {
			cameraPose_TargetSpace = new double[6];
			robotPose_FieldSpace = new double[6];
			robotPose_TargetSpace = new double[6];
			targetPose_CameraSpace = new double[6];
			targetPose_RobotSpace = new double[6];
		}
	}

	public class LimelightTarget_Classifier {

		@JsonProperty("class")
		public String className;

		@JsonProperty("classID")
		public double classID;

		@JsonProperty("conf")
		public double confidence;

		@JsonProperty("zone")
		public double zone;

		@JsonProperty("tx")
		public double tx;

		@JsonProperty("txp")
		public double tx_pixels;

		@JsonProperty("ty")
		public double ty;

		@JsonProperty("typ")
		public double ty_pixels;

		public LimelightTarget_Classifier() {
		}
	}

	public class LimelightTarget_Detector {

		@JsonProperty("class")
		public String className;

		@JsonProperty("classID")
		public double classID;

		@JsonProperty("conf")
		public double confidence;

		@JsonProperty("ta")
		public double ta;

		@JsonProperty("tx")
		public double tx;

		@JsonProperty("txp")
		public double tx_pixels;

		@JsonProperty("ty")
		public double ty;

		@JsonProperty("typ")
		public double ty_pixels;

		public LimelightTarget_Detector() {
		}
	}

	public class Results {

		@JsonProperty("pID")
		public double pipelineID;

		@JsonProperty("tl")
		public double latency_pipeline;

		@JsonProperty("cl")
		public double latency_capture;

		public double latency_jsonParse;

		@JsonProperty("ts")
		public double timestamp_LIMELIGHT_publish;

		@JsonProperty("ts_rio")
		public double timestamp_RIOFPGA_capture;

		@JsonProperty("v")
		@JsonFormat(shape = Shape.NUMBER)
		public boolean valid;

		@JsonProperty("botpose")
		public double[] botpose;

		@JsonProperty("botpose_wpired")
		public double[] botpose_wpired;

		@JsonProperty("botpose_wpiblue")
		public double[] botpose_wpiblue;

		@JsonProperty("t6c_rs")
		public double[] camerapose_robotspace;

		public Pose3d getBotPose3d() {
			return toPose3D(botpose);
		}

		public Pose3d getBotPose3d_wpiRed() {
			return toPose3D(botpose_wpired);
		}

		public Pose3d getBotPose3d_wpiBlue() {
			return toPose3D(botpose_wpiblue);
		}

		public Pose2d getBotPose2d() {
			return toPose2D(botpose);
		}

		public Pose2d getBotPose2d_wpiRed() {
			return toPose2D(botpose_wpired);
		}

		public Pose2d getBotPose2d_wpiBlue() {
			return toPose2D(botpose_wpiblue);
		}

		@JsonProperty("Retro")
		public LimelightTarget_Retro[] targets_Retro;

		@JsonProperty("Fiducial")
		public LimelightTarget_Fiducial[] targets_Fiducials;

		@JsonProperty("Classifier")
		public LimelightTarget_Classifier[] targets_Classifier;

		@JsonProperty("Detector")
		public LimelightTarget_Detector[] targets_Detector;

		public Results() {
			botpose = new double[6];
			botpose_wpired = new double[6];
			botpose_wpiblue = new double[6];
			camerapose_robotspace = new double[6];
			targets_Retro = new LimelightTarget_Retro[0];
			targets_Fiducials = new LimelightTarget_Fiducial[0];
			targets_Classifier = new LimelightTarget_Classifier[0];
			targets_Detector = new LimelightTarget_Detector[0];
		}
	}

	public class LimelightResults {
		@JsonProperty("Results")
		public Results targetingResults;

		public LimelightResults() {
			targetingResults = new Results();
		}
	}

	private ObjectMapper mapper;

	/**
	 * Print JSON Parse time to the console in milliseconds
	 */
	static boolean profileJSON = false;

	private Pose3d toPose3D(double[] inData) {
		if (inData.length < 6) {
			System.err.println("Bad LL 3D Pose Data!");
			return new Pose3d();
		}
		return new Pose3d(
				new Translation3d(inData[0], inData[1], inData[2]),
				new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
						Units.degreesToRadians(inData[5])));
	}

	private Pose2d toPose2D(double[] inData) {
		if (inData.length < 6) {
			System.err.println("Bad LL 2D Pose Data!");
			return new Pose2d();
		}
		Translation2d tran2d = new Translation2d(inData[0], inData[1]);
		Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
		return new Pose2d(tran2d, r2d);
	}

	public NetworkTable getLimelightNTTable() {
		return NetworkTableInstance.getDefault().getTable(name);
	}

	public NetworkTableEntry getLimelightNTTableEntry(String entryName) {
		return getLimelightNTTable().getEntry(entryName);
	}

	public double getLimelightNTDouble(String entryName) {
		return getLimelightNTTableEntry(entryName).getDouble(0.0);
	}

	public void setLimelightNTDouble(String entryName, double val) {
		getLimelightNTTableEntry(entryName).setDouble(val);
	}

	public void setLimelightNTDoubleArray(String entryName, double[] val) {
		getLimelightNTTableEntry(entryName).setDoubleArray(val);
	}

	public double[] getLimelightNTDoubleArray(String entryName) {
		return getLimelightNTTableEntry(entryName).getDoubleArray(new double[0]);
	}

	public String getLimelightNTString(String entryName) {
		return getLimelightNTTableEntry(entryName).getString("");
	}

	public URL getLimelightURLString(String request) {
		String urlString = "http://" + name + ".local:5807/" + request;
		URL url;
		try {
			url = new URL(urlString);
			return url;
		} catch (MalformedURLException e) {
			System.err.println("bad LL URL");
		}
		return null;
	}

	public double getTX() {
		return getLimelightNTDouble("tx");
	}

	public double getTY() {
		return getLimelightNTDouble("ty");
	}

	public double getTA() {
		return getLimelightNTDouble("ta");
	}

	public double getLatency_Pipeline() {
		return getLimelightNTDouble("tl");
	}

	public double getLatency_Capture() {
		return getLimelightNTDouble("cl");
	}

	public double getCurrentPipelineIndex() {
		return getLimelightNTDouble("getpipe");
	}

	public String getJSONDump() {
		return getLimelightNTString("json");
	}

	/**
	 * Switch to getBotPose
	 * 
	 * @param limelightName
	 * @return
	 */
	public double[] getBotPose() {
		return getLimelightNTDoubleArray("botpose");
	}

	public double[] getBotPose_TargetSpace() {
		return getLimelightNTDoubleArray("botpose_targetspace");
	}

	public double[] getCameraPose_TargetSpace() {
		return getLimelightNTDoubleArray("camerapose_targetspace");
	}

	public double[] getTargetPose_CameraSpace() {
		return getLimelightNTDoubleArray("targetpose_cameraspace");
	}

	public double[] getTargetPose_RobotSpace() {
		return getLimelightNTDoubleArray("targetpose_robotspace");
	}

	public double[] getTargetColor() {
		return getLimelightNTDoubleArray("tc");
	}

	public double getFiducialID() {
		return getLimelightNTDouble("tid");
	}

	public double getNeuralClassID() {
		return getLimelightNTDouble("tclass");
	}

	public double[] getAllianceBotPose() {
		Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			if (alliance.get() == DriverStation.Alliance.Blue) {
				return getLimelightNTDoubleArray("botpose_wpiblue");
			} else {
				return getLimelightNTDoubleArray("botpose_wpired");
			}
		}
		return getLimelightNTDoubleArray("botpose");
	}

	/**
	 * Gets the Pose2d for easy use with Odometry vision pose estimator
	 * (addVisionMeasurement)
	 * 
	 * @param limelightName
	 * @return
	 */
	public Pose2d getAllianceBotPose2d() {
		return toPose2D(getAllianceBotPose());
	}

	public Pose3d getAllianceBotPose3d() {
		return toPose3D(getAllianceBotPose());
	}

	/**
	 * Gets the Pose2d for easy use with Odometry vision pose estimator
	 * (addVisionMeasurement)
	 * 
	 * @param limelightName
	 * @return
	 */
	public Pose2d getBotPose2d() {

		double[] result = getBotPose();
		return toPose2D(result);

	}

	public boolean getTV() {
		return 1.0 == getLimelightNTDouble("tv");
	}

	public void setPipelineIndex(int pipelineIndex) {
		setLimelightNTDouble("pipeline", pipelineIndex);
	}

	/**
	 * The LEDs will be controlled by Limelight pipeline settings, and not by robot
	 * code.
	 */
	public void setLEDMode_PipelineControl() {
		setLimelightNTDouble("ledMode", 0);
	}

	public void setLEDMode_ForceOff() {
		setLimelightNTDouble("ledMode", 1);
	}

	public void setLEDMode_ForceBlink() {
		setLimelightNTDouble("ledMode", 2);
	}

	public void setLEDMode_ForceOn() {
		setLimelightNTDouble("ledMode", 3);
	}

	public void setCameraMode_Driver() {
		setLimelightNTDouble("camMode", 1);
	}

	/**
	 * Sets the crop window. The crop window in the UI must be completely open for
	 * dynamic cropping to work.
	 */
	public void setCropWindow(double cropXMin, double cropXMax, double cropYMin,
			double cropYMax) {
		double[] entries = new double[4];
		entries[0] = cropXMin;
		entries[1] = cropXMax;
		entries[2] = cropYMin;
		entries[3] = cropYMax;
		setLimelightNTDoubleArray("crop", entries);
	}

	public void setCameraPose_RobotSpace(double forward, double side, double up,
			double roll, double pitch, double yaw) {
		double[] entries = new double[6];
		entries[0] = forward;
		entries[1] = side;
		entries[2] = up;
		entries[3] = roll;
		entries[4] = pitch;
		entries[5] = yaw;
		setLimelightNTDoubleArray("camerapose_robotspace_set", entries);
	}

	/**
	 * Parses Limelight's JSON results dump into a LimelightResults Object
	 */
	public LimelightResults getLatestResults() {

		long start = System.nanoTime();
		LimelightResults results = new LimelightResults();
		if (mapper == null) {
			mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
		}

		try {
			results = mapper.readValue(getJSONDump(), LimelightResults.class);
		} catch (JsonProcessingException e) {
			System.err.println("lljson error: " + e.getMessage());
		}

		long end = System.nanoTime();
		double millis = (end - start) * .000001;
		results.targetingResults.latency_jsonParse = millis;
		if (profileJSON) {
			System.out.printf("lljson: %.2f\r\n", millis);
		}

		return results;
	}
}