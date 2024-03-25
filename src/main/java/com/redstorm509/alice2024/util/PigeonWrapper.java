package com.redstorm509.alice2024.util;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonWrapper {
	private Pigeon2 device;
	private double yawOffset = 0.0d;
	private double bootRoll;
	private double bootPitch;

	public PigeonWrapper(int deviceId, String bus) {
		this.device = new Pigeon2(deviceId, bus);
	}

	public void onEnable() {
		bootRoll = device.getRoll().getValue();
		bootPitch = device.getPitch().getValue();
	}

	public double getYaw() {
		return device.getYaw().getValue() + yawOffset;
	}

	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(device.getYaw().getValue() + yawOffset);
	}

	public void setYaw(double yaw) {
		yawOffset = yaw - device.getYaw().getValue();
	}

	public double getPitch() {
		return device.getPitch().getValueAsDouble() - bootPitch;
	}

	public double getRoll() {
		return device.getRoll().getValueAsDouble() - bootRoll;
	}

	public StatusSignal<Double> getAngularVelocityZWorld() {
		return device.getAngularVelocityZWorld();
	}
}
