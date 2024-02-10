package com.redstorm509.alice2024.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeStamp extends SubsystemBase {
	double previousTime;
	double delta;

	public TimeStamp() {
		this.previousTime = Timer.getFPGATimestamp();
		this.delta = 0.0d;
	}

	public void update() {
		double currentTime = Timer.getFPGATimestamp();
		delta = currentTime - previousTime;
		previousTime = currentTime;
	}

	public double deltaTime() {
		return delta;
	}
}
