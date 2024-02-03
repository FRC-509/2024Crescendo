package com.redstorm509.alice2024.util.devices;

import edu.wpi.first.wpilibj.PWM;

public class REVBlinkin {
	private PWM blinkin;

	public REVBlinkin(int pwmPort) {
		blinkin = new PWM(pwmPort);
	}

	public void setMode(BlinkinLedMode mode) {
		blinkin.setPulseTimeMicroseconds(mode.value);
	}

	public static enum BlinkinLedMode {
		// @formatter:off
	
		// --- Fixed palette patterns ---
		FixedRainbowRainbow(1005),
		FixedRainbowParty(1015),
		FixedRainbowOcean(1025),
		FixedRainbowLava(1035),
		FixedRainbowForest(1045), 
		FixedRainbowGlitter(1055),
		FixedConfetti(1065),
		FixedShotRed(1075),
		FixedShotBlue(1085),
		FixedShotWhite(1095),
		FixedSinelonRainbow(1105),
		FixedSinelonParty(1115),
		FixedSinelonOcean(1125),
		FixedSinelonLava(1135),
		FixedSinelonForest(1145),
		FixedBeatsRainbow(1155),
		FixedBeatsParty(1165),
		FixedBeatsOcean(1175),
		FixedBeatsLava(1185),
		FixedBeatsForest(1195),
		FixedFireMedium(1205),
		FixedFireLarge(1215),
		FixedTwinklesRainbow(1225),
		FixedTwinklesParty(1235),
		FixedTwinklesOcean(1245),
		FixedTwinklesLava(1255),
		FixedTwinklesForest(1265),
		FixedWavesRainbow(1275),
		FixedWavesParty(1285),
		FixedWavesOcean(1295),
		FixedWavesLava(1305),
		FixedWavesForest(1315),
		FixedLarsonRed(1325),
		FixedLarsonGray(1335),
		FixedChaseRed(1345),
		FixedChaseBlue(1355),
		FixedChaseGray(1365),
		FixedHeartbeatRed(1375),
		FixedHeartbeatBlue(1385),
		FixedHeartbeatWhite(1395),
		FixedHeartbeatGray(1405),
		FixedBreathRed(1415),
		FixedBreathBlue(1425),
		FixedBreathGray(1435),
		FixedStrobeRed(1445),
		FixedStrobeBlue(1455),
		FixedStrobeGold(1465),
		FixedStrobeWhite(1475),
	
		// --- Color one patterns ---
		OneBlendToBlack(1485),
		OneLarson(1495),
		OneChase(1505),
		OneHeartbeatSlow(1515),
		OneHeartbeatMedium(1525),
		OneHeartbeatFast(1535),
		OneBreathSlow(1545),
		OneBreathFast(1555),
		OneShot(1565),
		OneStrobe(1575),
	
		// --- Color two patterns ---
		TwoBlendToBlack(1585),
		TwoLarson(1595),
		TwoChase(1605),
		TwoHeartbeatSlow(1615),
		TwoHeartbeatMedium(1625),
		TwoHeartbeatFast(1635),
		TwoBreathSlow(1645),
		TwoBreathFast(1655),
		TwoShot(1665),
		TwoStrobe(1675),
	
		// --- Combined color patterns ---
		BothSparkleOneOnTwo(1685),
		BothSparkleTwoOnOne(1695),
		BothGradient(1705),
		BothBeats(1715),
		BothBlendOneToTwo(1725),
		BothBlend(1735),
		BothNoBlend(1745),
		BothTwinkles(1755),
		BothWaves(1765),
		BothSinelon(1775),
	
		// --- Solid colors ---
		SolidHotPink(1785),
		SolidDarkRed(1795),
		SolidRed(1805),
		SolidRedOrange(1815),
		SolidOrange(1825),
		SolidGold(1835),
		SolidYellow(1845),
		SolidLawnGreen(1855),
		SolidLime(1865),
		SolidDarkGreen(1875),
		SolidGreen(1885),
		SolidBlueGreen(1895),
		SolidAqua(1905),
		SolidSkyBlue(1915),
		SolidDarkBlue(1925),
		SolidBlue(1935),
		SolidBlueViolet(1945),
		SolidViolet(1955),
		SolidWhite(1965),
		SolidGray(1975),
		SolidDarkGray(1985),
		SolidBlack(1995);
	
		// @formatter:on

		private final int value;

		BlinkinLedMode(int value) {
			this.value = value;
		}
	}

}
