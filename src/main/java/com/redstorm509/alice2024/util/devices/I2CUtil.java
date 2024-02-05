// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.redstorm509.alice2024.util.devices;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.wpilibj.I2C;

public class I2CUtil implements AutoCloseable {
	private final int port;
	private final byte devAddr;

	public I2CUtil(I2C.Port port, byte devAddr) {
		this.port = port.value;
		this.devAddr = devAddr;
		I2CJNI.i2CInitialize(this.port);
	}

	/**
	 * Generic transaction.
	 *
	 * <p>
	 * This is a lower-level interface to the I2C hardware giving you more control
	 * over each transaction. If you intend to write multiple bytes in the same
	 * transaction and do not plan to receive anything back, use writeBulk()
	 * instead. Calling this with a receiveSize of 0 will result in an error.
	 *
	 * @param dataToSend   Buffer of data to send as part of the transaction.
	 * @param sendSize     Number of bytes to send as part of the transaction.
	 * @param dataReceived Buffer to read data into.
	 * @param receiveSize  Number of bytes to read from the device.
	 * @return True for success, false for aborted.
	 */
	public synchronized boolean transaction(byte[] dataToSend, byte sendSize, byte[] dataReceived, byte receiveSize) {
		if (dataToSend.length < sendSize) {
			throw new IllegalArgumentException("dataToSend is too small, must be at least " + sendSize);
		}
		if (dataReceived.length < receiveSize) {
			throw new IllegalArgumentException("dataReceived is too small, must be at least " + receiveSize);
		}
		return I2CJNI.i2CTransactionB(port, devAddr, dataToSend, sendSize, dataReceived, receiveSize) >= 0;
	}

	/**
	 * Execute a write transaction with the device.
	 *
	 * <p>
	 * Write multiple bytes to a register on a device and wait until the transaction
	 * is complete.
	 *
	 * @param data The data to write to the device.
	 * @param size The number of data bytes to write.
	 * @return True for success, false for aborted.
	 */
	public synchronized boolean write(byte[] data) {
		if (data.length > Byte.MAX_VALUE) {
			throw new IllegalArgumentException("buffer is too large, must be at most " + Byte.MAX_VALUE);
		}
		return I2CJNI.i2CWriteB(port, devAddr, data, (byte) data.length) >= 0;
	}

	/**
	 * Execute a read only transaction with the device.
	 *
	 * <p>
	 * Read bytes from a device. This method does not write any data to prompt the
	 * device.
	 *
	 * @param buffer A pointer to the array of bytes to store the data read from the
	 *               device.
	 * @param count  The number of bytes to read in the transaction.
	 * @return True for success, false for aborted.
	 */
	public boolean read(byte[] buffer, byte count) {
		if (count < 1) {
			throw new BoundaryException("Value must be at least 1, " + count + " given");
		}
		if (buffer.length < count) {
			throw new IllegalArgumentException("buffer is too small, must be at least " + count);
		}

		return I2CJNI.i2CReadB(port, devAddr, buffer, count) >= 0;
	}

	/**
	 * Execute a read transaction with the device.
	 *
	 * <p>
	 * Read bytes from a device. Most I2C devices will auto-increment the register
	 * pointer internally allowing you to read consecutive registers on a device in
	 * a single transaction.
	 *
	 * @param registerAddress The register to read first in the transaction.
	 * @param count           The number of bytes to read in the transaction.
	 * @param buffer          A pointer to the array of bytes to store the data read
	 *                        from the device.
	 * @return True for success, false for aborted.
	 */
	public boolean readFromAddress8bit(byte registerAddress, byte count, byte[] buffer) {
		if (count < 1) {
			throw new BoundaryException("Value must be at least 1, " + count + " given");
		}
		if (buffer.length < count) {
			throw new IllegalArgumentException("buffer is too small, must be at least " + count);
		}

		byte[] registerAddressArray = new byte[1];
		registerAddressArray[0] = registerAddress;

		return transaction(registerAddressArray, (byte) 1, buffer, count);
	}

	/**
	 * Execute a read transaction with the device.
	 *
	 * <p>
	 * Read bytes from a device. Most I2C devices will auto-increment the register
	 * pointer internally allowing you to read consecutive registers on a device in
	 * a single transaction.
	 *
	 * @param registerAddress The register to read first in the transaction.
	 * @param count           The number of bytes to read in the transaction.
	 * @param buffer          A pointer to the array of bytes to store the data read
	 *                        from the device.
	 * @return True for success, false for aborted.
	 */
	public boolean readFromAddress16bit(short registerAddress, byte count, byte[] buffer) {
		if (count < 1) {
			throw new BoundaryException("Value must be at least 1, " + count + " given");
		}
		if (buffer.length < count) {
			throw new IllegalArgumentException("buffer is too small, must be at least " + count);
		}

		byte[] registerAddressArray = new byte[2];
		registerAddressArray[0] = (byte) (registerAddress >>> 8);
		registerAddressArray[1] = (byte) (registerAddress);

		return transaction(registerAddressArray, (byte) 2, buffer, count);
	}

	/**
	 * Execute a write transaction with the device.
	 *
	 * <p>
	 * Write a single byte to a register on a device and wait until the transaction
	 * is complete.
	 *
	 * @param registerAddress The address of the register on the device to be
	 *                        written.
	 * @param data            The byte to write to the register on the device.
	 * @return True for success, false for aborted.
	 */
	public synchronized boolean writeToAddress8bit(byte registerAddress, byte data) {
		byte[] buffer = new byte[2];
		buffer[0] = (byte) registerAddress;
		buffer[1] = (byte) data;
		return I2CJNI.i2CWriteB(port, devAddr, buffer, (byte) 2) >= 0;
	}

	/**
	 * Execute a write transaction with the device.
	 *
	 * <p>
	 * Write a single byte to a register on a device and wait until the transaction
	 * is complete.
	 *
	 * @param registerAddress The address of the register on the device to be
	 *                        written.
	 * @param data            The bytes to write to the register on the device.
	 * @return True for success, false for aborted.
	 */
	public synchronized boolean writeToAddress8bit(byte registerAddress, byte[] data) {
		byte[] buffer = new byte[data.length + 1];
		buffer[0] = (byte) registerAddress;
		System.arraycopy(data, 0, buffer, 1, data.length);
		return I2CJNI.i2CWriteB(port, devAddr, buffer, (byte) (data.length + 1)) >= 0;
	}

	/**
	 * Execute a write transaction with the device.
	 *
	 * <p>
	 * Write a single byte to a register on a device and wait until the transaction
	 * is complete.
	 *
	 * @param registerAddress The address of the register on the device to be
	 *                        written.
	 * @param data            The byte to write to the register on the device.
	 * @return True for success, false for aborted.
	 */
	public synchronized boolean writeToAddress16bit(short registerAddress, byte data) {
		byte[] buffer = new byte[3];
		buffer[0] = (byte) (registerAddress >>> 8);
		buffer[1] = (byte) (registerAddress);
		buffer[2] = (byte) data;
		return I2CJNI.i2CWriteB(port, devAddr, buffer, (byte) 3) >= 0;
	}

	/**
	 * Execute a write transaction with the device.
	 *
	 * <p>
	 * Write a single byte to a register on a device and wait until the transaction
	 * is complete.
	 *
	 * @param registerAddress The address of the register on the device to be
	 *                        written.
	 * @param data            The bytes to write to the register on the device.
	 * @return True for success, false for aborted.
	 */
	public synchronized boolean writeToAddress16bit(short registerAddress, byte[] data) {
		byte[] buffer = new byte[2 + data.length];
		buffer[0] = (byte) (registerAddress >>> 8);
		buffer[1] = (byte) (registerAddress);
		System.arraycopy(data, 0, buffer, 2, data.length);
		return I2CJNI.i2CWriteB(port, devAddr, buffer, (byte) (2 + data.length)) >= 0;
	}

	@Override
	public void close() {
		I2CJNI.i2CClose(port);
	}
}