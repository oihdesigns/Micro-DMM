package com.microdmm.monitor120v

import android.hardware.usb.UsbConstants
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber

/** USB vendor ids the monitor firmware runs on: ST (Feather F405), Seeed (XIAO), Arduino (UNO R4). */
private val MONITOR_VIDS = setOf(0x0483, 0x2886, 0x2341)

class UsbSerialManager(private val usbManager: UsbManager) : SerialIo {

    private var port: UsbSerialPort? = null
    private val txLock = Any()

    /** Every serial device: the prober's, plus any CDC-ACM device it doesn't list
     *  (the Feather's ST VCP and the XIAO enumerate as plain CDC). */
    private fun drivers(): List<UsbSerialDriver> {
        val found = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager).toMutableList()
        for (dev in usbManager.deviceList.values) {
            if (found.any { it.device == dev }) continue
            val cdc = (0 until dev.interfaceCount).any { dev.getInterface(it).interfaceClass == UsbConstants.USB_CLASS_COMM }
            if (cdc) found += CdcAcmSerialDriver(dev)
        }
        return found
    }

    /** The monitor if one is plugged in, else the first serial device. */
    fun findDevice(): UsbDevice? {
        val all = drivers()
        return (all.firstOrNull { it.device.vendorId in MONITOR_VIDS } ?: all.firstOrNull())?.device
    }

    /** Opens the port; throws with a reason on failure. */
    fun connect(device: UsbDevice) {
        val driver = drivers().firstOrNull { it.device == device } ?: error("no serial driver for this device")
        val connection = usbManager.openDevice(device) ?: error("USB permission denied")
        val p = driver.ports[0]
        p.open(connection)
        try {
            p.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
        } catch (_: UnsupportedOperationException) {
            // CDC devices may ignore line coding
        }
        p.dtr = true
        p.rts = true
        port = p
    }

    override fun read(buffer: ByteArray, timeoutMs: Int): Int {
        val p = port ?: throw java.io.IOException("port closed")
        return p.read(buffer, timeoutMs)
    }

    override fun write(text: String) {
        synchronized(txLock) {
            val p = port ?: throw java.io.IOException("not connected")
            p.write(text.toByteArray(Charsets.US_ASCII), 1000)
        }
    }

    override fun disconnect() {
        try {
            port?.close()
        } catch (_: Exception) {
            // already gone
        }
        port = null
    }

    val isOpen: Boolean get() = port != null
}
