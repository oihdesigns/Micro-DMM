package com.microdmm.monitor120v

import java.io.IOException
import java.net.InetSocketAddress
import java.net.Socket
import java.net.SocketTimeoutException

/** What MonitorLink needs from a port: USB serial or a TCP socket. */
interface SerialIo {
    /** Bytes read, 0 on timeout (timeoutMs 0 = wait for data); throws once the port is gone. */
    fun read(buffer: ByteArray, timeoutMs: Int): Int
    fun write(text: String)
    fun disconnect()
}

/** The monitor protocol over TCP: sim_server.py (the simulator), or a board
 *  behind a serial-to-network bridge. "host:port", port default 8120. */
class TcpIo(hostPort: String) : SerialIo {
    private val socket = Socket()
    private val txLock = Any()

    init {
        val host = hostPort.substringBefore(':').trim()
        val port = hostPort.substringAfter(':', "8120").trim().toIntOrNull() ?: 8120
        socket.connect(InetSocketAddress(host, port), 3000)
        socket.tcpNoDelay = true
    }

    override fun read(buffer: ByteArray, timeoutMs: Int): Int {
        socket.soTimeout = timeoutMs
        return try {
            val n = socket.getInputStream().read(buffer)
            if (n < 0) throw IOException("connection closed") else n
        } catch (_: SocketTimeoutException) {
            0
        }
    }

    override fun write(text: String) {
        synchronized(txLock) {
            socket.getOutputStream().write(text.toByteArray(Charsets.US_ASCII))
            socket.getOutputStream().flush()
        }
    }

    override fun disconnect() {
        try {
            socket.close()
        } catch (_: Exception) {
        }
    }
}
