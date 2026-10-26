package com.microdmm.monitor120v

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeoutOrNull
import java.util.concurrent.ConcurrentHashMap

sealed interface LinkEvent {
    data class Line(val text: String) : LinkEvent
    data class Tx(val text: String) : LinkEvent
    class Pwr(val rec: PwrRecord) : LinkEvent
    class Cap(val cap: Capture) : LinkEvent
    data class Error(val text: String) : LinkEvent
}

/**
 * Serial link to the 120VMonitor firmware (port of monitor_link.py).
 *
 * One reader thread owns the port. Every line goes to [onEvent] except capture
 * data, which is assembled into a Capture first. Request helpers register a
 * one-shot waiter for the reply prefix they expect, so they can be called from
 * any coroutine.
 */
class MonitorLink(private val io: SerialIo, private val onEvent: (LinkEvent) -> Unit) {

    val cfg = ConcurrentHashMap<String, CfgRow>()          // key -> row
    val cfgOrder = mutableListOf<String>()                 // firmware's key order
    val status = ConcurrentHashMap<String, String>()
    private val waiters = HashMap<String, MutableList<Channel<Any>>>()
    private var cap: Capture? = null
    private var capWant = 0                                // int16 values the $CAP header promised
    private var capBad = 0                                 // $CD lines that didn't parse
    private var dumpSnap: Snapshot? = null
    @Volatile private var alive = true
    private val rx = Thread(::reader, "monitor-rx").apply { isDaemon = true }

    fun start() = rx.start()

    fun close() {
        try {
            send("!STOP")
            Thread.sleep(150)                              // let the board finish its reply
        } catch (_: Exception) {
        }
        alive = false
        io.disconnect()
    }

    fun send(cmd: String) {
        io.write(cmd + "\n")
        onEvent(LinkEvent.Tx(cmd))
    }

    private fun reader() {
        val buf = ByteArray(4096)
        val line = StringBuilder()
        while (alive) {
            val n = try {
                io.read(buf, 0)                            // 0 = block: the lossless USB path (close() ends it)
            } catch (e: Exception) {
                if (alive) onEvent(LinkEvent.Error("serial read failed: ${e.message}"))
                alive = false
                break
            }
            for (k in 0 until n) {
                val c = buf[k].toInt().toChar()
                if (c == '\n') {
                    handleSafely(line.toString().trim())
                    line.setLength(0)
                } else {
                    // every line starts with '$': one mid-line means bytes (and a newline) were lost, so
                    // end the damaged line there and keep the next one
                    if (c == '$' && line.isNotBlank()) {
                        handleSafely(line.toString().trim())
                        line.setLength(0)
                    }
                    if (line.length < 8192) line.append(c)
                }
            }
        }
    }

    /** A line that doesn't parse is logged, never fatal to the reader thread. */
    private fun handleSafely(line: String) {
        try {
            handle(line)
        } catch (e: Exception) {
            onEvent(LinkEvent.Line("!! unreadable line ($e): ${line.take(80)}"))
        }
    }

    private fun handle(line: String) {
        if (line.isEmpty()) return
        if (line.startsWith("\$CD,")) {
            val c = cap ?: return
            val hex = line.substring(4)
            val ok = hex.length % 4 == 0 && hex.all { it in '0'..'9' || it in 'A'..'F' || it in 'a'..'f' }
            if (!ok) { capBad++; return }                  // damaged in transfer: the count check reports it
            var k = 0
            while (k + 4 <= hex.length) {
                c.raw.add(hex.substring(k, k + 4).toInt(16).toShort())
                k += 4
            }
            return
        }
        if (line.startsWith("\$CAP,")) {
            val p = line.substring(5).split(",")
            cap = try {
                Capture(
                    ch = p[1].toInt(), dtUs = p[2].toDouble(), vPer = p[3].toDouble(), vOff = p[4].toDouble(),
                    iPer = p[5].toDouble(), iOff = p[6].toDouble(), trigIdx = p[7].toInt(), flags = p[8].toInt(),
                    relayIdx = p[9].toInt(),
                    openIdx = p.getOrNull(10)?.toInt() ?: -1,
                    openWhy = p.getOrNull(11)?.toInt() ?: 0,
                    avg = p.getOrNull(12)?.toInt() ?: 1,
                    rng = if (p.size > 17) p[13].toInt() else 0,
                    rngIdx = if (p.size > 17) p[14].toInt() else -1,
                    iPerHi = if (p.size > 17) p[15].toDouble() else null,
                    iOffHi = if (p.size > 17) p[16].toDouble() else null,
                    settleUs = if (p.size > 17) p[17].toDouble() else 0.0,
                )
            } catch (_: Exception) {
                null
            }
            capWant = (p[0].toIntOrNull() ?: 0) * (if (cap?.ch == 2) 2 else 1)
            capBad = 0
            onEvent(LinkEvent.Line(line))
            return
        }
        if (line.startsWith("\$CEND")) {
            val c = cap
            cap = null
            val got = c?.raw?.size ?: 0
            if (c == null || capBad > 0 || got != capWant) {
                // lost bytes: say so rather than plot a shifted or truncated waveform
                val why = if (c == null) "its header was lost" else "$got of $capWant values arrived"
                deliver("\$CEND", "\$ERR,data damaged in transfer ($why) - capture again")
                onEvent(LinkEvent.Line("!! capture damaged in transfer: $why"))
                return
            }
            c.finish()
            deliver("\$CEND", c)
            onEvent(LinkEvent.Cap(c))
            return
        }
        when {
            line.startsWith("\$CFG,") -> {
                val p = line.substring(5).split(",", limit = 5)
                if (p.size >= 4) {
                    val v = p[1].toDoubleOrNull()
                    val lo = p[2].toDoubleOrNull()
                    val hi = p[3].toDoubleOrNull()
                    if (v != null && lo != null && hi != null) {
                        synchronized(cfgOrder) { if (!cfg.containsKey(p[0])) cfgOrder += p[0] }
                        cfg[p[0]] = CfgRow(p[0], v, lo, hi, p.getOrElse(4) { "" })
                    }
                }
            }
            line.startsWith("\$DUMPHDR,") -> {
                val meta = linkedMapOf<String, String>()
                for (kv in line.substring(9).split(",")) {
                    val e = kv.indexOf('=')
                    if (e > 0) meta[kv.substring(0, e)] = kv.substring(e + 1)
                }
                dumpSnap = Snapshot(meta)
            }
            line.startsWith("\$DUMP,") -> {
                val p = line.substring(6).split(",")
                val d = dumpSnap
                if (d != null && p.size >= 3) p[2].toDoubleOrNull()?.let { d.rows += SnapRow(p[0], p[1], it) }
            }
            line.startsWith("\$STATUS,") -> {
                for (kv in line.substring(8).split(",")) {
                    val e = kv.indexOf('=')
                    if (e > 0) status[kv.substring(0, e)] = kv.substring(e + 1)
                }
            }
            line.startsWith("\$PWR,") -> {
                val vals = line.substring(5).split(",").map { it.toDoubleOrNull() }
                if (vals.all { it != null }) {
                    onEvent(LinkEvent.Pwr(PwrRecord(PWR_FIELDS.zip(vals.map { it!! }).toMap())))
                }
                return
            }
        }
        onEvent(LinkEvent.Line(line))
        deliver(line.substringBefore(','), line)
    }

    private fun deliver(prefix: String, obj: Any) {
        synchronized(waiters) { waiters[prefix]?.forEach { it.trySend(obj) } }
    }

    /** Send cmd and return the first reply whose prefix is in `prefixes`. */
    suspend fun request(cmd: String, prefixes: List<String>, timeoutMs: Long = 3000): Any {
        val ch = Channel<Any>(Channel.UNLIMITED)
        synchronized(waiters) { prefixes.forEach { waiters.getOrPut(it) { mutableListOf() } += ch } }
        try {
            withContext(Dispatchers.IO) { send(cmd) }     // never block the caller's (main) thread
            val obj = withTimeoutOrNull(timeoutMs) { ch.receive() }
                ?: throw LinkException("no reply to $cmd within ${timeoutMs / 1000.0} s")
            if (obj is String && obj.startsWith("\$ERR")) throw LinkException(obj.removePrefix("\$ERR,"))
            return obj
        } finally {
            synchronized(waiters) { prefixes.forEach { waiters[it]?.remove(ch) } }
        }
    }

    // ── commands ────────────────────────────────────────────────────────────
    suspend fun identify() = request("!ID", listOf("\$READY"), 2000) as String

    suspend fun readCfg(): Map<String, CfgRow> {
        request("!CFG", listOf("\$CFGEND", "\$ERR"), 3000)
        return cfg
    }

    fun cfgRows(): List<CfgRow> = synchronized(cfgOrder) { cfgOrder.mapNotNull { cfg[it] } }

    suspend fun getStatus(): Map<String, String> {
        request("!STATUS", listOf("\$STATUS"), 2000)
        return status
    }

    /** Snapshot of every key and its group: the running values, or with saved
     *  the flash/EEPROM block. Firmware before 1.4 has no !DUMP; then it is
     *  !CFG (RAM only), the group table and no uid. */
    suspend fun dump(saved: Boolean = false): Snapshot {
        val snap = try {
            request(if (saved) "!DUMP,SAVED" else "!DUMP", listOf("\$DUMPEND", "\$ERR"), 3000)
            dumpSnap.also { dumpSnap = null } ?: throw LinkException("empty dump")
        } catch (e: LinkException) {
            if (e.message?.contains("unknown command") != true) throw e
            readCfg()
            val st = getStatus()
            Snapshot(
                linkedMapOf(
                    "fw" to (st["fw"] ?: "?"), "board" to (st["board"] ?: "RA4M1"), "uid" to "",
                    "cfgver" to "?", "src" to "RAM-CFG", "dirty" to (st["dirty"] ?: "?"),
                ),
                cfgRows().map { SnapRow(GROUP_OF[it.key] ?: "OTHER", it.key, it.value) }.toMutableList(),
            )
        }
        snap.meta["taken"] = nowStamp()
        return snap
    }

    suspend fun set(key: String, value: Double) {
        request("!SET,$key,${fmtG(value, 7)}", listOf("\$CFG", "\$ERR"), 2000)
    }

    suspend fun save() {
        request("!SAVE", listOf("\$OK", "\$ERR"), 3000)
    }

    suspend fun relay(on: Boolean) {
        request("!RELAY,${if (on) 1 else 0}", listOf("\$RLY"), 2000)
    }

    suspend fun capture(a: CapArgs): Capture {
        val cmd = "!CAP,${a.rate.toLong()},${a.n},${a.ch},${a.trig},${fmtG(a.level, 6)},${fmtG(a.pre, 6)}," +
            "${a.timeoutMs},${a.relayUs},${fmtG(a.ipkA, 6)},${fmtG(a.ovA, 6)},${a.ovCyc},${a.autoMs}," +
            "${if (a.openAfter) 1 else 0},${a.avg},${a.rng},${if (a.mon) 1 else 0}"
        val wait = if (a.timeoutMs > 0) a.timeoutMs + 15000L else Long.MAX_VALUE   // 0: until abort()
        val obj = request(cmd, listOf("\$CEND", "\$CAPTMO", "\$ERR"), wait)
        if (obj is Capture) return obj
        val s = obj.toString()
        throw LinkException(
            when {
                !s.startsWith("\$CAPTMO") -> s
                "ABORT" in s -> "aborted"
                else -> "timed out waiting for the trigger"
            },
        )
    }

    /** Stop a capture that is waiting for its trigger; capture() then throws. */
    fun abort() = send("!ABORT")
}

/** !CAP arguments. Safe start (0 = off): ipkA instant trip; ovA for ovCyc cycles
 *  of per-cycle Irms; autoMs opens that long after the trigger; openAfter at the
 *  end. avg: scans per stored sample. rng: -1 follow RNGMODE, 0 HIGH, 1 LOW.
 *  timeoutMs 0 waits until abort(). */
data class CapArgs(
    val rate: Double, val n: Int, val ch: Int, val trig: Int, val level: Double, val pre: Double,
    val timeoutMs: Int, val relayUs: Int, val ipkA: Double, val ovA: Double, val ovCyc: Int,
    val autoMs: Int, val openAfter: Boolean, val avg: Int, val rng: Int,
    val mon: Boolean = false,       // keep $PWR readings coming while armed
)
