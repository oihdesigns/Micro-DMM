package com.microdmm.monitor120v

import android.app.Application
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.net.Uri
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import android.util.Log
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.BufferedWriter
import java.io.File
import java.util.Locale
import kotlin.math.roundToInt
import kotlin.math.roundToLong

enum class Conn { Disconnected, Connecting, Connected }

const val TAG = "Monitor120V"

val TIMING_MODES = listOf("Rate + samples", "Rate + duration", "Samples + duration")
val CH_OPTIONS = linkedMapOf(2 to "V + I", 0 to "V only", 1 to "I only")
val CAP_RANGES = linkedMapOf(-1 to "Follow mode", 0 to "HIGH", 1 to "LOW")   // !CAP rng
val FADES = linkedMapOf("Infinite" to 0.0, "Fade 30 s" to 30.0, "Fade 5 s" to 5.0, "Fade 1 s" to 1.0)

/** One live-persistence panel as drawn: counts (row 0 = -lim), range and label. */
class LiveChan(val h: DoubleArray, val lim: Double, val unit: String)

class LiveView(val chans: List<LiveChan>, val hz: Double, val ref: String)

val SPANS = linkedMapOf("30 s" to 30, "2 min" to 120, "10 min" to 600, "1 h" to 3600, "all" to 0)

/** Capture tab fields, kept as typed text like the desktop GUI's entries. */
data class CapSettings(
    val rate: String = "20000", val n: String = "3000", val dur: String = "150",
    val tmode: Int = 0, val ch: Int = 2, val trig: Int = 2,
    val level: String = "0.5", val pre: String = "10", val timeoutS: String = "5", val relayUs: String = "0",
    val avg: String = "7", val rng: Int = -1,
    val ipkOn: Boolean = false, val ipkA: String = "10",
    val ovOn: Boolean = false, val ovA: String = "5", val ovCyc: String = "3",
    val autoOn: Boolean = false, val autoMs: String = "500",
    val openAfter: Boolean = false,
    val mon: Boolean = false,
    val repeat: Boolean = false, val showHarm: Boolean = true,
    val heatMap: Boolean = false,      // show the capture folded onto one cycle
)

data class ApplyState(
    val name: String,
    val snap: Snapshot,
    val here: Snapshot,
    val diffs: List<SnapDiff>,
    val missing: List<String>,
    val otherBoard: Boolean,
    val sameUnit: Boolean,
    val groups: List<String>,
    val counts: Map<String, Int>,
    val on: Set<String>,
    val busy: Boolean = false,
    val msg: String = "",
) {
    val todo get() = diffs.filter { it.group in on }
}

data class UiState(
    val conn: Conn = Conn.Disconnected,
    val connMsg: String = "not connected",
    val ident: String = "",
    val board: String = "RA4M1",
    val relay: Boolean = false,
    val streaming: Boolean = true,
    val twoRanges: Boolean = false,
    val rng: Int? = null,
    val rngMode: Int = 0,
    val rngNote: String = "",
    val alert: String = "",
    val last: PwrRecord? = null,
    val trend: List<PwrRecord> = emptyList(),
    val span: String = "2 min",
    val logName: String? = null,
    val cap: CapSettings = CapSettings(),
    val timingNote: String = "",
    val spikeText: String = "",
    val rateText: String = "",
    val capBusy: Boolean = false,
    val liveOn: Boolean = false,          // Meter tab: live persistence capture loop
    val live: LiveView? = null,
    val liveMsg: String = "",
    val fade: String = "Infinite",
    val capMsg: String = "",
    val capture: Capture? = null,
    val capStats: CapStats? = null,
    val confirmCapture: String? = null,
    val cfg: List<CfgRow> = emptyList(),
    val dirty: Boolean = false,
    val console: List<String> = emptyList(),
    val apply: ApplyState? = null,
    val logFiles: List<String> = emptyList(),
    val toast: String? = null,
)

sealed interface Action {
    data class Connect(val device: UsbDevice) : Action
    data class ConnectTcp(val hostPort: String) : Action
    data object Disconnect : Action
    data object ToggleRelay : Action
    data class SetStream(val on: Boolean) : Action
    data class SetRangeMode(val mode: Int) : Action
    data object ResetWh : Action
    data object ClearTrend : Action
    data class SetLive(val on: Boolean) : Action
    data class SetFade(val fade: String) : Action
    data object ClearLive : Action
    data class SetSpan(val span: String) : Action
    data class StartLog(val uri: Uri, val name: String) : Action
    data object StopLog : Action
    data class UpdateCap(val cap: CapSettings) : Action
    data object Capture : Action
    data object AbortCapture : Action
    data class ConfirmCapture(val go: Boolean) : Action
    data class SaveCapture(val uri: Uri) : Action
    data object CfgRefresh : Action
    data object CfgSave : Action
    data object CfgLoad : Action
    data object CfgDefaults : Action
    data object CfgZero : Action
    data class CfgSet(val key: String, val value: String) : Action
    data class Export(val uri: Uri, val saved: Boolean) : Action
    data object ListLogFiles : Action
    data class ApplyFromLog(val name: String) : Action
    data class ApplyFromUri(val uri: Uri, val name: String) : Action
    data class ApplyToggle(val group: String) : Action
    data class ApplyRun(val save: Boolean) : Action
    data object ApplyClose : Action
    data class Console(val cmd: String) : Action
    data object DismissToast : Action
}

class MonitorViewModel(app: Application) : AndroidViewModel(app) {

    private val _ui = MutableStateFlow(UiState())
    val ui: StateFlow<UiState> = _ui.asStateFlow()

    private lateinit var usb: UsbSerialManager
    private var link: MonitorLink? = null
    private val events = Channel<LinkEvent>(Channel.UNLIMITED)
    private val hist = ArrayDeque<PwrRecord>()
    private var csv: BufferedWriter? = null
    private var capJob: Job? = null
    private var liveJob: Job? = null
    private val persist = Persistence()           // guarded by synchronized(persist)
    private val console = ArrayDeque<String>()

    val cfgLogDir: File get() = File(getApplication<Application>().getExternalFilesDir(null), "config_log")

    init {
        viewModelScope.launch { for (e in events) onLinkEvent(e) }   // reader thread -> main, in order
        recalcTiming()
    }

    fun initSerial(usbManager: UsbManager) {
        if (!::usb.isInitialized) usb = UsbSerialManager(usbManager)
    }

    fun findDevice(): UsbDevice? = usb.findDevice()

    private fun toast(msg: String) = _ui.update { it.copy(toast = msg) }

    fun onAction(a: Action) {
        when (a) {
            is Action.Connect -> connect(a.device.productName ?: a.device.deviceName) { usb.connect(a.device); usb }
            is Action.ConnectTcp -> connect(a.hostPort) { TcpIo(a.hostPort) }
            Action.Disconnect -> disconnect()
            Action.ToggleRelay -> {
                safe = ""
                _ui.update { it.copy(alert = "") }
                cmd("!RELAY,${if (_ui.value.relay) 0 else 1}")
            }
            is Action.SetStream -> {
                _ui.update { it.copy(streaming = a.on) }
                cmd(if (a.on) "!START" else "!STOP")
            }
            is Action.SetRangeMode -> cmd("!RANGE,${a.mode}")
            Action.ResetWh -> cmd("!EZERO")
            Action.ClearTrend -> { hist.clear(); _ui.update { it.copy(trend = emptyList()) } }
            is Action.SetLive -> if (a.on) startLive() else _ui.update { it.copy(liveOn = false) }
            is Action.SetFade -> {
                synchronized(persist) { persist.tauS = FADES[a.fade] ?: 0.0 }
                _ui.update { it.copy(fade = a.fade) }
            }
            Action.ClearLive -> {
                synchronized(persist) { persist.clear() }
                _ui.update { it.copy(live = null, liveMsg = "") }
            }
            is Action.SetSpan -> { _ui.update { it.copy(span = a.span) }; updateTrend() }
            is Action.StartLog -> startLog(a.uri, a.name)
            Action.StopLog -> stopLog()
            is Action.UpdateCap -> { _ui.update { it.copy(cap = a.cap) }; recalcTiming() }
            Action.Capture -> requestCapture()
            Action.AbortCapture -> {
                _ui.update { it.copy(cap = it.cap.copy(repeat = false)) }
                val l = link ?: return
                viewModelScope.launch(Dispatchers.IO) { try { l.abort() } catch (_: Exception) {} }
                // a waiting board answers at once; if nothing comes (say the end of a capture was
                // lost in transfer), stop waiting here as well
                val job = capJob
                viewModelScope.launch {
                    delay(2000)
                    if (_ui.value.capBusy && capJob === job) {
                        job?.cancel()
                        _ui.update { it.copy(capBusy = false, capMsg = "capture: aborted (no reply from the board)") }
                    }
                }
            }
            is Action.ConfirmCapture -> {
                _ui.update { it.copy(confirmCapture = null) }
                if (a.go) runCapture()
            }
            is Action.SaveCapture -> saveCapture(a.uri)
            Action.CfgRefresh -> { cmd("!CFG"); cmd("!STATUS") }
            Action.CfgSave -> cmd("!SAVE")
            Action.CfgLoad -> cmdRefresh("!LOAD")
            Action.CfgDefaults -> cmdRefresh("!DEFAULTS")
            Action.CfgZero -> cmdRefresh("!ZERO")
            is Action.CfgSet -> {
                cmd("!SET,${a.key},${a.value.trim()}")
                viewModelScope.launch { delay(300); cmd("!STATUS") }
            }
            is Action.Export -> export(a.uri, a.saved)
            Action.ListLogFiles -> _ui.update {
                it.copy(logFiles = cfgLogDir.listFiles { f -> f.name.endsWith(".csv") }
                    ?.map { f -> f.name }?.sortedDescending().orEmpty())
            }
            is Action.ApplyFromLog -> openApply(a.name) { File(cfgLogDir, a.name).readText() }
            is Action.ApplyFromUri -> openApply(a.name) {
                getApplication<Application>().contentResolver.openInputStream(a.uri)!!.bufferedReader().use { r -> r.readText() }
            }
            is Action.ApplyToggle -> _ui.update { s ->
                val ap = s.apply ?: return@update s
                s.copy(apply = ap.copy(on = if (a.group in ap.on) ap.on - a.group else ap.on + a.group))
            }
            is Action.ApplyRun -> runApply(a.save)
            Action.ApplyClose -> _ui.update { it.copy(apply = null) }
            is Action.Console -> if (a.cmd.isNotBlank()) cmd(a.cmd.trim())
            Action.DismissToast -> _ui.update { it.copy(toast = null) }
        }
    }

    // sticky safe-start / remote message, until the next relay action or capture
    private var safe = ""

    // ═══ connection ═════════════════════════════════════════════════════════
    private var io: SerialIo? = null

    private fun connect(name: String, open: () -> SerialIo) {
        if (_ui.value.conn != Conn.Disconnected) return
        _ui.update { it.copy(conn = Conn.Connecting, connMsg = "opening $name…") }
        viewModelScope.launch {
            try {
                val l = withContext(Dispatchers.IO) {
                    val port = open()
                    io = port
                    MonitorLink(port) { events.trySend(it) }.also { it.start() }
                }
                val ident = try {
                    l.identify()
                } catch (e: LinkException) {
                    try { l.identify() } catch (e2: LinkException) {
                        withContext(Dispatchers.IO) { l.close() }
                        throw LinkException("port opened but no 120VMonitor answered !ID")
                    }
                }
                l.readCfg()
                val st = l.getStatus()
                link = l
                val s4 = st["scanus4"]?.toDoubleOrNull() ?: 0.0
                val s2 = st["scanus2"]?.toDoubleOrNull() ?: 0.0
                _ui.update {
                    it.copy(
                        conn = Conn.Connected,
                        ident = ident.removePrefix("\$READY,"),
                        board = st["board"] ?: "RA4M1",
                        connMsg = "${st["board"] ?: ""}  ${ident.removePrefix("\$READY,")}  scan ${st["scanus4"] ?: "?"} µs",
                        relay = st["relay"] == "1",
                        rateText = if (s4 > 0 && s2 > 0) String.format(Locale.US, "max ≈ %,.0f S/s (V+I), %,.0f S/s (one channel)", 1e6 / s4, 1e6 / s2) else "",
                    )
                }
                rangeUi(st["rng"]?.toIntOrNull(), st["rngmode"]?.toIntOrNull())
                fillCfg()
                recalcTiming()                    // the board's real buffer size and scan times
                logConfig("connect")
                if (_ui.value.streaming) cmd("!START")
            } catch (e: Exception) {
                withContext(Dispatchers.IO) { io?.disconnect() }
                _ui.update { it.copy(conn = Conn.Disconnected, connMsg = "not connected") }
                Log.e(TAG, "connect failed", e)
                toast("Connect: ${e.message ?: e.toString()}")
            }
        }
    }

    private fun disconnect() {
        val l = link ?: run {
            _ui.update { it.copy(conn = Conn.Disconnected, connMsg = "not connected") }
            return
        }
        link = null
        capJob?.cancel()
        liveJob?.cancel()
        _ui.update { it.copy(conn = Conn.Disconnected, connMsg = "not connected", capBusy = false, liveOn = false, rngNote = "", alert = "") }
        viewModelScope.launch(Dispatchers.IO) {
            try {
                l.relay(false)                    // leave the load off when the app lets go
            } catch (_: Exception) {
            }
            l.close()
        }
    }

    override fun onCleared() {
        val l = link ?: return
        link = null
        Thread {
            try { l.send("!RELAY,0") } catch (_: Exception) {}
            l.close()
        }.start()
        stopLog()
    }

    private fun cmd(c: String) {
        val l = link ?: return
        viewModelScope.launch(Dispatchers.IO) {
            try {
                l.send(c)
            } catch (e: Exception) {
                toast("Serial: ${e.message}")
            }
        }
    }

    private fun cmdRefresh(c: String) {
        cmd(c)
        viewModelScope.launch { delay(400); onAction(Action.CfgRefresh) }
    }

    // ═══ events ═════════════════════════════════════════════════════════════
    private fun conAdd(s: String) {
        console.addLast(s)
        while (console.size > 600) console.removeFirst()
    }

    private fun pushConsole() = _ui.update { it.copy(console = console.toList()) }

    private fun onLinkEvent(e: LinkEvent) {
        when (e) {
            is LinkEvent.Pwr -> onPwr(e.rec)
            is LinkEvent.Line -> { conAdd(e.text); onLine(e.text); pushConsole() }
            is LinkEvent.Tx -> { conAdd("> ${e.text}"); pushConsole() }
            is LinkEvent.Cap -> Unit                  // delivered to the waiting capture()
            is LinkEvent.Error -> {
                conAdd("!! ${e.text}")
                pushConsole()
                if (link != null) {
                    link = null
                    capJob?.cancel()
                    liveJob?.cancel()
                    io?.disconnect()
                    _ui.update { it.copy(conn = Conn.Disconnected, connMsg = e.text, capBusy = false, liveOn = false) }
                }
            }
        }
    }

    private fun onLine(line: String) {
        when {
            line.startsWith("\$RLY,") -> _ui.update { it.copy(relay = line.endsWith("1")) }
            line.startsWith("\$RNG,") -> {
                val p = line.substring(5).split(",")
                val note = if (p.getOrNull(1) == "OVER" && p.size > 3)
                    "(overload ${String.format(Locale.US, "%.2f", p[3].toDoubleOrNull() ?: 0.0)} A)"
                else mapOf("AUTO" to "(auto)", "CLOSE" to "(load closing)", "CAP" to "(capture)")[p.getOrNull(1)] ?: ""
                rangeUi(p.getOrNull(0)?.toIntOrNull(), p.getOrNull(2)?.toIntOrNull(), note)
            }
            line.startsWith("\$TRIP") -> _ui.update { it.copy(alert = "TRIPPED: " + line.drop(6)) }
            line.startsWith("\$REMOTE,") -> {
                val ev = line.substring(8)
                safe = mapOf(
                    "PRIMED" to "REMOTE: primed — press * to close", "CLOSE" to "", "CANCEL" to "",
                    "TIMEOUT" to "", "OPEN" to "REMOTE opened the relay",
                )[ev] ?: "REMOTE: $ev"
                _ui.update { it.copy(alert = safe) }
            }
            line.startsWith("\$SAFE,") -> {
                val why = line.substring(6).substringBefore(",")
                val amps = line.substring(6).substringAfter(",", "").toDoubleOrNull()
                safe = "SAFE START opened relay: $why" +
                    if (why in setOf("INST", "OVERCURRENT", "TRIPA") && amps != null) String.format(Locale.US, " (%.2f A)", amps) else ""
                _ui.update { it.copy(alert = safe) }
            }
            line.startsWith("\$CFG,") || line.startsWith("\$OK") || line.startsWith("\$STATUS,") -> {
                fillCfg()
                if (line.startsWith("\$STATUS,")) {
                    val st = link?.status ?: return
                    _ui.update { it.copy(relay = st["relay"] == "1") }
                    rangeUi(st["rng"]?.toIntOrNull(), st["rngmode"]?.toIntOrNull())
                }
                if (line == "\$OK,save") logConfig("save")
            }
            line.startsWith("\$ERR") -> toast(line.removePrefix("\$ERR,"))
        }
    }

    private fun fillCfg() {
        val l = link ?: return
        val rows = l.cfgRows()
        _ui.update {
            it.copy(
                cfg = rows, dirty = l.status["dirty"] == "1",
                twoRanges = (l.cfg["RNGPIN"]?.value ?: 0.0) != 0.0,
            )
        }
    }

    private fun rangeUi(rng: Int?, mode: Int?, note: String? = null) = _ui.update {
        it.copy(
            rng = rng ?: it.rng,
            rngMode = mode ?: it.rngMode,
            rngNote = note ?: if (rng != null && rng != it.rng) "" else it.rngNote,
        )
    }

    private fun onPwr(r: PwrRecord) {
        hist.addLast(r)
        while (hist.size > 36000) hist.removeFirst()
        val fl = r["flags"].toInt()
        _ui.update {
            it.copy(
                last = r, relay = r["relay"] >= 1,
                rng = if (r.values.containsKey("rng")) r["rng"].toInt() else it.rng,
                alert = listOf(safe, flagText(fl)).filter { s -> s.isNotEmpty() }.joinToString("   "),
            )
        }
        updateTrend()
        csv?.let { w ->
            try {
                val keys = listOf("vrms", "irms", "p", "s", "q", "pf", "hz", "vpk", "ipk", "wh", "vdc", "idc", "relay", "flags")
                w.write(
                    (listOf(nowStamp(), String.format(Locale.US, "%.3f", r.t / 1000.0)) +
                        keys.map { k -> fmtG(r[k], 7) } + (RANGE_NAMES[r["rng"].toInt()] ?: "")).joinToString(",") + "\n",
                )
            } catch (e: Exception) {
                toast("CSV log: ${e.message}")
                stopLog()
            }
        }
    }

    private fun updateTrend() {
        val span = SPANS[_ui.value.span] ?: 120
        val now = System.currentTimeMillis()
        val rows = if (span == 0) hist.toList() else hist.filter { it.t >= now - span * 1000L }
        val step = (rows.size / 1500).coerceAtLeast(1)            // a phone screen can't show more
        _ui.update { it.copy(trend = if (step == 1) rows else rows.filterIndexed { k, _ -> k % step == 0 }) }
    }

    // ── CSV log ─────────────────────────────────────────────────────────────
    private fun startLog(uri: Uri, name: String) {
        stopLog()
        try {
            val w = getApplication<Application>().contentResolver.openOutputStream(uri)!!.bufferedWriter()
            w.write("time,unix,vrms,irms,w,va,var,pf,hz,vpk,ipk,wh,vdc,idc,relay,flags,range\n")
            csv = w
            _ui.update { it.copy(logName = name) }
        } catch (e: Exception) {
            toast("CSV log: ${e.message}")
        }
    }

    private fun stopLog() {
        try { csv?.close() } catch (_: Exception) {}
        csv = null
        _ui.update { it.copy(logName = null) }
    }

    // ═══ capture ════════════════════════════════════════════════════════════
    private fun capMax(): Int = link?.status?.get("capmax")?.toIntOrNull() ?: CAP_MAX

    /** Work out whichever of rate / samples / duration isn't being set. */
    private fun recalcTiming() {
        var c = _ui.value.cap
        val derived = listOf("dur", "n", "rate")[c.tmode]
        val maxN = capMax() / (if (c.ch == 2) 2 else 1)
        var note = ""
        val rate = c.rate.toDoubleOrNull()
        val n = c.n.toDoubleOrNull()?.toInt()
        val dur = c.dur.toDoubleOrNull()
        if ((derived != "rate" && rate == null) || (derived != "n" && n == null) || (derived != "dur" && dur == null)) {
            _ui.update { it.copy(timingNote = "", spikeText = spikeText(c)) }
            return
        }
        when (derived) {
            "dur" -> {
                c = c.copy(dur = if (rate!! > 0) fmtG(n!! / rate * 1000, 4) else "?")
                if (rate <= 0) note = "rate 0 = as fast as the ADC goes, so the duration isn't known in advance"
            }
            "n" -> {
                if (rate!! <= 0) {
                    c = c.copy(n = "?")
                    note = "set a rate: 0 (max) can't be combined with a duration"
                } else {
                    val want = (rate * dur!! / 1000).roundToLong()
                    c = c.copy(n = minOf(want, maxN.toLong()).toString())
                    if (want > maxN) note = "buffer holds $maxN samples here: ${fmtG(maxN / rate * 1000, 4)} ms at this rate"
                }
            }
            else -> c = c.copy(rate = if (dur!! > 0) fmtG(n!! / dur * 1000, 6) else "?")
        }
        if (derived != "n" && n != null && n > maxN) note = "buffer holds $maxN samples with these channels; the board will clamp"
        _ui.update { it.copy(cap = c, timingNote = note, spikeText = spikeText(c)) }
    }

    /** Shortest spike the averaged samples still show at full height: the N
     *  scans of a sample are d = T/N apart, so a spike needs (2N-1)·d to contain
     *  one whole sample's group; anything lasting d shows, at ≥ 1/N height. */
    private fun spikeText(c: CapSettings): String {
        val rate = c.rate.toDoubleOrNull() ?: return ""
        val nAvg = (c.avg.toDoubleOrNull()?.toInt() ?: return "").coerceAtLeast(1)
        val both = c.ch == 2
        var scanUs = link?.status?.get(if (both) "scanus4" else "scanus2")?.toDoubleOrNull() ?: 0.0
        val est = if (scanUs > 0) "" else "≈"
        if (scanUs <= 0) scanUs = if (both) 4.2 else 2.2           // typical, until a board reports it
        var d = if (rate > 0) 1e6 / (rate * nAvg) else scanUs
        val late = d < scanUs
        d = maxOf(d, scanUs)
        val full = (2 * nAvg - 1) * d
        fun fmt(us: Double) = if (us >= 1000) "${fmtG(us / 1000, 3)} ms" else "${fmtG(us, 3)} µs"
        var txt = "Spike fully captured if it lasts ≥ $est${fmt(full)}"
        if (nAvg > 1) txt += "  (≥ $est${fmt(d)} always shows, at ≥ 1/$nAvg of its height)"
        txt += String.format(Locale.US, "  max rate at N=%d: %s%,.0f S/s", nAvg, est, 1e6 / (scanUs * nAvg))
        if (late) txt += "  — rate × N is more than the ADC can do; the capture will run LATE"
        return txt
    }

    private fun capArgs(): CapArgs {
        val c = _ui.value.cap
        fun num(s: String, what: String) = s.toDoubleOrNull() ?: throw IllegalArgumentException("$what: \"$s\" is not a number")
        if (c.rate == "?" || c.n == "?") throw IllegalArgumentException(_ui.value.timingNote.ifEmpty { "incomplete timing" })
        val a = CapArgs(
            rate = num(c.rate, "rate"), n = num(c.n, "samples").toInt(), ch = c.ch, trig = c.trig,
            level = num(c.level, "level"), pre = num(c.pre, "pretrigger"),
            timeoutMs = (num(c.timeoutS, "timeout") * 1000).roundToInt(),
            // rising zero: close relay_us after it; level trigger: close at once
            relayUs = if (c.trig == 3) maxOf(0, num(c.relayUs, "close delay").toInt()) else -1,
            ipkA = if (c.ipkOn) num(c.ipkA, "instant trip") else 0.0,
            ovA = if (c.ovOn) num(c.ovA, "over-current") else 0.0,
            ovCyc = num(c.ovCyc, "cycles").toInt(),
            autoMs = if (c.autoOn) num(c.autoMs, "auto-open").toInt() else 0,
            openAfter = c.openAfter, avg = num(c.avg, "average N").toInt(), rng = c.rng, mon = c.mon,
        )
        if (a.trig in CURRENT_TRIGS && a.ch == 0) throw IllegalArgumentException("a current trigger needs the current channel (V + I or I only)")
        if (a.timeoutMs < 0) throw IllegalArgumentException("timeout must be 0 (none) or more")
        if ((a.ipkA != 0.0 || a.ovA != 0.0) && a.ch == 0) throw IllegalArgumentException("the current limits need the current channel (V + I or I only)")
        if (a.avg !in 1..64) throw IllegalArgumentException("Average N must be 1…64")
        return a
    }

    private fun requestCapture() {
        if (_ui.value.liveOn) {
            toast("Live persistence is running on the Meter tab: switch it to Trends to capture")
            return
        }
        if (link == null || _ui.value.capBusy) return
        val a = try {
            capArgs()
        } catch (e: IllegalArgumentException) {
            toast("Capture: bad setting: ${e.message}")
            return
        }
        if (a.trig in CLOSE_TRIGS) {
            val after = if (a.openAfter || a.autoMs > 0)
                "opens it again " + if (a.openAfter) "when the capture completes" else "${a.autoMs} ms after closing"
            else "LEAVES THE LOAD SWITCHED ON"
            _ui.update {
                it.copy(
                    confirmCapture = "This CLOSES THE RELAY and $after. If it is closed now it opens for 1 s first. Continue?",
                    cap = it.cap.copy(repeat = false),      // one inrush per tap
                )
            }
            return
        }
        runCapture()
    }

    private fun runCapture() {
        val l = link ?: return
        val a = try { capArgs() } catch (e: IllegalArgumentException) { toast("Capture: ${e.message}"); return }
        val closes = a.trig in CLOSE_TRIGS
        safe = ""
        val armedMsg = "armed — waiting for the trigger" + (if (a.timeoutMs == 0) " (no timeout)" else "") +
            (if (a.mon) ", readings continue" else "") + "…"
        _ui.update { it.copy(capBusy = true, capMsg = if (a.trig == 0) "capturing…" else armedMsg) }
        capJob = viewModelScope.launch {
            try {
                // an inrush test needs the load off first: open, let it settle 1 s
                if (closes && l.getStatus()["relay"] == "1") {
                    _ui.update { it.copy(capMsg = "relay was closed: opened, waiting 1 s…") }
                    l.relay(false)
                    delay(1000)
                }
                val cap = l.capture(a)
                val st = withContext(Dispatchers.Default) { captureStats(cap) }
                _ui.update { it.copy(capBusy = false, capture = cap, capStats = st, capMsg = "") }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {                  // LinkException, or the port failing mid-capture
                Log.e(TAG, "capture failed", e)
                _ui.update { it.copy(capBusy = false, capMsg = "capture: ${e.message ?: e.toString()}") }
            }
            val s = _ui.value
            if (s.cap.repeat && !closes && link != null) {
                delay(200)
                if (_ui.value.cap.repeat) runCapture()
            }
        }
    }

    // ── live persistence (Meter tab) ────────────────────────────────────────
    /** Back-to-back captures, each folded into the running cycle heat map. Readings
     *  while armed keep the meter going; switching back to Trends ends the loop
     *  after its current capture. */
    private fun startLive() {
        val s = _ui.value
        if (s.liveOn) return
        if (link == null) { toast("Connect first"); return }
        if (s.capBusy) { toast("A capture is running: try again when it ends"); return }
        synchronized(persist) { persist.clear(); persist.tauS = FADES[s.fade] ?: 0.0 }
        _ui.update { it.copy(liveOn = true, live = null, liveMsg = "starting…") }
        liveJob = viewModelScope.launch {
            try {
                while (_ui.value.liveOn) {
                    val l = link ?: break
                    val hz = hist.lastOrNull()?.get("hz")?.takeIf { it > 0 } ?: 60.0
                    val cyc = l.cfg["CYCLES"]?.value?.toInt() ?: 10
                    val cap = try {
                        l.capture(liveCaptureArgs(capMax(), hz, cyc))
                    } catch (e: CancellationException) {
                        throw e
                    } catch (e: Exception) {                  // keep going; a gap is fine
                        _ui.update { it.copy(liveMsg = "capture: ${e.message ?: e}") }
                        delay(500)
                        continue
                    }
                    val view = withContext(Dispatchers.Default) {
                        synchronized(persist) {
                            persist.add(cap, android.os.SystemClock.elapsedRealtime() / 1000.0)
                            val units = mapOf("v" to "V", "i" to "A")
                            val msg = String.format(
                                Locale.US, "%d captures, ≈%,.0f cycles · %.0f %% of the time sampled (the rest is the gap while each capture is sent)",
                                persist.captures, persist.cycles,
                                persist.coverage(android.os.SystemClock.elapsedRealtime() / 1000.0) * 100,
                            ) + if (persist.late > 0) " · ${persist.late} LATE: ${persist.dropped} slipped cycles left out" else ""
                            LiveView(persist.maps.map { (k, c) -> LiveChan(c.h.copyOf(), c.lim, units[k] ?: k) },
                                persist.hz, persist.ref) to msg
                        }
                    }
                    _ui.update { it.copy(live = view.first, liveMsg = view.second) }
                }
            } finally {
                _ui.update { it.copy(liveOn = false) }
            }
        }
    }

    private fun saveCapture(uri: Uri) {
        val cap = _ui.value.capture ?: return
        viewModelScope.launch(Dispatchers.IO) {
            try {
                getApplication<Application>().contentResolver.openOutputStream(uri)!!.bufferedWriter().use { w ->
                    w.write(
                        "# dt_us=${cap.dtUs},trig_idx=${cap.trigIdx},relay_idx=${cap.relayIdx},open_idx=${cap.openIdx}," +
                            "open_why=${SAFE_WHY[cap.openWhy] ?: ""},flags=${cap.flags},scans_per_sample=${cap.avg}," +
                            "range=${RANGE_NAMES[cap.rng] ?: ""},range_high_from_idx=${cap.rngIdx}\n",
                    )
                    w.write("t_ms" + (if (cap.v != null) ",v" else "") + (if (cap.i != null) ",i" else "") + "\n")
                    val sb = StringBuilder()
                    for (k in 0 until cap.n) {
                        sb.setLength(0)
                        sb.append(String.format(Locale.US, "%.5f", cap.tMs[k]))
                        cap.v?.let { sb.append(String.format(Locale.US, ",%.4f", it[k])) }
                        cap.i?.let { sb.append(String.format(Locale.US, ",%.5f", it[k])) }
                        sb.append('\n')
                        w.write(sb.toString())
                    }
                }
                toast("capture saved")
            } catch (e: Exception) {
                toast("Save capture: ${e.message}")
            }
        }
    }

    // ═══ config snapshots ═══════════════════════════════════════════════════
    /** Log the stored config to config_log/, unless it matches the newest
     *  snapshot of the same board. */
    private fun logConfig(why: String) {
        val l = link ?: return
        viewModelScope.launch {
            val msg = try {
                val snap = l.dump(saved = true)
                val f = withContext(Dispatchers.IO) { logSnapshot(snap, cfgLogDir) }
                if (f != null) "saved ${f.name}" else "unchanged since the last snapshot"
            } catch (e: Exception) {
                e.message ?: "failed"
            }
            conAdd("# config log ($why): $msg")
            pushConsole()
        }
    }

    private fun export(uri: Uri, saved: Boolean) {
        val l = link ?: return
        viewModelScope.launch {
            try {
                val snap = l.dump(saved)
                withContext(Dispatchers.IO) {
                    getApplication<Application>().contentResolver.openOutputStream(uri)!!.bufferedWriter().use { it.write(snap.toCsv()) }
                }
                toast("snapshot written")
            } catch (e: Exception) {
                toast("Export snapshot: ${e.message}")
            }
        }
    }

    private fun openApply(name: String, read: () -> String) {
        val l = link ?: return
        viewModelScope.launch {
            try {
                val snap = withContext(Dispatchers.IO) { Snapshot.parse(read(), name) }
                val here = l.dump()
                val cur = here.values()
                val diffs = snap.diff(cur).filter { it.key in cur }
                val counts = diffs.groupingBy { it.group }.eachCount()
                val groups = CFG_GROUPS.keys.toList() + (snap.rows.map { it.group }.toSet() - CFG_GROUPS.keys).sorted()
                val sm = snap.meta
                val hm = here.meta
                _ui.update {
                    it.copy(
                        apply = ApplyState(
                            name = name, snap = snap, here = here, diffs = diffs,
                            missing = snap.rows.map { r -> r.key }.filter { k -> k !in cur }.sorted(),
                            otherBoard = (sm["board"] ?: "?") != (hm["board"] ?: "?"),
                            sameUnit = !sm["uid"].isNullOrEmpty() && sm["uid"] == hm["uid"],
                            groups = groups, counts = counts,
                            on = if ((counts["CAL"] ?: 0) > 0) setOf("CAL") else emptySet(),
                            msg = if (diffs.isEmpty()) "This board already matches the snapshot." else "",
                        ),
                    )
                }
            } catch (e: Exception) {
                toast("Apply snapshot: ${e.message}")
            }
        }
    }

    private fun runApply(save: Boolean) {
        val l = link ?: return
        val ap = _ui.value.apply ?: return
        val todo = ap.todo
        if (todo.isEmpty()) return
        _ui.update { it.copy(apply = ap.copy(busy = true, msg = "sending ${todo.size} keys…")) }
        viewModelScope.launch {
            val msg = try {
                for (d in todo) {
                    try {
                        l.set(d.key, d.snap)
                    } catch (_: LinkException) {
                        // a refused pin etc.; the read-back below reports it
                    }
                }
                l.readCfg()
                val bad = todo.filter { d -> !sameValue(l.cfg[d.key]?.value ?: Double.NaN, d.snap) }.map { it.key }
                var m = "${todo.size - bad.size} of ${todo.size} keys set in RAM."
                if (bad.isNotEmpty()) m += " Not taken (refused or clamped): ${bad.joinToString(", ")}."
                m += when {
                    save && bad.isEmpty() -> { l.save(); " Saved." }
                    save -> " Not saved — fix those first or save from the Config tab."
                    else -> " Save from the Config tab to keep them."
                }
                m
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                "Error: ${e.message ?: e.toString()}"
            }
            _ui.update { s -> s.copy(apply = s.apply?.copy(busy = false, msg = msg)) }
            fillCfg()
            cmd("!STATUS")
        }
    }
}
