package com.microdmm.monitor120v

import java.io.File
import java.math.BigDecimal
import java.math.MathContext
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

// Port of monitor_link.py: the names and tables match it.

val PWR_FIELDS = listOf(
    "vrms", "irms", "p", "s", "q", "pf", "hz", "vpk", "ipk", "wh", "uptime",
    "n", "depth", "relay", "flags", "vdc", "idc", "rng",
)

val FLAG_NAMES = linkedMapOf(
    1 to "NO V ZERO-X", 2 to "V CLIP", 4 to "I CLIP", 8 to "TRIPPED", 16 to "TRUNCATED", 32 to "LATE",
)

val SAFE_WHY = mapOf(
    0 to "", 1 to "instant limit", 2 to "over-current (time delay)", 3 to "auto-open timer",
    4 to "capture complete", 5 to "TRIPPK",
)

val RANGE_NAMES = mapOf(0 to "HIGH", 1 to "LOW")             // 0.01 ohm / 0.1 ohm shunt
val RANGE_FULL = mapOf(0 to "HIGH ±25 A", 1 to "LOW ±2.5 A")
val RANGE_MODES = linkedMapOf(0 to "Auto", 1 to "HIGH only", 2 to "LOW", 3 to "Auto-delayed")

val TRIG_MODES = linkedMapOf(
    0 to "Immediate", 1 to "|I| above level", 2 to "V rising zero-cross",
    4 to "|V| above level", 6 to "Irms drops below level",
    3 to "Close relay - rising zero", 5 to "Close relay - level trigger",
)
val CLOSE_TRIGS = setOf(3, 5)                               // the capture closes the load relay itself
val CURRENT_TRIGS = setOf(1, 5, 6)                          // need the current channel

const val CAP_MAX = 7168                                    // XIAO buffer; $STATUS capmax wins

fun flagText(flags: Int): String =
    FLAG_NAMES.filter { (bit, _) -> flags and bit != 0 }.values.joinToString("  ")

class LinkException(msg: String) : Exception(msg)

/** One $PWR metering window. */
class PwrRecord(val values: Map<String, Double>, val t: Long = System.currentTimeMillis()) {
    operator fun get(key: String): Double = when (key) {
        "crest" -> {
            val irms = get("irms")
            if (irms > 1e-4) get("ipk") / irms else Double.NaN
        }
        else -> values[key] ?: Double.NaN
    }
}

class Capture(
    val ch: Int,              // 0 V, 1 I, 2 both
    val dtUs: Double,
    val vPer: Double, val vOff: Double,
    val iPer: Double, val iOff: Double,
    val trigIdx: Int,
    val flags: Int,
    val relayIdx: Int,
    val openIdx: Int = -1,
    val openWhy: Int = 0,
    val avg: Int = 1,
    val rng: Int = 0,
    val rngIdx: Int = -1,     // sample from which the range was HIGH (overload), -1 never
    val iPerHi: Double? = null,
    val iOffHi: Double? = null,
    val settleUs: Double = 0.0,
) {
    var raw = ShortArrayBuilder()
    var v: DoubleArray? = null    // volts
    var i: DoubleArray? = null    // amps
    var tMs = DoubleArray(0)
    var n = 0
        private set

    fun finish(): Capture {
        val a = raw.toArray()
        val vc: ShortArray?
        val ic: ShortArray?
        when (ch) {
            2 -> {
                vc = ShortArray(a.size / 2) { a[2 * it] }
                ic = ShortArray(a.size / 2) { a[2 * it + 1] }
            }
            0 -> { vc = a; ic = null }
            else -> { vc = null; ic = a }
        }
        if (vc != null) v = DoubleArray(vc.size) { (vc[it] - vOff) * vPer }
        if (ic != null) {
            val hiFrom = if (rngIdx >= 0 && iPerHi != null && iOffHi != null) rngIdx else Int.MAX_VALUE
            i = DoubleArray(ic.size) {
                if (it >= hiFrom) (ic[it] - iOffHi!!) * iPerHi!! else (ic[it] - iOff) * iPer
            }
        }
        n = (vc ?: ic!!).size
        val t0 = if (trigIdx >= 0) trigIdx else 0
        tMs = DoubleArray(n) { (it - t0) * dtUs / 1000.0 }
        raw = ShortArrayBuilder()
        return this
    }

    /** ms from the trigger (or the start) of sample k */
    fun msAt(k: Int) = (k - maxOf(trigIdx, 0)) * dtUs / 1000.0
}

class ShortArrayBuilder {
    private var a = ShortArray(4096)
    var size = 0
        private set

    fun add(x: Short) {
        if (size == a.size) a = a.copyOf(a.size * 2)
        a[size++] = x
    }

    fun toArray(): ShortArray = a.copyOf(size)
}

class CfgRow(val key: String, val value: Double, val lo: Double, val hi: Double, val help: String)

// ── config snapshots ────────────────────────────────────────────────────────
// What each key is for, so a snapshot can be copied one kind at a time. The
// firmware tags its own keys in !DUMP (fw 1.4+); this table only stands in for
// older firmware, which has !CFG alone.
val CFG_GROUPS = linkedMapOf(
    "CAL" to "VGAIN VOFF IGAIN IOFF IGAINLO IOFFLO PHASEUS ADCVREF",
    "FUSE" to "TRIPA TRIPPK",
    "RANGE" to "RNGMODE RNGUPA RNGDNA RNGHOLD RNGSETMS RNGCYC",
    "METER" to "MAINS SPC CYCLES",
    "WIRING" to "RLYPIN RLYINV RNGPIN RNGINV IRPIN",
    "UI" to "OLED OLEDADDR OLEDROT IRADDR IROK IRCONF IRARMMS PIXEL LIVEVAC",
)
val GROUP_OF: Map<String, String> =
    CFG_GROUPS.flatMap { (g, keys) -> keys.split(" ").map { it to g } }.toMap()
val GROUP_INFO = mapOf(
    "CAL" to "analog calibration: gains, offsets, phase, ADC reference",
    "FUSE" to "virtual fuses: TRIPA / TRIPPK",
    "RANGE" to "current range switching thresholds and mode",
    "METER" to "mains Hz, samples per cycle, window length",
    "WIRING" to "relay, range relay and IR pins and their polarity",
    "UI" to "OLED, IR remote codes, NeoPixel",
)

fun sameValue(a: Double, b: Double) = Math.abs(a - b) <= 1e-6 * maxOf(1.0, Math.abs(a), Math.abs(b))

/** Like Python's "%.{sig}g": shortest plain text with `sig` significant digits. */
fun fmtG(v: Double, sig: Int = 9): String {
    if (v.isNaN() || v.isInfinite()) return v.toString()
    if (v == 0.0) return "0"
    return BigDecimal(v).round(MathContext(sig)).stripTrailingZeros().toPlainString()
}

fun nowStamp(pattern: String = "yyyy-MM-dd HH:mm:ss"): String = SimpleDateFormat(pattern, Locale.US).format(Date())

data class SnapRow(val group: String, val key: String, val value: Double)

data class SnapDiff(val group: String, val key: String, val current: Double?, val snap: Double)

/** Every config key of one board, with its group and where it came from.
 *  meta: fw, board, uid, cfgver, src (RAM / SAVED / RAM-CFG), dirty, taken. */
class Snapshot(val meta: MutableMap<String, String> = linkedMapOf(), val rows: MutableList<SnapRow> = mutableListOf()) {

    fun values(): Map<String, Double> = rows.associate { it.key to it.value }

    fun sameValues(other: Snapshot): Boolean {
        val a = values()
        val b = other.values()
        return a.keys == b.keys && a.all { (k, v) -> sameValue(v, b.getValue(k)) }
    }

    /** every key whose value differs from `current` */
    fun diff(current: Map<String, Double>): List<SnapDiff> =
        rows.filter { r -> current[r.key]?.let { !sameValue(it, r.value) } ?: true }
            .map { SnapDiff(it.group, it.key, current[it.key], it.value) }

    fun toCsv(): String = buildString {
        append("# 120VMonitor config snapshot\n")
        for ((k, v) in meta) append("#$k,$v\n")
        append("group,key,value\n")
        for (r in rows) append("${r.group},${r.key},${fmtG(r.value)}\n")
    }

    companion object {
        fun parse(text: String, name: String = "file"): Snapshot {
            val snap = Snapshot()
            for (raw in text.lineSequence()) {
                val line = raw.trim()
                if (line.startsWith("#")) {
                    val c = line.indexOf(',')
                    if (c > 0) snap.meta[line.substring(1, c).trim()] = line.substring(c + 1).trim()
                    continue
                }
                val p = line.split(",")
                if (p.size >= 3 && p[0] != "group") {
                    p[2].toDoubleOrNull()?.let { snap.rows += SnapRow(p[0].uppercase(), p[1].uppercase(), it) }
                }
            }
            if (snap.rows.isEmpty()) throw IllegalArgumentException("$name holds no config rows")
            return snap
        }
    }
}

/** Write snap to folder/<board>_<uid>_<time>.csv unless the newest file for that
 *  board already holds the same values. Returns the file, or null. */
fun logSnapshot(snap: Snapshot, folder: File): File? {
    folder.mkdirs()
    val uid = snap.meta["uid"].orEmpty().takeLast(8).ifEmpty { "nouid" }
    val stem = "${snap.meta["board"] ?: "board"}_$uid"
    val newest = folder.listFiles { f -> f.name.startsWith(stem + "_") && f.name.endsWith(".csv") }
        ?.maxByOrNull { it.name }
    if (newest != null) {
        try {
            if (Snapshot.parse(newest.readText(), newest.name).sameValues(snap)) return null
        } catch (_: Exception) {
            // unreadable: write a fresh one
        }
    }
    val f = File(folder, "${stem}_${nowStamp("yyyyMMdd_HHmmss")}.csv")
    f.writeText(snap.toCsv())
    return f
}
