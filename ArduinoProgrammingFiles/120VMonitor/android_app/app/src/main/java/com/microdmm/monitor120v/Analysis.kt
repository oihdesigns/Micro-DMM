package com.microdmm.monitor120v

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

// Port of the analysis helpers in monitor_link.py.

/** Fractional sample positions of the rising zero crossings (mean removed), with
 *  hysteresis so noise around zero can't add crossings: a crossing counts only
 *  once the signal has been below -h and then gets above +h. */
fun risingCrossings(x0: DoubleArray): List<Double> {
    if (x0.size < 3) return emptyList()
    val mean = x0.average()
    val x = DoubleArray(x0.size) { x0[it] - mean }
    val sd = sqrt(x.sumOf { it * it } / x.size)
    val h = 0.2 * sd
    val crossings = ArrayList<Double>()
    var k = x.indexOfFirst { it < -h }
    if (k < 0) return crossings
    while (true) {
        var up = k
        while (up < x.size && x[up] <= h) up++                  // next sample above +h
        if (up >= x.size) break
        var m = up
        while (m > k && x[m] >= 0) m--                           // last sign change before `up`
        crossings += m + x[m] / (x[m] - x[m + 1])
        k = up
        while (k < x.size && x[k] >= -h) k++
        if (k >= x.size) break
    }
    return crossings
}

/** Mains frequency from the rising zero crossings. */
fun estimateFreq(x0: DoubleArray, dtS: Double): Double? {
    val crossings = risingCrossings(x0)
    if (crossings.size < 2) return null
    val span = (crossings.last() - crossings.first()) * dtS
    return if (span > 0) (crossings.size - 1) / span else null
}

class Harmonics(val amps: DoubleArray, val thd: Double)

/** Harmonic RMS amplitudes 1..nmax from a whole number of cycles of x. */
fun harmonics(x: DoubleArray, dtS: Double, f0: Double, nmax: Int = 40): Harmonics {
    val per = 1.0 / (f0 * dtS)
    val cycles = (x.size / per).toInt()
    if (cycles < 1) return Harmonics(DoubleArray(0), Double.NaN)
    val m = (cycles * per).roundToInt().coerceAtMost(x.size)
    var mean = 0.0
    for (k in 0 until m) mean += x[k]
    mean /= m
    val amps = ArrayList<Double>()
    for (h in 1..nmax) {
        if (h * f0 >= 0.5 / dtS) break
        val w = 2 * PI * h * f0 * dtS
        var a = 0.0
        var b = 0.0
        for (k in 0 until m) {
            val xs = x[k] - mean
            a += xs * cos(w * k)
            b += xs * sin(w * k)
        }
        a = 2 * a / m
        b = 2 * b / m
        amps += hypot(a, b) / sqrt(2.0)
    }
    val thd = if (amps.size > 1 && amps[0] > 0) sqrt(amps.drop(1).sumOf { it * it }) / amps[0] else Double.NaN
    return Harmonics(amps.toDoubleArray(), thd)
}

class ChanStats(val dc: Double, val rms: Double, val pk: Double, val crest: Double, val harm: Harmonics)

class CapStats(
    val rate: Double, val hz: Double,
    val v: ChanStats?, val i: ChanStats?,
    val p: Double?, val pf: Double?,
)

/** RMS / power / harmonics from a capture, over a whole number of cycles. */
fun captureStats(cap: Capture, fGuess: Double = 60.0): CapStats {
    val dt = cap.dtUs * 1e-6
    val ref = cap.v ?: cap.i!!
    val f = estimateFreq(ref, dt) ?: fGuess
    val per = 1.0 / (f * dt)
    val cyc = (ref.size / per).toInt()
    val m = (if (cyc >= 1) (cyc * per).roundToInt() else ref.size).coerceIn(1, ref.size)

    fun chan(x: DoubleArray?): ChanStats? {
        if (x == null || x.isEmpty()) return null
        var dc = 0.0
        for (k in 0 until m) dc += x[k]
        dc /= m
        var ss = 0.0
        for (k in 0 until m) ss += (x[k] - dc) * (x[k] - dc)
        val rms = sqrt(ss / m)
        val pk = x.maxOf { abs(it) }
        return ChanStats(dc, rms, pk, if (rms > 0) pk / rms else Double.NaN, harmonics(x, dt, f))
    }

    val v = chan(cap.v)
    val i = chan(cap.i)
    var p: Double? = null
    var pf: Double? = null
    if (v != null && i != null) {
        val vv = cap.v!!
        val ii = cap.i!!
        var acc = 0.0
        for (k in 0 until m) acc += (vv[k] - v.dc) * (ii[k] - i.dc)
        p = acc / m
        val s = v.rms * i.rms
        pf = if (s > 0) p / s else Double.NaN
    }
    return CapStats(if (cap.dtUs > 0) 1e6 / cap.dtUs else 0.0, f, v, i, p, pf)
}

/** Each sample's place in its mains cycle (0..1 from the rising zero crossing of
 *  ref, or from fGuess when ref has none) and the number of cycles. Each cycle is
 *  stretched between its own crossings, so drift doesn't smear an overlay; a
 *  crossing a dropout hid is filled in at the median period, and the ends are
 *  extrapolated. Port of cycle_phase in monitor_link.py. */
fun cyclePhase(ref: DoubleArray, dtS: Double, fGuess: Double = 60.0): Pair<DoubleArray, Double> {
    val n = ref.size
    val fr = risingCrossings(ref)
    if (fr.size < 2) {
        val per = 1.0 / (fGuess * dtS)
        return DoubleArray(n) { (it / per) % 1.0 } to n / per
    }
    val diffs = fr.zipWithNext { a, b -> b - a }.sorted()
    val per = diffs[diffs.size / 2]
    val full = ArrayList<Double>()
    full += fr[0]
    for (c in fr.drop(1)) {
        val gap = c - full.last()
        val k = (gap / per).roundToInt()
        if (k > 1) {                                       // missed crossings: fill them in evenly
            val last = full.last()
            for (j in 1 until k) full += last + gap * j / k
        }
        full += c
    }
    val ph = DoubleArray(n)
    var seg = 0
    for (s in 0 until n) {
        val cyc = when {
            s < full[0] -> (s - full[0]) / per
            s > full.last() -> full.size - 1 + (s - full.last()) / per
            else -> {
                while (seg < full.size - 2 && s > full[seg + 1]) seg++
                seg + (s - full[seg]) / (full[seg + 1] - full[seg])
            }
        }
        ph[s] = ((cyc % 1.0) + 1.0) % 1.0
    }
    return ph to n / per
}

const val FLG_LATE = 0x20
const val SLIP_TOL = 0.5           // samples: a timing slip shortens a cycle by at least one

/** (keep, dropped). The board sets LATE when it fell behind and restarted its
 *  sample clock, so somewhere the samples aren't evenly spaced and that cycle
 *  won't overlay the others. With LATE set, keep only the cycles whose length is
 *  within SLIP_TOL samples of the median; the partial cycles at the ends go too.
 *  Without LATE keep is null and nothing is dropped, so a real disturbance at a
 *  zero crossing is never hidden. Port of slip_mask in monitor_link.py. */
fun slipMask(ref: DoubleArray, flags: Int): Pair<BooleanArray?, Int> {
    if ((flags and FLG_LATE) == 0) return null to 0
    val keep = BooleanArray(ref.size)
    val fr = risingCrossings(ref)
    if (fr.size < 2) return keep to 1
    val d = fr.zipWithNext { a, b -> b - a }
    val med = d.sorted()[d.size / 2]
    var dropped = 0
    for (k in d.indices) {
        if (abs(d[k] - med) <= SLIP_TOL) {
            for (s in ceil(fr[k]).toInt()..minOf(floor(fr[k + 1]).toInt(), ref.size - 1)) keep[s] = true
        } else dropped++
    }
    return keep to dropped
}

/** Persistence map of x folded onto one cycle: counts[row * nx + col], row 0 at
 *  -lim, over phase 0..1 and value -lim..+lim. Traces are linearly upsampled so
 *  a cycle with few samples still draws a line rather than dots. keep (from
 *  slipMask) leaves out every segment that doesn't have both ends kept. */
fun cycleHeatmap(
    x: DoubleArray, phase: DoubleArray, lim: Double, nx: Int = 240, ny: Int = 160, keep: BooleanArray? = null,
): IntArray {
    val h = IntArray(nx * ny)
    if (x.isEmpty() || !(lim > 0)) return h
    var wraps = 0
    for (k in 1 until phase.size) if (phase[k] - phase[k - 1] < -0.5) wraps++
    val spc = x.size / (wraps + 1.0)
    val up = ceil(2.0 * nx / spc.coerceAtLeast(1.0)).toInt().coerceIn(1, 16)
    fun add(v: Double, p: Double) {
        val c = (p * nx).toInt().coerceIn(0, nx - 1)
        val r = ((v + lim) / (2 * lim) * ny).toInt().coerceIn(0, ny - 1)
        h[r * nx + c]++
    }
    for (k in 0 until x.size - 1) {
        if (keep != null && !(keep[k] && keep[k + 1])) continue
        var dp = phase[k + 1] - phase[k]
        if (dp < -0.5) dp += 1.0                           // across the 1 -> 0 wrap
        for (j in 0 until up) {
            val f = j.toDouble() / up
            add(x[k] + (x[k + 1] - x[k]) * f, (phase[k] + dp * f) % 1.0)
        }
    }
    if (keep == null || keep[x.size - 1]) add(x.last(), phase.last())
    return h
}

const val LIVE_RATE = 15000.0      // S/s for live persistence: 250 samples a 60 Hz cycle
const val LIVE_AVG = 4             // scans per sample
const val LIVE_MAX_CYCLES = 30     // per capture; fewer if the board's buffer is smaller

/** !CAP arguments for one live-persistence capture: V + I, as many cycles as the
 *  buffer holds (at least the meter window, so readings while armed still give a
 *  $PWR line per capture). Port of live_capture_args in monitor_link.py. */
fun liveCaptureArgs(capMax: Int, hz: Double = 60.0, cycles: Int = 10): CapArgs {
    val f = if (hz > 0) hz else 60.0
    val spc = LIVE_RATE / f
    val ncyc = maxOf(cycles, minOf(LIVE_MAX_CYCLES, (capMax / 2 / spc).toInt()))
    val rate = if (ncyc * spc * 2 <= capMax) LIVE_RATE else floor(capMax / 2 / ncyc.toDouble() * f)
    return CapArgs(
        rate = rate, n = (ncyc * rate / f).toInt(), ch = 2, trig = 0, level = 0.5, pre = 10.0,
        timeoutMs = 1000, relayUs = -1, ipkA = 0.0, ovA = 0.0, ovCyc = 3, autoMs = 0, openAfter = false,
        avg = LIVE_AVG, rng = -1, mon = true,
    )
}

/** Cycle heat maps that build up over many captures: a live persistence display.
 *  tauS 0 keeps everything; otherwise older captures fade with that time constant.
 *  Each channel's range only grows (25 % headroom), re-binning what's there.
 *  Port of Persistence in monitor_link.py; times in seconds from any fixed origin. */
class Persistence(val nx: Int = 240, val ny: Int = 160, var tauS: Double = 0.0) {
    class Chan(val h: DoubleArray, val lim: Double)

    val maps = LinkedHashMap<String, Chan>()      // "v" / "i"
    var cycles = 0.0; private set                 // cycles in the map (faded too)
    var captures = 0; private set
    var late = 0; private set                     // captures the board flagged LATE
    var dropped = 0; private set                  // cycles left out of those (slipMask)
    var hz = 60.0; private set
    var ref = "V"; private set
    private var capS = 0.0                        // seconds of mains actually sampled
    private var t0: Double? = null
    private var tLast: Double? = null

    fun clear() {
        maps.clear(); cycles = 0.0; captures = 0; late = 0; dropped = 0; capS = 0.0; t0 = null; tLast = null; hz = 60.0; ref = "V"
    }

    /** Fraction of the time since the start that was sampled; the rest fell in the
     *  gaps while each capture was sent. */
    fun coverage(now: Double): Double {
        val start = t0 ?: return 0.0
        return (capS / maxOf(now - start, 1e-9)).coerceAtMost(1.0)
    }

    private fun regrid(h: DoubleArray, lim0: Double, lim1: Double): DoubleArray {
        val out = DoubleArray(h.size)
        for (r in 0 until ny) {
            val v = -lim0 + (r + 0.5) * 2 * lim0 / ny
            val r1 = ((v + lim1) / (2 * lim1) * ny).toInt().coerceIn(0, ny - 1)
            for (c in 0 until nx) out[r1 * nx + c] += h[r * nx + c]
        }
        return out
    }

    fun add(cap: Capture, now: Double) {
        val dt = cap.dtUs * 1e-6
        val refData = cap.v ?: cap.i ?: return
        ref = if (cap.v != null) "V" else "I"
        hz = estimateFreq(refData, dt) ?: hz
        val (phase, ncyc0) = cyclePhase(refData, dt, hz)
        val (keep, slipped) = slipMask(refData, cap.flags)
        var ncyc = ncyc0
        if (keep != null) {
            late++
            dropped += slipped
            ncyc = keep.count { it } * dt * hz
        }
        val span = refData.size * dt
        val last = tLast
        val k = if (tauS > 0 && last != null) exp(-(now - last) / tauS) else 1.0
        for ((name, x) in listOf("v" to cap.v, "i" to cap.i)) {
            if (x == null || x.isEmpty()) continue
            val old = maps[name]
            var lim = old?.lim ?: 0.0
            var h = old?.h
            val pk = x.maxOf { abs(it) }
            if (pk > lim || lim <= 0) {
                val nl = maxOf(pk * 1.25, 1e-6)
                if (h != null) h = regrid(h, lim, nl)
                lim = nl
            }
            val add = cycleHeatmap(x, phase, lim, nx, ny, keep)
            val sum = DoubleArray(nx * ny) { (h?.get(it) ?: 0.0) * k + add[it] }
            maps[name] = Chan(sum, lim)
        }
        cycles = cycles * k + ncyc
        captures++
        capS += span
        if (t0 == null) t0 = now - span
        tLast = now
    }
}
