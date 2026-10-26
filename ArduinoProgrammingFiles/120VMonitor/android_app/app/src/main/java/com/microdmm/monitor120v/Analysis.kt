package com.microdmm.monitor120v

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

// Port of the analysis helpers in monitor_link.py.

/** Mains frequency from rising zero crossings, with hysteresis so noise around
 *  zero can't add crossings: a crossing counts only once the signal has been
 *  below -h and then gets above +h. */
fun estimateFreq(x0: DoubleArray, dtS: Double): Double? {
    if (x0.size < 3) return null
    val mean = x0.average()
    val x = DoubleArray(x0.size) { x0[it] - mean }
    val sd = sqrt(x.sumOf { it * it } / x.size)
    val h = 0.2 * sd
    val crossings = ArrayList<Double>()
    var k = x.indexOfFirst { it < -h }
    if (k < 0) return null
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
