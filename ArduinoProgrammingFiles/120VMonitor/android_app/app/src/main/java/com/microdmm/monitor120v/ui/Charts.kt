package com.microdmm.monitor120v.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.gestures.calculateCentroid
import androidx.compose.foundation.gestures.calculatePan
import androidx.compose.foundation.gestures.calculateZoom
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.sp
import com.microdmm.monitor120v.CapStats
import com.microdmm.monitor120v.Capture
import com.microdmm.monitor120v.PwrRecord
import com.microdmm.monitor120v.SAFE_WHY
import com.microdmm.monitor120v.fmtG
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.log10
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

private val GridColor = Color(0x33FFFFFF)
private val AxisText = TextStyle(color = MOnSurfaceDim, fontSize = 10.sp)

/** About n round-numbered ticks covering lo..hi. */
fun niceTicks(lo: Double, hi: Double, n: Int = 5): List<Double> {
    if (!(hi > lo) || lo.isNaN() || hi.isNaN()) return listOf(lo)
    val raw = (hi - lo) / n
    val mag = 10.0.pow(floor(log10(raw)))
    val step = listOf(1.0, 2.0, 2.5, 5.0, 10.0).first { it * mag >= raw } * mag
    val out = ArrayList<Double>()
    var t = ceil(lo / step) * step
    while (t <= hi + step * 1e-9) {
        out += if (abs(t) < step * 1e-9) 0.0 else t
        t += step
    }
    return out
}

/** Tick text with as many decimals as the tick spacing needs. */
private fun tickLabels(ticks: List<Double>): List<String> {
    val step = if (ticks.size > 1) ticks[1] - ticks[0] else 1.0
    val dp = (-floor(log10(step)) + 1e-9).toInt().coerceIn(0, 6)
    return ticks.map { String.format(java.util.Locale.US, "%.${dp}f", it) }
}

/** Plot area inside the axis labels. */
private class Frame(val left: Float, val top: Float, val right: Float, val bottom: Float) {
    val w get() = right - left
    val h get() = bottom - top
}

private fun DrawScope.frame(dual: Boolean): Frame {
    val l = 46.dp2px(this)
    val r = if (dual) size.width - 46.dp2px(this) else size.width - 8.dp2px(this)
    return Frame(l, 6.dp2px(this), r, size.height - 30.dp2px(this))
}

private fun Int.dp2px(d: DrawScope) = this * d.density

private fun DrawScope.yAxis(
    tm: TextMeasurer, f: Frame, lo: Double, hi: Double, color: Color, rightSide: Boolean, grid: Boolean,
) {
    val ticks = niceTicks(lo, hi, 4)
    for ((t, label) in ticks.zip(tickLabels(ticks))) {
        val y = (f.bottom - (t - lo) / (hi - lo) * f.h).toFloat()
        if (grid) drawLine(GridColor, Offset(f.left, y), Offset(f.right, y), 1f)
        val txt = tm.measure(label, AxisText.copy(color = color))
        val x = if (rightSide) f.right + 3 * density else f.left - txt.size.width - 3 * density
        drawText(txt, topLeft = Offset(x, y - txt.size.height / 2f))
    }
}

private fun DrawScope.xAxis(tm: TextMeasurer, f: Frame, lo: Double, hi: Double, unit: String) {
    val ticks = niceTicks(lo, hi, 5)
    for ((t, label) in ticks.zip(tickLabels(ticks))) {
        val x = (f.left + (t - lo) / (hi - lo) * f.w).toFloat()
        drawLine(GridColor, Offset(x, f.top), Offset(x, f.bottom), 1f)
        val txt = tm.measure(label, AxisText)
        drawText(txt, topLeft = Offset(x - txt.size.width / 2f, f.bottom + 2 * density))
    }
    val u = tm.measure(unit, AxisText)          // its own line under the tick labels
    drawText(u, topLeft = Offset(f.left + (f.w - u.size.width) / 2f, f.bottom + 15 * density))
}

private fun DrawScope.legend(tm: TextMeasurer, f: Frame, items: List<Pair<String, Color>>) {
    var x = f.left + 4 * density
    for ((name, c) in items) {
        val t = tm.measure(name, AxisText.copy(color = c))
        drawText(t, topLeft = Offset(x, f.top + 2 * density))
        x += t.size.width + 10 * density
    }
}

private fun range(xs: List<Double>, pad: Double = 0.05): Pair<Double, Double> {
    val ok = xs.filter { !it.isNaN() }
    if (ok.isEmpty()) return 0.0 to 1.0
    var lo = ok.min()
    var hi = ok.max()
    if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5 }
    val p = (hi - lo) * pad
    return lo - p to hi + p
}

/** Two quantities against time (seconds before now): `left` on the left axis,
 *  `right` on the right one (fixed range if rightFixed). */
@Composable
fun TrendChart(
    rows: List<PwrRecord>,
    left: Triple<String, String, Color>,           // key, label, colour
    right: Triple<String, String, Color>,
    modifier: Modifier = Modifier,
    rightFixed: Pair<Double, Double>? = null,
) {
    val tm = rememberTextMeasurer()
    Canvas(modifier) {
        val f = frame(dual = true)
        drawRect(MBorder, Offset(f.left, f.top), Size(f.w, f.h), style = Stroke(1f))
        if (rows.size < 2) {
            val t = tm.measure("waiting for readings…", AxisText)
            drawText(t, topLeft = Offset(f.left + 8 * density, f.top + f.h / 2))
            return@Canvas
        }
        val now = rows.last().t
        val ts = rows.map { (it.t - now) / 1000.0 }
        val x0 = ts.first()
        val x1 = 0.0
        xAxis(tm, f, x0, x1, "s")
        fun px(t: Double) = (f.left + (t - x0) / (x1 - x0) * f.w).toFloat()
        for ((k, side) in listOf(left to false, right to true)) {
            val ys = rows.map { it[k.first] }
            val (lo, hi) = if (side && rightFixed != null) rightFixed else range(ys)
            yAxis(tm, f, lo, hi, k.third, rightSide = side, grid = !side)
            val path = Path()
            var started = false
            for (j in rows.indices) {
                val y = ys[j]
                if (y.isNaN()) { started = false; continue }
                val py = (f.bottom - (y - lo) / (hi - lo) * f.h).toFloat()
                if (!started) { path.moveTo(px(ts[j]), py); started = true } else path.lineTo(px(ts[j]), py)
            }
            drawPath(path, k.third, style = Stroke(1.5f * density))
        }
        legend(tm, f, listOf(left.second to left.third, right.second to right.third))
    }
}

/** The time window after a pinch of `zoom` about pixel cx and a pan of panX px.
 *  Clamped inside `full`; never an empty range (rounding made one and crashed). */
private fun zoomView(
    view: Pair<Double, Double>, full: Pair<Double, Double>, minSpan: Double,
    width: Float, cx: Float, panX: Float, zoom: Float,
): Pair<Double, Double> {
    val (a, b) = view
    val fullSpan = full.second - full.first
    val span = b - a
    if (!(span > 0) || !(fullSpan > 0) || width <= 0f || !zoom.isFinite() || zoom <= 0f) return view
    val nspan = (span / zoom).coerceIn(min(minSpan, fullSpan), fullSpan)
    if (nspan >= fullSpan) return full
    val at = a + span * (cx / width)
    val na = (at - (at - a) * (nspan / span) - panX / width * nspan)
        .coerceIn(full.first, max(full.first, full.second - nspan))
    return na to na + nspan
}

/** Capture waveform: V on the left axis, I on the right, both centred on zero.
 *  Two fingers pinch / drag to zoom and pan in time; double-tap to reset. */
@Composable
fun WaveChart(cap: Capture, modifier: Modifier = Modifier) {
    val tm = rememberTextMeasurer()
    val full = cap.tMs.first() to cap.tMs.last().coerceAtLeast(cap.tMs.first() + 1e-6)
    var view by remember(cap) { mutableStateOf(full) }
    val vLim = remember(cap) { cap.v?.let { a -> (a.maxOf { abs(it) } * 1.08).takeIf { it > 0 } ?: 1.0 } }
    val iLim = remember(cap) { cap.i?.let { a -> (a.maxOf { abs(it) } * 1.08).takeIf { it > 0 } ?: 1.0 } }
    Canvas(
        modifier
            .pointerInput(cap) {
                // two fingers zoom and pan; one finger is left alone so the page still scrolls
                awaitEachGesture {
                    awaitFirstDown(requireUnconsumed = false)
                    do {
                        val ev = awaitPointerEvent()
                        if (ev.changes.count { it.pressed } >= 2) {
                            view = zoomView(view, full, cap.dtUs / 1000.0 * 10, size.width.toFloat(),
                                ev.calculateCentroid().x, ev.calculatePan().x, ev.calculateZoom())
                            ev.changes.forEach { it.consume() }
                        }
                    } while (ev.changes.any { it.pressed })
                }
            }
            .pointerInput(cap) { detectTapGestures(onDoubleTap = { view = full }) },
    ) {
        val dual = cap.v != null && cap.i != null
        val f = frame(dual)
        drawRect(MBorder, Offset(f.left, f.top), Size(f.w, f.h), style = Stroke(1f))
        val (x0, x1) = view
        xAxis(tm, f, x0, x1, if (cap.trigIdx >= 0) "ms from trigger" else "ms")
        fun px(t: Double) = (f.left + (t - x0) / (x1 - x0) * f.w).toFloat()
        drawLine(GridColor, Offset(f.left, f.top + f.h / 2), Offset(f.right, f.top + f.h / 2), 1.5f)

        // range relay moving (LOW -> HIGH), shaded
        if (cap.rngIdx >= 0 && cap.i != null) {
            val a = px(cap.msAt(cap.rngIdx)).coerceIn(f.left, f.right)
            val b = px(cap.msAt(cap.rngIdx) + cap.settleUs / 1000).coerceIn(f.left, f.right)
            drawRect(MAmber.copy(alpha = 0.15f), Offset(a, f.top), Size(max(b - a, 1f), f.h))
        }

        val k0 = ((x0 - cap.tMs[0]) / (cap.dtUs / 1000)).toInt().coerceIn(0, cap.n - 1)
        val k1 = (ceil((x1 - cap.tMs[0]) / (cap.dtUs / 1000)).toInt() + 1).coerceIn(0, cap.n - 1)
        val series = listOfNotNull(
            cap.v?.let { Triple(it, vLim!!, VColor) },
            cap.i?.let { Triple(it, iLim!!, IColor) },
        )
        series.forEachIndexed { s, (data, lim, color) ->
            yAxis(tm, f, -lim, lim, color, rightSide = s == 1, grid = s == 0)
            fun py(y: Double) = (f.bottom - (y + lim) / (2 * lim) * f.h).toFloat()
            val path = Path()
            val cols = f.w.toInt().coerceAtLeast(1)
            if (k1 - k0 <= cols * 2) {
                for (k in k0..k1) {
                    if (k == k0) path.moveTo(px(cap.tMs[k]), py(data[k])) else path.lineTo(px(cap.tMs[k]), py(data[k]))
                }
            } else {
                // min/max envelope per pixel column: every spike stays visible
                val per = (k1 - k0 + 1).toDouble() / cols
                for (c in 0 until cols) {
                    val a = k0 + (c * per).toInt()
                    val b = min(k1, k0 + ((c + 1) * per).toInt())
                    var lo = data[a]
                    var hi = data[a]
                    for (k in a..b) { lo = min(lo, data[k]); hi = max(hi, data[k]) }
                    val x = f.left + c + 0.5f
                    if (c == 0) path.moveTo(x, py(lo)) else path.lineTo(x, py(lo))
                    path.lineTo(x, py(hi))
                }
            }
            drawPath(path, color, style = Stroke(1.2f * density))
        }

        val dash = PathEffect.dashPathEffect(floatArrayOf(8f, 6f))
        fun vline(t: Double, c: Color, dashed: Boolean) {
            val x = px(t)
            if (x < f.left || x > f.right) return
            drawLine(c, Offset(x, f.top), Offset(x, f.bottom), 1.5f * density, pathEffect = if (dashed) dash else null)
        }
        if (cap.trigIdx >= 0) vline(0.0, MOnSurface, true)
        if (cap.relayIdx >= 0) vline(cap.msAt(cap.relayIdx), MAmber, false)
        if (cap.openIdx >= 0) vline(cap.msAt(cap.openIdx), MPurple, true)

        val items = ArrayList<Pair<String, Color>>()
        if (cap.v != null) items += "V" to VColor
        if (cap.i != null) items += "A" to IColor
        if (cap.relayIdx >= 0) items += "close cmd" to MAmber
        if (cap.openIdx >= 0) items += "open: ${SAFE_WHY[cap.openWhy] ?: "?"}" to MPurple
        if (cap.rngIdx >= 0 && cap.i != null) items += "range moving" to MAmber.copy(alpha = 0.7f)
        legend(tm, f, items)
    }
}

/** Harmonics as % of the fundamental, log scale 0.01…150 %. */
@Composable
fun HarmonicsChart(st: CapStats, modifier: Modifier = Modifier) {
    val tm = rememberTextMeasurer()
    Canvas(modifier) {
        val f = frame(dual = false)
        drawRect(MBorder, Offset(f.left, f.top), Size(f.w, f.h), style = Stroke(1f))
        val lo = -2.0
        val hi = log10(150.0)
        fun py(pct: Double) = (f.bottom - (log10(pct.coerceAtLeast(0.01)) - lo) / (hi - lo) * f.h).toFloat()
        for (e in listOf(0.01, 0.1, 1.0, 10.0, 100.0)) {
            val y = py(e)
            drawLine(GridColor, Offset(f.left, y), Offset(f.right, y), 1f)
            val t = tm.measure(fmtG(e, 2) + "%", AxisText)
            drawText(t, topLeft = Offset(f.left - t.size.width - 3 * density, y - t.size.height / 2f))
        }
        val sets = listOfNotNull(st.v?.harm?.amps?.let { it to VColor }, st.i?.harm?.amps?.let { it to IColor })
            .filter { it.first.size > 1 && it.first[0] > 0 }
        val nh = sets.maxOfOrNull { it.first.size } ?: 0
        if (nh == 0) return@Canvas
        val slot = f.w / nh
        sets.forEachIndexed { s, (amps, color) ->
            val bw = slot / (sets.size + 1)
            for (k in amps.indices) {
                val pct = 100 * amps[k] / amps[0]
                val x = f.left + k * slot + bw * (s + 0.5f)
                val y = py(pct)
                drawRect(color, Offset(x, y), Size(bw, f.bottom - y))
            }
        }
        for (h in niceTicks(1.0, nh.toDouble(), 6).filter { it >= 1 }) {
            val x = f.left + (h.toFloat() - 0.5f) * slot
            val t = tm.measure(h.toInt().toString(), AxisText)
            drawText(t, topLeft = Offset(x - t.size.width / 2f, f.bottom + 2 * density))
        }
        val u = tm.measure("harmonic", AxisText)
        drawText(u, topLeft = Offset(f.left + (f.w - u.size.width) / 2f, f.bottom + 15 * density))
        legend(tm, f, sets.map { (_, c) -> (if (c == VColor) "V" else "I") + " % of fundamental" to c })
    }
}
