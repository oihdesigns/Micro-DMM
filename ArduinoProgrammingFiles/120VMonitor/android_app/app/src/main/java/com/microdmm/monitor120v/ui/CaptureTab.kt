package com.microdmm.monitor120v.ui

import android.net.Uri
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.relocation.BringIntoViewRequester
import androidx.compose.foundation.relocation.bringIntoViewRequester
import androidx.compose.foundation.selection.toggleable
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Checkbox
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.SegmentedButton
import androidx.compose.material3.SegmentedButtonDefaults
import androidx.compose.material3.SingleChoiceSegmentedButtonRow
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.microdmm.monitor120v.Action
import com.microdmm.monitor120v.CAP_RANGES
import com.microdmm.monitor120v.CH_OPTIONS
import com.microdmm.monitor120v.CapSettings
import com.microdmm.monitor120v.CapStats
import com.microdmm.monitor120v.Capture
import com.microdmm.monitor120v.Conn
import com.microdmm.monitor120v.RANGE_NAMES
import com.microdmm.monitor120v.SAFE_WHY
import com.microdmm.monitor120v.TIMING_MODES
import com.microdmm.monitor120v.TRIG_MODES
import com.microdmm.monitor120v.UiState
import com.microdmm.monitor120v.cyclePhase
import com.microdmm.monitor120v.slipMask
import com.microdmm.monitor120v.flagText
import com.microdmm.monitor120v.nowStamp
import java.util.Locale

/** A save-as picker for a CSV; call the result with the suggested file name. */
@Composable
fun rememberCreateCsv(onPicked: (Uri, String) -> Unit): (String) -> Unit {
    var name by rememberSaveable { mutableStateOf("") }
    val launcher = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("text/csv")) { uri ->
        if (uri != null) onPicked(uri, name)
    }
    return { n -> name = n; launcher.launch(n) }
}

@Composable
fun NumField(label: String, value: String, onChange: (String) -> Unit, modifier: Modifier = Modifier, readOnly: Boolean = false) {
    OutlinedTextField(
        value = value, onValueChange = onChange, label = { Text(label, fontSize = 11.sp) },
        singleLine = true, readOnly = readOnly, enabled = !readOnly,
        textStyle = TextStyle(fontSize = 14.sp, fontFamily = FontFamily.Monospace),
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
        modifier = modifier,
    )
}

@Composable
private fun Labeled(label: String, content: @Composable () -> Unit) {
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        Text(label, style = MaterialTheme.typography.labelMedium, color = MOnSurfaceDim)
        content()
    }
}

@Composable
private fun Check(label: String, checked: Boolean, onChange: (Boolean) -> Unit) {
    // the label toggles it too
    Row(
        Modifier.toggleable(value = checked, role = Role.Checkbox, onValueChange = onChange),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Checkbox(checked = checked, onCheckedChange = null)
        Text(label, style = MaterialTheme.typography.bodySmall, modifier = Modifier.padding(start = 4.dp, end = 8.dp))
    }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun CaptureTab(state: UiState, onAction: (Action) -> Unit) {
    val c = state.cap
    fun up(f: (CapSettings) -> CapSettings) = onAction(Action.UpdateCap(f(c)))
    val saveLauncher = rememberCreateCsv { uri, _ -> onAction(Action.SaveCapture(uri)) }
    var showSettings by rememberSaveable { mutableStateOf(true) }
    val derived = listOf("dur", "n", "rate")[c.tmode]

    state.confirmCapture?.let { msg ->
        AlertDialog(
            onDismissRequest = { onAction(Action.ConfirmCapture(false)) },
            title = { Text("Inrush capture") },
            text = { Text(msg) },
            confirmButton = { TextButton(onClick = { onAction(Action.ConfirmCapture(true)) }) { Text("Close relay") } },
            dismissButton = { TextButton(onClick = { onAction(Action.ConfirmCapture(false)) }) { Text("Cancel") } },
        )
    }

    Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(8.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = { onAction(Action.Capture) }, enabled = state.conn == Conn.Connected && !state.capBusy && !state.liveOn) {
                Text(if (state.capBusy) "Capturing…" else "Capture")
            }
            OutlinedButton(onClick = { onAction(Action.AbortCapture) }, enabled = state.capBusy) { Text("Abort") }
            Check("Repeat", c.repeat) { v -> up { it.copy(repeat = v) } }
        }
        Check("Readings while armed (meter + log keep going; lower max rate)", c.mon) { v -> up { it.copy(mon = v) } }
        Row(verticalAlignment = Alignment.CenterVertically) {
            OutlinedButton(
                onClick = { saveLauncher("capture_${nowStamp("yyyyMMdd_HHmmss")}.csv") },
                enabled = state.capture != null,
            ) { Text("Save CSV") }
            TextButton(onClick = { showSettings = !showSettings }) { Text(if (showSettings) "Hide settings ▴" else "Settings ▾") }
            Check("Harmonics", c.showHarm) { v -> up { it.copy(showHarm = v) } }
        }
        // the same capture as a time series, or folded onto one mains cycle
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text("View", style = MaterialTheme.typography.labelMedium, color = MOnSurfaceDim,
                modifier = Modifier.padding(end = 8.dp))
            SingleChoiceSegmentedButtonRow {
                listOf("Time", "Heat map").forEachIndexed { k, label ->
                    SegmentedButton(
                        selected = c.heatMap == (k == 1), onClick = { up { it.copy(heatMap = k == 1) } },
                        shape = SegmentedButtonDefaults.itemShape(k, 2),
                    ) { Text(label, fontSize = 12.sp, maxLines = 1, softWrap = false) }
                }
            }
        }

        if (showSettings) {
            Card(colors = CardDefaults.cardColors(containerColor = MSurface), modifier = Modifier.fillMaxWidth()) {
                Column(Modifier.padding(8.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    Labeled("Trigger") {
                        Dropdown(TRIG_MODES[c.trig] ?: "?", TRIG_MODES.values.toList(),
                            onPick = { k -> up { it.copy(trig = TRIG_MODES.keys.toList()[k]) } })
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Labeled("Channels") {
                            Dropdown(CH_OPTIONS[c.ch] ?: "?", CH_OPTIONS.values.toList(),
                                onPick = { k -> up { it.copy(ch = CH_OPTIONS.keys.toList()[k]) } })
                        }
                        Labeled("Range") {
                            Dropdown(CAP_RANGES[c.rng] ?: "?", CAP_RANGES.values.toList(),
                                onPick = { k -> up { it.copy(rng = CAP_RANGES.keys.toList()[k]) } })
                        }
                    }
                    Labeled("Set by") {
                        Dropdown(TIMING_MODES[c.tmode], TIMING_MODES, onPick = { k -> up { it.copy(tmode = k) } })
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        NumField("Rate S/s (0=max)", c.rate, { v -> up { it.copy(rate = v) } }, Modifier.weight(1f), derived == "rate")
                        NumField("Samples", c.n, { v -> up { it.copy(n = v) } }, Modifier.weight(1f), derived == "n")
                        NumField("Duration ms", c.dur, { v -> up { it.copy(dur = v) } }, Modifier.weight(1f), derived == "dur")
                    }
                    if (state.timingNote.isNotEmpty()) Text(state.timingNote, color = MAmber, style = MaterialTheme.typography.bodySmall)
                    Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        NumField(if (c.trig == 6) "Level A rms" else "Level (A or V)", c.level, { v -> up { it.copy(level = v) } }, Modifier.weight(1f))
                        NumField("Pretrigger %", c.pre, { v -> up { it.copy(pre = v) } }, Modifier.weight(1f))
                        NumField("Timeout s (0 none)", c.timeoutS, { v -> up { it.copy(timeoutS = v) } }, Modifier.weight(1f))
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        NumField("Avg N scans/sample", c.avg, { v -> up { it.copy(avg = v) } }, Modifier.weight(1f))
                        NumField("Close delay after V↑0 (µs)", c.relayUs, { v -> up { it.copy(relayUs = v) } },
                            Modifier.weight(1.4f), readOnly = c.trig != 3)
                    }
                    if (state.spikeText.isNotEmpty()) Text(state.spikeText, color = MOnSurfaceDim, style = MaterialTheme.typography.bodySmall)
                    if (state.rateText.isNotEmpty()) Text(state.rateText, color = MOnSurfaceDim, style = MaterialTheme.typography.bodySmall)

                    Text("Safe start — the board opens the relay itself (recording continues)",
                        style = MaterialTheme.typography.labelLarge, modifier = Modifier.padding(top = 6.dp))
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Check("Instant trip above", c.ipkOn) { v -> up { it.copy(ipkOn = v) } }
                        NumField("A", c.ipkA, { v -> up { it.copy(ipkA = v) } }, Modifier.width(90.dp))
                    }
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Check("Over-current", c.ovOn) { v -> up { it.copy(ovOn = v) } }
                        NumField("A rms", c.ovA, { v -> up { it.copy(ovA = v) } }, Modifier.width(80.dp))
                        Spacer(Modifier.width(6.dp))
                        NumField("cycles", c.ovCyc, { v -> up { it.copy(ovCyc = v) } }, Modifier.width(80.dp))
                    }
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Check("Auto-open", c.autoOn) { v -> up { it.copy(autoOn = v) } }
                        NumField("ms after trigger", c.autoMs, { v -> up { it.copy(autoMs = v) } }, Modifier.width(130.dp))
                    }
                    Check("Open relay when capture completes", c.openAfter) { v -> up { it.copy(openAfter = v) } }
                }
            }
        }

        val cap = state.capture
        val st = state.capStats
        val results = remember { BringIntoViewRequester() }
        LaunchedEffect(cap) { if (cap != null) results.bringIntoView() }
        Column(Modifier.bringIntoViewRequester(results)) {
        if (state.capMsg.isNotEmpty()) Text(state.capMsg, color = MAmber, modifier = Modifier.padding(vertical = 4.dp))
        if (cap != null && st != null) {
            Text(
                statsText(cap, st, state.twoRanges), fontFamily = FontFamily.Monospace, fontSize = 10.sp,
                modifier = Modifier.padding(vertical = 6.dp),
            )
            if (c.heatMap) {
                val ref = cap.v ?: cap.i!!
                val (phase, cycles0) = remember(cap) { cyclePhase(ref, cap.dtUs * 1e-6, st.hz) }
                val (keep, dropped) = remember(cap) { slipMask(ref, cap.flags) }
                val cycles = if (keep != null) keep.count { it } * cap.dtUs * 1e-6 * st.hz else cycles0
                val refName = if (cap.v != null) "V" else "I"
                val chans = listOfNotNull(cap.v?.let { Triple(it, "V", VColor) }, cap.i?.let { Triple(it, "A", IColor) })
                for ((data, unit, color) in chans) {
                    HeatChart(data, phase, cycles, st.hz, unit, refName, color,
                        Modifier.fillMaxWidth().height(if (chans.size == 2) 220.dp else 300.dp), keep)
                }
                if (dropped > 0) Text("$dropped cycles left out: LATE timing slip", color = MAmber, fontSize = 10.sp)
                Text("every cycle overlaid: bright = where the waveform usually is, dim = the odd cycle out",
                    color = MOnSurfaceDim, fontSize = 10.sp)
            } else {
                WaveChart(cap, Modifier.fillMaxWidth().height(300.dp))
                Text("two fingers: pinch to zoom, drag to pan · double-tap to reset", color = MOnSurfaceDim, fontSize = 10.sp)
            }
            if (c.showHarm) {
                Spacer(Modifier.height(8.dp))
                HarmonicsChart(st, Modifier.fillMaxWidth().height(180.dp))
            }
        }
        }
    }
}

private fun statsText(cap: Capture, st: CapStats, twoRanges: Boolean): String {
    val f = { fmt: String, a: Array<Any> -> String.format(Locale.US, fmt, *a) }
    val out = StringBuilder()
    out.append(f("%d samples @ %,.0f S/s (%.2f µs)  f %.3f Hz", arrayOf(cap.n, st.rate, cap.dtUs, st.hz)))
    if (cap.avg > 1) out.append("\neach the mean of ${cap.avg} scans")
    if (cap.i != null && twoRanges) {
        out.append("\nrange ${RANGE_NAMES[cap.rng] ?: "?"}")
        if (cap.rngIdx >= 0) {
            out.append(f(" → HIGH (overload) at %.2f ms; the next %s ms is the relay moving",
                arrayOf(cap.msAt(cap.rngIdx), com.microdmm.monitor120v.fmtG(cap.settleUs / 1000, 4))))
        }
    }
    st.v?.let { v ->
        out.append(f("\nV %.3f Vrms  dc %+.3f  pk %.2f  crest %.3f  THD %.2f%%",
            arrayOf(v.rms, v.dc, v.pk, v.crest, v.harm.thd * 100)))
    }
    st.i?.let { i ->
        out.append(f("\nI %.4f Arms  dc %+.4f  pk %.3f  crest %.3f  THD %.2f%%",
            arrayOf(i.rms, i.dc, i.pk, i.crest, i.harm.thd * 100)))
    }
    if (st.p != null) out.append(f("\nP %.3f W  PF %.4f", arrayOf(st.p, st.pf ?: Double.NaN)))
    if (cap.openIdx >= 0) {
        out.append(f("\nrelay opened by %s at %.2f ms", arrayOf(SAFE_WHY[cap.openWhy] ?: "?", cap.msAt(cap.openIdx))))
    }
    if (cap.flags != 0) out.append("\nflags: " + flagText(cap.flags))
    return out.toString()
}
