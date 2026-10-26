package com.microdmm.monitor120v.ui

import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Checkbox
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.microdmm.monitor120v.Action
import com.microdmm.monitor120v.ApplyState
import com.microdmm.monitor120v.CfgRow
import com.microdmm.monitor120v.Conn
import com.microdmm.monitor120v.GROUP_INFO
import com.microdmm.monitor120v.GROUP_OF
import com.microdmm.monitor120v.UiState
import com.microdmm.monitor120v.fmtG
import com.microdmm.monitor120v.nowStamp
import kotlin.math.abs
import kotlin.math.roundToLong

private fun valueText(v: Double) =
    if (abs(v - Math.round(v)) > 1e-9 || abs(v) > 1e5) fmtG(v, 6) else v.roundToLong().toString()

private enum class Ask { None, Defaults, Zero, Export, Logs }

@Composable
fun ConfigTab(state: UiState, onAction: (Action) -> Unit) {
    val connected = state.conn == Conn.Connected
    var edit by remember { mutableStateOf<CfgRow?>(null) }
    var ask by remember { mutableStateOf(Ask.None) }
    var exportSaved by rememberSaveable { mutableStateOf(true) }
    val exportLauncher = rememberCreateCsv { uri, _ -> onAction(Action.Export(uri, exportSaved)) }
    val openLauncher = rememberLauncherForActivityResult(ActivityResultContracts.OpenDocument()) { uri ->
        if (uri != null) onAction(Action.ApplyFromUri(uri, uri.lastPathSegment?.substringAfterLast('/') ?: "snapshot"))
    }
    val board = state.board
    fun startExport(saved: Boolean) {
        exportSaved = saved
        exportLauncher("${board}_${nowStamp("yyyyMMdd_HHmmss")}.csv")
    }

    Column(Modifier.fillMaxSize().padding(6.dp)) {
        Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            OutlinedButton(onClick = { onAction(Action.CfgRefresh) }, enabled = connected) { Text("Refresh") }
            OutlinedButton(onClick = { onAction(Action.CfgSave) }, enabled = connected) { Text("Save to EEPROM") }
            OutlinedButton(onClick = { onAction(Action.CfgLoad) }, enabled = connected) { Text("Reload EEPROM") }
            OutlinedButton(onClick = { ask = Ask.Defaults }, enabled = connected) { Text("Factory defaults") }
            OutlinedButton(onClick = { ask = Ask.Zero }, enabled = connected) { Text("Zero offsets") }
        }
        Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            OutlinedButton(onClick = { if (state.dirty) ask = Ask.Export else startExport(true) }, enabled = connected) {
                Text("Export snapshot…")
            }
            OutlinedButton(onClick = { onAction(Action.ListLogFiles); ask = Ask.Logs }, enabled = connected) {
                Text("Apply from log…")
            }
            OutlinedButton(onClick = { openLauncher.launch(arrayOf("text/*", "application/octet-stream")) }, enabled = connected) {
                Text("Apply from file…")
            }
        }
        if (state.dirty) Text("unsaved changes", color = MAmber, style = MaterialTheme.typography.labelMedium)
        Text("Tap a key to change it (RAM, takes effect at once); Save to EEPROM keeps it.",
            color = MOnSurfaceDim, style = MaterialTheme.typography.labelSmall)
        LazyColumn(Modifier.fillMaxSize()) {
            items(state.cfg, key = { it.key }) { row ->
                Column(
                    Modifier.fillMaxWidth().clickable(enabled = connected) { edit = row }.padding(vertical = 5.dp),
                ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Text(row.key, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace, modifier = Modifier.width(96.dp))
                        Text(GROUP_OF[row.key] ?: "", color = MCyan, fontSize = 11.sp, modifier = Modifier.width(60.dp))
                        Text(valueText(row.value), color = MGreen, fontFamily = FontFamily.Monospace, modifier = Modifier.weight(1f))
                        Text("${fmtG(row.lo, 6)} … ${fmtG(row.hi, 6)}", color = MOnSurfaceDim, fontSize = 11.sp)
                    }
                    Text(row.help, color = MOnSurfaceDim, fontSize = 11.sp)
                }
                HorizontalDivider(color = MBorder.copy(alpha = 0.4f))
            }
        }
    }

    edit?.let { row ->
        var text by remember(row.key) { mutableStateOf(valueText(row.value)) }
        AlertDialog(
            onDismissRequest = { edit = null },
            title = { Text(row.key) },
            text = {
                Column {
                    Text(row.help, style = MaterialTheme.typography.bodySmall)
                    Text("range ${fmtG(row.lo, 6)} … ${fmtG(row.hi, 6)}", color = MOnSurfaceDim, style = MaterialTheme.typography.bodySmall)
                    OutlinedTextField(value = text, onValueChange = { text = it }, singleLine = true,
                        modifier = Modifier.padding(top = 8.dp))
                }
            },
            confirmButton = {
                TextButton(onClick = { onAction(Action.CfgSet(row.key, text)); edit = null }) { Text("Set") }
            },
            dismissButton = { TextButton(onClick = { edit = null }) { Text("Cancel") } },
        )
    }

    when (ask) {
        Ask.Defaults -> Confirm(
            "Defaults", "Reset every key (including calibration) to factory defaults in RAM? Nothing is saved until you Save.",
            "Reset", { onAction(Action.CfgDefaults) }, { ask = Ask.None },
        )
        Ask.Zero -> Confirm(
            "Zero offsets", "Store the present DC readings as VOFF / IOFF (RAM). Only with no mains on the input and no load.",
            "Zero", { onAction(Action.CfgZero) }, { ask = Ask.None },
        )
        Ask.Export -> AlertDialog(
            onDismissRequest = { ask = Ask.None },
            title = { Text("Export snapshot") },
            text = { Text("There are unsaved changes. Export the running values or the saved ones?") },
            confirmButton = { TextButton(onClick = { ask = Ask.None; startExport(false) }) { Text("Running") } },
            dismissButton = { TextButton(onClick = { ask = Ask.None; startExport(true) }) { Text("Saved") } },
        )
        Ask.Logs -> AlertDialog(
            onDismissRequest = { ask = Ask.None },
            title = { Text("Config log") },
            text = {
                if (state.logFiles.isEmpty()) {
                    Text("No snapshots logged yet. One is logged on every connect and save.")
                } else {
                    LazyColumn(Modifier.heightIn(max = 400.dp)) {
                        items(state.logFiles) { name ->
                            Text(
                                name, fontFamily = FontFamily.Monospace, fontSize = 13.sp,
                                modifier = Modifier.fillMaxWidth()
                                    .clickable { ask = Ask.None; onAction(Action.ApplyFromLog(name)) }
                                    .padding(vertical = 8.dp),
                            )
                        }
                    }
                }
            },
            confirmButton = { TextButton(onClick = { ask = Ask.None }) { Text("Close") } },
        )
        Ask.None -> Unit
    }

    state.apply?.let { ApplyDialog(it, onAction) }
}

@Composable
private fun Confirm(title: String, text: String, ok: String, onOk: () -> Unit, onDone: () -> Unit) {
    AlertDialog(
        onDismissRequest = onDone,
        title = { Text(title) },
        text = { Text(text) },
        confirmButton = { TextButton(onClick = { onOk(); onDone() }) { Text(ok) } },
        dismissButton = { TextButton(onClick = onDone) { Text("Cancel") } },
    )
}

/** Copy chosen groups of a snapshot onto the connected board (RAM, then
 *  optionally saved). Only keys whose values differ are sent. */
@Composable
private fun ApplyDialog(ap: ApplyState, onAction: (Action) -> Unit) {
    val sm = ap.snap.meta
    val hm = ap.here.meta
    AlertDialog(
        onDismissRequest = { if (!ap.busy) onAction(Action.ApplyClose) },
        title = { Text("Apply ${ap.name}", fontSize = 16.sp) },
        text = {
            Column(Modifier.verticalScroll(rememberScrollState())) {
                Text(
                    "File : ${sm["board"] ?: "?"} uid ${sm["uid"].orEmpty().ifEmpty { "?" }} fw ${sm["fw"] ?: "?"}\n" +
                        "       ${sm["src"] ?: "?"} values, taken ${sm["taken"] ?: "?"}\n" +
                        "Board: ${hm["board"] ?: "?"} uid ${hm["uid"].orEmpty().ifEmpty { "?" }} fw ${hm["fw"] ?: "?"}" +
                        if (ap.sameUnit) "  (same unit)" else "",
                    fontFamily = FontFamily.Monospace, fontSize = 11.sp,
                )
                if (ap.otherBoard) {
                    Text(
                        "Different board type. The CAL gains carry over if the isolation-amp front end moved with them, " +
                            "but zero the offsets again on this board and check PHASEUS. WIRING pin numbers do not carry over.",
                        color = MAmber, style = MaterialTheme.typography.bodySmall, modifier = Modifier.padding(top = 6.dp),
                    )
                }
                if (ap.missing.isNotEmpty()) {
                    Text("Not on this firmware, skipped: ${ap.missing.joinToString(", ")}", color = MOnSurfaceDim,
                        style = MaterialTheme.typography.bodySmall)
                }
                Text("Copy these groups", style = MaterialTheme.typography.labelLarge, modifier = Modifier.padding(top = 8.dp))
                for (g in ap.groups) {
                    val n = ap.counts[g] ?: 0
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Checkbox(checked = g in ap.on, onCheckedChange = { onAction(Action.ApplyToggle(g)) }, enabled = n > 0 && !ap.busy)
                        Text(
                            "$g — ${GROUP_INFO[g] ?: ""}: $n differ" +
                                if (g == "WIRING" && ap.otherBoard && n > 0) "  (pins from another board type!)" else "",
                            style = MaterialTheme.typography.bodySmall,
                            color = if (n > 0) MOnSurface else MOnSurfaceDim,
                        )
                    }
                }
                if (ap.diffs.isNotEmpty()) {
                    Row(Modifier.padding(top = 6.dp)) {
                        Text("Key", Modifier.weight(1f), fontSize = 11.sp, color = MOnSurfaceDim)
                        Text("This board", Modifier.weight(1f), fontSize = 11.sp, color = MOnSurfaceDim)
                        Text("Snapshot", Modifier.weight(1f), fontSize = 11.sp, color = MOnSurfaceDim)
                    }
                }
                for (d in ap.diffs) {
                    val col = if (d.group in ap.on) MOnSurface else MOnSurfaceDim.copy(alpha = 0.6f)
                    Row {
                        Text(d.key, Modifier.weight(1f), fontFamily = FontFamily.Monospace, fontSize = 11.sp, color = col)
                        Text(d.current?.let { fmtG(it, 7) } ?: "—", Modifier.weight(1f), fontFamily = FontFamily.Monospace, fontSize = 11.sp, color = col)
                        Text(fmtG(d.snap, 7), Modifier.weight(1f), fontFamily = FontFamily.Monospace, fontSize = 11.sp, color = col)
                    }
                }
                if (ap.msg.isNotEmpty()) Text(ap.msg, modifier = Modifier.padding(top = 8.dp), color = MGreen)
            }
        },
        confirmButton = {
            Row {
                TextButton(onClick = { onAction(Action.ApplyRun(false)) }, enabled = ap.todo.isNotEmpty() && !ap.busy) { Text("Apply to RAM") }
                TextButton(onClick = { onAction(Action.ApplyRun(true)) }, enabled = ap.todo.isNotEmpty() && !ap.busy) { Text("Apply and save") }
            }
        },
        dismissButton = { TextButton(onClick = { onAction(Action.ApplyClose) }, enabled = !ap.busy) { Text("Close") } },
    )
}
