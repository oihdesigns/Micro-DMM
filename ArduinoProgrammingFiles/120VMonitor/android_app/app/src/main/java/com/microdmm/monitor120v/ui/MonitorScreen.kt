package com.microdmm.monitor120v.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.imePadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.itemsIndexed
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Switch
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.microdmm.monitor120v.Action
import com.microdmm.monitor120v.Conn
import com.microdmm.monitor120v.RANGE_FULL
import com.microdmm.monitor120v.RANGE_MODES
import com.microdmm.monitor120v.UiState

private val TABS = listOf("Meter", "Capture", "Config", "Console")

@Composable
fun MonitorScreen(state: UiState, onAction: (Action) -> Unit, onConnectClick: () -> Unit) {
    var tab by rememberSaveable { mutableIntStateOf(0) }
    val snack = remember { SnackbarHostState() }
    LaunchedEffect(state.toast) {
        state.toast?.let {
            snack.showSnackbar(it)
            onAction(Action.DismissToast)
        }
    }
    Scaffold(
        containerColor = MBackground,
        snackbarHost = { SnackbarHost(snack) },
    ) { pad ->
        Column(Modifier.fillMaxSize().padding(pad).imePadding()) {
            TopBar(state, onAction, onConnectClick)
            TabRow(selectedTabIndex = tab, containerColor = MSurface) {
                TABS.forEachIndexed { k, name ->
                    Tab(selected = tab == k, onClick = { tab = k }, text = { Text(name) })
                }
            }
            Box(Modifier.fillMaxSize()) {
                when (tab) {
                    0 -> MeterTab(state, onAction)
                    1 -> CaptureTab(state, onAction)
                    2 -> ConfigTab(state, onAction)
                    else -> ConsoleTab(state, onAction)
                }
            }
        }
    }
}

@Composable
private fun TopBar(state: UiState, onAction: (Action) -> Unit, onConnectClick: () -> Unit) {
    val connected = state.conn == Conn.Connected
    var askNet by remember { mutableStateOf(false) }
    var hostPort by rememberSaveable { mutableStateOf("10.0.2.2:8120") }
    if (askNet) {
        AlertDialog(
            onDismissRequest = { askNet = false },
            title = { Text("Connect over the network") },
            text = {
                Column {
                    Text(
                        "host:port of sim_server.py (the simulator) or a serial-to-network bridge. " +
                            "From the Android emulator, the PC is 10.0.2.2.",
                        style = MaterialTheme.typography.bodySmall,
                    )
                    OutlinedTextField(value = hostPort, onValueChange = { hostPort = it }, singleLine = true,
                        modifier = Modifier.padding(top = 8.dp))
                }
            },
            confirmButton = {
                TextButton(onClick = { askNet = false; onAction(Action.ConnectTcp(hostPort.trim())) }) { Text("Connect") }
            },
            dismissButton = { TextButton(onClick = { askNet = false }) { Text("Cancel") } },
        )
    }
    Column(Modifier.fillMaxWidth().background(MSurface).padding(horizontal = 8.dp, vertical = 4.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(
                onClick = { if (state.conn == Conn.Disconnected) onConnectClick() else onAction(Action.Disconnect) },
                enabled = state.conn != Conn.Connecting,
                contentPadding = PaddingValues(horizontal = 12.dp),
            ) { Text(if (state.conn == Conn.Disconnected) "Connect" else "Disconnect") }
            if (state.conn == Conn.Disconnected) {
                TextButton(onClick = { askNet = true }, contentPadding = PaddingValues(horizontal = 6.dp)) { Text("Network…") }
            }
            Button(
                onClick = { onAction(Action.ToggleRelay) },
                enabled = connected,
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (state.relay) Color(0xFFC62828) else MGray,
                    contentColor = Color.White,
                ),
                contentPadding = PaddingValues(horizontal = 12.dp),
            ) { Text(if (state.relay) "LOAD ON" else "LOAD OFF", fontWeight = FontWeight.Bold) }
            Spacer(Modifier.weight(1f))
            Text("Stream", style = MaterialTheme.typography.labelMedium)
            Switch(
                checked = state.streaming,
                onCheckedChange = { onAction(Action.SetStream(it)) },
                enabled = connected,
            )
        }
        if (connected) {
            Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Range", style = MaterialTheme.typography.labelMedium)
                Dropdown(
                    value = RANGE_MODES[state.rngMode] ?: "?",
                    options = RANGE_MODES.values.toList(),
                    enabled = state.twoRanges,
                    onPick = { k -> onAction(Action.SetRangeMode(RANGE_MODES.keys.toList()[k])) },
                )
                Text(
                    when {
                        !state.twoRanges -> "single range (RNGPIN 0)"
                        state.rng == null -> ""
                        else -> (RANGE_FULL[state.rng] ?: "?") + if (state.rngNote.isNotEmpty()) "  ${state.rngNote}" else ""
                    },
                    color = if (state.twoRanges && state.rng == 1) MBlue else MOnSurfaceDim,
                    fontWeight = FontWeight.Bold,
                    style = MaterialTheme.typography.labelMedium,
                    maxLines = 1, overflow = TextOverflow.Ellipsis,
                )
            }
        }
        if (state.alert.isNotEmpty()) {
            Text(state.alert, color = MRed, fontWeight = FontWeight.Bold, style = MaterialTheme.typography.labelLarge)
        }
        Text(state.connMsg, color = MOnSurfaceDim, style = MaterialTheme.typography.labelSmall, maxLines = 1,
            overflow = TextOverflow.Ellipsis)
    }
}

/** A button that opens a list; onPick gets the index. */
@Composable
fun Dropdown(value: String, options: List<String>, onPick: (Int) -> Unit, enabled: Boolean = true, modifier: Modifier = Modifier) {
    var open by remember { mutableStateOf(false) }
    Box(modifier) {
        OutlinedButton(onClick = { open = true }, enabled = enabled, contentPadding = PaddingValues(horizontal = 10.dp)) {
            Text("$value ▾", maxLines = 1, overflow = TextOverflow.Ellipsis)
        }
        DropdownMenu(expanded = open, onDismissRequest = { open = false }) {
            options.forEachIndexed { k, o ->
                DropdownMenuItem(text = { Text(o) }, onClick = { open = false; onPick(k) })
            }
        }
    }
}

// ── Meter ───────────────────────────────────────────────────────────────────
private class Tile(val key: String, val label: String, val unit: String, val dp: Int)

private val TILES = listOf(
    Tile("vrms", "V rms", "V", 2), Tile("irms", "I rms", "A", 4), Tile("p", "Power", "W", 2),
    Tile("pf", "PF", "", 3), Tile("hz", "Freq", "Hz", 3), Tile("s", "Apparent", "VA", 2),
    Tile("q", "Non-active", "var", 2), Tile("vpk", "V peak", "V", 1), Tile("ipk", "I peak", "A", 3),
    Tile("crest", "I crest", "", 2), Tile("wh", "Energy", "Wh", 4), Tile("vdc", "V dc", "V", 3),
    Tile("idc", "I dc", "A", 4),
)

@Composable
private fun MeterTab(state: UiState, onAction: (Action) -> Unit) {
    val logLauncher = rememberCreateCsv { uri, name -> onAction(Action.StartLog(uri, name)) }
    Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(8.dp)) {
        val r = state.last
        TILES.chunked(3).forEachIndexed { row, tiles ->
            Row(Modifier.fillMaxWidth()) {
                for (t in tiles) {
                    val v = r?.get(t.key)
                    val txt = if (v == null || v.isNaN()) "---" else String.format(java.util.Locale.US, "%,.${t.dp}f %s", v, t.unit).trim()
                    Column(Modifier.weight(1f).padding(vertical = 4.dp, horizontal = 2.dp)) {
                        Text(t.label, color = MOnSurfaceDim, fontSize = 11.sp)
                        Text(
                            txt, color = MGreen, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                            fontSize = if (row == 0) 19.sp else 14.sp, maxLines = 1,
                        )
                    }
                }
                repeat(3 - tiles.size) { Spacer(Modifier.weight(1f)) }
            }
        }
        Row(
            Modifier.fillMaxWidth().padding(vertical = 6.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(6.dp),
        ) {
            Dropdown(state.span, com.microdmm.monitor120v.SPANS.keys.toList(),
                onPick = { k -> onAction(Action.SetSpan(com.microdmm.monitor120v.SPANS.keys.toList()[k])) })
            TextButton(onClick = { onAction(Action.ClearTrend) }) { Text("Clear") }
            TextButton(onClick = { onAction(Action.ResetWh) }, enabled = state.conn == Conn.Connected) { Text("Reset Wh") }
        }
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            if (state.logName == null) {
                OutlinedButton(onClick = { logLauncher("monitor_${com.microdmm.monitor120v.nowStamp("yyyyMMdd_HHmmss")}.csv") }) {
                    Text("Start CSV log…")
                }
            } else {
                Button(onClick = { onAction(Action.StopLog) }) { Text("Stop CSV log") }
                Text(state.logName, color = MOnSurfaceDim, style = MaterialTheme.typography.labelSmall)
            }
        }
        Spacer(Modifier.height(8.dp))
        TrendChart(
            state.trend, Triple("vrms", "Vrms", VColor), Triple("irms", "Irms (A)", IColor),
            Modifier.fillMaxWidth().height(200.dp),
        )
        Spacer(Modifier.height(8.dp))
        TrendChart(
            state.trend, Triple("p", "W", WColor), Triple("pf", "PF", PFColor),
            Modifier.fillMaxWidth().height(200.dp), rightFixed = -1.05 to 1.05,
        )
    }
}

// ── Console ─────────────────────────────────────────────────────────────────
@Composable
private fun ConsoleTab(state: UiState, onAction: (Action) -> Unit) {
    var cmd by rememberSaveable { mutableStateOf("") }
    val list = rememberLazyListState()
    LaunchedEffect(state.console.size) {
        if (state.console.isNotEmpty()) list.scrollToItem(state.console.size - 1)
    }
    Column(Modifier.fillMaxSize().padding(6.dp)) {
        LazyColumn(Modifier.weight(1f).fillMaxWidth(), state = list) {
            itemsIndexed(state.console) { _, line ->
                Text(
                    line, fontFamily = FontFamily.Monospace, fontSize = 11.sp,
                    color = when {
                        line.startsWith(">") -> MCyan
                        line.startsWith("\$ERR") || line.startsWith("!!") -> MRed
                        line.startsWith("#") -> MAmber
                        else -> MOnSurface
                    },
                )
            }
        }
        Row(verticalAlignment = Alignment.CenterVertically) {
            OutlinedTextField(
                value = cmd, onValueChange = { cmd = it }, modifier = Modifier.weight(1f), singleLine = true,
                placeholder = { Text("!STATUS") },
                keyboardOptions = KeyboardOptions(imeAction = ImeAction.Send),
                keyboardActions = KeyboardActions(onSend = { onAction(Action.Console(cmd)); cmd = "" }),
            )
            Spacer(Modifier.width(6.dp))
            Button(onClick = { onAction(Action.Console(cmd)); cmd = "" }, enabled = state.conn == Conn.Connected) { Text("Send") }
        }
    }
}
