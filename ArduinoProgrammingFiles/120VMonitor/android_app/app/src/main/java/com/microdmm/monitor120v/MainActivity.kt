package com.microdmm.monitor120v

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Bundle
import android.view.WindowManager
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.viewModels
import android.util.Log
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import java.io.File
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.microdmm.monitor120v.ui.MonitorScreen
import com.microdmm.monitor120v.ui.MonitorTheme

class MainActivity : ComponentActivity() {

    private val viewModel: MonitorViewModel by viewModels()
    private val usbPermissionAction = "com.microdmm.monitor120v.USB_PERMISSION"
    private var pendingDevice: UsbDevice? = null
    private val crashFile by lazy { File(filesDir, "last_crash.txt") }
    private var lastCrash by mutableStateOf<String?>(null)

    /** Keep the stack trace of a crash, so the next launch can show and share it. */
    private fun recordCrashes() {
        val previous = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, e ->
            try {
                crashFile.writeText(
                    "120V Monitor ${packageManager.getPackageInfo(packageName, 0).versionName}, " +
                        "Android ${Build.VERSION.RELEASE}, ${Build.MANUFACTURER} ${Build.MODEL}\n" +
                        "${nowStamp()}  thread ${thread.name}\n\n${Log.getStackTraceString(e)}",
                )
            } catch (_: Exception) {
            }
            previous?.uncaughtException(thread, e)
        }
        if (crashFile.exists()) lastCrash = crashFile.readText()
    }

    private fun shareCrash(text: String) {
        val send = Intent(Intent.ACTION_SEND).setType("text/plain")
            .putExtra(Intent.EXTRA_SUBJECT, "120V Monitor crash")
            .putExtra(Intent.EXTRA_TEXT, text)
        startActivity(Intent.createChooser(send, "Share crash report"))
    }

    private val usbReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            when (intent.action) {
                usbPermissionAction -> {
                    val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                    if (granted) pendingDevice?.let { viewModel.onAction(Action.Connect(it)) }
                    pendingDevice = null
                }
                UsbManager.ACTION_USB_DEVICE_ATTACHED -> connectToDevice(quiet = true)
                UsbManager.ACTION_USB_DEVICE_DETACHED -> viewModel.onAction(Action.Disconnect)
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        recordCrashes()

        val usbManager = getSystemService(USB_SERVICE) as UsbManager
        viewModel.initSerial(usbManager)

        val filter = IntentFilter().apply {
            addAction(usbPermissionAction)
            addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
            addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(usbReceiver, filter, RECEIVER_NOT_EXPORTED)
        } else {
            @Suppress("UnspecifiedRegisterReceiverFlag")
            registerReceiver(usbReceiver, filter)
        }

        setContent {
            MonitorTheme {
                val state by viewModel.ui.collectAsStateWithLifecycle()

                if (state.conn == Conn.Connected) {
                    window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                } else {
                    window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                }

                MonitorScreen(
                    state = state,
                    onAction = viewModel::onAction,
                    onConnectClick = { connectToDevice(quiet = false) },
                )
                lastCrash?.let { text ->
                    val done = { crashFile.delete(); lastCrash = null }
                    AlertDialog(
                        onDismissRequest = { done() },
                        title = { Text("The app crashed last time") },
                        text = {
                            Text(
                                text, fontFamily = FontFamily.Monospace, fontSize = 10.sp,
                                modifier = Modifier.heightIn(max = 420.dp).verticalScroll(rememberScrollState()),
                            )
                        },
                        confirmButton = { TextButton(onClick = { shareCrash(text); done() }) { Text("Share") } },
                        dismissButton = { TextButton(onClick = { done() }) { Text("Dismiss") } },
                    )
                }
            }
        }

        if (intent?.action == UsbManager.ACTION_USB_DEVICE_ATTACHED) connectToDevice(quiet = true)
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        if (intent.action == UsbManager.ACTION_USB_DEVICE_ATTACHED) connectToDevice(quiet = true)
    }

    private fun connectToDevice(quiet: Boolean) {
        val device = viewModel.findDevice() ?: run {
            if (!quiet) Toast.makeText(this, "No USB serial device found — plug the monitor in (OTG)", Toast.LENGTH_LONG).show()
            return
        }
        val usbManager = getSystemService(USB_SERVICE) as UsbManager
        if (usbManager.hasPermission(device)) {
            viewModel.onAction(Action.Connect(device))
        } else {
            pendingDevice = device
            val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) PendingIntent.FLAG_MUTABLE else 0
            val pi = PendingIntent.getBroadcast(this, 0, Intent(usbPermissionAction).setPackage(packageName), flags)
            usbManager.requestPermission(device, pi)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        try { unregisterReceiver(usbReceiver) } catch (_: Exception) { }
    }
}
