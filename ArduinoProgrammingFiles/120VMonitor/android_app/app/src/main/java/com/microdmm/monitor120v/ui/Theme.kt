package com.microdmm.monitor120v.ui

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

// Dark instrument-panel palette (shared with the other Micro-DMM apps)
val MBackground     = Color(0xFF1A1A1A)
val MSurface        = Color(0xFF222222)
val MSurfaceVariant = Color(0xFF2A2A30)
val MOnSurface      = Color(0xFFDCDCDC)
val MOnSurfaceDim   = Color(0xFF8C8C96)
val MBorder         = Color(0xFF555555)

val MGreen  = Color(0xFF00FF32)
val MCyan   = Color(0xFF00C8FF)
val MYellow = Color(0xFFFFDC00)
val MAmber  = Color(0xFFFFB000)
val MPurple = Color(0xFFC864FF)
val MRed    = Color(0xFFDD4444)
val MBlue   = Color(0xFF4488FF)
val MGray   = Color(0xFF444444)

// plot series, matching the desktop GUI: V blue, I red, W green, PF purple
val VColor  = Color(0xFF4FA3FF)
val IColor  = Color(0xFFFF5A4F)
val WColor  = Color(0xFF3DDC84)
val PFColor = MPurple

private val Scheme = darkColorScheme(
    primary          = MGreen,
    secondary        = MCyan,
    tertiary         = MAmber,
    background       = MBackground,
    surface          = MSurface,
    surfaceVariant   = MSurfaceVariant,
    onBackground     = MOnSurface,
    onSurface        = MOnSurface,
    onSurfaceVariant = MOnSurfaceDim,
    outline          = MBorder,
)

@Composable
fun MonitorTheme(content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = Scheme, content = content)
}
