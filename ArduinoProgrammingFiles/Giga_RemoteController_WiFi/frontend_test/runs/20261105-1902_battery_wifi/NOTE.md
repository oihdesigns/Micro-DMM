# Damaged run — don't use except the 500k noise row

Captured on battery over Wi-Fi with the firmware before tcp_stream.h. That firmware dropped whole print()
calls on the socket: lost commas glued samples together, lost numbers shifted A3 against A2. Only the
500 kS/s noise capture arrived intact (31.8 mV rms, corr +0.69, ripple 40.7 mV/pin, 6.8 mV diff). Every
other number here is garbage. The DC test crashed on a mangled DATA header.
