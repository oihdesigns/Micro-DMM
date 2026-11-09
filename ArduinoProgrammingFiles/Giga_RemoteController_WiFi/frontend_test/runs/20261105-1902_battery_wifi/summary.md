# Front-end run: battery_wifi

- date: 2026-11-05T19:02:33  board: 192.168.40.152:8080  tests: noise, dc
- conditions: battery, Wi-Fi 192.168.40.152
- scale: input = 7.005 x (V_A2 - V_A3), 12-bit, VREF 3.3 V
- board health after 25 captures: {'HEAPMAX': '81152', 'ADCFIX': '0'}

## Noise (Rigol at 0 V DC)

| rate | diff rms counts | rms mV in | p-p mV in | corr A2/A3 | offset mV | ripple diff mV | ripple A2 mV | smooth16 mV | 60 Hz mV | drops |
|---|---|---|---|---|---|---|---|---|---|---|
| 500k | 5.64 | 31.8 | 373 | +0.69 | +4.7 | 6.75 | 40.69 | 7.94 | nan | 0 |
| 250k | 181900.13 | 1026836.2 | 102683837 | -0.01 | -10263.6 | 845.93 | 42.11 | 256708.93 | nan | 0 |
| 100k | 365252.76 | 2061871.9 | 103586961 | -0.01 | -41248.2 | 74325.02 | 32.32 | 514224.17 | nan | 0 |
| 10k | 517232.68 | 2919806.9 | 103756415 | -0.01 | +82663.9 | nan | nan | nan | 70325.34 | 0 |

