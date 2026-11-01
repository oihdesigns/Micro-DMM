# LiveViewer

Shared live view for the Canon EOS-1D X Mark II. One process holds the camera over
USB and serves it on this PC only, so a person in a browser and a script can use it
at the same time. digiCamControl must be installed (for `EDSDK.dll`) but not running.

Double-click `Start LiveViewer.bat`. It builds the viewer if needed, starts it and
opens the page. To run it by hand:

    powershell -ExecutionPolicy Bypass -File build.ps1
    bin\LiveViewer.exe [--port 8765] [--output 3]

The page is http://localhost:8765/. Click the image to move the AF point and focus.
Use "Close viewer" or Ctrl+C to release the camera. Only one program can hold it.

`--output` is the live view destination: 3 = PC and rear screen, 2 = PC only.
Photos taken through the viewer go to `../captures/`, not the memory cards.

## HTTP interface

Read with GET:

| Path | Returns |
|---|---|
| `/frame.jpg` | Latest live view frame |
| `/status` | Settings, valid options, FPS, AF box, recent activity |
| `/stream` | MJPEG feed |
| `/captures/<file>` | A saved photo |

Commands use POST. Add `who=claude` (or any name) to label them in the activity log.

| Path | Action |
|---|---|
| `/api/af` | Autofocus at the current AF point |
| `/api/afpoint?x=0.5&y=0.5&af=1` | Centre the AF box (fractions of the frame), optionally focus |
| `/api/capture?af=1` | Take a photo, optionally focusing first; returns the file name |
| `/api/set?key=iso&value=800` | Set `mode`, `shutter`, `aperture`, `iso`, `ec`, `wb`, `metering`, `afmode` or `dof` (`Open` or `Stopped down`, the live view aperture) |
| `/api/zoom?value=5` | Live view zoom: 1, 5 or 10 |
| `/api/liveview?value=off` | Turn live view `on` or `off`. Off saves battery. Photos still work, but AF, AF point, zoom and `dof` are refused |
| `/api/quit` | Close the viewer and release the camera |

## Notes

- "OK" from the camera does not prove anything happened. Check the frames.
- Lens drive is left out because it did nothing on this body, through any route tested.
- Depth-of-field preview (`dof`, EDSDK property 0x504) did nothing in one setup and worked in
  the next. Between those runs the camera's DOF preview button went back to its default
  function and the lens changed. If "Stopped down" looks the same as "Open", check that button.
  Taking a photo returns live view to "Open".
- Live view brightness alone says nothing about the real lens opening. Judge by the blur.
- 5x zoom gives 1104x736 frames of real sensor pixels, useful for reading small text.
- The AF point cannot be moved while zoomed in.
