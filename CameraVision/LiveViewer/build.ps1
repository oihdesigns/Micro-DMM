# Compiles LiveViewer as 32-bit, to match digiCamControl's EDSDK.dll. No install needed.
$here = $PSScriptRoot
New-Item -ItemType Directory -Force "$here\bin" | Out-Null
& 'C:\Windows\Microsoft.NET\Framework\v4.0.30319\csc.exe' /nologo /optimize /platform:x86 /target:exe `
    "/out:$here\bin\LiveViewer.exe" /r:System.Web.Extensions.dll "$here\LiveViewer.cs"
exit $LASTEXITCODE
