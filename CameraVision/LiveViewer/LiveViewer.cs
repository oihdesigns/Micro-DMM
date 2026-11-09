// LiveViewer: owns the Canon body through EDSDK and shares it over localhost.
// One thread talks to the camera; the browser page and scripts queue work onto it.
// Built by build.ps1 as 32-bit, C# 5, because digiCamControl's EDSDK.dll is 32-bit.
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Web.Script.Serialization;

[StructLayout(LayoutKind.Sequential)]
struct EdsPropertyDesc
{
    public int Form;
    public int Access;
    public int NumElements;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)]
    public int[] PropDesc;
}

[StructLayout(LayoutKind.Sequential)]
struct EdsCapacity
{
    public int NumberOfFreeClusters;
    public int BytesPerSector;
    public int Reset;
}

[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
struct EdsDirectoryItemInfo
{
    public ulong Size;
    public int IsFolder;
    public uint GroupId;
    public uint Option;
    [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
    public string FileName;
    public uint Format;
    public uint DateTime;
}

delegate uint ObjectHandler(uint evt, IntPtr item, IntPtr context);
delegate uint StateHandler(uint evt, uint param, IntPtr context);

static class Eds
{
    const string D = "EDSDK.dll";
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode)] public static extern bool SetDllDirectory(string p);
    [DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint flags);
    [DllImport(D)] public static extern uint EdsInitializeSDK();
    [DllImport(D)] public static extern uint EdsTerminateSDK();
    [DllImport(D)] public static extern uint EdsGetCameraList(out IntPtr l);
    [DllImport(D)] public static extern uint EdsGetChildCount(IntPtr l, out int n);
    [DllImport(D)] public static extern uint EdsGetChildAtIndex(IntPtr l, int i, out IntPtr c);
    [DllImport(D)] public static extern uint EdsOpenSession(IntPtr c);
    [DllImport(D)] public static extern uint EdsCloseSession(IntPtr c);
    [DllImport(D)] public static extern uint EdsRelease(IntPtr o);
    [DllImport(D)] public static extern uint EdsGetEvent();
    [DllImport(D)] public static extern uint EdsSendCommand(IntPtr c, uint cmd, int p);
    [DllImport(D)] public static extern uint EdsGetPropertyData(IntPtr c, uint id, int p, int s, byte[] b);
    [DllImport(D)] public static extern uint EdsSetPropertyData(IntPtr c, uint id, int p, int s, byte[] b);
    [DllImport(D)] public static extern uint EdsGetPropertyDesc(IntPtr c, uint id, out EdsPropertyDesc d);
    [DllImport(D)] public static extern uint EdsSetCapacity(IntPtr c, EdsCapacity cap);
    [DllImport(D)] public static extern uint EdsCreateMemoryStream(ulong size, out IntPtr st);
    [DllImport(D)] public static extern uint EdsCreateEvfImageRef(IntPtr st, out IntPtr ev);
    [DllImport(D)] public static extern uint EdsDownloadEvfImage(IntPtr c, IntPtr ev);
    [DllImport(D)] public static extern uint EdsGetPointer(IntPtr st, out IntPtr p);
    [DllImport(D)] public static extern uint EdsGetLength(IntPtr st, out ulong n);
    [DllImport(D)] public static extern uint EdsGetDirectoryItemInfo(IntPtr item, out EdsDirectoryItemInfo info);
    [DllImport(D)] public static extern uint EdsDownload(IntPtr item, ulong size, IntPtr st);
    [DllImport(D)] public static extern uint EdsDownloadComplete(IntPtr item);
    [DllImport(D)] public static extern uint EdsSetObjectEventHandler(IntPtr c, uint evt, ObjectHandler h, IntPtr ctx);
    [DllImport(D)] public static extern uint EdsSetCameraStateEventHandler(IntPtr c, uint evt, StateHandler h, IntPtr ctx);

    public const uint PropBattery = 0x8, PropSaveTo = 0xB, PropWb = 0x106, PropAe = 0x400, PropIso = 0x402;
    public const uint PropMetering = 0x403, PropAfMode = 0x404, PropAv = 0x405, PropTv = 0x406, PropEc = 0x407;
    public const uint PropAeSelect = 0x436, PropEvfOutput = 0x500, PropEvfMode = 0x501, PropEvfDof = 0x504;
    public const uint PropEvfZoom = 0x507, PropEvfZoomPos = 0x508, PropEvfCoord = 0x540, PropEvfZoomRect = 0x541;
    public const uint CmdKeepAlive = 1, CmdShutter = 4, CmdEvfAf = 0x102;
    public const int ShutterOff = 0, ShutterNonAf = 0x10003;
    public const uint ObjectAll = 0x200, ObjectTransfer = 0x208, StateAll = 0x300, StateShutdown = 0x301, StateWillShutdown = 0x303;
    public const uint ErrBusy = 0x81, ErrNotReady = 0xA102;
    public const uint EsContinuous = 0x80000000, EsSystemRequired = 1;

    public static string Hex(uint e) { return e == 0 ? "OK" : "0x" + e.ToString("X"); }
}

class Prop
{
    public string Key;
    public uint Id;
    public bool Writable;
    public Dictionary<uint, string> Labels;

    public Prop(string key, uint id, bool writable, string table)
    {
        Key = key;
        Id = id;
        Writable = writable;
        Labels = new Dictionary<uint, string>();
        foreach (var pair in table.Split(','))
        {
            var kv = pair.Split('=');
            Labels[Convert.ToUInt32(kv[0], 16)] = kv[1];
        }
    }

    public string Label(uint code)
    {
        string s;
        return Labels.TryGetValue(code, out s) ? s : "0x" + code.ToString("X");
    }

    // Accepts a label ("1/100") or a raw code ("0x6D").
    public uint Code(string value)
    {
        foreach (var kv in Labels)
            if (kv.Value == value) return kv.Key;
        if (value.StartsWith("0x")) return Convert.ToUInt32(value.Substring(2), 16);
        throw new ArgumentException("unknown value for " + Key + ": " + value);
    }
}

static class Tables
{
    public static readonly Prop[] Props =
    {
        new Prop("mode", Eds.PropAeSelect, true, "0=P,1=Tv,2=Av,3=M,4=Bulb,5=A-DEP"),
        new Prop("iso", Eds.PropIso, true, "0=Auto,48=100,4B=125,4D=160,50=200,53=250,55=320,58=400,5B=500,5D=640,60=800,63=1000,65=1250,68=1600,6B=2000,6D=2500,70=3200,73=4000,75=5000,78=6400,7B=8000,7D=10000,80=12800,83=16000,85=20000,88=25600,8B=32000,8D=40000,90=51200,98=102400,A0=204800,A8=409600"),
        new Prop("aperture", Eds.PropAv, true, "8=1.0,B=1.1,D=1.2,C=1.2,10=1.4,13=1.6,14=1.8,15=1.8,18=2.0,1B=2.2,1C=2.5,1D=2.5,20=2.8,23=3.2,24=3.5,25=3.5,28=4.0,2B=4.5,2C=4.5,2D=5.0,30=5.6,33=6.3,34=6.7,35=7.1,38=8.0,3B=9.0,3C=9.5,3D=10,40=11,43=13,44=13,45=14,48=16,4B=18,4C=19,4D=20,50=22"),
        new Prop("shutter", Eds.PropTv, true, "0=Auto,C=Bulb,10=30,13=25,14=20,15=20,18=15,1B=13,1C=10,1D=10,20=8,23=6,24=6,25=5,28=4,2B=3.2,2C=3,2D=2.5,30=2,33=1.6,34=1.5,35=1.3,38=1,3B=0.8,3C=0.7,3D=0.6,40=0.5,43=0.4,44=0.3,45=0.3,48=1/4,4B=1/5,4C=1/6,4D=1/6,50=1/8,53=1/10,54=1/10,55=1/13,58=1/15,5B=1/20,5C=1/20,5D=1/25,60=1/30,63=1/40,64=1/45,65=1/50,68=1/60,6B=1/80,6C=1/90,6D=1/100,70=1/125,73=1/160,74=1/180,75=1/200,78=1/250,7B=1/320,7C=1/350,7D=1/400,80=1/500,83=1/640,84=1/750,85=1/800,88=1/1000,8B=1/1250,8C=1/1500,8D=1/1600,90=1/2000,93=1/2500,94=1/3000,95=1/3200,98=1/4000,9B=1/5000,9C=1/6000,9D=1/6400,A0=1/8000"),
        new Prop("ec", Eds.PropEc, true, "18=+3,15=+2 2/3,14=+2 1/2,13=+2 1/3,10=+2,D=+1 2/3,C=+1 1/2,B=+1 1/3,8=+1,5=+2/3,4=+1/2,3=+1/3,0=0,FD=-1/3,FC=-1/2,FB=-2/3,F8=-1,F5=-1 1/3,F4=-1 1/2,F3=-1 2/3,F0=-2,ED=-2 1/3,EC=-2 1/2,EB=-2 2/3,E8=-3"),
        new Prop("wb", Eds.PropWb, true, "0=Auto,1=Daylight,2=Cloudy,3=Tungsten,4=Fluorescent,5=Flash,6=Manual,8=Shade,9=Color temp"),
        new Prop("metering", Eds.PropMetering, true, "1=Spot,3=Evaluative,4=Partial,5=Center-weighted"),
        new Prop("afmode", Eds.PropAfMode, true, "0=One-Shot AF,1=AI Servo AF,2=AI Focus AF,3=Manual"),
        new Prop("dof", Eds.PropEvfDof, true, "0=Open,1=Stopped down"),
    };

    public static Prop Find(string key)
    {
        var p = Props.FirstOrDefault(x => x.Key == key);
        if (p == null) throw new ArgumentException("unknown setting: " + key);
        return p;
    }
}

class Job
{
    public Func<object> Work;
    public object Result;
    public Exception Error;
    public ManualResetEventSlim Done = new ManualResetEventSlim(false);
}

class Camera
{
    readonly string captureDir;
    readonly int outputDevice;
    readonly ConcurrentQueue<Job> jobs = new ConcurrentQueue<Job>();
    readonly object frameLock = new object();
    readonly object logLock = new object();
    readonly List<Dictionary<string, object>> log = new List<Dictionary<string, object>>();

    // Held in fields so the GC cannot collect delegates that native code still calls.
    readonly ObjectHandler onObject;
    readonly StateHandler onState;

    IntPtr cam = IntPtr.Zero, list = IntPtr.Zero;
    uint savedSaveTo;
    bool haveSaveTo, sessionLost, liveViewOn, pcOnly;
    string liveViewResult = "";
    byte[] frame;
    long frameSeq;
    int fpsCount;
    double fps;
    string lastCapture = "", lastCaptureAt = "", captureName;
    long captureSeq;
    int coordW = 5472, coordH = 3648, afX, afY, afW = 1096, afH = 730, zoom = 1, frameErrors;
    volatile bool stopping, liveViewWanted = true;
    volatile Dictionary<string, object> status = new Dictionary<string, object> { { "connected", false } };
    Dictionary<string, object> options = new Dictionary<string, object>();

    public Camera(string captureDir, int outputDevice)
    {
        this.captureDir = captureDir;
        this.outputDevice = outputDevice;
        onObject = HandleObject;
        onState = HandleState;
        FindLastCapture();
    }

    // So the page has a photo to show before the first one of this run.
    void FindLastCapture()
    {
        if (!Directory.Exists(captureDir)) return;
        var newest = new DirectoryInfo(captureDir).GetFiles()
            .Where(f => f.Name.StartsWith("capture_") || f.Name.StartsWith("interval_"))
            .OrderBy(f => f.LastWriteTime).LastOrDefault();
        if (newest == null) return;
        lastCapture = newest.Name;
        lastCaptureAt = newest.LastWriteTime.ToString("d MMM HH:mm:ss");
    }

    public void Log(string who, string message)
    {
        var line = new Dictionary<string, object> { { "t", DateTime.Now.ToString("HH:mm:ss") }, { "who", who }, { "msg", message } };
        lock (logLock)
        {
            log.Add(line);
            if (log.Count > 200) log.RemoveAt(0);
        }
        Console.WriteLine(line["t"] + " [" + who + "] " + message);
    }

    public object Run(Func<object> work, int timeoutMs)
    {
        var job = new Job { Work = work };
        jobs.Enqueue(job);
        if (!job.Done.Wait(timeoutMs)) throw new TimeoutException("camera did not answer in time");
        if (job.Error != null) throw job.Error;
        return job.Result;
    }

    public void Stop() { stopping = true; }

    public Dictionary<string, object> Status()
    {
        var s = new Dictionary<string, object>(status);
        s["fps"] = Math.Round(fps, 1);
        s["frameSeq"] = Interlocked.Read(ref frameSeq);
        s["lastCapture"] = lastCapture;
        s["lastCaptureAt"] = lastCaptureAt;
        s["liveViewWanted"] = liveViewWanted;
        s["options"] = options;
        lock (logLock) s["log"] = log.Skip(Math.Max(0, log.Count - 40)).ToList();
        return s;
    }

    public byte[] LatestFrame(out long seq)
    {
        lock (frameLock)
        {
            seq = frameSeq;
            return frame;
        }
    }

    public byte[] WaitFrame(ref long seq, int timeoutMs)
    {
        lock (frameLock)
        {
            if (frameSeq == seq) Monitor.Wait(frameLock, timeoutMs);
            seq = frameSeq;
            return frame;
        }
    }

    public void ThreadMain()
    {
        Eds.SetDllDirectory(@"C:\Program Files (x86)\digiCamControl");
        uint init = Eds.EdsInitializeSDK();
        Log("viewer", "SDK init " + Eds.Hex(init));
        while (!stopping)
        {
            if (!Open())
            {
                status = new Dictionary<string, object> { { "connected", false } };
                FailJobs("camera not connected");
                Thread.Sleep(2000);
                continue;
            }
            try { Session(); }
            catch (Exception ex) { Log("viewer", "session ended: " + ex.Message); }
            finally { Close(); }
        }
        Eds.EdsTerminateSDK();
    }

    void FailJobs(string message)
    {
        Job job;
        while (jobs.TryDequeue(out job))
        {
            job.Error = new InvalidOperationException(message);
            job.Done.Set();
        }
    }

    bool Open()
    {
        int count;
        if (Eds.EdsGetCameraList(out list) != 0) return false;
        Eds.EdsGetChildCount(list, out count);
        if (count < 1)
        {
            Eds.EdsRelease(list);
            list = IntPtr.Zero;
            return false;
        }
        Eds.EdsGetChildAtIndex(list, 0, out cam);
        Eds.EdsSetObjectEventHandler(cam, Eds.ObjectAll, onObject, IntPtr.Zero);
        Eds.EdsSetCameraStateEventHandler(cam, Eds.StateAll, onState, IntPtr.Zero);
        uint e = Eds.EdsOpenSession(cam);
        Log("viewer", "open session " + Eds.Hex(e));
        if (e != 0)
        {
            Eds.EdsRelease(cam);
            Eds.EdsRelease(list);
            cam = list = IntPtr.Zero;
            return false;
        }
        sessionLost = false;
        return true;
    }

    void Close()
    {
        StopLiveView();
        if (haveSaveTo) SetU(Eds.PropSaveTo, savedSaveTo);
        haveSaveTo = false;
        Eds.EdsCloseSession(cam);
        Eds.EdsRelease(cam);
        Eds.EdsRelease(list);
        cam = list = IntPtr.Zero;
        Log("viewer", "session closed");
    }

    void Session()
    {
        uint value;
        if (GetU(Eds.PropSaveTo, out value) == 0)
        {
            savedSaveTo = value;
            haveSaveTo = true;
        }
        // Photos go to the PC only, so tests do not fill the cards.
        SetU(Eds.PropSaveTo, 2);
        Eds.EdsSetCapacity(cam, new EdsCapacity { NumberOfFreeClusters = 0x7FFFFFFF, BytesPerSector = 0x1000, Reset = 1 });
        liveViewResult = "";
        RefreshOptions();

        DateTime nextStatus = DateTime.MinValue, nextOptions = DateTime.Now.AddSeconds(5), nextLiveView = DateTime.MinValue;
        DateTime nextKeepAlive = DateTime.Now.AddSeconds(30), fpsStart = DateTime.Now;
        while (!stopping && !sessionLost)
        {
            Eds.EdsGetEvent();
            Job job;
            while (jobs.TryDequeue(out job))
            {
                try { job.Result = job.Work(); }
                catch (Exception ex) { job.Error = ex; }
                job.Done.Set();
            }
            if (liveViewOn) GrabFrame();

            var now = DateTime.Now;
            if (liveViewWanted && !liveViewOn && now >= nextLiveView)
            {
                StartLiveView();
                nextLiveView = now.AddSeconds(1);
            }
            if (now >= nextStatus)
            {
                RefreshStatus();
                nextStatus = now.AddMilliseconds(700);
            }
            if (now >= nextOptions)
            {
                RefreshOptions();
                nextOptions = now.AddSeconds(5);
            }
            if (now >= nextKeepAlive)
            {
                Eds.EdsSendCommand(cam, Eds.CmdKeepAlive, 0);
                nextKeepAlive = now.AddSeconds(30);
            }
            double span = (now - fpsStart).TotalSeconds;
            if (span >= 1)
            {
                fps = fpsCount / span;
                fpsCount = 0;
                fpsStart = now;
            }
            Thread.Sleep(8);
        }
    }

    // Called again every second until it works: a camera that has just woken up answers busy.
    void StartLiveView()
    {
        uint a = SetU(Eds.PropEvfMode, 1);
        uint want = pcOnly ? 2 : (uint)outputDevice;
        uint b = SetU(Eds.PropEvfOutput, want);
        if (b != 0 && b != Eds.ErrBusy && want != 2)
        {
            Log("viewer", "live view output " + want + " refused (" + Eds.Hex(b) + "), using PC only");
            pcOnly = true;
            b = SetU(Eds.PropEvfOutput, 2);
        }
        liveViewOn = a == 0 && b == 0;
        string result = "live view: mode " + Eds.Hex(a) + ", output " + Eds.Hex(b);
        if (result != liveViewResult) Log("viewer", result);
        liveViewResult = result;
    }

    void StopLiveView()
    {
        if (cam == IntPtr.Zero) return;
        liveViewOn = false;
        SetU(Eds.PropEvfOutput, 0);
        Pump(500);
        SetU(Eds.PropEvfMode, 0);
        Pump(300);
    }

    void Pump(int ms)
    {
        var end = DateTime.Now.AddMilliseconds(ms);
        while (DateTime.Now < end)
        {
            Eds.EdsGetEvent();
            Thread.Sleep(15);
        }
    }

    // Like Pump, but the feed keeps moving so focus can be watched while it happens.
    void PumpWithFrames(int ms)
    {
        var end = DateTime.Now.AddMilliseconds(ms);
        while (DateTime.Now < end)
        {
            Eds.EdsGetEvent();
            if (liveViewOn) GrabFrame();
            Thread.Sleep(8);
        }
    }

    uint SetU(uint id, uint v)
    {
        return Eds.EdsSetPropertyData(cam, id, 0, 4, BitConverter.GetBytes(v));
    }

    uint GetU(uint id, out uint v)
    {
        var b = new byte[4];
        uint e = Eds.EdsGetPropertyData(cam, id, 0, 4, b);
        v = BitConverter.ToUInt32(b, 0);
        return e;
    }

    uint SetRetry(uint id, uint v)
    {
        uint e = 0;
        for (int i = 0; i < 12; i++)
        {
            e = SetU(id, v);
            if (e != Eds.ErrBusy) break;
            Pump(100);
        }
        return e;
    }

    uint SendRetry(uint cmd, int param)
    {
        uint e = 0;
        for (int i = 0; i < 20; i++)
        {
            e = Eds.EdsSendCommand(cam, cmd, param);
            if (e != Eds.ErrBusy) break;
            Pump(100);
        }
        return e;
    }

    void GrabFrame()
    {
        IntPtr stream, image;
        Eds.EdsCreateMemoryStream(0, out stream);
        Eds.EdsCreateEvfImageRef(stream, out image);
        uint e = Eds.EdsDownloadEvfImage(cam, image);
        if (e == 0)
        {
            IntPtr pointer;
            ulong length;
            Eds.EdsGetPointer(stream, out pointer);
            Eds.EdsGetLength(stream, out length);
            var bytes = new byte[(int)length];
            Marshal.Copy(pointer, bytes, 0, bytes.Length);

            var size = new byte[8];
            var rect = new byte[16];
            var position = new byte[8];
            if (Eds.EdsGetPropertyData(image, Eds.PropEvfCoord, 0, 8, size) == 0)
            {
                coordW = BitConverter.ToInt32(size, 0);
                coordH = BitConverter.ToInt32(size, 4);
            }
            if (Eds.EdsGetPropertyData(image, Eds.PropEvfZoomRect, 0, 16, rect) == 0)
            {
                afX = BitConverter.ToInt32(rect, 0);
                afY = BitConverter.ToInt32(rect, 4);
                afW = BitConverter.ToInt32(rect, 8);
                afH = BitConverter.ToInt32(rect, 12);
            }
            else if (Eds.EdsGetPropertyData(image, Eds.PropEvfZoomPos, 0, 8, position) == 0)
            {
                afX = BitConverter.ToInt32(position, 0);
                afY = BitConverter.ToInt32(position, 4);
            }
            lock (frameLock)
            {
                frame = bytes;
                frameSeq++;
                Monitor.PulseAll(frameLock);
            }
            fpsCount++;
            frameErrors = 0;
        }
        else if (e != Eds.ErrNotReady && e != Eds.ErrBusy && ++frameErrors == 40)
        {
            Log("viewer", "live view frames failing: " + Eds.Hex(e));
        }
        Eds.EdsRelease(image);
        Eds.EdsRelease(stream);
    }

    void RefreshStatus()
    {
        uint output;
        if (liveViewOn && GetU(Eds.PropEvfOutput, out output) == 0 && (output & 2) == 0)
        {
            liveViewOn = false;
            liveViewResult = "";
            Log("viewer", "camera stopped live view, restarting it");
        }
        var s = new Dictionary<string, object> { { "connected", true }, { "liveView", liveViewOn } };
        foreach (var p in Tables.Props)
        {
            uint v;
            if (GetU(p.Id, out v) == 0) s[p.Key] = p.Label(v);
        }
        uint value;
        if (GetU(Eds.PropBattery, out value) == 0) s["battery"] = (int)value;
        if (GetU(Eds.PropEvfZoom, out value) == 0)
        {
            zoom = (int)value;
            s["zoom"] = zoom;
        }
        s["sensor"] = new[] { coordW, coordH };
        s["afBox"] = new[] { afX, afY, afW, afH };
        status = s;
    }

    void RefreshOptions()
    {
        var o = new Dictionary<string, object>();
        foreach (var p in Tables.Props)
        {
            EdsPropertyDesc d;
            if (Eds.EdsGetPropertyDesc(cam, p.Id, out d) != 0 || d.PropDesc == null) continue;
            var labels = d.PropDesc.Take(Math.Min(d.NumElements, 128)).Select(v => p.Label((uint)v)).Distinct().ToList();
            if (labels.Count > 0) o[p.Key] = labels;
        }
        options = o;
    }

    uint HandleState(uint evt, uint param, IntPtr context)
    {
        if (evt == Eds.StateShutdown)
        {
            sessionLost = true;
            Log("viewer", "camera shut down or was disconnected");
        }
        else if (evt == Eds.StateWillShutdown)
        {
            Log("viewer", "camera about to sleep, keeping it awake");
            Eds.EdsSendCommand(cam, Eds.CmdKeepAlive, 0);
        }
        return 0;
    }

    uint HandleObject(uint evt, IntPtr item, IntPtr context)
    {
        if (evt == Eds.ObjectTransfer) SaveCapture(item);
        if (item != IntPtr.Zero) Eds.EdsRelease(item);
        return 0;
    }

    void SaveCapture(IntPtr item)
    {
        EdsDirectoryItemInfo info;
        uint e = Eds.EdsGetDirectoryItemInfo(item, out info);
        if (e != 0)
        {
            Log("viewer", "capture info failed " + Eds.Hex(e));
            return;
        }
        IntPtr stream;
        Eds.EdsCreateMemoryStream(info.Size, out stream);
        e = Eds.EdsDownload(item, info.Size, stream);
        if (e == 0) Eds.EdsDownloadComplete(item);
        if (e == 0)
        {
            IntPtr pointer;
            Eds.EdsGetPointer(stream, out pointer);
            var bytes = new byte[(int)info.Size];
            Marshal.Copy(pointer, bytes, 0, bytes.Length);
            string ext = Path.GetExtension(info.FileName);
            string name = (captureName ?? "capture_" + DateTime.Now.ToString("yyyyMMdd_HHmmss_fff")) + ext;
            Directory.CreateDirectory(captureDir);
            File.WriteAllBytes(Path.Combine(captureDir, name), bytes);
            lastCapture = name;
            lastCaptureAt = DateTime.Now.ToString("d MMM HH:mm:ss");
            Interlocked.Increment(ref captureSeq);
            Log("viewer", "saved " + name + " (" + bytes.Length / 1024 + " KB)");
        }
        else
        {
            Log("viewer", "capture download failed " + Eds.Hex(e));
        }
        Eds.EdsRelease(stream);
    }

    // These commands report OK with live view off, so they have to be refused here.
    void RequireLiveView()
    {
        if (!liveViewOn) throw new InvalidOperationException("turn live view on first");
    }

    public object SetLiveView(bool on)
    {
        liveViewWanted = on;
        if (on)
        {
            liveViewResult = "";
            if (!liveViewOn) StartLiveView();
        }
        else StopLiveView();
        return new Dictionary<string, object> { { "liveView", liveViewOn }, { "wanted", on } };
    }

    // With no camera there is nothing to run, so the choice waits for it to come back.
    public object RequestLiveView(bool on)
    {
        if (cam != IntPtr.Zero) return Run(() => SetLiveView(on), 8000);
        liveViewWanted = on;
        return new Dictionary<string, object> { { "liveView", false }, { "wanted", on } };
    }

    // Autofocus needs live view, so if it is off it comes on just for the focus.
    public object IntervalShot(string name, bool af)
    {
        if (af)
        {
            bool wasOff = !liveViewOn;
            try
            {
                if (wasOff) WaitForLiveView();
                AutoFocus();
            }
            finally
            {
                if (wasOff) StopLiveView();
            }
        }
        captureName = name;
        try { return Capture(); }
        finally { captureName = null; }
    }

    void WaitForLiveView()
    {
        long seq = Interlocked.Read(ref frameSeq);
        for (int i = 0; i < 50 && Interlocked.Read(ref frameSeq) == seq; i++)
        {
            if (!liveViewOn) StartLiveView();
            PumpWithFrames(100);
        }
        if (Interlocked.Read(ref frameSeq) == seq) throw new InvalidOperationException("live view did not start");
    }

    public object AutoFocus()
    {
        RequireLiveView();
        uint on = SendRetry(Eds.CmdEvfAf, 1);
        PumpWithFrames(2200);
        uint off = SendRetry(Eds.CmdEvfAf, 0);
        PumpWithFrames(300);
        return new Dictionary<string, object> { { "on", Eds.Hex(on) }, { "off", Eds.Hex(off) } };
    }

    // x and y are the wanted box centre, as fractions of the full frame.
    public object SetAfPoint(double x, double y)
    {
        RequireLiveView();
        if (zoom != 1) throw new InvalidOperationException("return to 1x before moving the AF point");
        int left = Math.Max(0, (int)(x * coordW) - afW / 2);
        int top = Math.Max(0, (int)(y * coordH) - afH / 2);
        var b = new byte[8];
        BitConverter.GetBytes(left).CopyTo(b, 0);
        BitConverter.GetBytes(top).CopyTo(b, 4);
        uint e = 0;
        for (int i = 0; i < 12; i++)
        {
            e = Eds.EdsSetPropertyData(cam, Eds.PropEvfZoomPos, 0, 8, b);
            if (e != Eds.ErrBusy) break;
            Pump(100);
        }
        if (e != 0) throw new InvalidOperationException("AF point refused: " + Eds.Hex(e));

        // The box only moves a few frames later, and focusing before then uses the old point.
        // Near the frame edge it stops short of the target, so this wait has to time out.
        for (int i = 0; i < 30; i++)
        {
            PumpWithFrames(50);
            if (Math.Abs(afX - left) < 300 && Math.Abs(afY - top) < 300) break;
        }
        return new Dictionary<string, object> { { "requested", new[] { left, top } }, { "box", new[] { afX, afY, afW, afH } } };
    }

    public object Capture()
    {
        long before = Interlocked.Read(ref captureSeq);
        uint press = SendRetry(Eds.CmdShutter, Eds.ShutterNonAf);
        uint release = SendRetry(Eds.CmdShutter, Eds.ShutterOff);
        if (press != 0) throw new InvalidOperationException("shutter refused: " + Eds.Hex(press));
        var end = DateTime.Now.AddSeconds(20);
        while (Interlocked.Read(ref captureSeq) == before && DateTime.Now < end)
        {
            Eds.EdsGetEvent();
            Thread.Sleep(15);
        }
        if (Interlocked.Read(ref captureSeq) == before) throw new TimeoutException("no photo arrived (release " + Eds.Hex(release) + ")");
        return new Dictionary<string, object> { { "file", lastCapture } };
    }

    public object Set(string key, string value)
    {
        var p = Tables.Find(key);
        if (key == "dof") RequireLiveView();
        uint code = p.Code(value);
        uint e = SetRetry(p.Id, code);
        if (e != 0) throw new InvalidOperationException(key + " refused: " + Eds.Hex(e));
        Pump(150);
        uint now;
        GetU(p.Id, out now);
        return new Dictionary<string, object> { { "key", key }, { "value", p.Label(now) } };
    }

    public object SetZoom(int value)
    {
        RequireLiveView();
        uint e = SetRetry(Eds.PropEvfZoom, (uint)value);
        if (e != 0) throw new InvalidOperationException("zoom refused: " + Eds.Hex(e));
        zoom = value;
        return new Dictionary<string, object> { { "zoom", value } };
    }
}

static class Raw
{
    // A CR2 is a TIFF whose first directory points at a full-size JPEG. Returns null for anything else.
    public static byte[] JpegPreview(byte[] file)
    {
        if (file.Length < 16 || file[0] != 'I' || file[1] != 'I' || file[8] != 'C' || file[9] != 'R') return null;
        int ifd = BitConverter.ToInt32(file, 4), offset = 0, length = 0;
        if (ifd < 0 || ifd > file.Length - 2) return null;
        int entries = BitConverter.ToUInt16(file, ifd);
        if (ifd + 2 + entries * 12 > file.Length) return null;
        for (int i = 0; i < entries; i++)
        {
            int entry = ifd + 2 + i * 12;
            int tag = BitConverter.ToUInt16(file, entry);
            if (tag == 0x111) offset = BitConverter.ToInt32(file, entry + 8);
            if (tag == 0x117) length = BitConverter.ToInt32(file, entry + 8);
        }
        if (offset <= 0 || length < 2 || length > file.Length - offset) return null;
        if (file[offset] != 0xFF || file[offset + 1] != 0xD8) return null;
        var jpeg = new byte[length];
        Buffer.BlockCopy(file, offset, jpeg, 0, length);
        return jpeg;
    }
}

// Takes photos on a fixed schedule from its own thread, so a run does not need the page open.
class Intervalometer
{
    readonly Camera camera;
    readonly object gate = new object();
    ManualResetEventSlim stop;
    bool running, af;
    int everySec, count, taken, failed, skipped;
    string run = "", lastError = "", ended = "";
    DateTime next;

    public Intervalometer(Camera camera) { this.camera = camera; }

    public object Start(int every, int photos, bool focus)
    {
        if (every < 1) throw new ArgumentException("interval must be at least 1 second");
        if (photos < 0) throw new ArgumentException("photo count cannot be negative");
        lock (gate)
        {
            if (running) throw new InvalidOperationException("an interval run is already going");
            running = true;
            everySec = every;
            count = photos;
            af = focus;
            taken = failed = skipped = 0;
            lastError = ended = "";
            run = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            next = DateTime.UtcNow;
            stop = new ManualResetEventSlim(false);
        }
        var thread = new Thread(Loop);
        thread.IsBackground = true;
        thread.Start();
        return Status();
    }

    public object Stop()
    {
        lock (gate)
        {
            if (running) stop.Set();
        }
        return Status();
    }

    public Dictionary<string, object> Status()
    {
        lock (gate)
        {
            int nextIn = running ? Math.Max(0, (int)Math.Ceiling((next - DateTime.UtcNow).TotalSeconds)) : 0;
            return new Dictionary<string, object>
            {
                { "running", running }, { "every", everySec }, { "count", count }, { "af", af },
                { "taken", taken }, { "failed", failed }, { "skipped", skipped }, { "nextIn", nextIn },
                { "lastError", lastError }, { "ended", ended },
            };
        }
    }

    TimeSpan UntilNext()
    {
        lock (gate)
        {
            var wait = next - DateTime.UtcNow;
            return wait > TimeSpan.Zero ? wait : TimeSpan.Zero;
        }
    }

    void Loop()
    {
        // A sleeping PC would drop the camera and miss every photo until it woke.
        Eds.SetThreadExecutionState(Eds.EsContinuous | Eds.EsSystemRequired);
        camera.Log("interval", "every " + everySec + " s, " + (count > 0 ? count + " photos" : "until stopped") + (af ? ", AF each photo" : ""));
        string result = "stopped", logged = "";
        while (!stop.Wait(UntilNext()))
        {
            string name = "interval_" + run + "_" + (taken + 1).ToString("D4");
            try
            {
                camera.Run(() => camera.IntervalShot(name, af), 50000);
                lock (gate) taken++;
                logged = "";
            }
            catch (Exception ex)
            {
                lock (gate)
                {
                    failed++;
                    lastError = ex.Message;
                }
                if (ex.Message != logged) camera.Log("interval", "photo failed: " + ex.Message);
                logged = ex.Message;
            }
            if (count > 0 && taken >= count)
            {
                result = "finished";
                break;
            }
            // The schedule stays fixed: a photo that overruns costs later slots, not drift.
            lock (gate)
            {
                next = next.AddSeconds(everySec);
                while (next <= DateTime.UtcNow)
                {
                    next = next.AddSeconds(everySec);
                    skipped++;
                }
            }
        }
        Eds.SetThreadExecutionState(Eds.EsContinuous);
        lock (gate)
        {
            running = false;
            ended = result;
        }
        camera.Log("interval", result + ": " + taken + " taken, " + failed + " failed, " + skipped + " skipped");
    }
}

static class Program
{
    static Camera camera;
    static Intervalometer interval;
    static HttpListener listener;
    static string htmlPath, captureDir;
    static int port = 8765;
    static readonly JavaScriptSerializer Json = new JavaScriptSerializer();

    static int Main(string[] args)
    {
        int output = 3;
        for (int i = 0; i + 1 < args.Length; i += 2)
        {
            if (args[i] == "--port") port = int.Parse(args[i + 1]);
            if (args[i] == "--output") output = int.Parse(args[i + 1]);
        }
        string exeDir = AppDomain.CurrentDomain.BaseDirectory;
        htmlPath = Path.GetFullPath(Path.Combine(exeDir, "..", "viewer.html"));
        captureDir = Path.GetFullPath(Path.Combine(exeDir, "..", "..", "captures"));

        // Take the port before the camera, so a second copy exits without disturbing the first.
        listener = new HttpListener();
        listener.Prefixes.Add("http://localhost:" + port + "/");
        try { listener.Start(); }
        catch (HttpListenerException)
        {
            Console.WriteLine("Viewer is already running on port " + port + ".");
            return 1;
        }

        camera = new Camera(captureDir, output);
        interval = new Intervalometer(camera);
        var thread = new Thread(camera.ThreadMain);
        thread.SetApartmentState(ApartmentState.STA);
        thread.Start();
        camera.Log("viewer", "listening on http://localhost:" + port + "/");
        Console.CancelKeyPress += delegate(object s, ConsoleCancelEventArgs e)
        {
            e.Cancel = true;
            Shutdown();
        };

        try
        {
            while (listener.IsListening)
            {
                var context = listener.GetContext();
                var worker = new Thread(() => Handle(context));
                worker.IsBackground = true;
                worker.Start();
            }
        }
        catch (HttpListenerException) { }
        catch (ObjectDisposedException) { }
        thread.Join(8000);
        return 0;
    }

    static void Shutdown()
    {
        camera.Stop();
        ThreadPool.QueueUserWorkItem(delegate
        {
            Thread.Sleep(300);
            listener.Stop();
        });
    }

    static void Handle(HttpListenerContext c)
    {
        try
        {
            string path = c.Request.Url.AbsolutePath;
            if (c.Request.HttpMethod == "POST") HandleCommand(c, path);
            else if (path == "/") SendFile(c, htmlPath, "text/html; charset=utf-8");
            else if (path == "/stream") Stream(c);
            else if (path == "/frame.jpg") SendFrame(c);
            else if (path == "/status")
            {
                var s = camera.Status();
                s["interval"] = interval.Status();
                SendJson(c, 200, s);
            }
            else if (path.StartsWith("/captures/"))
                SendCapture(c, Path.Combine(captureDir, Path.GetFileName(path)));
            else SendJson(c, 404, new Dictionary<string, object> { { "error", "not found" } });
        }
        catch (Exception ex)
        {
            try { SendJson(c, 500, new Dictionary<string, object> { { "error", ex.Message } }); }
            catch (Exception) { }
        }
    }

    static void HandleCommand(HttpListenerContext c, string path)
    {
        // Web pages from other sites can post to localhost, so only this page may send commands.
        string origin = c.Request.Headers["Origin"];
        if (origin != null && origin != "http://localhost:" + port)
        {
            SendJson(c, 403, new Dictionary<string, object> { { "error", "wrong origin" } });
            return;
        }
        var q = c.Request.QueryString;
        string who = q["who"] ?? "script";
        object result;
        try
        {
            switch (path)
            {
                case "/api/af":
                    camera.Log(who, "autofocus");
                    result = camera.Run(camera.AutoFocus, 8000);
                    break;
                case "/api/afpoint":
                    double x = double.Parse(q["x"], System.Globalization.CultureInfo.InvariantCulture);
                    double y = double.Parse(q["y"], System.Globalization.CultureInfo.InvariantCulture);
                    camera.Log(who, string.Format("AF point {0:0.00}, {1:0.00}", x, y));
                    result = camera.Run(() => camera.SetAfPoint(x, y), 8000);
                    if (q["af"] == "1") result = camera.Run(camera.AutoFocus, 8000);
                    break;
                case "/api/capture":
                    camera.Log(who, q["af"] == "1" ? "capture with AF" : "capture");
                    if (q["af"] == "1") camera.Run(camera.AutoFocus, 8000);
                    result = camera.Run(camera.Capture, 30000);
                    break;
                case "/api/set":
                    camera.Log(who, "set " + q["key"] + " = " + q["value"]);
                    result = camera.Run(() => camera.Set(q["key"], q["value"]), 8000);
                    break;
                case "/api/zoom":
                    int zoom = int.Parse(q["value"]);
                    camera.Log(who, "zoom " + zoom + "x");
                    result = camera.Run(() => camera.SetZoom(zoom), 8000);
                    break;
                case "/api/liveview":
                    bool wanted = q["value"] == "on";
                    camera.Log(who, "live view " + (wanted ? "on" : "off"));
                    result = camera.RequestLiveView(wanted);
                    break;
                case "/api/interval":
                    if (q["stop"] == "1")
                    {
                        camera.Log(who, "interval stop");
                        result = interval.Stop();
                        break;
                    }
                    camera.Log(who, "interval start");
                    result = interval.Start(int.Parse(q["every"]), int.Parse(q["count"] ?? "0"), q["af"] == "1");
                    break;
                case "/api/quit":
                    camera.Log(who, "viewer closing");
                    SendJson(c, 200, new Dictionary<string, object> { { "closing", true } });
                    Shutdown();
                    return;
                default:
                    SendJson(c, 404, new Dictionary<string, object> { { "error", "unknown command" } });
                    return;
            }
            SendJson(c, 200, result);
        }
        catch (Exception ex)
        {
            camera.Log("viewer", path + " failed: " + ex.Message);
            SendJson(c, 500, new Dictionary<string, object> { { "error", ex.Message } });
        }
    }

    static void Stream(HttpListenerContext c)
    {
        var r = c.Response;
        r.ContentType = "multipart/x-mixed-replace; boundary=frame";
        r.SendChunked = true;
        var body = r.OutputStream;
        long seq = 0;
        try
        {
            while (listener.IsListening)
            {
                byte[] jpeg = camera.WaitFrame(ref seq, 1000);
                if (jpeg == null) continue;
                var header = Encoding.ASCII.GetBytes("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + jpeg.Length + "\r\n\r\n");
                body.Write(header, 0, header.Length);
                body.Write(jpeg, 0, jpeg.Length);
                body.Write(new byte[] { 13, 10 }, 0, 2);
                body.Flush();
            }
        }
        catch (HttpListenerException) { }
        catch (IOException) { }
        try { r.Abort(); }
        catch (Exception) { }
    }

    static void SendFrame(HttpListenerContext c)
    {
        long seq;
        byte[] jpeg = camera.LatestFrame(out seq);
        if (jpeg == null)
        {
            SendJson(c, 503, new Dictionary<string, object> { { "error", "no frame yet" } });
            return;
        }
        c.Response.Headers["X-Frame-Seq"] = seq.ToString();
        Send(c, 200, "image/jpeg", jpeg);
    }

    // Browsers cannot show a CR2, so a RAW photo is sent as the JPEG stored inside it.
    static void SendCapture(HttpListenerContext c, string file)
    {
        if (!File.Exists(file))
        {
            SendJson(c, 404, new Dictionary<string, object> { { "error", "not found" } });
            return;
        }
        byte[] bytes = File.ReadAllBytes(file);
        Send(c, 200, "image/jpeg", Raw.JpegPreview(bytes) ?? bytes);
    }

    static void SendFile(HttpListenerContext c, string file, string type)
    {
        if (!File.Exists(file))
        {
            SendJson(c, 404, new Dictionary<string, object> { { "error", "not found" } });
            return;
        }
        Send(c, 200, type, File.ReadAllBytes(file));
    }

    static void SendJson(HttpListenerContext c, int code, object value)
    {
        Json.MaxJsonLength = int.MaxValue;
        Send(c, code, "application/json", Encoding.UTF8.GetBytes(Json.Serialize(value)));
    }

    static void Send(HttpListenerContext c, int code, string type, byte[] body)
    {
        c.Response.StatusCode = code;
        c.Response.ContentType = type;
        c.Response.Headers["Cache-Control"] = "no-store";
        c.Response.ContentLength64 = body.Length;
        c.Response.OutputStream.Write(body, 0, body.Length);
        c.Response.Close();
    }
}
