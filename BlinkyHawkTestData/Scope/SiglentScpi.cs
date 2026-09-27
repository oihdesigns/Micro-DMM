using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using System.Text;

// Raw-socket SCPI client for Siglent SDS800X HD (port 5025) plus fast waveform decode / CSV output.
public class SiglentScpi : IDisposable
{
    TcpClient client;
    NetworkStream stream;

    public SiglentScpi(string host, int port, int timeoutMs)
    {
        client = new TcpClient();
        IAsyncResult ar = client.BeginConnect(host, port, null, null);
        if (!ar.AsyncWaitHandle.WaitOne(3000)) { client.Close(); throw new TimeoutException("No response from " + host + ":" + port); }
        client.EndConnect(ar);
        client.NoDelay = true;
        stream = client.GetStream();
        stream.ReadTimeout = timeoutMs;
    }

    public void Write(string cmd)
    {
        byte[] b = Encoding.ASCII.GetBytes(cmd + "\n");
        stream.Write(b, 0, b.Length);
    }

    int ReadByteOrThrow()
    {
        int b = stream.ReadByte();
        if (b < 0) throw new IOException("Connection closed by scope");
        return b;
    }

    public string Query(string cmd)
    {
        Write(cmd);
        var sb = new StringBuilder();
        int b;
        while (true)
        {
            b = ReadByteOrThrow();
            if (b == '\n') { if (sb.Length == 0) continue; break; }
            sb.Append((char)b);
        }
        return sb.ToString().Trim();
    }

    void ReadExact(byte[] buf, int offset, int count)
    {
        while (count > 0)
        {
            int n = stream.Read(buf, offset, count);
            if (n <= 0) throw new IOException("Connection closed by scope");
            offset += n; count -= n;
        }
    }

    // IEEE 488.2 definite-length block: #<n><len><data>\n
    public byte[] QueryBlock(string cmd)
    {
        Write(cmd);
        int b;
        while ((b = ReadByteOrThrow()) != '#') { }
        int nd = ReadByteOrThrow() - '0';
        byte[] lenBytes = new byte[nd];
        ReadExact(lenBytes, 0, nd);
        int len = int.Parse(Encoding.ASCII.GetString(lenBytes));
        byte[] data = new byte[len];
        ReadExact(data, 0, len);
        // The trailing terminator (\n or \n\n) is left in the stream; Query() skips leading newlines.
        return data;
    }

    public void Dispose()
    {
        if (stream != null) stream.Dispose();
        if (client != null) client.Close();
    }

    // Convert 16-bit (WORD) waveform codes to volts.
    public static double[] DecodeWord(List<byte[]> chunks, double vPerDiv, double offsetV, double codePerDiv)
    {
        int total = 0;
        foreach (var c in chunks) total += c.Length / 2;
        var v = new double[total];
        int k = 0;
        double scale = vPerDiv / codePerDiv;
        foreach (var c in chunks)
            for (int i = 0; i + 1 < c.Length; i += 2)
                v[k++] = BitConverter.ToInt16(c, i) * scale - offsetV;
        return v;
    }

    public static double[] MakeTime(double t0, double dt, int n)
    {
        // decimal arithmetic keeps the time column free of binary rounding noise (e.g. 1.57E-11 at the trigger)
        var t = new double[n];
        decimal t0m = (decimal)t0, dtm = (decimal)dt;
        for (int i = 0; i < n; i++) t[i] = (double)(t0m + i * dtm);
        return t;
    }

    // Append the data rows of the CSV. isTime[c] picks the number format for column c; a column shorter than
    // the longest one is left blank in the remaining rows.
    public static void AppendColumns(string path, double[][] cols, bool[] isTime)
    {
        int rows = 0;
        foreach (var c in cols) if (c.Length > rows) rows = c.Length;
        var inv = CultureInfo.InvariantCulture;
        using (var w = new StreamWriter(path, true, new UTF8Encoding(false), 1 << 20))
        {
            var sb = new StringBuilder(256);
            for (int r = 0; r < rows; r++)
            {
                sb.Length = 0;
                for (int c = 0; c < cols.Length; c++)
                {
                    if (c > 0) sb.Append(',');
                    if (r < cols[c].Length) sb.Append(cols[c][r].ToString(isTime[c] ? "G9" : "G6", inv));
                }
                w.WriteLine(sb.ToString());
            }
        }
    }
}
