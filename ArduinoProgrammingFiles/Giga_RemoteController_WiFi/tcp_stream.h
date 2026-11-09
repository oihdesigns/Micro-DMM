#pragma once
// Reliable reply path for the Wi-Fi command socket.
//
// processCommand() prints a capture as thousands of tiny print() calls. Sent
// straight to a WiFiClient, each becomes its own TCP send, and MbedClient::write
// returns 0 (the bytes are simply gone) whenever the network stack is out of
// buffers. On a 10k-sample capture that dropped a few characters per reply: a
// lost comma glued two samples together ("1823,1811" -> 18231811), and a lost
// digit silently changed a value.
//
// This wrapper collects output into ~one TCP segment and sends it in a loop that
// keeps going on partial or failed sends until every byte is out. It only gives
// up if the client disconnects or makes no progress for STALL_MS. Reads go
// straight through to the client. Kept in a header so the IDE's prototype
// generator doesn't see it.
#include <Arduino.h>
#include <Client.h>

class ReliableTcpStream : public Stream {
public:
    static const uint32_t STALL_MS = 5000;

    explicit ReliableTcpStream(Client& c) : c_(c) {}
    ~ReliableTcpStream() { flush(); }

    int available() override { return c_.available(); }
    int read() override { return c_.read(); }
    int peek() override { return c_.peek(); }

    size_t write(uint8_t b) override { return write(&b, 1); }
    size_t write(const uint8_t* p, size_t len) override {
        size_t done = 0;
        while (done < len) {
            if (n_ == sizeof(buf_)) flush();
            if (failed_) return done;
            size_t room = sizeof(buf_) - n_, m = (len - done < room) ? len - done : room;
            memcpy(buf_ + n_, p + done, m); n_ += m; done += m;
        }
        return len;
    }

    void flush() override {
        size_t off = 0; uint32_t last = millis();
        while (off < n_ && !failed_) {
            size_t s = c_.write(buf_ + off, n_ - off);
            if (s > 0) { off += s; last = millis(); retries_ += (off < n_); }
            else if (!c_.connected() || millis() - last > STALL_MS) { failed_ = true; lost_ += n_ - off; }
            else { retries_++; delay(2); }
        }
        n_ = 0;
    }

    bool failed() const { return failed_; }
    uint32_t retries() const { return retries_; }   // sends that needed another try
    uint32_t lost() const { return lost_; }         // bytes abandoned (only after a stall/disconnect)

private:
    Client& c_;
    uint8_t buf_[1436];   // fits one TCP segment on a 1500-byte MTU
    size_t n_ = 0;
    bool failed_ = false;
    uint32_t retries_ = 0, lost_ = 0;
};
