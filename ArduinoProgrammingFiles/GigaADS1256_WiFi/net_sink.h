#pragma once
// Non-blocking output path, so the acquisition never waits on the network.
//
// Giga_RemoteController_WiFi taught two things about replying over Wi-Fi:
//  1. MbedClient::write() returns 0 (the bytes are gone) when lwIP is out of
//     buffers, so every send has to be checked and retried.
//  2. Its retry loop (ReliableTcpStream) blocks. That was fine there, because
//     the ADC had already stopped when the reply went out. Here the ADC never
//     stops: a blocked loop means full DMA buffers and lost samples.
//
// So every line is formatted into a LineBuf first, then copied whole into a
// TxRing per output (USB serial, TCP client). loop() drains each ring with
// non-blocking writes, keeping what the socket refused for next time. When a
// ring has no room for a whole line, the line is dropped and counted. A line is
// never split, so a slow client sees missing lines, not glued-together ones.
// Kept in a header so the IDE's prototype generator doesn't see it.
#include <Arduino.h>

class TxRing {
public:
    static const size_t N = 24576;

    // all or nothing: a line either fits whole or is dropped
    bool put(const uint8_t* p, size_t n) {
        if (n > room()) { drops_++; return false; }
        size_t idx = head_ % N, first = (n < N - idx) ? n : N - idx;
        memcpy(buf_ + idx, p, first);
        if (n > first) memcpy(buf_, p + first, n - first);
        head_ += n;
        return true;
    }
    size_t used() const { return (size_t)(head_ - tail_); }
    size_t room() const { return N - used(); }
    // longest contiguous run of unsent bytes, for one write() call
    size_t peek(const uint8_t** p) const {
        size_t idx = tail_ % N, n = used();
        if (n > N - idx) n = N - idx;
        *p = buf_ + idx;
        return n;
    }
    void consume(size_t n) { tail_ += n; sent_ += n; }
    void clear() { tail_ = head_; }
    uint32_t drops() const { return drops_; }
    uint32_t sent() const { return sent_; }

private:
    uint8_t buf_[N];
    uint32_t head_ = 0, tail_ = 0, drops_ = 0, sent_ = 0;
};

// One output line under construction. Print supplies print(float, digits) etc.,
// so the formatting reads like the Serial.print() code it replaces.
class LineBuf : public Print {
public:
    static const size_t CAP = 4096;
    size_t write(uint8_t c) override {
        if (n_ < CAP) { b_[n_++] = c; return 1; }
        over_ = true; return 0;
    }
    size_t write(const uint8_t* p, size_t len) override {
        size_t m = (len < CAP - n_) ? len : CAP - n_;
        memcpy(b_ + n_, p, m); n_ += m;
        if (m < len) over_ = true;
        return m;
    }
    void reset() { n_ = 0; over_ = false; }
    const uint8_t* data() const { return b_; }
    size_t size() const { return n_; }
    bool overflowed() const { return over_; }
private:
    uint8_t b_[CAP];
    size_t n_ = 0;
    bool over_ = false;
};
