#if defined(ARDUINO_GIGA)
#ifndef MODBUS_RTU_GIGA_BUFFERED_SERIAL_H_
#define MODBUS_RTU_GIGA_BUFFERED_SERIAL_H_

// Ownership marker for ODR compile gates. Exactly one package may
// define the global GigaBufferedSerial compatibility type in a firmware image.
#define MBUS_RTU_GIGA_BUFFERED_SERIAL_OWNER_MODBUS_RTU_COMM 1

#include <Arduino.h>
#include <chrono>
#include "mbed.h"
#include "drivers/BufferedSerial.h"
#include "GigaSerialFormat.h"

class GigaBufferedSerial : public arduino::HardwareSerial {
public:
    static constexpr uint32_t kWriteTimeoutUs = 20000UL;
    static constexpr uint32_t kFlushTimeoutUs = 20000UL;

    using ReadableCallback = void (*)(void*);

    explicit GigaBufferedSerial(PinName txPin, PinName rxPin)
        : _bufSerial(txPin, rxPin) {
        _bufSerial.set_blocking(false);
        _registerInstance();
    }

    explicit GigaBufferedSerial(int txPin, int rxPin)
        : GigaBufferedSerial(digitalPinToPinName(txPin), digitalPinToPinName(rxPin)) {}

    ~GigaBufferedSerial() noexcept {
        detachReadableCallback();
        _unregisterInstance();
        _bufSerial.close();
    }

    GigaBufferedSerial(const GigaBufferedSerial&) = delete;
    GigaBufferedSerial& operator=(const GigaBufferedSerial&) = delete;
    GigaBufferedSerial(GigaBufferedSerial&&) = delete;
    GigaBufferedSerial& operator=(GigaBufferedSerial&&) = delete;

    void begin(unsigned long baud) override {
        _bufSerial.set_baud(baud);
        _bufSerial.set_format(8, mbed::BufferedSerial::None, 1);
    }

    void begin(unsigned long baud, uint16_t config) override {
        _bufSerial.set_baud(baud);
        modbus_rtu::platform::giga::applySerialConfig(_bufSerial, config);
    }

    void end() override {
        detachReadableCallback();
        _bufSerial.close();
    }

    int available() override {
        if (_peekBuf >= 0) {
            return 1;
        }
        uint8_t byte = 0;
        const ssize_t n = _bufSerial.read(&byte, 1);
        if (n == 1) {
            _peekBuf = static_cast<int>(byte);
            return 1;
        }
        return 0;
    }

    int peek() override {
        if (_peekBuf >= 0) {
            return _peekBuf;
        }
        if (available() > 0) {
            return _peekBuf;
        }
        return -1;
    }

    int read() override {
        if (_peekBuf >= 0) {
            const int c = _peekBuf;
            _peekBuf = -1;
            return c;
        }
        uint8_t byte = 0;
        const ssize_t n = _bufSerial.read(&byte, 1);
        return (n == 1) ? static_cast<int>(byte) : -1;
    }

    size_t write(uint8_t c) override {
        return write(&c, 1);
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        if (!buffer || size == 0) {
            return 0;
        }
        size_t written = 0;
        const uint32_t startUs = micros();
        while (written < size) {
            const ssize_t n = _bufSerial.write(buffer + written, size - written);
            if (n > 0) {
                written += static_cast<size_t>(n);
                continue;
            }
            if (static_cast<uint32_t>(micros() - startUs) >= kWriteTimeoutUs) {
                break;
            }
            rtos::ThisThread::yield();
        }
        return written;
    }

    bool flushWithTimeoutUs(uint32_t timeoutUs = kFlushTimeoutUs) {
        const uint32_t startUs = micros();
        while (true) {
            if (_bufSerial.sync() == 0) {
                return true;
            }
            if (timeoutUs == 0 ||
                static_cast<uint32_t>(micros() - startUs) >= timeoutUs) {
                return false;
            }
            rtos::ThisThread::yield();
        }
    }

    bool waitForTxDrainEstimateUs(size_t byteCount, uint32_t charTimeUs) {
        if (byteCount == 0 || charTimeUs == 0) {
            return true;
        }

        uint64_t waitUs64 = (static_cast<uint64_t>(byteCount) + 1ULL) *
                            static_cast<uint64_t>(charTimeUs);
        if (waitUs64 > 0xFFFFFFFFULL) {
            waitUs64 = 0xFFFFFFFFULL;
        }

        const uint32_t waitUs = static_cast<uint32_t>(waitUs64);
        const uint32_t startUs = micros();
        while (static_cast<uint32_t>(micros() - startUs) < waitUs) {
            const uint32_t elapsedUs = static_cast<uint32_t>(micros() - startUs);
            const uint32_t remainingUs = waitUs - elapsedUs;
            if (remainingUs >= 1000U) {
                rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
            } else if (remainingUs > 50U) {
                delayMicroseconds(remainingUs);
            } else {
                rtos::ThisThread::yield();
            }
        }
        return true;
    }

    void flush() override {
        (void)flushWithTimeoutUs();
    }

    operator bool() override {
        return true;
    }

    bool attachReadableCallback(ReadableCallback cb, void* ctx) {
        _readableCb = cb;
        _readableCbCtx = ctx;
        if (_readableCb) {
            _bufSerial.sigio(mbed::callback(this, &GigaBufferedSerial::_onReadableSigio));
        } else {
            _bufSerial.sigio(mbed::Callback<void()>{});
        }
        return true;
    }

    void detachReadableCallback() {
        _readableCb = nullptr;
        _readableCbCtx = nullptr;
        _bufSerial.sigio(mbed::Callback<void()>{});
    }

    static bool attachReadableCallback(arduino::Stream& stream, ReadableCallback cb, void* ctx) {
        GigaBufferedSerial* instance = _fromStream(stream);
        if (!instance) {
            return false;
        }
        return instance->attachReadableCallback(cb, ctx);
    }

    static void detachReadableCallback(arduino::Stream& stream) {
        GigaBufferedSerial* instance = _fromStream(stream);
        if (instance) {
            instance->detachReadableCallback();
        }
    }

    static bool flushWithTimeout(arduino::Stream& stream, uint32_t timeoutUs = kFlushTimeoutUs) {
        GigaBufferedSerial* instance = _fromStream(stream);
        if (!instance) {
            stream.flush();
            return true;
        }
        return instance->flushWithTimeoutUs(timeoutUs);
    }

    static bool waitForTxDrainEstimate(
        arduino::Stream& stream,
        size_t byteCount,
        uint32_t charTimeUs) {
        GigaBufferedSerial* instance = _fromStream(stream);
        if (!instance) {
            stream.flush();
            return true;
        }
        return instance->waitForTxDrainEstimateUs(byteCount, charTimeUs);
    }

private:
    static constexpr uint8_t kRegistryCap = 4;

    static GigaBufferedSerial** _registry() {
        static GigaBufferedSerial* slots[kRegistryCap] = {nullptr, nullptr, nullptr, nullptr};
        return slots;
    }

    void _registerInstance() {
        GigaBufferedSerial** slots = _registry();
        for (uint8_t i = 0; i < kRegistryCap; ++i) {
            if (slots[i] == nullptr) {
                slots[i] = this;
                return;
            }
        }
    }

    void _unregisterInstance() {
        GigaBufferedSerial** slots = _registry();
        for (uint8_t i = 0; i < kRegistryCap; ++i) {
            if (slots[i] == this) {
                slots[i] = nullptr;
                return;
            }
        }
    }

    static GigaBufferedSerial* _fromStream(arduino::Stream& stream) {
        GigaBufferedSerial** slots = _registry();
        arduino::Stream* target = &stream;
        for (uint8_t i = 0; i < kRegistryCap; ++i) {
            GigaBufferedSerial* cur = slots[i];
            if (!cur) {
                continue;
            }
            if (static_cast<arduino::Stream*>(cur) == target) {
                return cur;
            }
        }
        return nullptr;
    }

    void _onReadableSigio() {
        ReadableCallback cb = _readableCb;
        if (cb) {
            cb(_readableCbCtx);
        }
    }

    mbed::BufferedSerial _bufSerial;
    int _peekBuf = -1;
    ReadableCallback _readableCb = nullptr;
    void* _readableCbCtx = nullptr;
};

#endif
#endif
