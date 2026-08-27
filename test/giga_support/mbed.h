#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

#include "Arduino.h"

namespace mbed {

template<typename Signature>
class Callback;

template<>
class Callback<void()> {
public:
    Callback() : context_(nullptr), thunk_(nullptr) {}

    Callback(void* context, void (*thunk)(void*))
        : context_(context), thunk_(thunk) {}

    void operator()() const {
        if (thunk_) {
            thunk_(context_);
        }
    }

private:
    void* context_;
    void (*thunk_)(void*);
};

template<typename T>
Callback<void()> callback(T* object, void (T::*method)()) {
    // Tests never invoke the installed callback. The stub only needs to model
    // its presence without introducing platform allocation behavior.
    (void)object;
    (void)method;
    return Callback<void()>();
}

class BufferedSerial {
public:
    enum Parity {
        None = 0,
        Even,
        Odd
    };

    BufferedSerial(PinName, PinName) {}

    void set_blocking(bool) {}
    void set_baud(unsigned long) {}
    void set_format(int, Parity, int) {}
    ssize_t read(void*, std::size_t) { return 0; }
    ssize_t write(const void*, std::size_t size) {
        return static_cast<ssize_t>(size);
    }
    int sync() { return 0; }
    void close() {}
    void sigio(Callback<void()>) {}
};

} // namespace mbed

namespace rtos {

struct ThisThread {
    static void yield() {}

    template<typename Rep, typename Period>
    static void sleep_for(const std::chrono::duration<Rep, Period>&) {}
};

} // namespace rtos
