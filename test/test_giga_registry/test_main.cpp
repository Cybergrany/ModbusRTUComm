#include <unity.h>

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "platform/giga/GigaBufferedSerial.h"

namespace {

using RegistryStorage = typename std::aligned_storage<
    GigaBufferedSerialRegistry::storageSize(),
    GigaBufferedSerialRegistry::storageAlignment()>::type;

void test_direct_construction_remains_registered() {
    TEST_ASSERT_EQUAL_UINT8(0U, GigaBufferedSerialRegistry::count());
    {
        GigaBufferedSerial serial(1, 0);
        TEST_ASSERT_TRUE(serial.registered());
        TEST_ASSERT_TRUE(GigaBufferedSerialRegistry::contains(&serial));
        TEST_ASSERT_EQUAL_PTR(
            &serial,
            GigaBufferedSerialRegistry::find(serial));
        TEST_ASSERT_EQUAL_UINT8(1U, GigaBufferedSerialRegistry::count());
    }
    TEST_ASSERT_EQUAL_UINT8(0U, GigaBufferedSerialRegistry::count());
}

void test_placement_construction_uses_caller_storage() {
    RegistryStorage storage;
    GigaBufferedSerialRegistryResult result =
        GigaBufferedSerialRegistry::constructAt(
            &storage,
            sizeof(storage),
            1,
            0);

    TEST_ASSERT_TRUE(static_cast<bool>(result));
    TEST_ASSERT_NOT_NULL(result.serial);
    TEST_ASSERT_TRUE(result.serial->registered());
    TEST_ASSERT_EQUAL_UINT8(1U, GigaBufferedSerialRegistry::count());

    GigaBufferedSerial* serial = result.serial;
    TEST_ASSERT_TRUE(GigaBufferedSerialRegistry::destroyAt(serial));
    TEST_ASSERT_NULL(serial);
    TEST_ASSERT_EQUAL_UINT8(0U, GigaBufferedSerialRegistry::count());
}

void test_invalid_and_misaligned_storage_fail_closed() {
    GigaBufferedSerialRegistryResult invalid =
        GigaBufferedSerialRegistry::constructAt(nullptr, 0U, 1, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(GigaBufferedSerialRegistryStatus::InvalidStorage),
        static_cast<uint8_t>(invalid.status));
    TEST_ASSERT_NULL(invalid.serial);

    alignas(GigaBufferedSerial) unsigned char bytes[
        sizeof(GigaBufferedSerial) + alignof(GigaBufferedSerial)];
    void* misaligned = bytes + 1U;
    GigaBufferedSerialRegistryResult alignment =
        GigaBufferedSerialRegistry::constructAt(
            misaligned,
            sizeof(bytes) - 1U,
            1,
            0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(GigaBufferedSerialRegistryStatus::MisalignedStorage),
        static_cast<uint8_t>(alignment.status));
    TEST_ASSERT_NULL(alignment.serial);
    TEST_ASSERT_EQUAL_UINT8(0U, GigaBufferedSerialRegistry::count());
}

void test_full_registry_rejects_placement_construction() {
    static_assert(
        GigaBufferedSerialRegistry::capacity() == 4U,
        "The characterized GIGA adapter registry capacity changed");

    RegistryStorage storage;
    {
        GigaBufferedSerial serial0(1, 0);
        GigaBufferedSerial serial1(18, 19);
        GigaBufferedSerial serial2(16, 17);
        GigaBufferedSerial serial3(14, 15);
        TEST_ASSERT_EQUAL_UINT8(
            GigaBufferedSerialRegistry::capacity(),
            GigaBufferedSerialRegistry::count());

        GigaBufferedSerialRegistryResult result =
            GigaBufferedSerialRegistry::constructAt(
                &storage,
                sizeof(storage),
                2,
                3);
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<uint8_t>(GigaBufferedSerialRegistryStatus::Full),
            static_cast<uint8_t>(result.status));
        TEST_ASSERT_NULL(result.serial);
        TEST_ASSERT_EQUAL_UINT8(
            GigaBufferedSerialRegistry::capacity(),
            GigaBufferedSerialRegistry::count());
    }
    TEST_ASSERT_EQUAL_UINT8(0U, GigaBufferedSerialRegistry::count());
}

} // namespace

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_direct_construction_remains_registered);
    RUN_TEST(test_placement_construction_uses_caller_storage);
    RUN_TEST(test_invalid_and_misaligned_storage_fail_closed);
    RUN_TEST(test_full_registry_rejects_placement_construction);
    return UNITY_END();
}
