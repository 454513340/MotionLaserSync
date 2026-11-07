#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace p0 {
struct TableRingHeader {
    int capacity = 0;
    int write = 0;
    int read = 0;
    bool overflow = false;
};

// Reject NaN/fractional/out-of-range fields before float->int conversion.
inline bool parseTableRingHeader(const float* values, int maxCapacity,
                                TableRingHeader& out) {
    if (!values || maxCapacity <= 0) return false;
    for (int i = 0; i < 4; ++i) {
        const double v = values[i];
        if (!std::isfinite(v) || v < 0 || std::floor(v) != v
            || v > static_cast<double>((std::numeric_limits<int>::max)())) return false;
    }
    const double n = values[0];
    if (n < 1 || n > maxCapacity || values[1] >= n || values[2] >= n) return false;
    out = {static_cast<int>(n), static_cast<int>(values[1]),
           static_cast<int>(values[2]), values[3] != 0};
    return true;
}

enum class TableReadStatus { Ok, ReadFailed, AckUncertain, Overflow, InvalidHeader };
struct TableReadBatch {
    TableReadStatus status = TableReadStatus::InvalidHeader;
    std::vector<float> values;
    int records = 0;
};

// Preserve the V17 protocol and call order. read(startRecord, count, float*) and
// acknowledge(nextReadRecord) return bool. No host data is published until ALL
// segments AND the read-index acknowledgement succeeded. An ACK error may mean
// the write executed but its reply was lost: the caller must latch a scan fault,
// not retry and silently renumber poses. This is not a hardware-atomic snapshot.
template<class Read, class Acknowledge>
TableReadBatch readTableRingBatch(const TableRingHeader& h, int dimension,
                                 Read&& read, Acknowledge&& acknowledge) {
    TableReadBatch result;
    if (dimension <= 0 || h.capacity <= 0 || h.read < 0 || h.write < 0
        || h.read >= h.capacity || h.write >= h.capacity
        || h.capacity > (std::numeric_limits<int>::max)() / dimension) return result;
    if (h.overflow) { result.status = TableReadStatus::Overflow; return result; }
    const int count = h.write >= h.read ? h.write - h.read : h.capacity - h.read + h.write;
    if (count == 0) { result.status = TableReadStatus::Ok; return result; }
    std::vector<float> staged(static_cast<std::size_t>(count) * dimension);
    const int firstCount = h.write > h.read ? count : h.capacity - h.read;
    if (!read(h.read, firstCount, staged.data())
        || (count > firstCount && !read(0, count - firstCount,
                                        staged.data() + firstCount * dimension))) {
        result.status = TableReadStatus::ReadFailed;
        return result;
    }
    if (!acknowledge(h.write)) {
        result.status = TableReadStatus::AckUncertain;
        return result;
    }
    result.status = TableReadStatus::Ok;
    result.records = count;
    result.values.swap(staged);
    return result;
}
} // namespace p0
