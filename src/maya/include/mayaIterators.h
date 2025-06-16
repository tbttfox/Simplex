#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnAttribute.h>

#include <iterator>
#include <type_traits>
#include <unordered_map>
#include <utility>  // For std::pair
#include <vector>

// Wrapper for range-based iteration
class MArrayInputDataHandleRange {
   public:
    class Iterator {
       public:
        using iterator_category = std::bidirectional_iterator_tag;
        using value_type = std::pair<unsigned int, MDataHandle>;
        using difference_type = std::ptrdiff_t;
        using pointer = value_type*;
        using reference = value_type&;

        explicit Iterator(MArrayDataHandle* handle, unsigned int index = 0)
            : m_handle(handle), m_index(index), m_count(handle ? handle->elementCount() : 0) {
            if (m_handle && m_index < m_count) {
                m_handle->jumpToArrayElement(m_index);
            }
        }

        // Dereference operator returning {index, inputHandle}
        value_type operator*() const { return {m_handle->elementIndex(), m_handle->inputValue()}; }

        // Pre-increment
        Iterator& operator++() {
            if (m_handle && ++m_index < m_count) {
                m_handle->jumpToArrayElement(m_index);
            }
            return *this;
        }

        // Post-increment
        Iterator operator++(int) {
            Iterator temp = *this;
            ++(*this);
            return temp;
        }

        // Pre-decrement
        Iterator& operator--() {
            if (m_handle && m_index > 0) {
                --m_index;
                m_handle->jumpToArrayElement(m_index);
            }
            return *this;
        }

        // Post-decrement
        Iterator operator--(int) {
            Iterator temp = *this;
            --(*this);
            return temp;
        }

        // Comparisons
        bool operator==(const Iterator& other) const {
            return m_index == other.m_index && m_handle == other.m_handle;
        }
        bool operator!=(const Iterator& other) const { return !(*this == other); }

       private:
        MArrayDataHandle* m_handle;
        unsigned int m_index;
        unsigned int m_count;
    };

    explicit MArrayInputDataHandleRange(MArrayDataHandle& handle) : m_handle(handle) {}

    Iterator begin() { return Iterator(&m_handle, 0); }
    Iterator end() { return Iterator(&m_handle, m_handle.elementCount()); }

   private:
    MArrayDataHandle& m_handle;
};

template <typename T>
T getTypedMDataHandleValue(const MDataHandle& handle) {
    if constexpr (std::is_same_v<T, int>) {
        return handle.asInt();
    } else if constexpr (std::is_same_v<T, float>) {
        return handle.asFloat();
    } else if constexpr (std::is_same_v<T, double>) {
        return handle.asDouble();
    } else if constexpr (std::is_same_v<T, bool>) {
        return handle.asBool();
    } else if constexpr (std::is_same_v<T, MVector>) {
        return handle.asVector();
    } else if constexpr (std::is_same_v<T, MFloatVector>) {
        return handle.asFloatVector();
    } else if constexpr (std::is_same_v<T, MMatrix>) {
        return handle.asMatrix();
    } else if constexpr (std::is_same_v<T, MString>) {
        return handle.asString();
    } else {
        static_assert(false, "Unsupported MDataHandle type.");
    }
}

template <typename T>
inline void getDenseArrayHandleData(MArrayDataHandle& arrayHandle, std::vector<T>& ret) {
    int prevIdx = 0;
    for (auto [index, handle] : MArrayInputDataHandleRange(arrayHandle)) {
        for (; prevIdx < index; ++prevIdx) {
            ret.push_back(T());
        }
        ret.push_back(getTypedMDataHandleValue<T>(handle));
        prevIdx = index + 1;
    }
}

template <typename T>
inline void getDenseArrayHandleData(MDataBlock& dataBlock, MObject& attr, std::vector<T>& ret) {
    MArrayDataHandle handle = dataBlock.inputArrayValue(attr);
    getDenseArrayHandleData<T>(handle, ret);
}

template <typename T>
inline void getSparseArrayHandleData(
    MArrayDataHandle& arrayHandle, std::unordered_map<UINT, T>& ret
) {
    for (auto [index, handle] : MArrayInputDataHandleRange(arrayHandle)) {
        ret[index] = getTypedMDataHandleValue<T>(handle);
    }
}

template <typename T>
inline void getSparseArrayHandleData(
    MDataBlock& dataBlock, MObject& attr, std::unordered_map<UINT, T>& ret
) {
    MArrayDataHandle handle = dataBlock.inputArrayValue(attr);
    getSparseArrayHandleData<T>(handle, ret);
}
