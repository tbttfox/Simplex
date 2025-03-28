/*

Some Example usage:

Get all the aOrigMatrix non-array child values from the aOrig array plug and store it in restMats
Similar for the two others

    std::vector<MVector> restAAs;
    std::vector<MMatrix> restMats;
    std::vector<bool> restUseMats;
    getDenseArrayChildHandleData(dataBlock, aOrig, aOrigMatrix, restMats);
    getDenseArrayChildHandleData(dataBlock, aOrig, aOrigAxisAngle, restAAs);
    getDenseArrayChildHandleData(dataBlock, aOrig, aOrigUseMatrix, restUseMats);



All as one loop to a struct. Probably a little faster

    auto valueGetter = [&](MDataHandle& h) {
        return std::make_tuple(
            h.child(aOrigMatrix).asMatrix(),
            h.child(aOrigAxisAngle).asVector(),
            h.child(aOrigUseMatrix).asBool()
        );
    };
    std::vector<std::tuple<MMatrix, MVector, bool>> restMVBs;
    getDenseArrayHandleData(dataBlock, aOrig, restMVBs, valueGetter);


*/

#pragma once

#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MIndexMapper.h>
#include <maya/MItGeometry.h>
#include <maya/MPxGeometryFilter.h>
#include <maya/MDistance.h>
#include <maya/MAngle.h>
#include <maya/MTime.h>

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

// Wrapper for range-based iteration
class MArrayInputDataHandleComponentRange {
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

    explicit MArrayInputDataHandleComponentRange(MArrayDataHandle& handle, MObject& components)
        : m_handle(handle), m_components(components) {}

    Iterator begin() { return Iterator(&m_handle, 0); }
    Iterator end() { return Iterator(&m_handle, m_handle.elementCount()); }

   private:
    MArrayDataHandle& m_handle;
    MObject& m_components;
};

template <typename T>
struct DefaultHandleValueGetter {
    T operator()(const MDataHandle& handle) const {
        if constexpr (std::is_same_v<T, MAngle>) {
            return handle.asAngle();
        } else if constexpr (std::is_same_v<T, MTime>) {
            return handle.asTime();
        } else if constexpr (std::is_same_v<T, MDistance>) {
            return handle.asDistance();
        } else if constexpr (std::is_same_v<T, MString>) {
            return handle.asString();
        } else if constexpr (std::is_same_v<T, MVector>) {
            return handle.asVector();
        } else if constexpr (std::is_same_v<T, bool>) {
            return handle.asBool();
        } else if constexpr (std::is_same_v<T, char>) {
            return handle.asChar();
        } else if constexpr (std::is_same_v<T, double>) {
            return handle.asDouble();
        } else if constexpr (std::is_same_v<T, double2>) {
            return handle.asDouble2();
        } else if constexpr (std::is_same_v<T, double3>) {
            return handle.asDouble3();
        } else if constexpr (std::is_same_v<T, double4>) {
            return handle.asDouble4();
        } else if constexpr (std::is_same_v<T, float>) {
            return handle.asFloat();
        } else if constexpr (std::is_same_v<T, float2>) {
            return handle.asFloat2();
        } else if constexpr (std::is_same_v<T, float3>) {
            return handle.asFloat3();
        } else if constexpr (std::is_same_v<T, int>) {
            return handle.asInt();
        } else if constexpr (std::is_same_v<T, int2>) {
            return handle.asInt2();
        } else if constexpr (std::is_same_v<T, int3>) {
            return handle.asInt3();
        } else if constexpr (std::is_same_v<T, short>) {
            return handle.asShort();
        } else if constexpr (std::is_same_v<T, short2>) {
            return handle.asShort2();
        } else if constexpr (std::is_same_v<T, short3>) {
            return handle.asShort3();
        } else if constexpr (std::is_same_v<T, unsigned char>) {
            return handle.asUChar();
        } else if constexpr (std::is_same_v<T, MFloatMatrix>) {
            return handle.asFloatMatrix();
        } else if constexpr (std::is_same_v<T, MFloatVector>) {
            return handle.asFloatVector();
        } else if constexpr (std::is_same_v<T, MInt64>) {
            return handle.asInt64();
        } else if constexpr (std::is_same_v<T, MMatrix>) {
            return handle.asMatrix();
        } else {
            static_assert(false, "Unsupported MDataHandle type.");
        }
    }
};

/************************************
Dense getter templates
************************************/

template <typename T, typename DefaultPusher, typename ValuePusher>
inline void getDenseArrayMultiHandleData(
    MArrayDataHandle& arrayHandle, unsigned int minSize, DefaultPusher defaultPusher,
    ValuePusher valuePusher
) {
    unsigned int prevIdx = 0;
    for (auto [index, handle] : MArrayInputDataHandleRange(arrayHandle)) {
        for (; prevIdx < index; ++prevIdx) {
            defaultPusher();
        }
        valuePusher(index, handle);
        prevIdx = index + 1;
    }

    // Fill up to the requested min size
    for (; prevIdx < minSize; ++prevIdx) {
        defaultPusher();
    }
}

template <typename T, typename ValueGetter = DefaultHandleValueGetter<T>>
inline void getDenseArrayHandleData(
    MArrayDataHandle& arrayHandle, std::vector<T>& ret, unsigned int minSize = 0,
    ValueGetter valueGetter = ValueGetter()
) {
    auto defaultPusher = [&ret]() { ret.emplace_back(); };
    auto valuePusher = [&ret, &valueGetter](unsigned int index, MDataHandle& handle) {
        ret.push_back(valueGetter(handle));
    };
    getDenseArrayMultiHandleData(arrayHandle, minSize, defaultPusher, valuePusher);
}

template <typename T, typename ValueGetter = DefaultHandleValueGetter<T>>
inline void getDenseArrayHandleData(
    MDataBlock& dataBlock, MObject& attr, std::vector<T>& ret, unsigned int minSize = 0,
    ValueGetter valueGetter = ValueGetter()

) {
    MArrayDataHandle arrayHandle = dataBlock.inputArrayValue(attr);
    getDenseArrayHandleData<T>(arrayHandle, ret, minSize, valueGetter);
}

template <typename T, typename ValueGetter = DefaultHandleValueGetter<T>>
inline void getDenseArrayChildHandleData(
    MArrayDataHandle& arrayHandle, MObject& childAttr, std::vector<T>& ret,
    unsigned int minSize = 0, ValueGetter valueGetter = ValueGetter()
) {
    auto childValueGetter = [&childAttr, &valueGetter](MDataHandle& h) {
        return valueGetter(h.child(childAttr));
    };
    getDenseArrayHandleData<T>(arrayHandle, ret, minSize, childValueGetter);
}

template <typename T, typename ValueGetter = DefaultHandleValueGetter<T>>
inline void getDenseArrayChildHandleData(
    MDataBlock& dataBlock, MObject& attr, MObject& childAttr, std::vector<T>& ret,
    unsigned int minSize = 0, ValueGetter valueGetter = ValueGetter()
) {
    MArrayDataHandle handle = dataBlock.inputArrayValue(attr);
    getDenseArrayChildHandleData<T>(handle, ret, childAttr, minSize, valueGetter);
}

/************************************
Sparse getter templates
************************************/
template <typename T, typename ValuePusher>
inline void getSparseArrayMultiHandleData(MArrayDataHandle& arrayHandle, ValuePusher valuePusher) {
    for (auto [index, handle] : MArrayInputDataHandleRange(arrayHandle)) {
        valuePusher(handle);
    }
}

template <typename T, typename ValueGetter = DefaultHandleValueGetter<T>>
inline void getSparseArrayHandleData(
    MArrayDataHandle& arrayHandle, std::unordered_map<unsigned int, T>& ret,
    ValueGetter valueGetter = ValueGetter()
) {
    auto valuePusher = [&ret, &valueGetter](unsigned int index, MDataHandle& handle) {
        ret[index] = valueGetter(handle);
    };
    getSparseArrayMultiHandleData(arrayHandle, valuePusher);
}

template <typename T, typename ValueGetter = DefaultHandleValueGetter<T>>
inline void getSparseArrayHandleData(
    MDataBlock& dataBlock, MObject& attr, std::unordered_map<unsigned int, T>& ret,
    ValueGetter valueGetter = ValueGetter()
) {
    MArrayDataHandle arrayHandle = dataBlock.inputArrayValue(attr);
    getSparseArrayHandleData<T>(arrayHandle, ret, valueGetter);
}

template <typename T, typename ValueGetter = DefaultHandleValueGetter<T>>
inline void getSparseArrayHandleData(
    MArrayDataHandle& arrayHandle, MObject& childAttr, std::unordered_map<unsigned int, T>& ret,
    ValueGetter valueGetter = ValueGetter()
) {
    auto childValueGetter = [&childAttr, &valueGetter](MDataHandle& h) {
        return valueGetter(h.child(childAttr));
    };
    getSparseArrayChildHandleData<T>(arrayHandle, ret, childValueGetter);
}

template <typename T, typename ValueGetter = DefaultHandleValueGetter<T>>
inline void getSparseArrayChildHandleData(
    MDataBlock& dataBlock, MObject& attr, std::unordered_map<unsigned int, T>& ret,
    ValueGetter valueGetter = ValueGetter()
) {
    MArrayDataHandle arrayHandle = dataBlock.inputArrayValue(attr);
    getSparseArrayChildHandleData<T>(arrayHandle, ret, valueGetter);
}

/************************************
Component getter templates
************************************/

/*
Get the vertex indices that are deformerd by *self
geomIndex is normally 0
This isn't for actualy *use*   It's just so I remember it exists

Returns
    bool: True if we're working on the whole mesh

Return By Ref
    affectMap: The list of indices we're affecting
    affectCount: The number of vertices we're affecting
*/
template <typename T>
bool getAffectedIndices(
    T* self,  // MPxGeometryFilter or MPxDeformerNode
    unsigned int geomIndex, MUintArray& affectMap, unsigned int& affectCount
) {
    MIndexMapper mapper = self->indexMapper(geomIndex);
    affectCount = mapper.affectCount();
    affectMap = mapper.affectMap();
    return mapper.isIdentityMap();
}
