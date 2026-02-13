//
// Created by ww on 2026/2/13.
//

#ifndef YANDY_ARM_NBUF_HPP
#define YANDY_ARM_NBUF_HPP

#include <atomic>
#include <cstddef>
#include <new>
#include <utility>
#include <optional>

namespace yandy
{
    // SPMC N-Buffer Class (Standard C++ implementation)
    template<typename T, size_t N>
    requires std::is_standard_layout_v<T> && (N >= 2)
    class NBuf {
    public:
        NBuf() = default;

        NBuf(const NBuf&) = delete;
        NBuf& operator=(const NBuf&) = delete;
        NBuf(NBuf&& other) = delete;
        NBuf& operator=(NBuf&& other) = delete;

        void write(const T& data) noexcept
        {
            auto next_slot = (m_next_write_idx + 1) % N;
            auto& slot = m_slots[next_slot];
            
            slot.version.fetch_add(1, std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            slot.data = data;
            std::atomic_thread_fence(std::memory_order_seq_cst);
            slot.version.fetch_add(1, std::memory_order_release);

            m_latest_idx.store(next_slot, std::memory_order_release);
            m_next_write_idx = next_slot;
        }

        void write(T&& data) noexcept
        {
            auto next_slot = (m_next_write_idx + 1) % N;
            auto& slot = m_slots[next_slot];
            
            slot.version.fetch_add(1, std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            slot.data = std::move(data);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            slot.version.fetch_add(1, std::memory_order_release);

            m_latest_idx.store(next_slot, std::memory_order_release);
            m_next_write_idx = next_slot;
        }

        template <typename Func>
        void manipulate(const Func& func)
        {
            auto next_slot = (m_next_write_idx + 1) % N;
            auto& slot = m_slots[next_slot];

            slot.version.fetch_add(1, std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            func(slot.data);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            slot.version.fetch_add(1, std::memory_order_release);

            m_latest_idx.store(next_slot, std::memory_order_release);
            m_next_write_idx = next_slot;
        }

        std::optional<T> try_read() const noexcept
        {
            T out_data;
            auto& slot = m_slots[m_latest_idx.load(std::memory_order_acquire)];
            auto v1 = slot.version.load(std::memory_order_acquire);

            std::atomic_thread_fence(std::memory_order_seq_cst);
            out_data = slot.data;
            std::atomic_thread_fence(std::memory_order_seq_cst);

            auto v2 = slot.version.load(std::memory_order_acquire);
            return (v1 == v2) ? std::make_optional(out_data) : std::nullopt;
        }

        T read() const noexcept
        {
            T copy{};
            auto& slot = m_slots[m_latest_idx.load(std::memory_order_acquire)];
            size_t v1, v2;
            do {
                v1 = slot.version.load(std::memory_order_acquire);

                // 检查是否正在写入
                if (v1 & 1) {
                    // 在实际应用中，这里可能需要短暂等待
                    // 但在用户态程序中，我们可以简单地继续循环
                    continue;
                }

                std::atomic_thread_fence(std::memory_order_seq_cst);
                copy = slot.data;
                std::atomic_thread_fence(std::memory_order_seq_cst);

                v2 = slot.version.load(std::memory_order_acquire);
            } while (v1 != v2);
            
            return copy;
        }

    private:
        struct alignas(std::hardware_destructive_interference_size) Slot
        {
            mutable std::atomic<size_t> version{0};
            T data;
        };

        std::array<Slot, N> m_slots;
        mutable std::atomic<size_t> m_latest_idx{0};
        size_t m_next_write_idx{0};
    };
}

#endif //YANDY_ARM_NBUF_HPP
