#pragma once

#include "ed_shared_protocol/payloads.hpp"

namespace ed::shared_protocol {

constexpr std::uint32_t kMaxForwardSequenceGap = 1024;
constexpr std::size_t kRetiredEpochLimit = 32;
constexpr std::uint32_t kTelemetryStaleAfterMs = 750;

struct PeerEndpoint {
    std::uint32_t address = 0;
    std::uint16_t port = 0;
};

enum class SessionDecision : std::uint8_t {
    Accepted,
    SourceMismatch,
    SenderMismatch,
    Replay,
    Reordered,
    SequenceGap,
    RetiredEpoch,
};

class SessionTracker {
public:
    SessionTracker(const char* sender_id, PeerEndpoint endpoint);

    SessionDecision accept(const Frame& frame, PeerEndpoint source, std::uint32_t receipt_ms);
    bool stale(std::uint32_t now_ms) const;
    void reset();
    bool has_session() const { return has_epoch_; }
    std::uint64_t boot_epoch() const { return epoch_; }
    std::uint32_t last_sequence() const { return last_sequence_; }

private:
    char sender_id_[9]{};
    PeerEndpoint endpoint_{};
    std::uint64_t epoch_ = 0;
    std::uint32_t last_sequence_ = 0;
    std::uint32_t last_receipt_ms_ = 0;
    bool has_epoch_ = false;
    bool has_sequence_ = false;
    std::uint64_t retired_epochs_[kRetiredEpochLimit]{};
    std::size_t retired_count_ = 0;
};

class RouteValidator {
public:
    bool accept(bool start_event, RouteStage stage, bool complete);
    void reset();
    RouteStage stage() const { return stage_; }
    bool started() const { return started_; }

private:
    RouteStage stage_ = RouteStage::Start;
    bool have_stage_ = false;
    bool started_ = false;
};

}  // namespace ed::shared_protocol
