#include "ed_shared_protocol/session.hpp"

#include <cstring>

namespace ed::shared_protocol {
namespace {

bool same_endpoint(PeerEndpoint left, PeerEndpoint right) {
    return left.address == right.address && left.port == right.port;
}

bool same_sender(const char left[9], const char right[9]) {
    return std::strncmp(left, right, 8) == 0;
}

bool retired(const std::uint64_t* epochs, std::size_t count, std::uint64_t epoch) {
    for (std::size_t index = 0; index < count; ++index) {
        if (epochs[index] == epoch) return true;
    }
    return false;
}

}  // namespace

SessionTracker::SessionTracker(const char* sender_id, PeerEndpoint endpoint) : endpoint_(endpoint) {
    if (sender_id != nullptr) std::strncpy(sender_id_, sender_id, sizeof(sender_id_) - 1);
}

SessionDecision SessionTracker::accept(const Frame& frame, PeerEndpoint source, std::uint32_t receipt_ms) {
    if (!same_endpoint(source, endpoint_)) return SessionDecision::SourceMismatch;
    if (!same_sender(frame.sender_id, sender_id_)) return SessionDecision::SenderMismatch;
    if (!has_epoch_) {
        epoch_ = frame.boot_epoch;
        has_epoch_ = true;
        has_sequence_ = false;
    } else if (frame.boot_epoch != epoch_) {
        if (retired(retired_epochs_, retired_count_, frame.boot_epoch)) return SessionDecision::RetiredEpoch;
        if (retired_count_ < kRetiredEpochLimit) retired_epochs_[retired_count_++] = epoch_;
        else {
            std::memmove(retired_epochs_, retired_epochs_ + 1, (kRetiredEpochLimit - 1) * sizeof(std::uint64_t));
            retired_epochs_[kRetiredEpochLimit - 1] = epoch_;
        }
        epoch_ = frame.boot_epoch;
        has_sequence_ = false;
    }
    if (has_sequence_) {
        const std::uint32_t delta = frame.sequence - last_sequence_;
        if (delta == 0) return SessionDecision::Replay;
        if (delta >= 0x80000000U) return SessionDecision::Reordered;
        if (delta > kMaxForwardSequenceGap) return SessionDecision::SequenceGap;
    }
    last_sequence_ = frame.sequence;
    last_receipt_ms_ = receipt_ms;
    has_sequence_ = true;
    return SessionDecision::Accepted;
}

bool SessionTracker::stale(std::uint32_t now_ms) const {
    return !has_sequence_ || static_cast<std::uint32_t>(now_ms - last_receipt_ms_) > kTelemetryStaleAfterMs;
}

void SessionTracker::reset() {
    epoch_ = 0;
    last_sequence_ = 0;
    last_receipt_ms_ = 0;
    has_epoch_ = false;
    has_sequence_ = false;
    retired_count_ = 0;
}

bool RouteValidator::accept(bool start_event, RouteStage stage, bool complete) {
    if (complete != (stage == RouteStage::Complete)) return false;
    if (start_event && (started_ || stage != RouteStage::Start)) return false;
    if (!have_stage_) {
        if (stage != RouteStage::Start) return false;
        have_stage_ = true;
        stage_ = stage;
        started_ = start_event;
        return true;
    }
    if (start_event) return false;
    if (stage == stage_) return true;
    if (!started_ || static_cast<std::uint8_t>(stage) != static_cast<std::uint8_t>(stage_) + 1) return false;
    stage_ = stage;
    return true;
}

void RouteValidator::reset() {
    stage_ = RouteStage::Start;
    have_stage_ = false;
    started_ = false;
}

}  // namespace ed::shared_protocol
