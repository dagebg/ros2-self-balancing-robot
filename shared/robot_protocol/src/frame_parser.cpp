#include "robot_protocol/frame_parser.hpp"

namespace robot::protocol {

std::optional<DecodeResult> FrameParser::push(std::uint8_t byte) noexcept {
  if (byte != kFrameDelimiter) {
    if (overflow_) {
      return std::nullopt;  // discard until the next delimiter
    }
    if (length_ < buffer_.size()) {
      buffer_[length_++] = byte;
    } else {
      overflow_ = true;
    }
    return std::nullopt;
  }

  if (overflow_) {
    reset();
    DecodeResult result;
    result.error = DecodeError::kOverflow;
    count(result.error);
    return result;
  }
  if (length_ == 0) {
    return std::nullopt;  // empty frame: resynchronization marker
  }
  DecodeResult result = decodeFrame(buffer_.data(), length_);
  length_ = 0;
  count(result.error);
  return result;
}

void FrameParser::reset() noexcept {
  length_ = 0;
  overflow_ = false;
}

void FrameParser::count(DecodeError error) noexcept {
  switch (error) {
    case DecodeError::kNone:
      ++stats_.frames_ok;
      break;
    case DecodeError::kCobs:
      ++stats_.cobs_errors;
      break;
    case DecodeError::kCrcMismatch:
      ++stats_.crc_errors;
      break;
    case DecodeError::kOverflow:
      ++stats_.overflows;
      break;
    case DecodeError::kTooShort:
    case DecodeError::kUnsupportedVersion:
    case DecodeError::kUnknownType:
    case DecodeError::kBadLength:
    case DecodeError::kBadValue:
      ++stats_.other_errors;
      break;
  }
}

std::uint8_t SequenceTracker::observe(std::uint8_t sequence) noexcept {
  ++received_;
  std::uint8_t missed = 0;
  if (has_last_) {
    missed = static_cast<std::uint8_t>(sequence - last_ - 1U);
    lost_ += missed;
  }
  last_ = sequence;
  has_last_ = true;
  return missed;
}

}  // namespace robot::protocol
