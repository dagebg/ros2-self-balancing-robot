#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "logging/logger.hpp"

namespace {

using robot::logging::Level;
using robot::logging::Logger;

class RecordingSink final : public robot::logging::Sink {
 public:
  void write(Level level, std::string_view message) override {
    records.emplace_back(level, std::string(message));
  }
  std::vector<std::pair<Level, std::string>> records;
};

TEST(LoggerTest, FormatsPrintfStyleMessages) {
  Logger log;
  RecordingSink sink;
  ASSERT_TRUE(log.addSink(sink));

  log.info("pitch=%.1f deg, ticks=%d", 2.5, 42);

  ASSERT_EQ(sink.records.size(), 1U);
  EXPECT_EQ(sink.records[0].first, Level::kInfo);
  EXPECT_EQ(sink.records[0].second, "pitch=2.5 deg, ticks=42");
}

TEST(LoggerTest, DropsMessagesBelowMinimumLevel) {
  Logger log;
  RecordingSink sink;
  log.addSink(sink);
  log.setMinLevel(Level::kWarn);

  log.debug("d");
  log.info("i");
  log.warn("w");
  log.error("e");

  ASSERT_EQ(sink.records.size(), 2U);
  EXPECT_EQ(sink.records[0].first, Level::kWarn);
  EXPECT_EQ(sink.records[1].first, Level::kError);
}

TEST(LoggerTest, DefaultMinimumLevelIsInfo) {
  Logger log;
  EXPECT_EQ(log.minLevel(), Level::kInfo);
}

TEST(LoggerTest, FansOutToEverySink) {
  Logger log;
  RecordingSink first;
  RecordingSink second;
  log.addSink(first);
  log.addSink(second);

  log.log(Level::kError, "boom");

  EXPECT_EQ(first.records.size(), 1U);
  EXPECT_EQ(second.records.size(), 1U);
}

TEST(LoggerTest, RejectsSinksBeyondCapacity) {
  Logger log;
  std::vector<RecordingSink> sinks(Logger::kMaxSinks + 1);
  for (std::size_t i = 0; i < Logger::kMaxSinks; ++i) {
    EXPECT_TRUE(log.addSink(sinks[i]));
  }
  EXPECT_FALSE(log.addSink(sinks.back()));
}

TEST(LoggerTest, TruncatesLongMessagesWithEllipsis) {
  Logger log;
  RecordingSink sink;
  log.addSink(sink);
  const std::string long_text(Logger::kMaxMessageLength + 50, 'x');

  log.info("%s", long_text.c_str());

  ASSERT_EQ(sink.records.size(), 1U);
  const std::string& message = sink.records[0].second;
  EXPECT_EQ(message.size(), Logger::kMaxMessageLength);
  EXPECT_EQ(message.substr(message.size() - 3), "...");
}

TEST(LoggerTest, MessageOfExactlyMaxLengthIsNotTruncated) {
  Logger log;
  RecordingSink sink;
  log.addSink(sink);
  const std::string text(Logger::kMaxMessageLength, 'y');

  log.info("%s", text.c_str());

  ASSERT_EQ(sink.records.size(), 1U);
  EXPECT_EQ(sink.records[0].second, text);
}

TEST(LoggerTest, LevelTagsAreThreeCharacters) {
  for (const Level level : {Level::kDebug, Level::kInfo, Level::kWarn, Level::kError}) {
    EXPECT_EQ(robot::logging::toString(level).size(), 3U);
  }
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
