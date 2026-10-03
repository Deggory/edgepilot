/* 알림음 합성: 모든 알림이 0에서 시작해 0으로 끝나고(딸깍 소리 없음), 봉우리가 같고, 길이가
 * 알림답게 짧으며, 서로 다른 소리인지 본다. */
#include "alert_tones.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <vector>

namespace {

constexpr int kRate = 48000;

TEST(AlertTones, ShortCleanAndDistinct) {
    std::vector<std::vector<int16_t>> tones;
    for (int i = 0; i < static_cast<int>(AlertSoundId::count); ++i) {
        const AlertSoundId id = static_cast<AlertSoundId>(i);
        const std::vector<int16_t> pcm = render_alert_tone(id, kRate);
        ASSERT_FALSE(pcm.empty()) << alert_sound_name(id);
        const double seconds = static_cast<double>(pcm.size()) / kRate;
        EXPECT_GT(seconds, 0.3) << alert_sound_name(id);
        EXPECT_LT(seconds, 2.5) << alert_sound_name(id);
        // 무음에서 서서히 올라가 무음으로 끝나 스피커가 딸깍거리지 않는다: 첫 샘플과 끝 샘플은 0,
        // 첫 0.25 ms는 풀스케일의 2% 아래
        EXPECT_EQ(pcm.front(), 0) << alert_sound_name(id);
        for (int s = 0; s < kRate / 4000; ++s) EXPECT_LT(std::abs(pcm[s]), 655) << alert_sound_name(id);
        EXPECT_EQ(pcm.back(), 0) << alert_sound_name(id);
        int peak = 0;
        for (int16_t s : pcm) peak = std::max(peak, std::abs(static_cast<int>(s)));
        EXPECT_NEAR(peak, kAlertTonePeak * 32767, 2) << alert_sound_name(id);
        tones.push_back(pcm);
    }
    for (size_t a = 0; a < tones.size(); ++a)
        for (size_t b = a + 1; b < tones.size(); ++b) EXPECT_NE(tones[a], tones[b]);
    EXPECT_TRUE(render_alert_tone(AlertSoundId::count, kRate).empty());
}

}  // namespace
