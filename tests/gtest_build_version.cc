// 빌드 버전(cmake/build_version.cmake): 실행 파일 안의 표식 바로 뒤가 edgepilot_version()이고, 웹 콘솔
// (process_status.py)이 그대로 보여 줄 수 있는 64자 이하의 글자만 쓴다.
#include "common/build_version.h"

#include <gtest/gtest.h>

#include <cctype>
#include <cstring>
#include <string>

TEST(BuildVersion, MarkerPrecedesTheVersion) {
    const std::string version = edgepilot_version();
    EXPECT_EQ(std::string(kBuildVersionMarker), EDGEPILOT_VERSION_MARKER_PREFIX + version);
    EXPECT_EQ(edgepilot_version(), kBuildVersionMarker + std::strlen(EDGEPILOT_VERSION_MARKER_PREFIX));
}

TEST(BuildVersion, PrintableAndShort) {
    const std::string version = edgepilot_version();
    ASSERT_FALSE(version.empty());
    EXPECT_LE(version.size(), 64u);
    for (const char c : version)
        EXPECT_TRUE(std::isalnum(static_cast<unsigned char>(c)) || std::strchr("._+-", c)) << version;
}
