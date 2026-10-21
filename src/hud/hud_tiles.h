#pragma once

// 그린 자리(damage): 버퍼 행마다 가로 칸의 비트. 캔버스가 지울 칸과 화면(maix_display)이 옮길 칸을 같은
// 방식으로 훑는다. 칸은 31개까지라 맨 위 비트는 늘 비어 있다.

#include <cstdint>

// tiles의 이어진 칸마다 run(first, end)를 부른다(칸 [first, end)). 맨 위 비트가 비어 있어 이어진 칸의 끝을
// 셀 때 ctz에 0이 들어가지 않는다(정의되지 않는다).
template <typename Run>
inline void for_each_tile_run(uint32_t tiles, Run run) {
    while (tiles) {
        const int first = __builtin_ctz(tiles);
        const int end = first + __builtin_ctz(~(tiles >> first));
        run(first, end);
        tiles &= ~((1U << end) - (1U << first));
    }
}
