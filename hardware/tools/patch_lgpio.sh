#!/usr/bin/env bash
# lgpio EINVAL 무한 스핀 패치 — Pi에서 실행 (lgpio_patch.md "2026-08-25" 절 참고)
#
# 무엇을 고치나: liblgpio의 송출 스레드(lgPthTx.c)는 다음 깨어날 시각을
#   pthTxDelayMicros로 계산하는데, 큐 소진 이벤트 시 이 값이 음수가 될 수 있다.
#   음수면 timespec.tv_nsec이 음수가 되고 clock_nanosleep이 EINVAL을 즉시
#   반환하는데, while(clock_nanosleep(...))이 같은 인자로 무한 재시도해서
#   송출 스레드가 100% CPU 스핀에 빠진다 → 전 핀의 펄스 송출이 영구 정지.
#   (실기 확정 2026-07-16, 재확정 2026-08-25: gdb 스택 lgPthTx→clock_nanosleep
#    + top에서 해당 스레드 R 99.9%)
#
# 두 겹으로 고친다 (각각 단독으로도 충분):
#   ① 딜레이 음수 → 0 클램프 (원인 차단)
#   ② sleep 루프를 EINTR만 재시도로 바꾸고, 그 외 오류면 현재 시각으로
#      재동기화 후 계속 (미지의 경로까지 복구 보장)
#
# 설치 방식: 시스템 파일은 하나도 수정하지 않는다. 패치 빌드본을
#   /usr/local/lib에 설치하면 ldconfig 우선순위로 시스템 구본
#   (/lib/aarch64-linux-gnu/liblgpio.so.1, gdb로 확인된 실제 로드 경로)보다
#   먼저 로드된다. venv를 새로 만들어도 패치가 유지된다.
#
# 되돌리기 (원본 백업 불필요 — 시스템 파일 무수정이므로 사본 삭제가 곧 원복):
#   sudo rm -f /usr/local/lib/liblgpio.* /usr/local/lib/liblgrpio.*
#   sudo ldconfig
#
# 사용법 (Pi에서, verify 스크립트를 돌리는 그 venv를 활성화한 채):
#   bash hardware/tools/patch_lgpio.sh [소스디렉터리=~/lg-src]
# 재실행해도 안전하다(이미 패치된 소스는 건너뜀). 기록은 $SRC_DIR/patch_install.log.
set -euo pipefail

SRC_DIR="${1:-$HOME/lg-src}"
MARKER="lgpio-einval-clamp-20260825"
# joan2937/lg master @ 2026-02-24 (패치 검증 기준 커밋 — 올릴 때는 lgPthTx.c 재확인)
PIN="bcccd782eceedc5b278b3056ea81d5fbbb89c489"
LOG="$SRC_DIR/patch_install.log"

resolve_loaded_lib() {
    python3 - <<'PYEOF'
import os
try:
    import lgpio
except Exception:
    print("")
else:
    for line in open(f"/proc/{os.getpid()}/maps"):
        if "liblgpio" in line:
            print(line.split()[-1])
            break
PYEOF
}

echo "== 0/5 현재 상태 기록"
BEFORE=$(resolve_loaded_lib)
PKG_VER=$(dpkg-query -W liblgpio1 2>/dev/null || echo "liblgpio1 (dpkg 미설치)")
echo "   설치 전 로드 경로: ${BEFORE:-<lgpio import 실패>}"
echo "   시스템 패키지: $PKG_VER"

echo "== 1/5 소스 준비: $SRC_DIR (커밋 고정 $PIN)"
if [ ! -d "$SRC_DIR/.git" ]; then
    git init -q "$SRC_DIR"
    git -C "$SRC_DIR" fetch -q --depth 1 https://github.com/joan2937/lg "$PIN"
    git -C "$SRC_DIR" checkout -q FETCH_HEAD
fi
HEAD=$(git -C "$SRC_DIR" rev-parse HEAD)
if [ "$HEAD" != "$PIN" ]; then
    echo "   ✘ $SRC_DIR 의 커밋($HEAD)이 고정 커밋과 다름."
    echo "     rm -rf $SRC_DIR 후 재실행할 것 (로컬 수정이 있다면 먼저 확인)."
    exit 1
fi

echo "== 2/5 lgPthTx.c 패치"
python3 - "$SRC_DIR/lgPthTx.c" "$MARKER" <<'PYEOF'
import re, sys

path, marker = sys.argv[1], sys.argv[2]
src = open(path).read()

if marker in src:
    print("   이미 패치됨 — 건너뜀")
    sys.exit(0)

# ① 딜레이 클램프: tv_nsec에 더하기 직전에 음수를 0으로
pat_add = re.compile(r"(\n([ \t]*)pthTxReq\.tv_nsec\s*\+=\s*\(pthTxDelayMicros\s*\*\s*1000\)\s*;)")
n = len(pat_add.findall(src))
if n != 1:
    sys.exit(f"   실패: 'tv_nsec += (pthTxDelayMicros * 1000)' 매칭 {n}개 (정확히 1개여야 함) — 수동 확인 필요")
src = pat_add.sub(
    r"\n\g<2>if (pthTxDelayMicros < 0) pthTxDelayMicros = 0; /* %s */\g<1>" % marker,
    src, count=1)

# ② sleep 루프: EINTR만 재시도, 그 외(EINVAL 등)는 현재 시각으로 재동기화
pat_sleep = re.compile(
    r"([ \t]*)while\s*\(\s*clock_nanosleep\s*\(\s*\n?"
    r"\s*CLOCK_MONOTONIC\s*,\s*TIMER_ABSTIME\s*,\s*&pthTxReq\s*,\s*NULL\s*\)\s*\)\s*;")
matches = list(pat_sleep.finditer(src))
if len(matches) != 1:
    sys.exit(f"   실패: 'while (clock_nanosleep(...));' 매칭 {len(matches)}개 (정확히 1개여야 함) — 수동 확인 필요")
m = matches[0]
ind = m.group(1)
repl = "\n".join([
    f"{ind}{{ /* {marker}: retry only on EINTR, resync otherwise */",
    f"{ind}   int sleepErr;",
    f"{ind}   while ((sleepErr = clock_nanosleep(",
    f"{ind}      CLOCK_MONOTONIC, TIMER_ABSTIME, &pthTxReq, NULL)))",
    f"{ind}   {{",
    f"{ind}      if (sleepErr != EINTR)",
    f"{ind}      {{",
    f"{ind}         clock_gettime(CLOCK_MONOTONIC, &pthTxReq);",
    f"{ind}         break;",
    f"{ind}      }}",
    f"{ind}   }}",
    f"{ind}}}",
])
src = src[:m.start()] + repl + src[m.end():]

# EINTR에 필요한 errno.h 보장
if not re.search(r"#include\s*<errno\.h>", src):
    src = src.replace("#include", "#include <errno.h>\n#include", 1)

# 설치본(.so)에서 패치 여부를 확인할 수 있는 마커 문자열
src += f'\n\nconst char lgpio_patch_marker[] __attribute__((used)) = "{marker}";\n'

open(path, "w").write(src)

patched = open(path).read()
assert "pthTxDelayMicros < 0" in patched and "sleepErr != EINTR" in patched
print("   패치 완료 (클램프 + sleep 재동기화 + 마커)")
PYEOF

echo "== 3/5 빌드·설치 (/usr/local/lib — 시스템 파일 무수정)"
make -C "$SRC_DIR" -j"$(nproc)"
sudo make -C "$SRC_DIR" install
sudo ldconfig

echo "== 4/5 실제 로드되는 라이브러리가 패치본인지 확인"
AFTER=$(resolve_loaded_lib)
if [ -z "$AFTER" ]; then
    echo "   ✘ 이 python에서 lgpio를 import하지 못함."
    echo "     verify 스크립트를 돌리는 venv를 활성화하고 재실행할 것."
    exit 1
fi
echo "   설치 후 로드 경로: $AFTER"
if ! strings "$AFTER" | grep -q "$MARKER"; then
    echo "   ✘ 아직 미패치 구본이 로드된다 — /usr/local/lib이 우선순위에서 밀림."
    echo "     시스템 파일을 옮기거나 지우지 말 것. 아래 진단 정보로 원인 확인:"
    echo "     --- ldconfig -p | grep lgpio ---"
    ldconfig -p | grep -i lgpio || true
    echo "     --- /etc/ld.so.conf.d 순서 ---"
    cat /etc/ld.so.conf /etc/ld.so.conf.d/*.conf 2>/dev/null || true
    echo "     해결은 lgpio_patch.md의 수동 절차(dpkg-divert) 참고."
    exit 1
fi
echo "   ✔ 패치본이 로드된다"

echo "== 5/5 기록: $LOG"
{
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $MARKER"
    echo "  commit:       $PIN"
    echo "  system pkg:   $PKG_VER"
    echo "  loaded(전):   ${BEFORE:-<import 실패>}"
    echo "  loaded(후):   $AFTER"
} >> "$LOG"
cat "$LOG"
echo ""
echo "완료. 재검증: verify_E2E_v2를 20초 이상(BLE 전원/모드 토글 포함) 돌려 정지 재현 여부 확인."
echo "재현 시 'top -H'에서 FF 스레드가 다시 99% 스핀이면 패치 무효 — 4/5 확인부터 다시."
