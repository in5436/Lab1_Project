set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/docs/metrics/lab3"

mkdir -p "$OUT"
cd "$ROOT"

make clean >/dev/null

if ! make build > "$OUT/build-memory.log" 2>&1; then
    tail -n 60 "$OUT/build-memory.log"
    exit 1
fi

{
    printf 'Git commit: %s\n' "$(git rev-parse --short HEAD)"

    if test -n "$(git status --porcelain --untracked-files=no)"; then
        echo 'Tracked worktree: MODIFIED'
    else
        echo 'Tracked worktree: CLEAN'
    fi

    echo '=== LINKER MEMORY USAGE ==='
    grep -A 3 'Memory region' "$OUT/build-memory.log"

    docker run --rm \
        --user "$(id -u):$(id -g)" \
        -v "$ROOT:/workspace" \
        -w /workspace/firmware \
        rack-monitor-build:local sh -eu -c '

            echo "=== ELF SHA256 ==="
            sha256sum build/firmware.elf

            echo "=== SIZE ==="
            arm-none-eabi-size build/firmware.elf

            echo "=== SECTIONS ==="
            arm-none-eabi-size -A build/firmware.elf

            echo "=== LARGEST OBJECT FILES ==="
            arm-none-eabi-size build/*.o |
                sort -k4,4n | tail -15

            echo "=== LARGEST LINKED SYMBOLS ==="
            arm-none-eabi-nm -S --size-sort --radix=d \
                build/firmware.elf | tail -20

            echo "=== LARGEST DATA/BSS SYMBOLS ==="
            arm-none-eabi-nm -S --size-sort --radix=d \
                build/firmware.elf |
                grep -E " [bBdD] " | tail -10 || true

            echo "=== LARGEST STATIC STACK FRAMES ==="
            test -f build/main.su
            cat build/*.su | sort -k2,2n | tail -20

            echo "=== LINK MAP ==="
            echo "firmware/build/firmware.map"
        '

} > "$OUT/memory-build.txt"

printf 'Saved %s and %s\n' \
    "$OUT/build-memory.log" \
    "$OUT/memory-build.txt"