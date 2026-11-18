#!/bin/bash
set -e

ROOT_DIR="$(pwd)"
OUT_DIR="$ROOT_DIR/out"
DTS_DIR="$OUT_DIR/arch/arm64/boot/dts/samsung"
A23_DTS_SUBDIR="arch/arm64/boot/dts/samsung/a23/a23"
MKDTIMG="$ROOT_DIR/tools/mkdtimg"
DTC_EXT_BIN="$ROOT_DIR/tools/dtc"
MKBOOTIMG="$ROOT_DIR/mkbootimg/mkbootimg.py"
UNPACK_BOOTIMG="$ROOT_DIR/mkbootimg/unpack_bootimg.py"

build_kernel() {
    echo "-----------------------------------------------"
    echo "Building kernel..."
    echo "-----------------------------------------------"

    export ARCH=arm64
    export SUBARCH=arm64
    export PATH="$ROOT_DIR/llvm-21/bin:$PATH"

    if [ ! -f "$DTC_EXT_BIN" ]; then
        echo "ERROR: tools/dtc tidak ditemukan (dibutuhkan untuk overlay)."
        exit 1
    fi
    chmod +x "$DTC_EXT_BIN"

    mkdir -p "$OUT_DIR"

    BUILD_VAR=(
        -j"$(nproc)"
        -C "$ROOT_DIR"
        O="$OUT_DIR"
        ARCH=arm64
        CROSS_COMPILE=aarch64-linux-gnu-
        LLVM=1
        LLVM_IAS=1
        KCFLAGS=-w
        DTC_EXT="$DTC_EXT_BIN"
        CONFIG_BUILD_ARM64_DT_OVERLAY=y
    )

    TEMP_DEFCONFIG="$ROOT_DIR/arch/arm64/configs/temp_defconfig"

    cp "$ROOT_DIR/arch/arm64/configs/a23_eur_open_defconfig" \
       "$TEMP_DEFCONFIG"

    cat >> "$TEMP_DEFCONFIG" <<'EOF'

CONFIG_THINLTO=y
# CONFIG_LTO_NONE is not set
CONFIG_LTO_CLANG=y
EOF

    make "${BUILD_VAR[@]}" temp_defconfig
    rm -f "$TEMP_DEFCONFIG"

    make "${BUILD_VAR[@]}"

    if [ ! -s "$OUT_DIR/arch/arm64/boot/Image" ]; then
        echo "ERROR: Kernel Image tidak ditemukan."
        exit 1
    fi

    echo "Kernel berhasil dibangun."
    ls -lh "$OUT_DIR/arch/arm64/boot/Image"
}

build_dtb() {
    echo "-----------------------------------------------"
    echo "Building khaje.dtb and overlay A23..."
    echo "-----------------------------------------------"

    make "${BUILD_VAR[@]}" "$A23_DTS_SUBDIR/"
}

build_dtbo() {
    echo "-----------------------------------------------"
    echo "Building dtbo.img..."
    echo "-----------------------------------------------"

    if [ ! -f "$MKDTIMG" ]; then
        echo "ERROR: tools/mkdtimg tidak ditemukan."
        exit 1
    fi

    mapfile -t DTBO_FILES < <(find "$DTS_DIR/a23/a23" -maxdepth 1 \
        -name 'a23_eur_open_*.dtbo' | sort)

    if [ "${#DTBO_FILES[@]}" -eq 0 ]; then
        echo "ERROR: overlay a23_eur_open_*.dtbo tidak ditemukan di:"
        echo "$DTS_DIR/a23/a23"
        exit 1
    fi

    printf 'Overlay: %s\n' "${DTBO_FILES[@]##*/}"

    chmod +x "$MKDTIMG"

    "$MKDTIMG" create "$OUT_DIR/dtbo.img" \
        --page_size=4096 "${DTBO_FILES[@]}"

    cp "$OUT_DIR/dtbo.img" "$ROOT_DIR/dtbo.img"

    echo "dtbo.img selesai dibuat."
    ls -lh "$ROOT_DIR/dtbo.img"
}

build_boot() {
    echo "-----------------------------------------------"
    echo "Building boot.img..."
    echo "-----------------------------------------------"

    BASE_BOOT="$ROOT_DIR/boot-stock.img"
    OUT_KERNEL="$OUT_DIR/arch/arm64/boot/Image"
    UNPACK_DIR="$OUT_DIR/boot-stock-unpacked"

    for f in "$MKBOOTIMG" "$UNPACK_BOOTIMG"; do
        if [ ! -f "$f" ]; then
            echo "ERROR: $f tidak ditemukan."
            echo "Jalankan: git submodule update --init"
            exit 1
        fi
    done

    if [ ! -s "$BASE_BOOT" ]; then
        echo "ERROR: boot-stock.img tidak ditemukan."
        echo "Sediakan boot.img stock SM-A235F yang sesuai di root repo."
        exit 1
    fi

    if [ ! -s "$OUT_KERNEL" ]; then
        echo "ERROR: Kernel Image tidak ditemukan."
        exit 1
    fi

    rm -rf "$UNPACK_DIR"
    mkdir -p "$UNPACK_DIR"
    python3 "$UNPACK_BOOTIMG" --boot_img "$BASE_BOOT" \
        --out "$UNPACK_DIR" --format=mkbootimg > "$OUT_DIR/boot-stock.args"
    eval "STOCK_ARGS=( $(cat "$OUT_DIR/boot-stock.args") )"

    echo "Parameter dari boot stock:"
    cat "$OUT_DIR/boot-stock.args"

    BOOT_ARGS=()
    i=0
    while [ "$i" -lt "${#STOCK_ARGS[@]}" ]; do
        opt="${STOCK_ARGS[$i]}"
        val="${STOCK_ARGS[$((i + 1))]}"
        case "$opt" in
            --kernel)
                val="$OUT_KERNEL"
                ;;
            --ramdisk)
                [ -s "$ROOT_DIR/boot/ramdisk" ] && val="$ROOT_DIR/boot/ramdisk"
                ;;
            --dtb)
                [ -s "$ROOT_DIR/boot/dtb" ] && val="$ROOT_DIR/boot/dtb"
                ;;
        esac
        BOOT_ARGS+=("$opt" "$val")
        i=$((i + 2))
    done

    python3 "$MKBOOTIMG" "${BOOT_ARGS[@]}" --output "$OUT_DIR/boot.img"

    cp "$OUT_DIR/boot.img" "$ROOT_DIR/boot.img"

    echo "boot.img selesai dibuat."
    ls -lh "$ROOT_DIR/boot.img"
}

if [ "${SKIP_KERNEL:-0}" != "1" ]; then
    build_kernel
else
    export ARCH=arm64 SUBARCH=arm64
    export PATH="$ROOT_DIR/llvm-21/bin:$PATH"
    BUILD_VAR=(
        -j"$(nproc)" -C "$ROOT_DIR" O="$OUT_DIR" ARCH=arm64
        CROSS_COMPILE=aarch64-linux-gnu- LLVM=1 LLVM_IAS=1 KCFLAGS=-w
        DTC_EXT="$DTC_EXT_BIN" CONFIG_BUILD_ARM64_DT_OVERLAY=y
    )
fi
build_dtb
build_dtbo
build_boot

echo "-----------------------------------------------"
echo "Build Done."
echo "-----------------------------------------------"
