"""把 128x64 开机动画图片量化并编码成供 TC387 使用的 RLE 头文件。"""

from pathlib import Path
import sys

from PIL import Image


PALETTE = (
    (255, 255, 255),  # 白色背景
    (101, 47, 36),    # 深棕轮廓
    (218, 174, 131),  # 毛色
    (238, 208, 174),  # 浅毛色/抗锯齿
    (45, 105, 190),   # 蓝色眼泪
    (189, 139, 106),  # 棕色抗锯齿
)

FRAME_SIZE = (112, 122)


def rgb565(color: tuple[int, int, int]) -> int:
    r, g, b = color
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def nearest_palette(pixel: tuple[int, int, int]) -> int:
    # JPEG 白底有轻微噪点，先直接吸到纯白，压缩率会高很多。
    if min(pixel) >= 242:
        return 0
    return min(
        range(len(PALETTE)),
        key=lambda i: sum((pixel[c] - PALETTE[i][c]) ** 2 for c in range(3)),
    )


def content_bbox(image: Image.Image) -> tuple[int, int, int, int]:
    pixels = image.load()
    xs: list[int] = []
    ys: list[int] = []
    for y in range(image.height):
        for x in range(image.width):
            if min(pixels[x, y]) < 242:
                xs.append(x)
                ys.append(y)
    if not xs:
        return (0, 0, image.width, image.height)
    return (min(xs), min(ys), max(xs) + 1, max(ys) + 1)


def prepare_frame(path: Path, crop: tuple[int, int, int, int]) -> Image.Image:
    image = Image.open(path).convert("RGB").crop(crop)
    # 固定裁剪框确保动画不抖；contain 保持小猫比例，并留少量白边防轮廓贴边。
    canvas = Image.new("RGB", FRAME_SIZE, "white")
    image.thumbnail((FRAME_SIZE[0] - 4, FRAME_SIZE[1] - 4), Image.Resampling.LANCZOS)
    canvas.paste(image, ((FRAME_SIZE[0] - image.width) // 2,
                         (FRAME_SIZE[1] - image.height) // 2))
    return canvas


def encode_frame(path: Path, crop: tuple[int, int, int, int]) -> list[int]:
    image = prepare_frame(path, crop)
    pixels = image.load()
    indexes = [
        nearest_palette(pixels[x, y])
        for y in range(image.height)
        for x in range(image.width)
    ]
    encoded: list[int] = []
    start = 0
    while start < len(indexes):
        color = indexes[start]
        count = 1
        while start + count < len(indexes) and indexes[start + count] == color and count < 255:
            count += 1
        encoded.extend((count, color))
        start += count
    return encoded


def frame_number(path: Path) -> int:
    return int(path.stem.split("-")[-1])


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: generate_boot_anim.py INPUT_DIR OUTPUT_HEADER")
    source = Path(sys.argv[1])
    output = Path(sys.argv[2])
    frames = sorted(source.glob("frame-*.jpg"), key=frame_number)
    if len(frames) != 30:
        raise ValueError(f"expected 30 frames, found {len(frames)}")

    # 30 帧取共同包围盒，避免逐帧自动裁剪造成主体忽大忽小。再外扩约 4%。
    boxes = [content_bbox(Image.open(frame).convert("RGB")) for frame in frames]
    left = min(box[0] for box in boxes)
    top = min(box[1] for box in boxes)
    right = max(box[2] for box in boxes)
    bottom = max(box[3] for box in boxes)
    first = Image.open(frames[0])
    pad_x = max(4, (right - left) // 25)
    pad_y = max(4, (bottom - top) // 25)
    crop = (max(0, left - pad_x), max(0, top - pad_y),
            min(first.width, right + pad_x), min(first.height, bottom + pad_y))

    data: list[int] = []
    offsets = [0]
    for frame in frames:
        data.extend(encode_frame(frame, crop))
        offsets.append(len(data))

    lines = [
        "/* 由 tools/generate_boot_anim.py 生成，请勿手改。 */",
        "#ifndef KART_BOOT_ANIM_DATA_H_",
        "#define KART_BOOT_ANIM_DATA_H_",
        "",
        f"#define KART_BOOT_FRAME_W      ({FRAME_SIZE[0]}u)",
        f"#define KART_BOOT_FRAME_H      ({FRAME_SIZE[1]}u)",
        f"#define KART_BOOT_FRAME_COUNT  ({len(frames)}u)",
        "",
        "static const uint16 kart_boot_palette[] =",
        "{",
        "    " + ", ".join(f"0x{rgb565(c):04X}u" for c in PALETTE),
        "};",
        "",
        "static const uint32 kart_boot_frame_offsets[] =",
        "{",
    ]
    for i in range(0, len(offsets), 8):
        lines.append("    " + ", ".join(f"{v}u" for v in offsets[i : i + 8]) + ",")
    lines.extend(("};", "", "static const uint8 kart_boot_rle[] =", "{"))
    for i in range(0, len(data), 24):
        lines.append("    " + ", ".join(str(v) for v in data[i : i + 24]) + ",")
    lines.extend(("};", "", "#endif", ""))
    output.write_text("\n".join(lines), encoding="utf-8", newline="\n")
    print(f"{len(frames)} frames: {len(data)} RLE bytes -> {output}")


if __name__ == "__main__":
    main()
