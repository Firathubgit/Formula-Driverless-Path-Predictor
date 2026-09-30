"""Arrange Blender review frames for a four-car visual sign-off."""

from argparse import ArgumentParser
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


CARS = [
    ("amr23", "ASTON MARTIN  /  AMR23"),
    ("rb19", "ORACLE RED BULL  /  RB19"),
    ("jesko", "KOENIGSEGG  /  JESKO ATTACK"),
    ("urus", "LAMBORGHINI  /  URUS PERFORMANTE"),
]
SHOTS = [(12, "REAR / LAMPS"), (25, "WHEEL / SURFACE"),
         (44, "FRONT / LIGHTS"), (73, "SELECTION POSE")]


def main():
    parser = ArgumentParser()
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--environment", action="store_true", help="Four large selection poses in a 2x2 sheet")
    parser.add_argument("--cinematic", action="store_true", help="Entrance details, hero and reserved selection angle")
    args = parser.parse_args()
    tile_w, tile_h, label_h = (960, 540, 54) if args.environment else (480, 270, 44)
    columns, rows = (2, 2) if args.environment else (4, 4)
    sheet = Image.new("RGB", (tile_w * columns, (tile_h + label_h) * rows), "#101216")
    draw = ImageDraw.Draw(sheet)
    font_path = Path("C:/Windows/Fonts/segoeui.ttf")
    font = ImageFont.truetype(str(font_path), 26 if args.environment else 18) if font_path.is_file() else ImageFont.load_default()
    for index, (car, title) in enumerate(CARS):
        shots = [(12, 'DETAIL 1'), (36, 'DETAIL 2'), (73, 'HERO / IDLE'), (185, 'SELECT / NEW ANGLE')] if args.cinematic else SHOTS
        for col, (frame, label) in enumerate([(73, title)] if args.environment else shots):
            source = args.frames / car / f"{frame:04d}.png"
            with Image.open(source) as raw:
                image = raw.convert("RGB").resize((tile_w, tile_h), Image.Resampling.LANCZOS)
            row = index // 2 if args.environment else index
            col = index % 2 if args.environment else col
            x, y = col * tile_w, row * (tile_h + label_h)
            sheet.paste(image, (x, y))
            draw.text((x + 12, y + tile_h + 10), title if col == 0 else label,
                      fill="#eef1f4", font=font)
            draw.line((x, y + tile_h + label_h - 1, x + tile_w, y + tile_h + label_h - 1),
                      fill="#4a5055", width=1)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.out, quality=94)
    print(args.out.resolve())


if __name__ == "__main__":
    main()
