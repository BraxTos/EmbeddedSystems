from PIL import Image
import sys
import os


def merge_forward_commands(commands):
    merged = []
    acc = 0

    for cmd in commands:
        if cmd.startswith("F "):
            acc += int(cmd.split()[1])
        else:
            if acc > 0:
                merged.append(f"F {acc}")
                acc = 0
            merged.append(cmd)

    if acc > 0:
        merged.append(f"F {acc}")

    return merged

DIRS = {
    (1, 0): 0,
    (0, 1): 90,
    (-1, 0): 180,
    (0, -1): 270
}

VECT = {
    0: (1, 0),
    90: (0, 1),
    180: (-1, 0),
    270: (0, -1)
}


def turn(commands, current, target):
    diff = (target - current) % 360
    if diff == 90:
        commands.append("R 90")
    elif diff == 270:
        commands.append("R -90")
    elif diff == 180:
        commands.append("R 90")
        commands.append("R 90")
    return target

def find_contours(pixels, w, h):
    visited = [[False]*w for _ in range(h)]
    contours = []

    def is_black(x, y):
        return 0 <= x < w and 0 <= y < h and pixels[x, y] == 0

    for y in range(h):
        for x in range(w):
            if not is_black(x, y) or visited[y][x]:
                continue

            stack = [(x, y)]
            contour = []

            while stack:
                cx, cy = stack.pop()
                if visited[cy][cx]:
                    continue
                visited[cy][cx] = True

                boundary = False
                for dx, dy in DIRS:
                    nx, ny = cx + dx, cy + dy
                    if not is_black(nx, ny):
                        boundary = True

                if boundary:
                    contour.append((cx, cy))

                for dx, dy in DIRS:
                    nx, ny = cx + dx, cy + dy
                    if is_black(nx, ny) and not visited[ny][nx]:
                        stack.append((nx, ny))

            if len(contour) > 1:
                contours.append(contour)

    return contours



def contour_to_commands(contour, step):
    commands = ["paintON"]
    direction = 0

    cx, cy = contour[0]

    for nx, ny in contour[1:]:
        dx, dy = nx - cx, ny - cy

        if (dx, dy) not in DIRS:
            continue

        target_dir = DIRS[(dx, dy)]
        direction = turn(commands, direction, target_dir)
        commands.append(f"F {step}")

        cx, cy = nx, ny

    commands.append("paintOFF")
    return commands



def image_to_commands_arduino(
    image_path,
    output_path="arduino_commands.txt",
    grid_size=12,
    step_length=10
):
    img = Image.open(image_path).convert("L")
    img = img.resize((grid_size, grid_size))
    img_bw = img.point(lambda x: 0 if x < 128 else 255, "1")

    pixels = img_bw.load()

    contours = find_contours(pixels, grid_size, grid_size)

    commands = []
    for contour in contours:
        commands.extend(contour_to_commands(contour, step_length))

    # 🔥 ОБЪЕДИНЕНИЕ F
    commands = merge_forward_commands(commands)

    with open(output_path, "w", encoding="utf-8") as f:
        for c in commands:
            f.write("\"" + c + "\",\n")

    print(f"Контуров: {len(contours)}")
    print(f"Команд: {len(commands)}")
    print(f"Файл: {output_path}")

    print("\nУпрощённое изображение:")
    for y in range(grid_size):
        print("".join("X" if pixels[x, y] == 0 else "." for x in range(grid_size)))



def main():
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    else:
        image_path = input("Путь к изображению: ").strip()

    if not os.path.exists(image_path):
        print("Файл не найден")
        return

    grid = input("Размер сетки (Enter=12): ").strip()
    grid = int(grid) if grid else 12

    step = input("Шаг (Enter=10): ").strip()
    step = int(step) if step else 10

    image_to_commands_arduino(
        image_path,
        grid_size=grid,
        step_length=step
    )


if __name__ == "__main__":
    main()
