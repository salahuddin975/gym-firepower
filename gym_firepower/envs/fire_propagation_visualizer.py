import json
import random
from PIL import Image, ImageDraw, ImageFont

DARK_GREEN = (0, 96, 0)
DARK_BLUE = (0, 0, 160)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

PPC = 4  # pixel per cell
LINE_WIDTH = 2
FONT_SIZE = PPC * 6
FONT_STROKE_WIDTH = FONT_SIZE // 16


class Visualizer:
    def __init__(self, conf_file):
        with open(conf_file) as fin:
            self.conf_data = json.load(fin)

    def draw_map(self, step, fire_cells):
        image = Image.new('RGB', (self.conf_data['cols'], self.conf_data['rows']), DARK_GREEN)

        for cell in self.conf_data['fuel_type']:
            assert (cell[2] == 0)
            image.putpixel((cell[0], cell[1]), DARK_BLUE)

        draw = ImageDraw.Draw(image)

        buses = self.conf_data['bus_ids']
        for branch in self.conf_data['branches']:
            xy_from = buses[branch[0]][1:3]
            xy_to = buses[branch[1]][1:3]
            draw.line(xy_from + xy_to, fill=YELLOW)

        for bus in self.conf_data['bus_ids']:
            image.putpixel((bus[1], bus[2]), WHITE)

        for fire_cell in fire_cells:
            color = list(image.getpixel(fire_cell))
            color[0] = 255
            color[1] = round(color[1] * 0.8)
            color[2] = round(color[2] * 0.8)
            image.putpixel(fire_cell, tuple(color))

        image = image.resize((self.conf_data['cols'] * PPC, self.conf_data['rows'] * PPC), Image.NEAREST)

        font = ImageFont.load_default() # ImageFont.truetype("FreeSansBold.ttf", 24)

        draw = ImageDraw.Draw(image)
        draw.text((FONT_SIZE // 2, FONT_SIZE // 2), f"Step: {step}", font=font, fill=WHITE, stroke_width=FONT_STROKE_WIDTH,
                  stroke_fill=BLACK)

        for bus in self.conf_data['bus_ids']:
            x = bus[1]
            y = bus[2]
            draw.rectangle((x * PPC - LINE_WIDTH, y * PPC - LINE_WIDTH, (x + 1) * PPC, (y + 1) * PPC), outline=BLACK,
                           width=LINE_WIDTH)
            draw.text((x * PPC + FONT_SIZE // 4, y * PPC + FONT_SIZE // 4), str(bus[0]), font=font, fill=WHITE,
                      stroke_width=FONT_STROKE_WIDTH, stroke_fill=BLACK)

        return image


if __name__ == "__main__":
    grid_size = 350
    conf_file = "./../../../FirePower-agent-private/configurations/configuration.json"

    images = []
    visualizer = Visualizer(conf_file)
    fire_cells = [(random.randrange(0, grid_size), random.randrange(0, grid_size))]

    for step in range(20):
        spread = []
        for fire_cell in fire_cells:
            x = fire_cell[0]
            y = fire_cell[1]
            spread_x = random.randint(x - 1, x + 1)
            spread_y = random.randint(y - 1, y + 1)
            if (0 <= spread_x < grid_size) and (0 <= spread_y < grid_size):
                spread.append((spread_x, spread_y))
        fire_cells = fire_cells + spread

        print(f"Drawing step {step}")
        image = visualizer.draw_map(step, fire_cells)
        image.save(f"map_{step}.png")
        images.append(image)

    print(images)
    images[0].save("map.gif", save_all=True, append_images=images[1:], loop=True)