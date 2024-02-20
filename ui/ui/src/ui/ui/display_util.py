from PIL import ImageFont
from pathlib import Path
import psutil
import socket

font_size = 12
font_size_full = 10
top_bar_height = 12

margin_y_line = [l + top_bar_height for l in [0, 13, 25, 38, 51]]
margin_x_figure = 78
margin_x_bar = 31
bar_width = 52
bar_width_full = 95
bar_height = 8
bar_margin_top = 3

font_default = ImageFont.truetype(
    str(Path(__file__).resolve().parent.joinpath("fonts", "DejaVuSansMono.ttf")),
    font_size,
)
font_full = ImageFont.truetype(
    str(Path(__file__).resolve().parent.joinpath("fonts", "DejaVuSansMono.ttf")),
    font_size_full,
)


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def bytes2human(n):
    """
    >>> bytes2human(10000)
    '9K'
    >>> bytes2human(100001221)
    '95M'
    """
    symbols = ("K", "M", "G", "T", "P", "E", "Z", "Y")
    prefix = {}
    for i, s in enumerate(symbols):
        prefix[s] = 1 << (i + 1) * 10
    for s in reversed(symbols):
        if n >= prefix[s]:
            value = int(float(n) / prefix[s])
            return "%s%s" % (value, s)
    return f"{n}B"


def get_temp():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as temp:
        temp = int(temp.read()[:2])

    return temp


def get_cpu():
    return psutil.cpu_percent()


def get_mem():
    return psutil.virtual_memory().percent


def get_disk_usage():
    usage = psutil.disk_usage("/")
    return usage.used / usage.total * 100


def format_percent(percent):
    return "%5.1f" % (percent)


def draw_text(draw, margin_x, line_num, text):
    draw.text(
        (margin_x, margin_y_line[line_num]), text, font=font_default, fill="white"
    )


def draw_bar(draw, line_num, percent):
    top_left_y = margin_y_line[line_num] + bar_margin_top
    draw.rectangle(
        (margin_x_bar, top_left_y, margin_x_bar + bar_width, top_left_y + bar_height),
        outline="white",
    )
    draw.rectangle(
        (
            margin_x_bar,
            top_left_y,
            margin_x_bar + bar_width * percent / 100,
            top_left_y + bar_height,
        ),
        fill="white",
    )


def draw_bar_full(draw, line_num):
    top_left_y = margin_y_line[line_num] + bar_margin_top
    draw.rectangle(
        (
            margin_x_bar,
            top_left_y,
            margin_x_bar + bar_width_full,
            top_left_y + bar_height,
        ),
        fill="white",
    )
    draw.text((65, top_left_y - 2), "100 %", font=font_full, fill="black")


def show_stats(draw):
    temp = get_temp()
    draw_text(draw, 0, 0, "Temp")
    draw_text(draw, margin_x_figure, 0, "%s'C" % (format_percent(temp)))

    cpu = get_cpu()
    draw_text(draw, 0, 1, "CPU")
    if cpu < 100:
        draw_text(draw, margin_x_figure, 1, "%s %%" % (format_percent(cpu)))
        draw_bar(draw, 1, cpu)
    else:
        draw_bar_full(draw, 1)

    mem = get_mem()
    draw_text(draw, 0, 2, "Mem")
    if mem < 100:
        draw_text(draw, margin_x_figure, 2, "%s %%" % (format_percent(mem)))
        draw_bar(draw, 2, mem)
    else:
        draw_bar_full(draw, 2)

    disk = get_disk_usage()
    draw_text(draw, 0, 3, "Disk")
    if disk < 100:
        draw_text(draw, margin_x_figure, 3, "%s %%" % (format_percent(disk)))
        draw_bar(draw, 3, disk)
    else:
        draw_bar_full(draw, 3)


def show_battery(bb, percentage, draw):
    _, _, w, h = bb
    padding = 10
    tip_length = 4
    tip_height = 20
    body_l = padding
    body_r = w - padding
    body_t = top_bar_height + padding
    body_b = h
    tip_t = body_t + (body_b - body_t) / 2 - tip_height / 2
    tip_b = tip_t + tip_height

    for i in range(2):
        draw.rectangle(
            (body_l + i, body_t + i, body_r - tip_length - i, body_b - i),
            outline="white",
            fill="black",
        )
        draw.rectangle(
            (body_r - tip_length + i, tip_t + i, body_r + tip_length - i, tip_b - i),
            outline="white",
            fill="black",
        )

    fill_r = map(percentage, 0.0, 1.0, body_l, body_r - tip_length)
    draw.rectangle((body_l, body_t, fill_r, body_b), outline="white", fill="white")


def show_network(iface, draw):
    if_addrs = psutil.net_if_addrs()
    if iface not in if_addrs:
        draw_text(draw, 0, 0, f"{iface} not found")
    else:
        isup = psutil.net_if_stats()[iface].isup
        status_str = "up" if isup else "down"
        draw_text(draw, 0, 0, f"{iface} {status_str}")
        adrv4 = [adr for adr in if_addrs[iface] if adr.family == socket.AF_INET]
        if isup and len(adrv4) == 1:
            stat = psutil.net_io_counters(pernic=True)[iface]
            draw_text(draw, 0, 1, adrv4[0].address)
            data = f"Tx{bytes2human(stat.bytes_sent)}, Rx{bytes2human(stat.bytes_recv)}"
            draw_text(draw, 0, 2, data)
