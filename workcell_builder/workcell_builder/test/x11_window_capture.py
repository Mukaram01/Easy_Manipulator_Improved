#!/usr/bin/env python3
"""Capture one X11 window by id to PNG (acceptance screenshots).

Qt widget grabs cannot read the composited Product View (QtWebEngine/GL), and
root-window grabs fail under XWayland; reading the application's own window
works for both. Usage: x11_window_capture.py <window-id> <output.png>
"""
import sys

from PIL import Image
from Xlib import X, display


def main():
    window_id, path = int(sys.argv[1], 0), sys.argv[2]
    d = display.Display()
    window = d.create_resource_object('window', window_id)
    geometry = window.get_geometry()
    raw = window.get_image(0, 0, geometry.width, geometry.height, X.ZPixmap, 0xffffffff)
    Image.frombytes('RGB', (geometry.width, geometry.height), raw.data, 'raw', 'BGRX').save(path)
    print(path, geometry.width, geometry.height)


if __name__ == '__main__':
    main()
