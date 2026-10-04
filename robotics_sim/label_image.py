"""Tiny RGBA label backplates without an additional imaging dependency."""

import struct
import zlib


def solid_rgba_png(width, height, rgb, alpha):
    """Encode a uniform PNG; Tk composites its alpha over the canvas scene."""
    if width <= 0 or height <= 0:
        raise ValueError("Image dimensions must be positive.")

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    pixel = bytes((*rgb, alpha))
    scanline = b"\x00" + pixel * width
    header = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
            + chunk(b"IDAT", zlib.compress(scanline * height)) + chunk(b"IEND", b""))
