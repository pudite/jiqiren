#!/usr/bin/env python3
"""Generate Adafruit GFX 1-bit bitmaps for Chinese characters (12x12) using tkinter."""
import tkinter as tk
import ctypes

chars = "未连接同步中桌面机器人启动设备天气阴晴雨雪雾雷阵风转小大多云"

# Create a hidden window
root = tk.Tk()
root.withdraw()

# Create a canvas
canvas = tk.Canvas(root, width=200, height=200, bg='black', highlightthickness=0)
canvas.pack()

# Try to find a Chinese font using Windows
font_name = "Microsoft YaHei"

def create_char_bitmap(ch, size=12):
    """Render character to canvas and read pixels."""
    canvas.delete('all')
    # Draw white text
    canvas.create_text(6, 0, text=ch, fill='white', font=(font_name, 11), anchor='nw')

    # Force update
    canvas.update_idletasks()
    root.update()

    # Read the canvas as PostScript (not ideal), use screenshot instead
    # Actually, let's use a PhotoImage to capture
    img = tk.PhotoImage(width=size, height=size)
    # Copy the canvas region to the image
    img.put(canvas.postscript(), (0, 0, size, size))  # This won't work

    # Different approach: use a label with the font, then read pixel data
    return None

# Actually, let me try a simpler approach using ctypes + CreateFont + GetGlyphOutline
# Or even simpler: use Windows D2D/GDI+ via a proper bitmap

# Let me try yet another approach: use a Label widget and grab its pixel data
# This won't work easily either in tkinter.

# Best approach: use PIL with Windows native font loading via ctypes
# Since PIL isn't available, let me manually encode the characters
# using a built-in 8x16 or 12x12 font table

# Actually, let me use the Windows API GetGlyphOutline which gives us the bitmap
# of a character glyph. This is the most reliable method.

print("// Using Windows GetGlyphOutline for 12x12 Chinese character bitmaps\n")

# Windows constants
GGO_BITMAP = 1
GDI_ERROR = 0xFFFFFFFF

# Load gdi32
gdi32 = ctypes.windll.gdi32
user32 = ctypes.windll.user32
kernel32 = ctypes.windll.kernel32

class POINT(ctypes.Structure):
    _fields_ = [('x', ctypes.c_long), ('y', ctypes.c_long)]

class MAT2(ctypes.Structure):
    _fields_ = [('eM11', ctypes.c_ulong), ('eM12', ctypes.c_long),
                ('eM21', ctypes.c_long), ('eM22', ctypes.c_ulong)]

class FIXED(ctypes.Structure):
    _fields_ = [('fract', ctypes.c_ushort), ('value', ctypes.c_short)]

class GLYPHMETRICS(ctypes.Structure):
    _fields_ = [
        ('gmBlackBoxX', ctypes.c_ulong),
        ('gmBlackBoxY', ctypes.c_ulong),
        ('gmptGlyphOrigin', POINT),
        ('gmCellIncX', ctypes.c_short),
        ('gmCellIncY', ctypes.c_short),
    ]

class LOGFONTW(ctypes.Structure):
    _fields_ = [
        ('lfHeight', ctypes.c_long),
        ('lfWidth', ctypes.c_long),
        ('lfEscapement', ctypes.c_long),
        ('lfOrientation', ctypes.c_long),
        ('lfWeight', ctypes.c_long),
        ('lfItalic', ctypes.c_ubyte),
        ('lfUnderline', ctypes.c_ubyte),
        ('lfStrikeOut', ctypes.c_ubyte),
        ('lfCharSet', ctypes.c_ubyte),
        ('lfOutPrecision', ctypes.c_ubyte),
        ('lfClipPrecision', ctypes.c_ubyte),
        ('lfQuality', ctypes.c_ubyte),
        ('lfPitchAndFamily', ctypes.c_ubyte),
        ('lfFaceName', ctypes.c_wchar * 32),
    ]

def get_char_bitmap(ch, target_w=12, target_h=12):
    """Get bitmap data for a character using GetGlyphOutline."""
    hdc = user32.GetDC(0)

    # Create font - use height that will produce approximately target_h
    lf = LOGFONTW()
    lf.lfHeight = -12  # 12 pixel height
    lf.lfWidth = 0
    lf.lfWeight = 400  # FW_NORMAL
    lf.lfCharSet = 134  # GB2312_CHARSET
    lf.lfQuality = 3  # PROOF_QUALITY for best rendering
    lf.lfFaceName = "Microsoft YaHei"

    hFont = gdi32.CreateFontIndirectW(ctypes.byref(lf))
    oldFont = gdi32.SelectObject(hdc, hFont)

    # Get the glyph outline to determine bitmap size
    mat = MAT2()
    mat.eM11 = 0x00010000  # 1.0 in fixed point
    mat.eM22 = 0x00010000

    gm = GLYPHMETRICS()
    buf_size = gdi32.GetGlyphOutlineW(hdc, ord(ch), GGO_BITMAP, ctypes.byref(gm), 0, None, ctypes.byref(mat))

    if buf_size == GDI_ERROR:
        gdi32.SelectObject(hdc, oldFont)
        gdi32.DeleteObject(hFont)
        user32.ReleaseDC(0, hdc)
        # Fallback: empty bitmap
        return [0] * (target_w * target_h // 8)

    if buf_size == 0:
        gdi32.SelectObject(hdc, oldFont)
        gdi32.DeleteObject(hFont)
        user32.ReleaseDC(0, hdc)
        return [0] * (target_w * target_h // 8)

    # Allocate buffer and get the glyph bitmap
    buf = ctypes.create_string_buffer(buf_size)
    result = gdi32.GetGlyphOutlineW(hdc, ord(ch), GGO_BITMAP, ctypes.byref(gm),
                                     buf_size, buf, ctypes.byref(mat))

    # Clean up
    gdi32.SelectObject(hdc, oldFont)
    gdi32.DeleteObject(hFont)
    user32.ReleaseDC(0, hdc)

    if result == GDI_ERROR:
        return [0] * (target_w * target_h // 8)

    # The glyph bitmap from GetGlyphOutline is in a specific format:
    # - Each row is DWORD-aligned (4 bytes)
    # - Scan lines are bottom-to-top for bitmaps
    # - Each byte represents 8 pixels horizontally

    glyph_w = gm.gmBlackBoxX
    glyph_h = gm.gmBlackBoxY

    # Calculate row stride (DWORD aligned)
    stride = ((glyph_w + 31) // 32) * 4

    # Create our target bitmap
    # We need to copy the glyph into a 12x12 canvas, positioned centrally
    target_bytes = target_w * (target_h + 7) // 8  # horizontal format

    # For simplicity, let's create a 12x12 bitmap in horizontal format
    # First, extract glyph pixels into a 2D array
    pixel_grid = [[0] * target_w for _ in range(target_h)]

    # Glyph offset within the target (center it horizontally)
    offset_x = (target_w - glyph_w) // 2 + gm.gmptGlyphOrigin.x
    offset_y = 0  # NO flip needed - buffer is already top-to-bottom

    # GetGlyphOutline buffer is already stored top-to-bottom
    for gy in range(glyph_h):
        src_row = buf[gy * stride : gy * stride + stride]
        for gx in range(glyph_w):
            byte_idx = gx // 8
            bit_idx = 7 - (gx % 8)
            pixel_val = (src_row[byte_idx] >> bit_idx) & 1

            tx = offset_x + gx
            ty = offset_y + gy
            if 0 <= tx < target_w and 0 <= ty < target_h:
                pixel_grid[ty][tx] = pixel_val

    # Convert to Adafruit GFX horizontal format (1 bit per pixel, MSB first)
    result_data = []
    for y in range(target_h):
        for byte_idx in range((target_w + 7) // 8):
            val = 0
            for bit in range(8):
                px = byte_idx * 8 + bit
                if px < target_w:
                    val |= (pixel_grid[y][px] << (7 - bit))
            result_data.append(val)

    return result_data

# Generate C code
print(f"// Adafruit GFX 12x12 Chinese character bitmaps")
print(f"// Horizontal scan, MSB first, 1 bit per pixel")
print(f"// Each char = 24 bytes (12 rows x 2 bytes)\n")

char_data = []
for ch in chars:
    code = ord(ch)
    data = get_char_bitmap(ch)
    hex_str = ",".join(f"0x{b:02X}" for b in data)
    var_name = f"char_0x{code:04X}"
    char_data.append((ch, code, data, var_name))
    print(f"static const unsigned char {var_name}[] = {{ {hex_str} }};")
    print(f"  // {ch} (U+{code:04X})")

print(f"\n// Lookup table:")
print("static const CharBitmap chineseChars[] = {")
for ch, code, data, var_name in char_data:
    print(f"  {{ 0x{code:04X}, {var_name} }},")
print("};")
