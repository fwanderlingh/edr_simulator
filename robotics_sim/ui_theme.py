"""Shared, native ttk styling for the teaching laboratory (no extra dependency)."""

import tkinter.font as tkfont
from tkinter import ttk


PAGE = "#edf2f6"
SURFACE = "#ffffff"
INK = "#203545"
MUTED = "#627687"
ACCENT = "#087f83"
VIEWPORT = "#e7eef4"
LINE = "#d6e0e7"


def configure_theme(root):
    style = ttk.Style(root)
    if "clam" in style.theme_names():
        style.theme_use("clam")
    families = set(tkfont.families(root))
    family = next((name for name in ("Segoe UI", "DejaVu Sans") if name in families),
                  tkfont.nametofont("TkDefaultFont").actual("family"))
    fixed = tkfont.nametofont("TkFixedFont").actual("family")
    root.configure(background=PAGE)
    style.configure(".", font=(family, 10), foreground=INK)
    style.configure("TFrame", background=PAGE)
    style.configure("Card.TFrame", background=SURFACE)
    style.configure("TLabel", background=SURFACE, foreground=INK)
    style.configure("Title.TLabel", background=PAGE, font=(family, 21, "bold"))
    style.configure("Subtitle.TLabel", background=PAGE, foreground=MUTED, font=(family, 9))
    style.configure("Section.TLabel", font=(family, 9, "bold"), foreground=MUTED)
    style.configure("Muted.TLabel", foreground=MUTED, font=(family, 9))
    style.configure("Code.TLabel", font=(fixed, 9), foreground=ACCENT)
    style.configure("Readout.TLabel", font=(fixed, 10), foreground=INK)
    style.configure("Footer.TLabel", background=PAGE, foreground=MUTED, font=(family, 9))
    style.configure("TButton", background="#f1f5f8", foreground=INK, bordercolor=LINE,
                    lightcolor=LINE, darkcolor=LINE, relief="flat", padding=(12, 7))
    style.map("TButton", background=[("active", "#e2ebf0"), ("disabled", "#f4f6f8")],
              foreground=[("disabled", "#8b9aa5")])
    style.configure("Primary.TButton", background=ACCENT, foreground=SURFACE,
                    bordercolor=ACCENT, lightcolor=ACCENT, darkcolor=ACCENT,
                    font=(family, 10, "bold"))
    style.map("Primary.TButton", background=[("disabled", "#d7e4e6"), ("active", "#06676b")],
              foreground=[("disabled", "#718b8e"), ("!disabled", SURFACE)])
    style.configure("TCombobox", padding=(8, 6), fieldbackground=SURFACE,
                    background="#f0f5f8", bordercolor=LINE, arrowcolor=MUTED)
    style.map("TCombobox", fieldbackground=[("readonly", SURFACE)],
              selectbackground=[("readonly", SURFACE)], selectforeground=[("readonly", INK)])
    style.configure("TEntry", padding=(8, 6), fieldbackground=SURFACE, bordercolor=LINE)
    style.configure("TCheckbutton", background=SURFACE, foreground=MUTED, font=(family, 9))
    style.map("TCheckbutton", background=[("active", SURFACE)])
    style.configure("Horizontal.TProgressbar", background=ACCENT, troughcolor="#e4ecef",
                    borderwidth=0, lightcolor=ACCENT, darkcolor=ACCENT, thickness=5)
    style.configure("Vertical.TScrollbar", background="#d2dde5", troughcolor=SURFACE,
                    borderwidth=0, arrowsize=11)
    style.configure("NotImplemented.TLabel", background="#fff2cd", foreground="#78551a")
    for name, foreground, background in (
        ("Ready", ACCENT, "#dff0ef"), ("Running", ACCENT, "#dff0ef"),
        ("Paused", "#94651c", "#fff0d4"), ("Completed", "#3463a1", "#e6edf9"),
    ):
        style.configure(f"{name}.Badge.TLabel", foreground=foreground, background=background,
                        font=(family, 9, "bold"), padding=(12, 6))
    return style
