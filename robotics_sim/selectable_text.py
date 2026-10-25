"""Read-only sidebar text that supports selection and copying."""

import tkinter as tk
from tkinter import ttk


class SelectableText(tk.Text):
    def __init__(self, parent, *, textvariable, style="TLabel", **options):
        super().__init__(parent, wrap="word", width=1, height=1, borderwidth=0,
                         highlightthickness=0, padx=0, pady=0, cursor="xterm",
                         exportselection=False, takefocus=True, state="disabled", **options)
        self._variable = textvariable
        self._fit_timer = None
        self.set_style(style)
        self.bind("<Configure>", self._schedule_fit)
        self.bind("<Control-a>", self._select_all)
        self.bind("<Control-c>", self._copy)
        self.bind("<Control-Insert>", self._copy)
        self._menu = tk.Menu(self, tearoff=False)
        self._menu.add_command(label="Copy", command=self._copy)
        self._menu.add_command(label="Select all", command=self._select_all)
        self.bind("<Button-3>", self._show_menu)
        self._trace = textvariable.trace_add("write", self._sync)
        self._sync()

    def set_style(self, style):
        theme = ttk.Style(self)
        self.configure(font=theme.lookup(style, "font") or "TkDefaultFont",
                       foreground=theme.lookup(style, "foreground") or "#203545",
                       background=theme.lookup(style, "background") or "#ffffff",
                       selectbackground="#087f83", selectforeground="#ffffff")
        self._schedule_fit()

    def _sync(self, *_):
        value = self._variable.get()
        if self.get("1.0", "end-1c") == value:
            return
        selection = self.tag_ranges("sel")
        insertion = self.index("insert")
        self.configure(state="normal")
        self.delete("1.0", "end")
        self.insert("1.0", value)
        self.configure(state="disabled")
        # Live measurements refresh frequently; keep a selection usable while
        # the values change instead of clearing it on every physics frame.
        if selection:
            self.tag_add("sel", *selection)
        self.mark_set("insert", insertion)
        self._schedule_fit()

    def _schedule_fit(self, event=None):
        if self._fit_timer is None:
            self._fit_timer = self.after_idle(self._fit)

    def _fit(self):
        self._fit_timer = None
        if self.winfo_width() <= 1:
            return
        count = self.count("1.0", "end-1c", "displaylines")
        height = (count[0] if count else 0) + 1
        if int(self.cget("height")) != height:
            self.configure(height=height)

    def _select_all(self, event=None):
        self.focus_set()
        self.tag_add("sel", "1.0", "end-1c")
        return "break"

    def _copy(self, event=None):
        if self.tag_ranges("sel"):
            self.clipboard_clear()
            self.clipboard_append(self.get("sel.first", "sel.last"))
        return "break"

    def _show_menu(self, event):
        self.focus_set()
        try:
            self._menu.tk_popup(event.x_root, event.y_root)
        finally:
            self._menu.grab_release()

    def destroy(self):
        if self._fit_timer is not None:
            self.after_cancel(self._fit_timer)
        self._variable.trace_remove("write", self._trace)
        super().destroy()
