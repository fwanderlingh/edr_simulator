"""Single-window teaching application with crisp, screen-space annotations."""

import math
import time
import tkinter as tk
from collections import OrderedDict
from pathlib import Path
from tkinter import colorchooser, filedialog, messagebox, ttk

import numpy as np

from .exercises.lesson02.frames import point_in_world
from .exercises.lesson02 import activity as frames_activity
from .exercises.lesson02.planar import solve_planar_task
from .exercises.lesson02 import module2
from .session import ACTIVITY_FILES, ActivitySession, DESCRIPTIONS, LESSONS, TASKS
from .sim.camera import Camera
from .sim.scene_renderer import SceneRenderer
from .exercises.lesson04.gui import SENSOR_EXERCISES
from .preferences import DEFAULT_PATH, load_preferences, save_preferences
from .experiment_data import capture_experiment
from .label_image import solid_rgba_png
from .selectable_text import SelectableText
from .ui_theme import configure_theme, ACCENT, SURFACE, VIEWPORT


class SimulatorApp:
    _notice = ""
    def __init__(self, root, *, lesson=None, duration=None, dt=1 / 240, csv_path=None, settings_path=DEFAULT_PATH):
        self.root = root
        self.settings_path = settings_path
        self._loading_preferences = True
        self._save_timer = None
        saved = load_preferences(settings_path)
        lesson = lesson or saved.get("lesson", "1")
        if lesson not in LESSONS:
            lesson = "1"
        duration = duration if duration is not None else saved.get("duration", 16)
        self.dt, self.csv_path = dt, csv_path
        self.session = None
        self.last_experiment = None
        self._recorded_experiment_key = None
        self._experiment_generation = 0
        self._plot_window = None
        self._plot_canvas = None
        self._plot_figure = None
        self.camera = Camera()
        self.closed = False
        self._accumulator = 0.0
        self._last_tick = time.perf_counter()
        self._dirty = True
        self._drag_position = None
        self.lesson = tk.StringVar(value=LESSONS[lesson])
        self.task = tk.StringVar()
        self.duration = tk.StringVar(value=f"{duration:g}")
        self.status = tk.StringVar()
        self.values = tk.StringVar()
        self.description = tk.StringVar()
        self.kinematics = tk.StringVar(value=saved.get("kinematics", "reference")
                                     if saved.get("kinematics") in ("reference", "student") else "reference")
        self.sensor_mode = tk.StringVar(value=saved.get("sensor_mode", "ideal")
                                       if saved.get("sensor_mode") in ("ideal", "noisy", "student") else "ideal")
        self.exercise_model = tk.StringVar(value=saved.get("exercise_model", "reference")
            if saved.get("exercise_model") in ("reference", "student") else "reference")
        self.goal_x = tk.StringVar(value="1")
        self.goal_y = tk.StringVar(value="1")
        self.goal_yaw = tk.StringVar(value="0")
        self.show_frames = tk.BooleanVar(value=saved.get("show_frames", True))
        self.label_size = int(saved.get("label_size", 10))
        self.label_color = self._valid_color(saved.get("label_color"), "#172d3d")
        self.label_background = saved.get("label_background", True)
        self.label_background_color = self._valid_color(saved.get("label_background_color"), "#ffffff")
        self.label_background_transparency = saved.get("label_background_transparency", 0)
        self._label_images = OrderedDict()
        self._label_dialog = None
        root.title("Robotics Laboratory")
        self._app_icon = tk.PhotoImage(master=root, file=str(
            Path(__file__).resolve().parents[1] / "img" / "rlab_app_icon.png"))
        root.iconphoto(True, self._app_icon)
        # Keep restored windows visible if the display dimensions have changed.
        screen_width, screen_height = root.winfo_screenwidth(), root.winfo_screenheight()
        width = min(int(saved.get("width", 1120)), max(900, screen_width))
        height = min(int(saved.get("height", 760)), max(620, screen_height - 80))
        x = max(0, min(int(saved.get("x", 40)), screen_width - width))
        y = max(0, min(int(saved.get("y", 40)), screen_height - height - 80))
        self._normal_window = {"width": width, "height": height, "x": x, "y": y}
        root.geometry(f"{width}x{height}+{x}+{y}")
        root.minsize(900, 620)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.style = configure_theme(root)
        self.time_readout = tk.StringVar(value="0.00 / 0.00 s")
        self._build_menu()
        self._build_widgets()
        self.select_lesson()
        if saved.get("lesson") == lesson and saved.get("task") in TASKS[lesson]:
            self.task.set(TASKS[lesson][saved["task"]])
            self.load_task()
        for name in ("yaw", "pitch", "distance"):
            if f"camera_{name}" in saved:
                setattr(self.camera, name, saved[f"camera_{name}"])
        root.bind("<Configure>", self._window_changed, add="+")
        self._loading_preferences = False
        self._restore_timer = root.after_idle(lambda: self._restore_maximized(saved.get("maximized", False)))
        self._timer = root.after(20, self._tick)

    def _valid_color(self, color, default):
        try:
            self.root.winfo_rgb(color or default)
            return color or default
        except tk.TclError:
            return default

    def _is_maximized(self):
        if self.root.state() == "zoomed":
            return True
        try:
            return self.root.tk.getboolean(self.root.attributes("-zoomed"))
        except tk.TclError:
            return False

    def _restore_maximized(self, maximized):
        if maximized:
            try:
                self.root.state("zoomed")
            except tk.TclError:
                try:
                    self.root.attributes("-zoomed", True)
                except tk.TclError:
                    pass

    def _window_changed(self, event):
        if event.widget is not self.root or self.closed:
            return
        self._capture_window_geometry()
        self._queue_save()

    def _capture_window_geometry(self):
        # Configure notifications can lag behind the geometry reported by Tk.
        # Never replace the normal bounds with maximized or unmapped geometry.
        if (self.root.winfo_ismapped() and self.root.state() == "normal"
                and not self._is_maximized()):
            self._normal_window = {"width": self.root.winfo_width(), "height": self.root.winfo_height(),
                                   "x": self.root.winfo_x(), "y": self.root.winfo_y()}

    def _queue_save(self, event=None):
        if self._loading_preferences or self.closed:
            return
        if self._save_timer is not None:
            self.root.after_cancel(self._save_timer)
        self._save_timer = self.root.after(500, self._save_settings)

    def _save_settings(self):
        self._save_timer = None
        self._capture_window_geometry()
        duration = self.session.duration
        try:
            candidate = float(self.duration.get())
            if math.isfinite(candidate) and 0 < candidate <= 86400:
                duration = candidate
        except ValueError:
            pass
        data = {**self._normal_window, "maximized": self._is_maximized(),
                "lesson": self.session.lesson, "task": self.session.task, "duration": duration,
                "kinematics": self.kinematics.get(),
                "sensor_mode": self.sensor_mode.get(), "exercise_model": self.exercise_model.get(),
                "show_frames": self.show_frames.get(), "label_size": self.label_size,
                "label_color": self.label_color, "label_background": self.label_background,
                "label_background_color": self.label_background_color,
                "label_background_transparency": self.label_background_transparency,
                "camera_yaw": self.camera.yaw, "camera_pitch": self.camera.pitch,
                "camera_distance": self.camera.distance}
        try:
            save_preferences(self.settings_path, data)
        except OSError as error:
            # A read-only location must not prevent students from using or closing the app.
            self.status.set(f"Could not save settings: {error}")

    def _display_settings_changed(self):
        self._redraw()
        self._queue_save()

    def _build_menu(self):
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="Export measurements...", command=self.export)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.close)
        menu.add_cascade(label="File", menu=file_menu)
        lesson_menu = tk.Menu(menu, tearoff=False)
        for title in LESSONS.values():
            lesson_menu.add_radiobutton(
                label=f"   {title}", variable=self.lesson, value=title,
                command=self.select_lesson, indicatoron=True, hidemargin=False)
        menu.add_cascade(label="Lessons", menu=lesson_menu)
        view_menu = tk.Menu(menu, tearoff=False)
        view_menu.add_checkbutton(
            label="   Show coordinate frames", variable=self.show_frames,
            command=self._display_settings_changed, indicatoron=True,
            hidemargin=False)
        view_menu.add_command(label="Label settings...", command=self.label_settings)
        view_menu.add_command(label="Reset camera", command=self.reset_camera)
        menu.add_cascade(label="View", menu=view_menu)
        help_menu = tk.Menu(menu, tearoff=False)
        help_menu.add_command(label="How to use", command=lambda: messagebox.showinfo(
            "Robotics Laboratory", "Select a lesson and a task, then press Start.\n\n"
            "Start runs the activity; Pause freezes time; Resume continues it.\n"
            "Step advances one physics step while paused.\n"
            "Reset / Apply length starts the selected task again and clears its measurements.\n"
            "Completed activities remain open for inspection.\n\n"
            "In the 3D view, left-drag to orbit, middle-drag or Shift + left-drag to pan, and scroll to zoom.\n"
            "Use View > Reset camera to restore the default view.\n"
            "Use View > Label settings to change label size, colors, and backgrounds.\n\n"
            "Use File > Export measurements... to save the recorded data.\n"
            "After editing a student exercise, save the file and restart the simulator.", parent=self.root))
        menu.add_cascade(label="Help", menu=help_menu)
        self.root.configure(menu=menu)

    def _build_widgets(self):
        panel = ttk.Frame(self.root, padding=(16, 12))
        panel.pack(fill="both", expand=True)
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(2, weight=1)

        header = ttk.Frame(panel)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        header.columnconfigure(1, weight=1)
        mark = tk.Label(header, text="R", background=ACCENT, foreground=SURFACE,
                        font=("TkDefaultFont", 20, "bold"), width=2, padx=6, pady=5)
        mark.grid(row=0, column=0, rowspan=2, padx=(0, 12))
        title = ttk.Label(header, text="Robotics Laboratory", style="Title.TLabel")
        title.grid(row=0, column=1, sticky="w")
        subtitle = ttk.Label(header, text="Observe the model. Predict the motion. Test your implementation.",
                             style="Subtitle.TLabel")
        subtitle.grid(row=1, column=1, sticky="w")
        self.state_badge = ttk.Label(header, text="Ready", style="Ready.Badge.TLabel")
        self.state_badge.grid(row=0, column=2, rowspan=2, sticky="e")

        selectors = ttk.Frame(panel, style="Card.TFrame", padding=(14, 10))
        selectors.grid(row=1, column=0, sticky="ew", pady=(0, 12))
        selectors.columnconfigure(0, weight=2, minsize=200)
        selectors.columnconfigure(1, weight=3, minsize=230)
        ttk.Label(selectors, text="LESSON", style="Section.TLabel").grid(row=0, column=0, sticky="w", pady=(0, 5))
        ttk.Label(selectors, text="ACTIVITY", style="Section.TLabel").grid(row=0, column=1, sticky="w", padx=12, pady=(0, 5))
        ttk.Label(selectors, text="DURATION · s", style="Section.TLabel").grid(row=0, column=2, sticky="w", pady=(0, 5))
        self.lesson_selector = ttk.Combobox(selectors, textvariable=self.lesson,
            values=list(LESSONS.values()), state="readonly", width=1)
        self.lesson_selector.grid(row=1, column=0, sticky="ew")
        self.lesson_selector.bind("<<ComboboxSelected>>", self.select_lesson)
        self.tasks = ttk.Combobox(selectors, textvariable=self.task, state="readonly", width=1)
        self.tasks.grid(row=1, column=1, sticky="ew", padx=12)
        self.tasks.bind("<<ComboboxSelected>>", self.load_task)
        duration_entry = ttk.Entry(selectors, textvariable=self.duration, width=8)
        duration_entry.grid(row=1, column=2, sticky="ew")
        duration_entry.bind("<FocusOut>", self._queue_save)
        duration_entry.bind("<Return>", self._queue_save)

        workspace = ttk.Frame(panel)
        workspace.grid(row=2, column=0, sticky="nsew", pady=(0, 12))
        workspace.columnconfigure(0, weight=1)
        workspace.columnconfigure(1, minsize=294)
        workspace.rowconfigure(0, weight=1)
        viewport = ttk.Frame(workspace, style="Card.TFrame", padding=1)
        viewport.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        viewport.columnconfigure(0, weight=1)
        viewport.rowconfigure(1, weight=1)
        tools = ttk.Frame(viewport, style="Card.TFrame", padding=(12, 6))
        tools.grid(row=0, column=0, sticky="ew")
        ttk.Label(tools, text="SCENE", style="Section.TLabel").pack(side="left")
        ttk.Button(tools, text="Reset camera", command=self.reset_camera, width=12).pack(side="right")
        ttk.Button(tools, text="Labels…", command=self.label_settings, width=7).pack(side="right", padx=6)
        ttk.Checkbutton(tools, text="Frames", variable=self.show_frames,
                        command=self._display_settings_changed).pack(side="right", padx=(5, 10))
        self.canvas = tk.Canvas(viewport, background=VIEWPORT, highlightthickness=0,
                                width=450, height=260)
        self.canvas.grid(row=1, column=0, sticky="nsew")
        self.renderer = SceneRenderer(self.canvas)
        self.canvas.bind("<Configure>", self._redraw)
        self.canvas.bind("<ButtonPress-1>", self._begin_drag)
        self.canvas.bind("<B1-Motion>", self._orbit)
        self.canvas.bind("<ButtonPress-2>", self._begin_drag)
        self.canvas.bind("<B2-Motion>", lambda event: self._orbit(event, pan=True))
        self.canvas.bind("<MouseWheel>", lambda event: self._zoom(1 if event.delta > 0 else -1))
        self.canvas.bind("<Button-4>", lambda event: self._zoom(1))
        self.canvas.bind("<Button-5>", lambda event: self._zoom(-1))
        legend = ttk.Frame(viewport, style="Card.TFrame", padding=(10, 6))
        legend.grid(row=2, column=0, sticky="ew")
        ttk.Label(legend, text="Left: orbit  ·  Middle / Shift: pan  ·  Scroll: zoom", style="Muted.TLabel").pack(side="left")
        for text, color in (("z", "#245dcc"), ("y", "#14823e"), ("x", "#c92d35")):
            ttk.Label(legend, text=text, foreground=color, font=("TkDefaultFont", 9, "bold")).pack(side="right", padx=3)

        inspector = ttk.Frame(workspace, style="Card.TFrame")
        inspector.grid(row=0, column=1, sticky="nsew")
        inspector.columnconfigure(0, weight=1)
        inspector.rowconfigure(0, weight=1)
        self.inspector_canvas = tk.Canvas(inspector, background=SURFACE,
                                         highlightthickness=0, width=278)
        self.inspector_canvas.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(inspector, orient="vertical", command=self.inspector_canvas.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.inspector_canvas.configure(yscrollcommand=scroll.set)
        details = ttk.Frame(self.inspector_canvas, style="Card.TFrame", padding=(14, 12))
        details.columnconfigure(0, weight=1)
        window = self.inspector_canvas.create_window(0, 0, window=details, anchor="nw")
        details.bind("<Configure>", lambda event:
                     self.inspector_canvas.configure(scrollregion=self.inspector_canvas.bbox("all")))
        self.inspector_canvas.bind("<Configure>", lambda event:
                                  self.inspector_canvas.itemconfigure(window, width=event.width))
        self.model_controls = ttk.Frame(details, style="Card.TFrame")
        self.model_controls.grid(row=0, column=0, sticky="ew", pady=(0, 14))
        self.model_heading = ttk.Label(self.model_controls, text="KINEMATICS MODEL", style="Section.TLabel")
        self.model_heading.pack(anchor="w", pady=(0, 6))
        self.model_selector = ttk.Combobox(self.model_controls, textvariable=self.kinematics,
            values=("reference", "student"), state="readonly", width=16)
        self.model_selector.pack(fill="x")
        self.model_selector.bind("<<ComboboxSelected>>", self.load_task)
        self.sensor_controls = ttk.Frame(details, style="Card.TFrame")
        self.sensor_controls.grid(row=1, column=0, sticky="ew", pady=(0, 14))
        ttk.Label(self.sensor_controls, text="SENSOR MODE", style="Section.TLabel").pack(anchor="w", pady=(0, 6))
        self.sensor_selector = ttk.Combobox(self.sensor_controls, textvariable=self.sensor_mode,
            values=("ideal", "noisy", "student"), state="readonly", width=16)
        self.sensor_selector.pack(fill="x")
        self.sensor_selector.bind("<<ComboboxSelected>>", self.load_task)
        ttk.Label(self.sensor_controls, text="10 Hz nominal · A mode change resets the activity.",
                  style="Muted.TLabel", wraplength=245).pack(anchor="w", pady=(6, 0))
        self.goal_controls = ttk.Frame(details, style="Card.TFrame")
        self.goal_controls.grid(row=2, column=0, sticky="ew", pady=(0, 14))
        self.goal_controls.columnconfigure((0, 1), weight=1)
        ttk.Label(self.goal_controls, text="TARGET", style="Section.TLabel").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))
        for column, name, variable in ((0, "x · m", self.goal_x), (1, "y · m", self.goal_y)):
            ttk.Label(self.goal_controls, text=name, style="Muted.TLabel").grid(row=1, column=column, sticky="w")
            ttk.Entry(self.goal_controls, textvariable=variable, width=8).grid(row=2, column=column, sticky="ew", padx=(0, 5))
        self.heading_controls = ttk.Frame(self.goal_controls, style="Card.TFrame")
        self.heading_controls.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Label(self.heading_controls, text="Heading · degrees", style="Muted.TLabel").pack(side="left")
        ttk.Entry(self.heading_controls, textvariable=self.goal_yaw, width=7).pack(side="right")
        ttk.Button(self.goal_controls, text="Apply / reset", command=self.load_task).grid(row=4, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Label(details, text="LIVE MEASUREMENTS", style="Section.TLabel").grid(row=3, column=0, sticky="w", pady=(0, 8))
        self.values_label = SelectableText(details, textvariable=self.values, style="Readout.TLabel")
        self.values_label.grid(row=4, column=0, sticky="ew")
        ttk.Separator(details).grid(row=5, column=0, sticky="ew", pady=16)
        ttk.Label(details, text="THE EXPERIMENT", style="Section.TLabel").grid(row=6, column=0, sticky="w", pady=(0, 8))
        description = SelectableText(details, textvariable=self.description)
        description.grid(row=7, column=0, sticky="ew")
        ttk.Separator(details).grid(row=8, column=0, sticky="ew", pady=16)
        ttk.Label(details, text="YOUR IMPLEMENTATION", style="Section.TLabel").grid(row=9, column=0, sticky="w", pady=(0, 8))
        self.edit_hint = ttk.Label(details, wraplength=246, justify="left", style="Muted.TLabel")
        self.edit_hint.grid(row=10, column=0, sticky="ew", pady=(0, 8))
        self.edit_file = ttk.Label(details, wraplength=246, justify="left", style="Code.TLabel")
        self.edit_file.grid(row=11, column=0, sticky="ew")

        def scroll_details(event):
            direction = -1 if getattr(event, "num", None) == 4 else 1
            if getattr(event, "delta", 0):
                direction = -1 if event.delta > 0 else 1
            self.inspector_canvas.yview_scroll(direction * 3, "units")
            return "break"
        for widget in (self.inspector_canvas, details, self.values_label, description, self.edit_hint, self.edit_file):
            widget.bind("<MouseWheel>", scroll_details)
            widget.bind("<Button-4>", scroll_details)
            widget.bind("<Button-5>", scroll_details)

        transport = ttk.Frame(panel, style="Card.TFrame", padding=(12, 10))
        transport.grid(row=3, column=0, sticky="ew")
        self.start_button = ttk.Button(transport, text="Start", command=self.toggle_running,
                                       style="Primary.TButton", width=9)
        self.start_button.pack(side="left")
        self.step_button = ttk.Button(transport, text="Step", command=self.single_step, width=6)
        self.step_button.pack(side="left", padx=6)
        ttk.Button(transport, text="Reset ■ / Apply length", command=self.load_task).pack(side="left")
        analysis = ttk.Frame(transport, style="Card.TFrame")
        analysis.pack(side="right")
        ttk.Button(analysis, text="Export CSV…", command=self.export).pack(side="left")
        self.plot_button = ttk.Button(analysis, text="Plot results…", command=self.plot_results,
                                       state="disabled")
        self.plot_button.pack(side="left", padx=(6, 0))
        timeline = ttk.Frame(transport, style="Card.TFrame")
        timeline.pack(side="right", padx=(12, 20))
        ttk.Label(timeline, textvariable=self.time_readout, style="Muted.TLabel").pack(anchor="e", pady=(0, 4))
        self.progress = ttk.Progressbar(timeline, orient="horizontal", length=115, maximum=100)
        self.progress.pack(fill="x")
        self.status_label = ttk.Label(panel, textvariable=self.status, style="Footer.TLabel", wraplength=1000)
        self.status_label.grid(row=4, column=0, sticky="ew", pady=(8, 0))
        panel.bind("<Configure>", lambda event:
                   self.status_label.configure(wraplength=max(300, event.width - 32)))
        self._compact_layout = None
        title_font = self.root.tk.splitlist(self.style.lookup("Title.TLabel", "font"))[0]

        def adapt_layout(event):
            if event.widget is not self.root:
                return
            compact = event.height < 700
            if compact == self._compact_layout:
                return
            self._compact_layout = compact
            panel.configure(padding=(12, 8) if compact else (16, 12))
            title.configure(font=(title_font, 16 if compact else 21, "bold"))
            header.grid_configure(pady=(0, 8 if compact else 12))
            selectors.configure(padding=(12, 6) if compact else (14, 10))
            selectors.grid_configure(pady=(0, 8 if compact else 12))
            workspace.grid_configure(pady=(0, 8 if compact else 12))
            tools.configure(padding=(8, 4) if compact else (12, 6))
            transport.configure(padding=(12, 8) if compact else (12, 10))
            if compact:
                subtitle.grid_remove()
                legend.grid_remove()
            else:
                subtitle.grid()
                legend.grid()
        self.root.bind("<Configure>", adapt_layout, add="+")

        for variable, label, default_style in (
            (self.values, self.values_label, "Readout.TLabel"),
            (self.status, self.status_label, "Footer.TLabel"),
        ):
            def highlight_notice(*_, variable=variable, label=label, default_style=default_style):
                style = "NotImplemented.TLabel" if "not implemented" in variable.get().lower() else default_style
                if isinstance(label, SelectableText):
                    label.set_style(style)
                else:
                    label.configure(style=style)
            variable.trace_add("write", highlight_notice)
            highlight_notice()

    def select_lesson(self, event=None):
        self.lesson_id = next(key for key, value in LESSONS.items() if value == self.lesson.get())
        self.tasks.configure(values=list(TASKS[self.lesson_id].values()))
        self.task.set(next(iter(TASKS[self.lesson_id].values())))
        self.load_task()

    def load_task(self, event=None):
        try:
            duration = float(self.duration.get())
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError
        except ValueError:
            if self.session is not None:
                self.lesson_id = self.session.lesson
                self.lesson.set(LESSONS[self.lesson_id])
                self.tasks.configure(values=list(TASKS[self.lesson_id].values()))
                self.task.set(TASKS[self.lesson_id][self.session.task])
            messagebox.showerror("Activity length", "Enter a positive number of seconds.", parent=self.root)
            return
        task = next(key for key, value in TASKS[self.lesson_id].items() if value == self.task.get())
        changed_task = (self.session is None or self.session.lesson != self.lesson_id or self.session.task != task)
        editable_target = task in ("drive_goal", "drive_goal_behind", "sensor_avoidance") or self.lesson_id == "5"
        if changed_task and editable_target:
            default = ((1, 0.5, 90) if task == "drive_goal" else
                       (-1, 0, 0) if task == "drive_goal_behind" else
                       (3, 0, 0) if task == "sensor_avoidance" else (1, 1, 0))
            for variable, value in zip((self.goal_x, self.goal_y, self.goal_yaw), default):
                variable.set(str(value))
        try:
            goal = (float(self.goal_x.get()), float(self.goal_y.get()),
                    math.radians(float(self.goal_yaw.get()))) if editable_target else None
            if task in SENSOR_EXERCISES and self.sensor_mode.get() == "student":
                self.sensor_mode.set("ideal")
            new_session = ActivitySession(self.lesson_id, task, duration=duration, dt=self.dt,
                                          kinematics=self.kinematics.get(), sensor_mode=self.sensor_mode.get(),
                                          exercise_model=self.exercise_model.get(), goal=goal)
        except Exception as error:
            if self.session is None:
                raise
            self.lesson_id = self.session.lesson
            self.lesson.set(LESSONS[self.lesson_id])
            self.tasks.configure(values=list(TASKS[self.lesson_id].values()))
            self.task.set(TASKS[self.lesson_id][self.session.task])
            messagebox.showerror("Could not load activity",
                f"{type(error).__name__}: {error}\nCheck your exercise implementation and try again.", parent=self.root)
            return
        if self.session:
            self._remember_experiment()
            self.session.close()
        self.session = new_session
        self._experiment_generation += 1
        self.model_selector.configure(state="readonly", textvariable=(self.kinematics
                                      if self.lesson_id == "3" else self.exercise_model))
        if self.lesson_id in ("3", "5") or task in SENSOR_EXERCISES:
            self.model_controls.grid()
            self.model_heading.configure(text=("CONTROLLER MODEL" if self.session.controller is not None else
                "ARM MODEL" if self.lesson_id == "5" else "SUPERVISOR MODEL" if task == "sensor_avoidance" else
                "ESTIMATOR MODEL" if task in SENSOR_EXERCISES else "KINEMATICS MODEL"))
        else:
            self.model_controls.grid_remove()
        if editable_target:
            self.goal_controls.grid()
        else:
            self.goal_controls.grid_remove()
        if task in ("drive_goal", "drive_goal_behind"):
            self.heading_controls.grid()
        else:
            self.heading_controls.grid_remove()
        self.sensor_selector.configure(values=("ideal", "noisy") if task in SENSOR_EXERCISES
                                       else ("ideal", "noisy", "student"))
        self.inspector_canvas.yview_moveto(0)
        if self.lesson_id == "4":
            self.sensor_controls.grid()
        else:
            self.sensor_controls.grid_remove()
        if self.lesson_id == "2":
            self.camera.target = ((2, 1, 0.2) if task == "planar" else
                                  (1, 2, 0.7) if task == "spatial" else
                                  (0.6, 0, 0.8) if task in module2.TASKS else (0.25, 0, 0.2))
        else:
            self.camera.target = (0.8, 0.7, 0.1)
        self._camera_home_target = self.camera.target
        self.description.set(DESCRIPTIONS[task])
        hint, path = ACTIVITY_FILES[task]
        self.edit_hint.configure(text=hint)
        files = f"File (relative to simulator/): {path}"
        if self.lesson_id != "1":
            scene_path = f"robotics_sim/exercises/lesson{int(self.lesson_id):02d}/" + (
                "gui.py" if self.session.laboratory is not None else "activity.py")
            if scene_path != path:
                files += f"\nScene and robot commands: {scene_path}"
        self.edit_file.configure(text=files)
        self._notice = ""
        self._accumulator = 0
        self._last_tick = time.perf_counter()
        self._redraw()
        self._update_status()
        self._update_plot_button()

        self._queue_save()

    def toggle_running(self):
        if self.session.state == "Running":
            self.session.pause()
            self._remember_experiment()
        else:
            self.session.start()
        self._accumulator = 0
        self._last_tick = time.perf_counter()
        self._update_status()

    def single_step(self):
        self.session.pause()
        self._guarded_step(single=True)
        self._after_steps()

    def _after_steps(self):
        self._update_plot_button()
        if self.session.state == "Completed":
            self._remember_experiment()
        if self.session.state == "Completed" and self.csv_path:
            try:
                self.session.export_csv(self.csv_path)
            except OSError as error:
                self.csv_path = None
                messagebox.showerror("Export failed", str(error), parent=self.root)
        self._redraw()
        self._update_status()

    def _guarded_step(self, *, single=False):
        """Step the session; pause with a hint if an exercise is still a TODO stub."""
        try:
            self.session.step(single=single)
        except NotImplementedError as error:
            self.session.pause()
            self.session.sim.stop()
            self._notice = f"Exercise not implemented yet: {error}"
            return False
        except (ValueError, TypeError, ArithmeticError, np.linalg.LinAlgError) as error:
            self.session.pause()
            self.session.sim.stop()
            self._notice = f"Check your implementation: {error}"
            return False
        return True

    def _planar_output(self):
        scene = self.session.planar_scene
        try:
            return solve_planar_task(
                scene.world_from_robot(), scene.robot_from_sensor,
                scene.point_in_sensor, scene.direction_in_sensor)
        except NotImplementedError:
            return None

    def _module2_output(self):
        try:
            return module2.evaluate(self.session.task)
        except NotImplementedError:
            return None

    def _update_status(self):
        pose = self.session.robot.pose()
        self.values.set(f"x     {pose.x:8.3f} m\ny     {pose.y:8.3f} m\nyaw   {pose.yaw:8.3f} rad")
        if self.session.task in module2.TASKS:
            output = self._module2_output()
            self.values.set(self.values.get() + "\n" + (output[2] if output else
                "Module 2 not implemented yet: complete the exercise file shown below."))
        if self.session.task == "planar":
            output = self._planar_output()
            if output is None:
                self.values.set(self.values.get() +
                    "\nModule 1 not implemented yet: complete solve_planar_task in exercises/lesson02/planar.py")
            else:
                point, direction = output
                self.values.set(self.values.get() +
                    f"\nModule 1 output: point in world ({point[0]:.2f}, {point[1]:.2f}) m"
                    f" | direction in world ({direction[0]:.2f}, {direction[1]:.2f})")
        if self.session.lesson == "3":
            left, right = self.session.robot.wheel_velocities
            command = self.session.robot.command
            self.values.set(self.values.get() +
                f"\nWheels L/R: {left:.2f} / {right:.2f} rad/s    v: {command.vx:.3f} m/s    omega: {command.omega:.3f} rad/s")
        if self.session.sensors is not None:
            sensors = self.session.sensors
            left, right = sensors.encoder.read()
            gyro, ax, ay = sensors.imu.read()
            middle = len(sensors.lidar.ranges) // 2
            distance = f"{sensors.lidar.ranges[middle]:.3f} m" if sensors.lidar.hits[middle] else "no return (4 m)"
            self.values.set(self.values.get() +
                f"\nSample: {sensors.time:.2f} s | Encoder L/R: {left:.4f} / {right:.4f} rad | Front: {distance}"
                f"\nGyro z: {gyro:.3f} rad/s | Accel x/y: {ax:.3f} / {ay:.3f} m/s² | {self.session.sensor_mode}")
        if self.session.controller is not None:
            ctrl = self.session.controller
            active = min(ctrl.index, len(ctrl.goals) - 1)
            gx, gy, gt = ctrl.goals[active]
            distance = math.hypot(gx - pose.x, gy - pose.y)
            heading = (gt - pose.yaw + math.pi) % (2 * math.pi) - math.pi
            self.values.set(self.values.get() + f"\nWaypoints: {ctrl.index}/{len(ctrl.goals)}\nPhase: {ctrl.phase}"
                            f"\nGoal error: {distance:.3f} m\nHeading error: {heading:.3f} rad\nOutcome: {self.session.outcome}")
        if self.session.laboratory is not None:
            self.values.set(self.values.get() + "\n" + self.session.laboratory.readout)
        state = self.session.state
        self.state_badge.configure(text=state, style=f"{state}.Badge.TLabel")
        self.progress.configure(value=(100 if state == "Completed" and self.session.outcome != "TIMEOUT"
                                       else min(100, 100 * self.session.sim.time / self.session.duration)))
        self.time_readout.set(f"{self.session.sim.time:.2f} / {self.session.duration:g} s")
        text = {"Ready": "Ready - select a task and press Start.",
                "Running": "Running - Pause to inspect the scene.",
                "Paused": "Paused - inspect, single-step, or resume.",
                "Completed": "Completed - the scene remains open. Reset to repeat or choose another task."}
        if state == "Completed" and self.session.outcome is not None:
            text[state] = f"{self.session.outcome} — the scene remains open. Reset to repeat or choose another task."
        self.status.set(self._notice or text[state])
        self.start_button.configure(text="Pause ⏸︎" if state == "Running" else "Resume" if state == "Paused" else "Start ▶",
                                    state="disabled" if state == "Completed" else "normal")
        self.step_button.configure(state="disabled" if state in ("Running", "Completed") else "normal")

    def _tick(self):
        if self.closed:
            return
        now = time.perf_counter()
        elapsed = min(now - self._last_tick, 0.1)
        self._last_tick = now
        if self.session.state == "Running":
            self._accumulator += elapsed
            while self._accumulator >= self.dt and self.session.state == "Running":
                if not self._guarded_step():
                    break
                self._accumulator -= self.dt
            self._after_steps()
        if self._dirty:
            self._render()
            self._dirty = False
        # Budget includes our work; do not add a full frame delay after drawing.
        remaining = 1 / 60 - (time.perf_counter() - now)
        self._timer = self.root.after(max(1, round(remaining * 1000)), self._tick)

    def _redraw(self, event=None):
        self._dirty = True

    def _begin_drag(self, event):
        self._drag_position = event.x, event.y

    def _orbit(self, event, *, pan=False):
        if self._drag_position:
            dx, dy = event.x - self._drag_position[0], event.y - self._drag_position[1]
            if pan or event.state & 0x0001:  # Tk Shift modifier, on Windows and Linux.
                self.camera.pan(dx, dy, self.canvas.winfo_width(), self.canvas.winfo_height())
            else:
                self.camera.orbit(dx, dy)
            self._drag_position = event.x, event.y
            self._redraw()
            self._queue_save()

    def _zoom(self, direction):
        self.camera.zoom(direction)
        self._redraw()
        self._queue_save()

    def reset_camera(self):
        self.camera.target = self._camera_home_target
        self.camera.reset()
        self._redraw()
        self._queue_save()

    def label_settings(self):
        if self._label_dialog is not None and self._label_dialog.winfo_exists():
            self._label_dialog.lift()
            return
        dialog = tk.Toplevel(self.root)
        self._label_dialog = dialog
        dialog.title("Label settings")
        dialog.transient(self.root)
        dialog.resizable(False, False)
        panel = ttk.Frame(dialog, padding=20)
        panel.pack(fill="both", expand=True)
        size = tk.StringVar(dialog, value=str(self.label_size))
        background = tk.BooleanVar(dialog, value=self.label_background)
        text_color = tk.StringVar(dialog, value=self.label_color)
        background_color = tk.StringVar(dialog, value=self.label_background_color)
        transparency = tk.StringVar(dialog, value=f"{self.label_background_transparency:g}")
        ttk.Label(panel, text="Text size (points)").grid(row=0, column=0, sticky="w", pady=6)
        ttk.Spinbox(panel, from_=8, to=36, textvariable=size, width=8).grid(row=0, column=1, sticky="w", padx=16)
        ttk.Checkbutton(panel, text="Show label background", variable=background).grid(row=1, column=0, columnspan=2, sticky="w", pady=10)

        def color_control(row, title, variable):
            ttk.Label(panel, text=title).grid(row=row, column=0, sticky="w", pady=6)
            row_controls = ttk.Frame(panel)
            row_controls.grid(row=row, column=1, sticky="w", padx=16)
            swatch = tk.Label(row_controls, background=variable.get(), width=3, relief="solid", borderwidth=1)
            swatch.pack(side="left", padx=(0, 8))
            def choose():
                _, selected = colorchooser.askcolor(color=variable.get(), title=title, parent=dialog)
                if selected:
                    variable.set(selected)
                    swatch.configure(background=selected)
            ttk.Button(row_controls, text="Choose...", command=choose).pack(side="left")

        color_control(2, "Text color", text_color)
        color_control(3, "Background color", background_color)
        ttk.Label(panel, text="Background transparency (%)").grid(row=4, column=0, sticky="w", pady=6)
        ttk.Spinbox(panel, from_=0, to=100, increment=5, textvariable=transparency, width=8).grid(row=4, column=1, sticky="w", padx=16)
        ttk.Label(panel, text="0 = opaque; 100 = clear. Text stays opaque.\nApplies to all frame and point labels.").grid(row=5, column=0, columnspan=2, pady=12)

        def apply():
            try:
                point_size = int(size.get())
                if not 8 <= point_size <= 36:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Text size", "Enter a whole number from 8 to 36.", parent=dialog)
                return
            try:
                percent = float(transparency.get())
                if not math.isfinite(percent) or not 0 <= percent <= 100:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Transparency", "Enter a percentage from 0 to 100.", parent=dialog)
                return
            self.set_label_style(size=point_size, color=text_color.get(),
                                 background=background.get(), background_color=background_color.get(),
                                 transparency=percent)

        buttons = ttk.Frame(panel)
        buttons.grid(row=6, column=0, columnspan=2, sticky="e")
        ttk.Button(buttons, text="Apply", command=apply).pack(side="left", padx=6)
        ttk.Button(buttons, text="Close", command=dialog.destroy).pack(side="left")

    def set_label_style(self, *, size, color, background, background_color, transparency=None):
        if not isinstance(size, int) or not 8 <= size <= 36:
            raise ValueError("Label size must be an integer between 8 and 36 points.")
        self.root.winfo_rgb(color)
        self.root.winfo_rgb(background_color)
        percent = self.label_background_transparency if transparency is None else transparency
        if not math.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError("Background transparency must be between 0 and 100 percent.")
        self.label_size = size
        self.label_color = color
        self.label_background = bool(background)
        self.label_background_color = background_color
        self.label_background_transparency = percent
        self._redraw()
        self._queue_save()

    def _label(self, point, text, offset=(8, -10)):
        screen = self.camera.project(point)
        if screen is None:
            return
        x, y = screen[0] + offset[0], screen[1] + offset[1]
        item = self.canvas.create_text(x, y, text=text, anchor="sw", fill=self.label_color,
                                      font=("TkDefaultFont", self.label_size, "bold"), tags=("overlay", "label-text"))
        bounds = self.canvas.bbox(item)
        if bounds:
            width, height = bounds[2] - bounds[0], bounds[3] - bounds[1]
            candidates = [offset, (-width-35, -25), (35, -height-45),
                          (-width-35, height+45), (35, height+45),
                          (-width/2, -2*height-70), (-width/2, 2*height+90)]
            for dx, dy in candidates:
                x = max(8, min(screen[0] + dx, self.camera.width - width - 8))
                y = max(height+8, min(screen[1] + dy, self.camera.height - 8))
                self.canvas.coords(item, x, y)
                bounds = self.canvas.bbox(item)
                if not any(bounds[0] < old[2]+12 and bounds[2] > old[0]-12
                           and bounds[1] < old[3]+12 and bounds[3] > old[1]-12
                           for old in self._label_bounds):
                    break
            self._label_bounds.append(bounds)
            # Quiet leaders keep the geometric vectors visually dominant.
            self.canvas.create_line(*screen, max(bounds[0], min(screen[0], bounds[2])),
                                    max(bounds[1], min(screen[1], bounds[3])),
                                    fill="#93a5b4", width=1, dash=(2, 3), tags="overlay")
            if self.label_background and self.label_background_transparency < 100:
                background = self._background_item((bounds[0]-4, bounds[1]-2, bounds[2]+4, bounds[3]+2))
                self.canvas.tag_lower(background, item)

    def _background_item(self, bounds):
        tags = ("overlay", "label-background")
        if self.label_background_transparency == 0:
            return self.canvas.create_rectangle(*bounds, fill=self.label_background_color, outline="", tags=tags)
        width, height = bounds[2] - bounds[0], bounds[3] - bounds[1]
        alpha = round(255 * (1 - self.label_background_transparency / 100))
        rgb = tuple(channel // 257 for channel in self.root.winfo_rgb(self.label_background_color))
        key = (width, height, rgb, alpha)
        if key not in self._label_images:
            png = solid_rgba_png(width, height, rgb, alpha)
            self._label_images[key] = tk.PhotoImage(data=png, format="PNG", master=self.root)
        self._label_images.move_to_end(key)
        image = self._label_images[key]
        while len(self._label_images) > 64:
            self._label_images.popitem(last=False)
        return self.canvas.create_image(bounds[0], bounds[1], image=image, anchor="nw", tags=tags)

    def _line(self, start, end, color, *, arrow=False, width=3, dash=None):
        a, b = self.camera.project(start), self.camera.project(end)
        if a is not None and b is not None:
            self.canvas.create_line(*a, *b, fill=color, width=width, dash=dash, arrow="last" if arrow else "none",
                                    arrowshape=(9, 11, 4), capstyle="round", tags="overlay")

    def _frame(self, transform, name, scale=0.5, offset=(8, -10)):
        for axis, color in enumerate(("#c92d35", "#14823e", "#245dcc")):
            self._line(transform[:3, 3], transform[:3, 3] + scale * transform[:3, axis], color, arrow=True)
        self._label(transform[:3, 3], name, offset=offset)

    def _draw_goal(self, position, label, color, *, heading=None, radius=0.1):
        circle = [(position[0] + radius * math.cos(a), position[1] + radius * math.sin(a), position[2])
                  for a in np.linspace(0, 2 * math.pi, 25)]
        for start, end in zip(circle, circle[1:]):
            self._line(start, end, color, width=2)
        if heading is not None:
            end = (position[0] + 0.35 * math.cos(heading), position[1] + 0.35 * math.sin(heading), position[2])
            self._line(position, end, color, arrow=True, width=2)
        self._label(position, label, offset=(12, -18))

    def _render(self):
        width, height = max(2, self.canvas.winfo_width()), max(2, self.canvas.winfo_height())
        ground_paths = ()
        if self.session.task == "square":
            ground_paths = (([(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)], "#b5660b"),)
        self.renderer.draw(self.session.world, self.camera, width, height, ground_paths=ground_paths)
        self.canvas.delete("overlay")
        self._label_bounds = []
        laboratory = self.session.laboratory
        if self.session.controller is not None:
            controller = self.session.controller
            for index, (gx, gy, heading) in enumerate(controller.goals):
                color = "#53958a" if index < controller.index else "#dd8b24" if index == controller.index else "#8b9eb1"
                self._draw_goal((gx, gy, 0.025), f"Goal {index + 1}", color, heading=heading)
        if laboratory is not None and self.session.lesson == "4":
            if laboratory.has_goal:
                self._draw_goal((*laboratory.target, 0.025), "Goal", "#dd8b24")
            else:
                for key, color in (("encoder", "#c88925"), ("imu", "#8558ae")):
                    points = laboratory.traces[key]
                    stride = max(1, len(points) // 250)
                    simplified = points[::stride] + [points[-1]]
                    for start, end in zip(simplified, simplified[1:]):
                        self._line(start, end, color, width=2)
        if self.session.lesson == "5":
            chains = laboratory.branches if len(laboratory.branches) else (laboratory.points,)
            for index, chain in enumerate(chains):
                color = "#087f83" if index == 0 else "#8558ae"
                for start, end in zip(chain, chain[1:]):
                    self._line(start, end, "#263f4b", width=11)
                    self._line(start, end, color, width=8)
                for joint in chain:
                    point = self.camera.project(joint)
                    if point is not None:
                        x, y = point
                        self.canvas.create_oval(x-5, y-5, x+5, y+5, fill="#f4f8fb", outline=color, width=2, tags="overlay")
            self._draw_goal(laboratory.target, "Target", "#dd8b24")
            for prediction in laboratory.predictions:
                self._draw_goal(prediction, "Prediction", "#8558ae", radius=0.04)
            if self.show_frames.get():
                self._frame(np.eye(4), "Arm base", scale=0.35)
                self._label(laboratory.points[-1], "End-effector")
            return
        if self.session.sensors is not None:
            lidar = self.session.sensors.lidar
            for endpoint, hit in zip(lidar.endpoints, lidar.hits):
                self._line(lidar.origin, endpoint, "#32947d" if hit else "#aab8c5",
                           width=1.5 if hit else 1, dash=None if hit else (3, 5))
                if hit:
                    screen = self.camera.project(endpoint)
                    if screen is not None:
                        x, y = screen
                        self.canvas.create_oval(x-2.5, y-2.5, x+2.5, y+2.5,
                                                fill="#16816c", outline=SURFACE, tags="overlay")
            if self.show_frames.get():
                self._label(lidar.origin, "Rangefinder" if len(lidar.angles) == 1 else "Lidar", offset=(12, -55))
        if self.session.lesson == "3":
            # Bound trail rendering cost even for long activities.
            samples = self.session.samples
            stride = max(1, len(samples) // 300)
            points = [(0, 0, 0.02)] + [(s[1], s[2], 0.02) for s in samples[::stride]]
            if samples:
                points.append((samples[-1][1], samples[-1][2], 0.02))
            for start, end in zip(points, points[1:]):
                self._line(start, end, "#b5660b")
        if self.show_frames.get():
            robot_frame = self.session.robot.pose().matrix()
            self._frame(np.eye(4), "World", offset=(-70, 35))
            self._frame(robot_frame, "Robot", offset=(12, -25))
            if self.session.task == "planar":
                world_from_sensor = self.session.planar_scene.sensor_frame()
                self._frame(world_from_sensor, "Sensor", scale=0.3, offset=(12, -55))
                output = self._planar_output()
                if output is not None:
                    point, direction = output
                    point_world = (*point, 0.03)
                    direction_world = (*direction, 0)
                    self._line(world_from_sensor[:3, 3], point_world, "#773baa")
                    self._line(world_from_sensor[:3, 3],
                               world_from_sensor[:3, 3] + np.asarray(direction_world), "#1976a5", arrow=True)
                    self._label(point_world, "Sensor point", offset=(12, 20))
                    self._label(world_from_sensor[:3, 3] + np.asarray(direction_world),
                                "Direction (no translation)", offset=(12, -15))
            elif self.session.task in module2.TASKS:
                output = self._module2_output()
                if output is not None:
                    frames, arrows, _ = output
                    for transform, name in frames:
                        self._frame(transform, name, scale=0.4, offset=(12, -40))
                    for start, end, name, color in arrows:
                        self._line(start, end, color, arrow=True)
                        self._label(end, name, offset=(12, 20))
            elif self.session.lesson == "2":
                sensor_frame = frames_activity.ROBOT_FROM_SENSOR
                world_sensor = robot_frame @ sensor_frame
                self._frame(world_sensor, "Sensor", scale=0.3, offset=(12, -55))
                try:
                    point = point_in_world(robot_frame, sensor_frame, frames_activity.POINT_IN_SENSOR)
                except NotImplementedError:
                    point = None
                if point is not None:
                    self._line(world_sensor[:3, 3], point, "#773baa")
                    self._label(point, "Sensor point", offset=(12, 20))

    def _update_plot_button(self):
        available = bool((self.session is not None and self.session.samples) or self.last_experiment)
        self.plot_button.configure(state="normal" if available else "disabled")

    def _remember_experiment(self):
        if self.session is None or not self.session.samples:
            return self.last_experiment
        key = (self._experiment_generation, self.session.sim.steps)
        if key != self._recorded_experiment_key:
            self.last_experiment = capture_experiment(self.session)
            self._recorded_experiment_key = key
        return self.last_experiment

    def _close_plot(self):
        if self._plot_canvas is not None:
            # A pending draw_idle must not outlive its Tk widget.
            pending = getattr(self._plot_canvas, "_idle_draw_id", None)
            if pending:
                self._plot_canvas.get_tk_widget().after_cancel(pending)
            self._plot_canvas = None
        if self._plot_window is not None and self._plot_window.winfo_exists():
            self._plot_window.destroy()
        self._plot_window = self._plot_figure = None

    def plot_results(self):
        snapshot = self._remember_experiment()
        if snapshot is None:
            messagebox.showinfo("Plot results", "Run an experiment first, then plot its recorded measurements.", parent=self.root)
            return
        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
            from .experiment_plots import create_experiment_figure
        except ImportError as error:
            messagebox.showerror("Plot results", "Matplotlib is required for experiment plots. "
                "Restart using the simulator launcher to update its private environment.\n\n" + str(error), parent=self.root)
            return
        figure = create_experiment_figure(snapshot)
        self._close_plot()
        self._plot_figure = figure
        window = tk.Toplevel(self.root)
        self._plot_window = window
        window.title("Experiment plots · " + TASKS[snapshot["lesson"]][snapshot["task"]])
        width = min(1120, max(700, self.root.winfo_screenwidth() - 120))
        height = min(820, max(520, self.root.winfo_screenheight() - 130))
        window.geometry(f"{width}x{height}")
        window.minsize(650, 480)
        panel = ttk.Frame(window, padding=(12, 10))
        panel.pack(fill="both", expand=True)
        elapsed = snapshot["metadata"]["elapsed"]
        ttk.Label(panel, text=f"Recorded experiment · {elapsed:.2f} s · "
                  "fixed snapshot; rerun Plot results to refresh", style="Footer.TLabel").pack(anchor="w", pady=(0, 8))
        canvas = FigureCanvasTkAgg(figure, master=panel)
        self._plot_canvas = canvas
        # Toolbar provides zoom/pan and saving the figure without exporting CSV first.
        toolbar = NavigationToolbar2Tk(canvas, panel, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(side="bottom", fill="x")
        canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        canvas.draw()
        window.protocol("WM_DELETE_WINDOW", self._close_plot)

    def export(self):
        destination = filedialog.asksaveasfilename(parent=self.root, title="Export measurements",
            defaultextension=".csv", filetypes=[("CSV measurements", "*.csv")],
            initialfile=f"lesson{self.session.lesson}_{self.session.task}.csv")
        if destination:
            try:
                self.session.export_csv(destination)
                extra = " (plus companion _sensors.csv)" if self.session.sensors is not None else ""
                self.status.set(f"Measurements exported to {destination}{extra}")
            except OSError as error:
                messagebox.showerror("Export failed", str(error), parent=self.root)

    def close(self):
        if self.closed:
            return
        if self._save_timer is not None:
            self.root.after_cancel(self._save_timer)
        self._save_settings()
        self.closed = True
        self.root.after_cancel(self._timer)
        self.root.after_cancel(self._restore_timer)
        if self.session:
            self.session.close()
        self._close_plot()
        self._label_images.clear()
        self.root.destroy()


def enable_high_dpi():
    """Keep native text sharp on Windows displays with scaling enabled."""
    import sys
    if sys.platform == "win32":
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass


def run_app(**kwargs):
    enable_high_dpi()
    root = tk.Tk()
    app = SimulatorApp(root, **kwargs)
    try:
        root.mainloop()
    finally:
        if not app.closed:
            app.close()
