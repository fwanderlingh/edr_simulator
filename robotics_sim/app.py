"""Single-window teaching application with crisp, screen-space annotations."""

import math
import time
import tkinter as tk
from collections import OrderedDict
from tkinter import colorchooser, filedialog, messagebox, ttk

import numpy as np

from .exercises.lesson02.frames import point_in_world
from .exercises.lesson02.planar import (
    PLANAR_DIRECTION_IN_SENSOR,
    PLANAR_GOAL_IN_WORLD,
    PLANAR_POINT_IN_SENSOR,
    PLANAR_ROBOT_FROM_SENSOR,
    PLANAR_WORLD_FROM_ROBOT,
    solve_planar_task,
)
from .geometry.rotations import rot_z
from .geometry.transforms import homogeneous
from .session import ActivitySession, DESCRIPTIONS, LESSONS, TASKS
from .sim.camera import Camera
from .sim.scene_renderer import SceneRenderer
from .preferences import DEFAULT_PATH, load_preferences, save_preferences
from .label_image import solid_rgba_png


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
        self.show_frames = tk.BooleanVar(value=saved.get("show_frames", True))
        self.label_size = int(saved.get("label_size", 12))
        self.label_color = self._valid_color(saved.get("label_color"), "#172d3d")
        self.label_background = saved.get("label_background", True)
        self.label_background_color = self._valid_color(saved.get("label_background_color"), "#ffffff")
        self.label_background_transparency = saved.get("label_background_transparency", 0)
        self._label_images = OrderedDict()
        self._label_dialog = None
        root.title("Robotics Laboratory")
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
                "sensor_mode": self.sensor_mode.get(),
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
            lesson_menu.add_radiobutton(label=title, variable=self.lesson, value=title, command=self.select_lesson)
        menu.add_cascade(label="Lessons", menu=lesson_menu)
        view_menu = tk.Menu(menu, tearoff=False)
        view_menu.add_checkbutton(label="Show coordinate frames", variable=self.show_frames, command=self._display_settings_changed)
        view_menu.add_command(label="Label settings...", command=self.label_settings)
        view_menu.add_command(label="Reset camera", command=self.reset_camera)
        menu.add_cascade(label="View", menu=view_menu)
        help_menu = tk.Menu(menu, tearoff=False)
        help_menu.add_command(label="How to use", command=lambda: messagebox.showinfo(
            "Robotics Laboratory", "Select a lesson and a task, then press Start.\n\n"
            "Pause freezes simulation time. Step advances one physics step.\n"
            "Reset starts a fresh activity and clears measurements.\n"
            "Completed activities stay open for inspection.\n\n"
            "Drag the view to orbit; scroll to zoom.\n"
            "Use View > Label settings to change label size, colors, and backgrounds.", parent=self.root))
        menu.add_cascade(label="Help", menu=help_menu)
        self.root.configure(menu=menu)

    def _build_widgets(self):
        panel = ttk.Frame(self.root, padding=16)
        panel.pack(fill="both", expand=True)
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(4, weight=1)
        ttk.Label(panel, text="Robotics Laboratory", font=("TkDefaultFont", 20, "bold")).grid(row=0, column=0, sticky="w")
        selectors = ttk.Frame(panel)
        selectors.grid(row=1, column=0, sticky="ew", pady=(12, 8))
        ttk.Label(selectors, text="Lesson").grid(row=0, column=0, sticky="w")
        ttk.Label(selectors, text="Task").grid(row=0, column=1, sticky="w", padx=12)
        ttk.Label(selectors, text="Activity length (s)").grid(row=0, column=2, sticky="w")
        lessons = ttk.Combobox(selectors, textvariable=self.lesson, values=list(LESSONS.values()), state="readonly", width=34)
        lessons.grid(row=1, column=0, sticky="ew")
        lessons.bind("<<ComboboxSelected>>", self.select_lesson)
        self.tasks = ttk.Combobox(selectors, textvariable=self.task, state="readonly", width=32)
        self.tasks.grid(row=1, column=1, sticky="ew", padx=12)
        self.tasks.bind("<<ComboboxSelected>>", self.load_task)
        duration_entry = ttk.Entry(selectors, textvariable=self.duration, width=12)
        duration_entry.grid(row=1, column=2, sticky="w")
        duration_entry.bind("<FocusOut>", self._queue_save)
        duration_entry.bind("<Return>", self._queue_save)
        ttk.Label(selectors, text="Lesson 3 kinematics").grid(row=0, column=3, padx=12, sticky="w")
        self.model_selector = ttk.Combobox(selectors, textvariable=self.kinematics,
            values=("reference", "student"), state="readonly", width=12)
        self.model_selector.grid(row=1, column=3, padx=12, sticky="w")
        self.model_selector.bind("<<ComboboxSelected>>", self.load_task)
        self.sensor_controls = ttk.Frame(selectors)
        self.sensor_controls.grid(row=2, column=0, columnspan=4, sticky="w", pady=(8, 0))
        ttk.Label(self.sensor_controls, text="Sensor mode").pack(side="left")
        sensor_selector = ttk.Combobox(self.sensor_controls, textvariable=self.sensor_mode,
            values=("ideal", "noisy", "student"), state="readonly", width=12)
        sensor_selector.pack(side="left", padx=10)
        sensor_selector.bind("<<ComboboxSelected>>", self.load_task)
        ttk.Label(self.sensor_controls, text="Nominal 10 Hz | Student mode replaces lidar | Mode change resets activity").pack(side="left")
        selectors.columnconfigure(0, weight=1)
        selectors.columnconfigure(1, weight=1)
        description = ttk.Label(panel, textvariable=self.description, wraplength=980)
        description.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        panel.bind("<Configure>", lambda event: description.configure(wraplength=max(300, event.width - 32)))
        controls = ttk.Frame(panel)
        controls.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        self.start_button = ttk.Button(controls, text="Start", command=self.toggle_running)
        self.start_button.pack(side="left")
        self.step_button = ttk.Button(controls, text="Step", command=self.single_step)
        self.step_button.pack(side="left", padx=6)
        ttk.Button(controls, text="Reset / Apply length", command=self.load_task).pack(side="left")
        ttk.Button(controls, text="Export CSV...", command=self.export).pack(side="left", padx=6)
        ttk.Button(controls, text="Reset camera", command=self.reset_camera).pack(side="right")
        ttk.Button(controls, text="Labels...", command=self.label_settings).pack(side="right", padx=6)
        self.canvas = tk.Canvas(panel, background="#edf1f4", highlightthickness=1, highlightbackground="#bcc8d0")
        self.canvas.grid(row=4, column=0, sticky="nsew")
        self.renderer = SceneRenderer(self.canvas)
        self.canvas.bind("<Configure>", self._redraw)
        self.canvas.bind("<ButtonPress-1>", self._begin_drag)
        self.canvas.bind("<B1-Motion>", self._orbit)
        self.canvas.bind("<MouseWheel>", lambda event: self._zoom(1 if event.delta > 0 else -1))
        self.canvas.bind("<Button-4>", lambda event: self._zoom(1))
        self.canvas.bind("<Button-5>", lambda event: self._zoom(-1))
        ttk.Label(panel, text="Drag to orbit  |  Scroll to zoom  |  Axes: x red, y green, z blue").grid(row=5, column=0, sticky="w", pady=(6, 0))
        ttk.Label(panel, textvariable=self.values, font=("TkFixedFont", 11)).grid(row=6, column=0, sticky="w", pady=6)
        ttk.Label(panel, textvariable=self.status, wraplength=1000).grid(row=7, column=0, sticky="ew")

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
        try:
            new_session = ActivitySession(self.lesson_id, task, duration=duration, dt=self.dt,
                                          kinematics=self.kinematics.get(), sensor_mode=self.sensor_mode.get())
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
            self.session.close()
        self.session = new_session
        self.model_selector.configure(state="readonly" if self.lesson_id == "3" else "disabled")
        if self.lesson_id == "4":
            self.sensor_controls.grid()
        else:
            self.sensor_controls.grid_remove()
        if self.lesson_id == "2":
            self.camera.target = (2, 1, 0.2) if task == "planar" else (0.25, 0, 0.2)
        else:
            self.camera.target = (0.8, 0.7, 0.1)
        self.description.set(DESCRIPTIONS[task])
        self._notice = ""
        self._accumulator = 0
        self._last_tick = time.perf_counter()
        self._redraw()
        self._update_status()

        self._queue_save()

    def toggle_running(self):
        if self.session.state == "Running":
            self.session.pause()
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
            self._notice = f"Exercise not implemented yet: {error}"
            return False
        return True

    def _planar_output(self):
        try:
            return solve_planar_task(
                PLANAR_WORLD_FROM_ROBOT, PLANAR_ROBOT_FROM_SENSOR,
                PLANAR_POINT_IN_SENSOR, PLANAR_DIRECTION_IN_SENSOR,
                PLANAR_GOAL_IN_WORLD)
        except NotImplementedError:
            return None

    def _update_status(self):
        pose = self.session.robot.pose()
        self.values.set(f"Time: {self.session.sim.time:.2f} s    x: {pose.x:.3f} m    y: {pose.y:.3f} m    yaw: {pose.yaw:.3f} rad")
        if self.session.task == "planar":
            output = self._planar_output()
            if output is None:
                self.values.set(self.values.get() +
                    "\nModule 1 not implemented yet: complete solve_planar_task in exercises/lesson02/planar.py")
            else:
                point, direction, goal = output
                self.values.set(self.values.get() +
                    f"\nModule 1 output: point in world ({point[0]:.2f}, {point[1]:.2f}) m"
                    f" | direction in world ({direction[0]:.2f}, {direction[1]:.2f})")
                self.values.set(self.values.get() +
                    f"\nGoal in robot: ({goal[0]:.2f}, {goal[1]:.2f}) m")
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
        state = self.session.state
        text = {"Ready": "Ready - select a task and press Start.",
                "Running": "Running - Pause to inspect the scene.",
                "Paused": "Paused - inspect, single-step, or resume.",
                "Completed": "Completed - the scene remains open. Reset to repeat or choose another task."}
        self.status.set(self._notice or text[state])
        self.start_button.configure(text="Pause" if state == "Running" else "Resume" if state == "Paused" else "Start",
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

    def _orbit(self, event):
        if self._drag_position:
            self.camera.orbit(event.x - self._drag_position[0], event.y - self._drag_position[1])
            self._drag_position = event.x, event.y
            self._redraw()
            self._queue_save()

    def _zoom(self, direction):
        self.camera.zoom(direction)
        self._redraw()
        self._queue_save()

    def reset_camera(self):
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
            '''Dashed line from label to point, clipped to canvas bounds.'''
            self.canvas.create_line(*screen, max(bounds[0], min(screen[0], bounds[2])),
                                    max(bounds[1], min(screen[1], bounds[3])),
                                    fill="#34495e", width=3, dash=(6, 3), tags="overlay")
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

    def _line(self, start, end, color, *, arrow=False):
        a, b = self.camera.project(start), self.camera.project(end)
        if a is not None and b is not None:
            self.canvas.create_line(*a, *b, fill=color, width=3, arrow="last" if arrow else "none", tags="overlay")

    def _frame(self, transform, name, scale=0.5, offset=(8, -10)):
        for axis, color in enumerate(("#c92d35", "#14823e", "#245dcc")):
            self._line(transform[:3, 3], transform[:3, 3] + scale * transform[:3, axis], color, arrow=True)
        self._label(transform[:3, 3], name, offset=offset)

    def _render(self):
        width, height = max(2, self.canvas.winfo_width()), max(2, self.canvas.winfo_height())
        self.renderer.draw(self.session.world, self.camera, width, height)
        self.canvas.delete("overlay")
        self._label_bounds = []
        if self.session.sensors is not None:
            lidar = self.session.sensors.lidar
            for endpoint, hit in zip(lidar.endpoints, lidar.hits):
                self._line(lidar.origin, endpoint, "#14823e" if hit else "#99a4ae")
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
        if self.session.task == "square":
            path = [(0, 0, 0.02), (2, 0, 0.02), (2, 2, 0.02), (0, 2, 0.02), (0, 0, 0.02)]
            for start, end in zip(path, path[1:]):
                self._line(start, end, "#b5660b")
        if self.show_frames.get():
            robot_frame = self.session.robot.pose().matrix()
            self._frame(np.eye(4), "World", offset=(-70, 35))
            self._frame(robot_frame, "Robot", offset=(12, -25))
            if self.session.task == "planar":
                world_from_robot = np.eye(4)
                world_from_robot[:2, :2] = PLANAR_WORLD_FROM_ROBOT[:2, :2]
                world_from_robot[:2, 3] = PLANAR_WORLD_FROM_ROBOT[:2, 2]
                robot_from_sensor = np.eye(4)
                robot_from_sensor[:2, :2] = PLANAR_ROBOT_FROM_SENSOR[:2, :2]
                robot_from_sensor[:2, 3] = PLANAR_ROBOT_FROM_SENSOR[:2, 2]
                world_from_sensor = world_from_robot @ robot_from_sensor
                self._frame(world_from_sensor, "Sensor", scale=0.3, offset=(12, -55))
                output = self._planar_output()
                if output is not None:
                    point, direction, _ = output
                    point_world = (*point, 0.03)
                    goal_world = (*PLANAR_GOAL_IN_WORLD, 0.03)
                    direction_world = (*direction, 0)
                    self._line(world_from_sensor[:3, 3], point_world, "#773baa")
                    self._line(world_from_sensor[:3, 3],
                               world_from_sensor[:3, 3] + np.asarray(direction_world), "#1976a5", arrow=True)
                    self._line(world_from_robot[:3, 3], goal_world, "#b5660b")
                    self._label(point_world, "Sensor point", offset=(12, 20))
                    self._label(goal_world, "World goal", offset=(12, 20))
                    self._label(world_from_sensor[:3, 3] + np.asarray(direction_world),
                                "Direction (no translation)", offset=(12, -15))
            elif self.session.lesson == "2":
                sensor_frame = homogeneous(rot_z(np.pi / 4), (0.25, 0, 0.3))
                world_sensor = robot_frame @ sensor_frame
                self._frame(world_sensor, "Sensor", scale=0.3, offset=(12, -55))
                try:
                    point = point_in_world(robot_frame, sensor_frame, np.array([0.6, 0.2, 0.1]))
                except NotImplementedError:
                    point = None
                if point is not None:
                    self._line(world_sensor[:3, 3], point, "#773baa")
                    self._label(point, "Sensor point", offset=(12, 20))

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
