"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: hsv-p_tuner.py

Title: HSV + P-Controller Tuner (single-window Tk edition)

Purpose: Live-tune the HSV color threshold and P-controller gains used by the
line-following labs. All UI lives in one Tk window: sliders on the left, mask
and camera/contour previews on the right, line-position plot below.

Controls:
    Right trigger = release RACECAR failsafe (drive)
    A button      = save tuned HSV + speed/angle values
    B button      = print current speed/angle
"""

########################################################################################
# Imports
########################################################################################

import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

import cv2 as cv
import matplotlib
matplotlib.use('TkAgg')  # noqa: E402  -- backend must be set before pyplot import
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageTk  # noqa: E402

import racecar_core  # noqa: E402
import racecar_utils as rc_utils  # noqa: E402

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# HSV bounds
H_low, H_high = 0, 179
S_low, S_high = 0, 255
V_low, V_high = 0, 255

# Camera frame dimensions used in update() — larger than v1's 320x240 so
# the previews are readable inside the embedded Tk window.
FRAME_W, FRAME_H = 640, 480

# Mask preview is downscaled before display to keep it from dominating the
# UI; tuning still happens against the full-resolution mask internally.
MASK_PREVIEW_W, MASK_PREVIEW_H = 320, 240

# Camera + contour (cropped floor strip) preview is also downscaled.
DISPLAY_PREVIEW_W, DISPLAY_PREVIEW_H = 480, 90

# Cropped floor strip: bottom (FRAME_H - CROP_TOP) px of the resized frame.
CROP_TOP = 360
CROP_FLOOR = ((CROP_TOP, 0), (FRAME_H, FRAME_W))

# P-control gains (0..1)
tune_speed = 0.5  # default 50% so the car drives as soon as the trigger is held
tune_angle = 0.5

# Rolling line-position history
HIST_LEN = 300
loc_history = [0] * HIST_LEN

# Per-frame outputs (read by tk refresh callback)
speed = 0.0
angle = 0.0
contour_center = None
contour_area = 0
MIN_CONTOUR_AREA = 30

# Tk widget handles populated by build_tk_ui(). update() pushes frames into
# `latest_mask_bgr` and `latest_display_bgr`; the Tk refresh callback reads
# them and pushes them onto the Label widgets. None until UI is built.
tk_root = None
mask_label = None
display_label = None
latest_mask_bgr = None
latest_display_bgr = None
# Keep PhotoImage refs alive — Tk drops images when the ref is GC'd.
_mask_photo_ref = None
_display_photo_ref = None

# Flag flipped by start() to deiconify the Tk window. Until then the
# window stays withdrawn so the terminal stays visible/focused.
_ui_visible_requested = False


########################################################################################
# Slider callbacks (run on tk's thread; just mutate globals)
########################################################################################

def on_low_h_change(val):
    global H_low
    H_low = int(float(val))


def on_low_s_change(val):
    global S_low
    S_low = int(float(val))


def on_low_v_change(val):
    global V_low
    V_low = int(float(val))


def on_high_h_change(val):
    global H_high
    H_high = int(float(val))


def on_high_s_change(val):
    global S_high
    S_high = int(float(val))


def on_high_v_change(val):
    global V_high
    V_high = int(float(val))


def on_speed_change(val):
    global tune_speed
    tune_speed = float(val) / 100


def on_angsens_change(val):
    global tune_angle
    tune_angle = float(val) / 100


########################################################################################
# Tk UI construction (one window, four panes)
########################################################################################

def _make_slider(parent, label, from_, to, start, command, custom_font):
    """Build a single labeled slider inside parent. Returns the ttk.Scale."""
    frame = ttk.Frame(parent, style="Outline.TFrame")
    frame.pack(fill='x', expand=False, pady=4)

    tk.Label(frame, text=str(from_), bg='black', fg='white', font=custom_font
             ).pack(side='left')
    tk.Label(frame, text=str(to), bg='black', fg='white', font=custom_font
             ).pack(side='right')
    current_value_label = tk.Label(frame, text=str(start),
                                   bg='black', fg='white', font=custom_font)
    current_value_label.pack(side='top')

    def _on_change(v, lbl=current_value_label):
        command(v)
        lbl.config(text=f"{int(float(v))}")

    scale = ttk.Scale(frame, from_=from_, to=to, orient='horizontal',
                      style="Horizontal.TScale", command=_on_change)
    scale.set(start)
    scale.pack(fill='x', expand=True, padx=4, pady=2)

    tk.Label(frame, text=label, bg='black', fg='white', font=custom_font
             ).pack(side='bottom')
    return scale


def build_tk_ui():
    """Construct the single-window Tk UI. Must be called on the main thread."""
    global tk_root, mask_label, display_label

    root = tk.Tk()
    tk_root = root
    root.title("HSV + P-Controller Tuner")
    root.configure(background='black')
    # Hide until start() flips _ui_visible_requested. Keeps the terminal
    # uncovered so the user can read the start-message prompts first.
    root.withdraw()
    # Target screen size — covers the bulk of a 1920x1080 HDMI display.
    # We re-assert geometry after content packs (see end of build_tk_ui).
    root.geometry("1000x620+30+30")

    title_font = tkfont.Font(family="Helvetica", size=18, weight="bold")
    subtitle_font = tkfont.Font(family="Helvetica", size=13, weight="bold",
                                slant="italic")
    custom_font = tkfont.Font(family="Helvetica", size=11, weight="bold")

    style = ttk.Style()
    style.theme_use('clam')
    style.configure("Horizontal.TScale", background='black',
                    troughcolor='grey', bordercolor='white')
    style.configure("TLabel", background='black', foreground='white',
                    font=custom_font)
    style.configure("Outline.TFrame", background='black', borderwidth=1,
                    relief='solid')

    # ─── Left column: titles + sliders ──────────────────────────────────────
    # Height shrink-wraps to content; width pinned by zero-height spacer.
    left = tk.Frame(root, bg='black')
    left.grid(row=0, column=0, sticky='nw', padx=10, pady=10)
    tk.Frame(left, bg='black', width=480, height=1).pack(side='top')

    tk.Label(left, text="RACECAR Neo Demo UI", background='black',
             foreground='red', font=title_font).pack()
    tk.Label(left, text="Line Following Tuner App", background='black',
             foreground='orange', font=subtitle_font).pack()

    _make_slider(left, "H_Low", 1, 179, 1, on_low_h_change, custom_font)
    _make_slider(left, "H_High", 1, 179, 179, on_high_h_change, custom_font)
    _make_slider(left, "S_Low", 1, 255, 1, on_low_s_change, custom_font)
    _make_slider(left, "S_High", 1, 255, 255, on_high_s_change, custom_font)
    _make_slider(left, "V_Low", 1, 255, 1, on_low_v_change, custom_font)
    _make_slider(left, "V_High", 1, 255, 255, on_high_v_change, custom_font)
    _make_slider(left, "Speed (%)", 0, 100, 50, on_speed_change, custom_font)
    _make_slider(left, "Angle Sensitivity (%)", 0, 100, 50,
                 on_angsens_change, custom_font)

    # ─── Right column: mask + camera/contour + line plot, stacked ──────────
    right = tk.Frame(root, bg='black')
    right.grid(row=0, column=1, sticky='nw', padx=6, pady=6)

    # NB: tk.Label width/height are character-cell units when no image is
    # set. Omit them entirely so the label sizes to whatever PhotoImage we
    # configure() onto it. This keeps the previews at their native pixel
    # dimensions (FRAME_W x FRAME_H for the mask, cropped for display).
    tk.Label(right, text="HSV Mask", bg='black', fg='white',
             font=custom_font).grid(row=0, column=0, sticky='w')
    mask_label = tk.Label(right, bg='black')
    mask_label.grid(row=1, column=0, padx=2, pady=2, sticky='w')

    tk.Label(right, text="Camera + Contour (cropped floor strip)",
             bg='black', fg='white', font=custom_font
             ).grid(row=2, column=0, sticky='w', pady=(8, 0))
    display_label = tk.Label(right, bg='black')
    display_label.grid(row=3, column=0, padx=2, pady=2, sticky='w')

    # ─── Plot row inside the right column, below the camera/contour ─────────
    tk.Label(right, text="Line Position", bg='black', fg='white',
             font=custom_font).grid(row=4, column=0, sticky='w', pady=(8, 0))
    bottom = tk.Frame(right, bg='black')
    bottom.grid(row=5, column=0, sticky='nw', padx=2, pady=2)

    fig, ax = plt.subplots(figsize=(4.8, 1.8), dpi=100)
    fig.patch.set_facecolor('black')
    ax.set_facecolor('black')
    ax.tick_params(colors='white')
    for spine in ax.spines.values():
        spine.set_color('white')

    line, = ax.plot(range(HIST_LEN), list(loc_history),
                    color='cyan', label="Line Position")
    setpoint_line = ax.axhline(y=FRAME_W // 2, color='red', linestyle='--',
                               label='Setpoint')
    ax.set_xlabel("Frame", color='white')
    ax.set_ylabel("Px", color='white')
    ax.set_ylim(0, FRAME_W)
    ax.set_xlim(0, HIST_LEN - 1)
    ax.legend(loc='upper right', facecolor='black', edgecolor='white',
              labelcolor='white', fontsize=8)
    fig.tight_layout()

    canvas = FigureCanvasTkAgg(fig, master=bottom)
    canvas.get_tk_widget().pack()

    def _update_plot(_frame):
        line.set_ydata(loc_history)
        return line, setpoint_line

    # Keep a ref so the animation doesn't get GC'd.
    bottom._anim = FuncAnimation(fig, _update_plot, interval=66, blit=True,
                                 cache_frame_data=False)

    # Force the requested geometry after content has packed. Tk otherwise
    # often shrinks the window back to content-fit size on first draw.
    root.update_idletasks()
    root.geometry("1000x620+30+30")


def _bgr_to_photo(bgr):
    """Convert a BGR numpy image to a Tk-compatible PhotoImage."""
    rgb = cv.cvtColor(bgr, cv.COLOR_BGR2RGB)
    return ImageTk.PhotoImage(Image.fromarray(rgb))


def _tk_refresh():
    """Push the latest mask + display frames into their Tk Labels."""
    global _mask_photo_ref, _display_photo_ref

    # Deiconify the window the first time start() has been pressed.
    if _ui_visible_requested and tk_root.state() == 'withdrawn':
        tk_root.deiconify()

    # Wrap each image conversion so a malformed numpy frame can't crash the
    # Tk event loop and freeze the entire UI. update() can produce empty or
    # off-shape arrays in the moments before the camera node settles.
    try:
        if latest_mask_bgr is not None and latest_mask_bgr.size > 0:
            mask_small = cv.resize(latest_mask_bgr,
                                   (MASK_PREVIEW_W, MASK_PREVIEW_H))
            _mask_photo_ref = _bgr_to_photo(mask_small)
            mask_label.configure(image=_mask_photo_ref)
    except (cv.error, ValueError):
        pass

    try:
        if latest_display_bgr is not None and latest_display_bgr.size > 0:
            display_small = cv.resize(latest_display_bgr,
                                      (DISPLAY_PREVIEW_W, DISPLAY_PREVIEW_H))
            _display_photo_ref = _bgr_to_photo(display_small)
            display_label.configure(image=_display_photo_ref)
    except (cv.error, ValueError):
        pass

    tk_root.after(33, _tk_refresh)  # ~30 Hz refresh


########################################################################################
# Contour detection (writes into latest_display_bgr instead of rc.display)
########################################################################################

def update_contour(img):
    global contour_center, contour_area, latest_display_bgr

    image = rc_utils.crop(img, CROP_FLOOR[0], CROP_FLOOR[1])

    if image is None:
        contour_center = None
        contour_area = 0
        return

    # Live HSV bounds from the sliders — changes take effect immediately,
    # no controller-button save step required.
    contours = rc_utils.find_contours(image,
                                      (H_low, S_low, V_low),
                                      (H_high, S_high, V_high))
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if contour is not None:
        contour_center = rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)
        rc_utils.draw_contour(image, contour)
        if contour_center is not None:
            rc_utils.draw_circle(image, contour_center)
    else:
        contour_center = None
        contour_area = 0

    latest_display_bgr = image


########################################################################################
# RACECAR start/update
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global _ui_visible_requested
    rc.drive.set_speed_angle(0, 0)
    _ui_visible_requested = True  # _tk_refresh will deiconify the window
    print(
        ">> RACECAR Neo OneShot Demo - Line Follower with Live HSV Tuning\n"
        "\n"
        "Slider changes take effect immediately — no save step needed.\n"
        "\n"
        "Controls:\n"
        "   Right trigger = release RACECAR failsafe (drive)\n"
        "   A button = print current HSV + speed/angle to terminal\n"
        "   B button = print system speed/angle"
    )


# [FUNCTION] Tune HSV values to RACECAR camera while in main loop
def update():
    global loc_history, speed, angle, latest_mask_bgr

    img = rc.camera.get_color_image()
    # The camera node may not have published yet on the first few frames
    # after start(); skip the rest of update() until a real frame arrives.
    if img is None or img.size == 0:
        return
    img = cv.resize(img, (FRAME_W, FRAME_H))
    hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    hsv_low = np.array([H_low, S_low, V_low], np.uint8)
    hsv_high = np.array([H_high, S_high, V_high], np.uint8)
    mask = cv.inRange(hsv_image, hsv_low, hsv_high)
    latest_mask_bgr = cv.bitwise_and(img, img, mask=mask)

    update_contour(img)

    # P-controller — reads live tune_angle directly from the slider.
    # Setpoint is the horizontal center of the processed frame; must scale
    # with FRAME_W (a contour exactly centered must yield error=0, else
    # the car steers hard one direction even when it's lined up correctly).
    if contour_center is not None:
        setpoint = FRAME_W // 2
        error = setpoint - contour_center[1]
        kp = -(2 / setpoint) * tune_angle * 2
        angle = rc_utils.clamp(kp * error, -1, 1)

    # Live speed from the slider; no controller-button save step required.
    speed = tune_speed

    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0.1:
        rc.drive.set_speed_angle(speed, angle)
        if len(loc_history) >= HIST_LEN:
            loc_history.pop(-1)
        loc_history.insert(0,
                           contour_center[1] if contour_center is not None
                           else 0)
    else:
        rc.drive.set_speed_angle(0, 0)

    # A: snapshot the current tuning to the terminal so the student can
    # transcribe it. Tuning itself is already applied live from the sliders.
    if rc.controller.was_pressed(rc.controller.Button.A):
        print(f"HSV Threshold: ({H_low}, {S_low}, {V_low}), "
              f"({H_high}, {S_high}, {V_high})")
        print(f"Speed/Angle: tune_speed={tune_speed:.2f}, "
              f"tune_angle={tune_angle:.2f}")

    if rc.controller.was_pressed(rc.controller.Button.B):
        print(f"System Speed/Angle: Speed={speed:.2f}, Angle={angle:.2f}")


########################################################################################
# Entry point
########################################################################################

# Tk MUST run on the main thread, and rc.go() also blocks the main thread
# spinning the ROS executor. We resolve this by:
#   1. Building the Tk UI on the main thread first.
#   2. Starting rc.go() on a background thread (rc's __run_thread keeps
#      firing update() via its internal executor regardless).
#   3. Running tk_root.mainloop() on the main thread so Tk owns the GUI
#      event loop. The mainloop polls update()'s outputs via _tk_refresh.

if __name__ == "__main__":
    import threading

    rc.set_start_update(start, update)

    build_tk_ui()

    # rc.go() spins the executor synchronously; run it off-main so Tk owns
    # the main thread. rc's internal __run_thread continues to call update()
    # on its own. rc.go() never returns until BACK+START exits the program,
    # so daemon=True ensures it dies with the Tk window.
    rc_thread = threading.Thread(target=rc.go, daemon=True)
    rc_thread.start()

    # Kick off the Tk-side refresh loop and enter the mainloop.
    tk_root.after(100, _tk_refresh)
    tk_root.mainloop()
