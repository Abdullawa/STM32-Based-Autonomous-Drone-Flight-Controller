import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ---- config ----
PORT    = 'COM3'    # change to your port
BAUD    = 115200
HISTORY = 200       # samples visible on screen
# ----------------

data = {k: deque([0.0] * HISTORY, maxlen=HISTORY) for k in
        ['pitch', 'roll', 'yaw', 'pr', 'rr', 'yr', 'fl', 'fr', 'bl', 'br']}

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Connected to {PORT}")
except Exception as e:
    print(f"ERROR: Could not open {PORT}: {e}")
    exit()

fig, (ax_ang, ax_rate, ax_pwm) = plt.subplots(3, 1, figsize=(13, 8))
fig.suptitle('Drone Bench Telemetry', fontsize=13)

x = list(range(HISTORY))

# ---------------- Angles ----------------
ln_pitch, = ax_ang.plot(x, list(data['pitch']), label='Pitch', color='tab:red')
ln_roll,  = ax_ang.plot(x, list(data['roll']),  label='Roll',  color='tab:blue')
ln_yaw,   = ax_ang.plot(x, list(data['yaw']),   label='Yaw',   color='tab:green')

ax_ang.set_title('Angles (deg)')
ax_ang.set_ylim(-90, 90)
ax_ang.legend(loc='upper right', fontsize=8)
ax_ang.grid(True)

# Live values box
txt_ang = ax_ang.text(
    0.01, 0.95, "",
    transform=ax_ang.transAxes,
    verticalalignment='top',
    fontsize=9,
    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
)

# ---------------- Rates ----------------
ln_pr, = ax_rate.plot(x, list(data['pr']), label='Pitch Rate', color='tab:red')
ln_rr, = ax_rate.plot(x, list(data['rr']), label='Roll Rate',  color='tab:blue')
ln_yr, = ax_rate.plot(x, list(data['yr']), label='Yaw Rate',   color='tab:green')

ax_rate.set_title('Rates (deg/s)')
ax_rate.set_ylim(-50, 50)
ax_rate.legend(loc='upper right', fontsize=8)
ax_rate.grid(True)

# Live values box
txt_rate = ax_rate.text(
    0.01, 0.95, "",
    transform=ax_rate.transAxes,
    verticalalignment='top',
    fontsize=9,
    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
)

# ---------------- PWM ----------------
ln_fl, = ax_pwm.plot(x, list(data['fl']), label='FL', color='tab:red')
ln_fr, = ax_pwm.plot(x, list(data['fr']), label='FR', color='tab:blue')
ln_bl, = ax_pwm.plot(x, list(data['bl']), label='BL', color='tab:orange')
ln_br, = ax_pwm.plot(x, list(data['br']), label='BR', color='tab:green')

ax_pwm.set_title('PWM (us)')
ax_pwm.set_ylim(900, 2100)
ax_pwm.legend(loc='upper right', fontsize=8)
ax_pwm.grid(True)

# Live values box
txt_pwm = ax_pwm.text(
    0.01, 0.95, "",
    transform=ax_pwm.transAxes,
    verticalalignment='top',
    fontsize=9,
    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
)

all_lines = [
    ln_pitch, ln_roll, ln_yaw,
    ln_pr, ln_rr, ln_yr,
    ln_fl, ln_fr, ln_bl, ln_br
]

keys = [
    'pitch', 'roll', 'yaw',
    'pr', 'rr', 'yr',
    'fl', 'fr', 'bl', 'br'
]


def parse_line(raw):
    raw = raw.strip()

    if not (raw.startswith('/*') and raw.endswith('*/')):
        return None

    try:
        parts = raw[2:-2].split(',')

        if len(parts) != 10:
            return None

        return [float(p) for p in parts]

    except ValueError:
        return None


def update(_frame):

    # Drain serial buffer
    while ser.in_waiting:
        try:
            raw = ser.readline().decode('utf-8', errors='ignore')
        except Exception:
            break

        values = parse_line(raw)

        if values is None:
            continue

        for k, v in zip(keys, values):
            data[k].append(v)

    # Update graph lines
    for ln, k in zip(all_lines, keys):
        ln.set_ydata(list(data[k]))

    # Update live angle values
    txt_ang.set_text(
        f"Pitch: {data['pitch'][-1]:7.2f}\n"
        f"Roll : {data['roll'][-1]:7.2f}\n"
        f"Yaw  : {data['yaw'][-1]:7.2f}"
    )

    # Update live rate values
    txt_rate.set_text(
        f"PR: {data['pr'][-1]:7.2f}\n"
        f"RR: {data['rr'][-1]:7.2f}\n"
        f"YR: {data['yr'][-1]:7.2f}"
    )

    # Update live PWM values
    txt_pwm.set_text(
        f"FL: {data['fl'][-1]:4.0f}\n"
        f"FR: {data['fr'][-1]:4.0f}\n"
        f"BL: {data['bl'][-1]:4.0f}\n"
        f"BR: {data['br'][-1]:4.0f}"
    )

    return all_lines + [txt_ang, txt_rate, txt_pwm]


ani = animation.FuncAnimation(
    fig,
    update,
    interval=50,
    blit=True
)

plt.tight_layout()
plt.show()

ser.close()