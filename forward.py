# ===============================================================================
# ИМПОРТ БИБЛИОТЕК
# ===============================================================================
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
from fractions import Fraction
import textwrap

# ===============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНЫХ ВЫЧИСЛЕНИЙ
# ===============================================================================
def to_fraction(value, max_denominator=1000000):
    """Float yoki stringni aniq kasrga aylantiradi"""
    if isinstance(value, str):
        # Kasr formati: "surat/maxraj" yoki oddiy son
        if '/' in value:
            parts = value.split('/')
            return Fraction(int(parts[0]), int(parts[1]))
        else:
            return Fraction(float(value)).limit_denominator(max_denominator)
    else:
        return Fraction(value).limit_denominator(max_denominator)

def format_value(value):
    """Sonni kasr ko'rinishida formatlaydi"""
    frac = to_fraction(float(value), max_denominator=1000000)
    if frac.denominator == 1:
        return str(frac.numerator)
    else:
        return f"{frac.numerator}/{frac.denominator}"

def format_high_precision(value):
    """10^-9 gacha aniqlikda formatlash (kasr bilan birga)"""
    frac = to_fraction(float(value), max_denominator=1000000)
    decimal = f"{float(value):.9f}"
    return f"{frac.numerator}/{frac.denominator} (≈{decimal})"

def format_kasr(value):
    """Faqat kasr ko'rinishida qaytaradi"""
    frac = to_fraction(float(value), max_denominator=1000000)
    return f"{frac.numerator}/{frac.denominator}"

# ===============================================================================
# ROBOT O'LCHAMLARI VA CHEKLOVLARI
# ===============================================================================
L = Fraction(30, 1)  # 30 sm - aniq kasr

# h_CB cheklovlari
H_CB_MIN = Fraction(30, 1)
H_CB_MAX = Fraction(57, 1)

# Burchak cheklovlari
BETA_MIN_DEG = Fraction(27, 1)
BETA_MAX_DEG = Fraction(120, 1)
GAMMA_MIN_DEG = Fraction(-60, 1)
GAMMA_MAX_DEG = Fraction(120, 1)
ELBOW_MIN_DEG = Fraction(30, 1)
ELBOW_MAX_DEG = Fraction(160, 1)

# ===============================================================================
# TO'G'RI KINEMATIKA (FK) - KASRLAR BILAN
# ===============================================================================
def forward_kinematics(alpha_deg, beta_deg, gamma_deg, h_CB, oa_dist):
    """
    TO'G'RI KINEMATIKA - KASRLAR BILAN ANIQ HISOBLASH
    """
    
    # Kasrlarga aylantirish
    alpha_deg = float(alpha_deg)
    beta_deg = float(beta_deg)
    gamma_deg = float(gamma_deg)
    h_CB = float(h_CB)
    oa_dist = float(oa_dist)
    
    # h_CB cheklovlari
    if h_CB < float(H_CB_MIN) or h_CB > float(H_CB_MAX):
        return None, None, None, None, None, None, None, None, False, \
               f"h_CB = {format_value(h_CB)} sm chegaradan tashqarida! Ruxsat etilgan: [{format_value(H_CB_MIN)}, {format_value(H_CB_MAX)}] sm"
    
    # Beta cheklovi
    if beta_deg < float(BETA_MIN_DEG) or beta_deg > float(BETA_MAX_DEG):
        return None, None, None, None, None, None, None, None, False, \
               f"β = {format_value(beta_deg)}° chegaradan tashqarida! Ruxsat etilgan: [{format_value(BETA_MIN_DEG)}°, {format_value(BETA_MAX_DEG)}°]"
    
    # Gamma cheklovi
    if gamma_deg < float(GAMMA_MIN_DEG) or gamma_deg > float(GAMMA_MAX_DEG):
        return None, None, None, None, None, None, None, None, False, \
               f"γ = {format_value(gamma_deg)}° chegaradan tashqarida! Ruxsat etilgan: [{format_value(GAMMA_MIN_DEG)}°, {format_value(GAMMA_MAX_DEG)}°]"
    
    # Radianlarga o'tkazish (numpy float ishlatamiz, lekin kasrlar aniqligi saqlanadi)
    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)
    
    # A, B nuqtalari
    A = np.array([oa_dist, 0.0, 0.0])
    B = A.copy()
    
    # C nuqtasi
    C = np.array([oa_dist, 0.0, h_CB])
    
    # CD vektori
    L_float = float(L)
    cd_h = L_float * np.cos(beta)
    cd_v = L_float * np.sin(beta)
    
    D_x = C[0] + cd_h * np.cos(alpha)
    D_y = C[1] + cd_h * np.sin(alpha)
    D_z = C[2] + cd_v
    D = np.array([D_x, D_y, D_z])
    
    # CD uzunligi tekshiruvi
    cd_length = np.linalg.norm(D - C)
    if abs(cd_length - L_float) > 0.01:
        return None, None, None, None, None, None, None, None, False, \
               f"CD uzunligi noto'g'ri: {format_value(cd_length)} sm (L={format_value(L_float)} sm bo'lishi kerak)"
    
    # DE vektori
    de_h = L_float * np.cos(gamma)
    de_v = L_float * np.sin(gamma)
    
    E_x = D_x + de_h * np.cos(alpha)
    E_y = D_y + de_h * np.sin(alpha)
    E_z = D_z + de_v
    E = np.array([E_x, E_y, E_z])
    
    # n vektori
    de_vec = E - D
    de_length = np.linalg.norm(de_vec)
    n_x, n_y, n_z = de_vec / de_length if de_length > 0 else (1, 0, 0)
    
    # Tirsak burchagi
    elbow_angle = 180.0 - abs(beta_deg - gamma_deg)
    
    if elbow_angle < float(ELBOW_MIN_DEG) or elbow_angle > float(ELBOW_MAX_DEG):
        return None, None, None, None, None, None, None, None, False, \
               f"Tirsak burchagi = {format_value(elbow_angle)}° chegaradan tashqarida! Ruxsat etilgan: [{format_value(ELBOW_MIN_DEG)}°, {format_value(ELBOW_MAX_DEG)}°]"
    
    return A, B, C, D, E, n_x, n_y, n_z, True, "OK"


# ===============================================================================
# 3D OBYEKTLAR
# ===============================================================================
def create_platform_vertices(cx, w=40, ln=25, h=15):
    hw, hl = w/2, ln/2
    return np.array([
        [cx-hw, -hl, 0], [cx+hw, -hl, 0], [cx+hw, hl, 0], [cx-hw, hl, 0],
        [cx-hw, -hl, h], [cx+hw, -hl, h], [cx+hw, hl, h], [cx-hw, hl, h]
    ])

def create_fence_vertices(x_off=70, y_off=35, w=120, ln=40, h=60):
    hw, hl = w/2, ln/2
    return np.array([
        [x_off-hw, y_off-hl, 0], [x_off+hw, y_off-hl, 0],
        [x_off+hw, y_off+hl, 0], [x_off-hw, y_off+hl, 0],
        [x_off-hw, y_off-hl, h], [x_off+hw, y_off-hl, h],
        [x_off+hw, y_off+hl, h], [x_off-hw, y_off+hl, h]
    ])


# ===============================================================================
# BOSHLANG'ICH PARAMETRLAR (Kasr formatida)
# ===============================================================================
alpha0 = Fraction(0, 1)
beta0 = Fraction(90, 1)
gamma0 = Fraction(0, 1)
h_CB0 = Fraction(45, 1)
oa0 = Fraction(0, 1)

current_alpha = float(alpha0)
current_beta = float(beta0)
current_gamma = float(gamma0)
current_h_CB = float(h_CB0)
current_oa = float(oa0)

# ===============================================================================
# GRAFIK OYNA
# ===============================================================================
plt.close('all')
fig = plt.figure(figsize=(16, 10))
ax = fig.add_axes([0.26, 0.05, 0.72, 0.90], projection='3d')
ax.set_facecolor('#f5f5f5')

# O'qlar
ax.plot([-100, 200], [0, 0], [0, 0], color='red', linewidth=1, alpha=0.5)
ax.plot([0, 0], [-100, 100], [0, 0], color='green', linewidth=1, alpha=0.5)
ax.plot([0, 0], [0, 0], [-10, 140], color='blue', linewidth=1, alpha=0.5)

# Boshlang'ich holat
A, B, C, D, E, n_x, n_y, n_z, valid, msg = forward_kinematics(
    current_alpha, current_beta, current_gamma, current_h_CB, current_oa
)

if not valid:
    print(f"XATO: {msg}")
    A = np.array([current_oa, 0, 0])
    B = A.copy()
    C = np.array([current_oa, 0, current_h_CB])
    D = np.array([current_oa, 0, current_h_CB + float(L)])
    E = np.array([current_oa + float(L), 0, current_h_CB + float(L)])
    n_x, n_y, n_z = 1, 0, 0

# ===============================================================================
# PLATFORMA
# ===============================================================================
pv = create_platform_vertices(current_oa)
platform_lines = []

for face in [[0,1,2,3,0], [4,5,6,7,4]]:
    line, = ax.plot(pv[face,0], pv[face,1], pv[face,2], color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

for i in range(4):
    line, = ax.plot([pv[i,0], pv[i+4,0]], [pv[i,1], pv[i+4,1]], [pv[i,2], pv[i+4,2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# ===============================================================================
# POMIDOR EGATLARI
# ===============================================================================
for y_off in [-50, 50]:
    fv = create_fence_vertices(x_off=70, y_off=y_off)
    for face in [[0,1,2,3,0], [4,5,6,7,4]]:
        ax.plot(fv[face,0], fv[face,1], fv[face,2], color='brown', linewidth=2, alpha=0.7)
    for i in range(4):
        ax.plot([fv[i,0], fv[i+4,0]], [fv[i,1], fv[i+4,1]], [fv[i,2], fv[i+4,2]], 
                color='brown', linewidth=2, alpha=0.7)

# ===============================================================================
# MANIPULYATOR
# ===============================================================================
bc_line, = ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], color='black', linewidth=3, label='BC')
cd_line, = ax.plot([C[0], D[0]], [C[1], D[1]], [C[2], D[2]], color='orange', linewidth=3, marker='o', label='CD')
de_line, = ax.plot([D[0], E[0]], [D[1], E[1]], [D[2], E[2]], color='blue', linewidth=3, marker='o', label='DE')
end_sc = ax.scatter(E[0], E[1], E[2], color='purple', s=100, marker='*', label='E')

# Tirsak burchagi
elbow_angle = 180.0 - abs(current_beta - current_gamma)

# ===============================================================================
# SAHNA SOZLAMALARI
# ===============================================================================
ax.set_xlim(-80, 180)
ax.set_ylim(-130, 130)
ax.set_zlim(-10, 140)
ax.set_box_aspect([260, 260, 150])
ax.set_xlabel('X (sm)', fontsize=11)
ax.set_ylabel('Y (sm)', fontsize=11)
ax.set_zlabel('Z (sm)', fontsize=11)
ax.view_init(elev=20, azim=-60)
ax.legend(loc='upper left', fontsize=9)
ax.set_title(f"FK | β:[{format_value(BETA_MIN_DEG)}°,{format_value(BETA_MAX_DEG)}°] γ:[{format_value(GAMMA_MIN_DEG)}°,{format_value(GAMMA_MAX_DEG)}°] Tirsak:[{format_value(ELBOW_MIN_DEG)}°,{format_value(ELBOW_MAX_DEG)}°]")

# ===============================================================================
# MA'LUMOT PANELI (Kasr formatida)
# ===============================================================================
info_ax = fig.add_axes([0.02, 0.42, 0.20, 0.20])
info_ax.text(0.05, 0.5, 
    f"α = {format_value(current_alpha)}°\nβ = {format_value(current_beta)}°\nγ = {format_value(current_gamma)}°\n"
    f"h_CB = {format_value(current_h_CB)} sm\nOA = {format_value(current_oa)} sm\n"
    f"Tirsak = {format_value(elbow_angle)}°",
    fontsize=10, va='center', fontweight='bold')
info_ax.set_xlim(0,1)
info_ax.set_ylim(0,1)
info_ax.axis('off')

coord_ax = fig.add_axes([0.02, 0.05, 0.20, 0.32])
coord_ax.text(0.05, 0.5, 
    f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
    f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
    f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
    f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
    f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
    f"n: ({format_value(n_x)}, {format_value(n_y)}, {format_value(n_z)})",
    fontsize=9, va='center', fontweight='bold')
coord_ax.set_xlim(0,1)
coord_ax.set_ylim(0,1)
coord_ax.axis('off')

# ===============================================================================
# BOSHQARUV ELEMENTLARI (Kasr kiritish uchun)
# ===============================================================================
alpha_ax = plt.axes([0.08, 0.87, 0.08, 0.025])
beta_ax  = plt.axes([0.08, 0.83, 0.08, 0.025])
gamma_ax = plt.axes([0.08, 0.79, 0.08, 0.025])
h_ax     = plt.axes([0.08, 0.75, 0.08, 0.025])
oa_ax    = plt.axes([0.08, 0.71, 0.08, 0.025])

button_ax = plt.axes([0.09, 0.65, 0.10, 0.035])
btn = Button(button_ax, '▶ YURISH', color='lightgreen', hovercolor='lime')

# Yorliqlar
fig.text(0.02, 0.88, 'α (°):', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.84, 'β (°):', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.80, 'γ (°):', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.76, 'h_CB (sm):', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.72, 'OA (sm):', fontsize=10, fontweight='bold', va='center')

# Hintlar
fig.text(0.17, 0.88, f"[-180,180]", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.84, f"[{format_value(BETA_MIN_DEG)},{format_value(BETA_MAX_DEG)}]", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.80, f"[{format_value(GAMMA_MIN_DEG)},{format_value(GAMMA_MAX_DEG)}]", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.76, f"[{format_value(H_CB_MIN)},{format_value(H_CB_MAX)}]", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.72, "(cheksiz)", fontsize=7, color='gray', va='center')

alpha_tb = TextBox(alpha_ax, '', initial=format_value(alpha0))
beta_tb  = TextBox(beta_ax,  '', initial=format_value(beta0))
gamma_tb = TextBox(gamma_ax, '', initial=format_value(gamma0))
h_tb     = TextBox(h_ax,     '', initial=format_value(h_CB0))
oa_tb    = TextBox(oa_ax,    '', initial=format_value(oa0))

# ===============================================================================
# XATOLIK PANELI
# ===============================================================================
err_ax = fig.add_axes([0.02, 0.92, 0.20, 0.06])
err_ax.set_facecolor('#fafafa')
for spine in err_ax.spines.values():
    spine.set_visible(True)
    spine.set_color('#cccccc')
    spine.set_linewidth(1)

err_ax.text(0.5, 0.5, "✓ Holat: Tayyor", fontsize=9, va='center', ha='center', 
            fontweight='bold', color='green', transform=err_ax.transAxes, wrap=True)
err_ax.set_xlim(0, 1)
err_ax.set_ylim(0, 1)
err_ax.set_xticks([])
err_ax.set_yticks([])


def update_error(msg, col):
    err_ax.clear()
    for spine in err_ax.spines.values():
        spine.set_visible(True)
        spine.set_color('#cccccc')
        spine.set_linewidth(1)
    
    wrapped_msg = textwrap.fill(msg, width=30)
    
    err_ax.text(0.5, 0.5, wrapped_msg, fontsize=8, va='center', ha='center', 
                fontweight='bold', color=col, transform=err_ax.transAxes, wrap=True)
    err_ax.set_xlim(0, 1)
    err_ax.set_ylim(0, 1)
    err_ax.set_xticks([])
    err_ax.set_yticks([])
    fig.canvas.draw()


def parse_input(value_str):
    """Kasr yoki oddiy sonni floatga aylantiradi"""
    value_str = value_str.strip()
    if '/' in value_str:
        parts = value_str.split('/')
        return float(Fraction(int(parts[0]), int(parts[1])))
    else:
        return float(Fraction(value_str))

# ===============================================================================
# ANIMATSIYA
# ===============================================================================
def start_anim(event):
    try:
        ta = parse_input(alpha_tb.text)
        tb = parse_input(beta_tb.text)
        tg = parse_input(gamma_tb.text)
        th = parse_input(h_tb.text)
        to = parse_input(oa_tb.text)
        animate(ta, tb, tg, th, to)
    except (ValueError, ZeroDivisionError) as e:
        update_error(f"✗ XATO: Noto'g'ri son formati! ({str(e)})", 'red')

def animate(ta, tb, tg, th, to):
    global current_alpha, current_beta, current_gamma, current_h_CB, current_oa
    
    At, Bt, Ct, Dt, Et, nxt, nyt, nzt, vld, msg = forward_kinematics(ta, tb, tg, th, to)
    
    if not vld:
        update_error(f"✗ {msg}", 'red')
        return
    
    update_error("⟳ Animatsiya bajarilmoqda...", 'blue')
    
    steps = 30
    da = (ta - current_alpha) / steps
    db = (tb - current_beta) / steps
    dg = (tg - current_gamma) / steps
    dh = (th - current_h_CB) / steps
    do = (to - current_oa) / steps
    
    for _ in range(steps):
        current_alpha += da
        current_beta += db
        current_gamma += dg
        current_h_CB += dh
        current_oa += do
        
        A, B, C, D, E, nx, ny, nz, vld, _ = forward_kinematics(
            current_alpha, current_beta, current_gamma, current_h_CB, current_oa
        )
        if not vld:
            continue
        
        elbow = 180.0 - abs(current_beta - current_gamma)
        
        pv = create_platform_vertices(current_oa)
        platform_lines[0].set_data(pv[[0,1,2,3,0],0], pv[[0,1,2,3,0],1])
        platform_lines[0].set_3d_properties(pv[[0,1,2,3,0],2])
        platform_lines[1].set_data(pv[[4,5,6,7,4],0], pv[[4,5,6,7,4],1])
        platform_lines[1].set_3d_properties(pv[[4,5,6,7,4],2])
        for j in range(4):
            platform_lines[j+2].set_data([pv[j,0], pv[j+4,0]], [pv[j,1], pv[j+4,1]])
            platform_lines[j+2].set_3d_properties([pv[j,2], pv[j+4,2]])
        
        bc_line.set_data([A[0], C[0]], [A[1], C[1]])
        bc_line.set_3d_properties([A[2], C[2]])
        cd_line.set_data([C[0], D[0]], [C[1], D[1]])
        cd_line.set_3d_properties([C[2], D[2]])
        de_line.set_data([D[0], E[0]], [D[1], E[1]])
        de_line.set_3d_properties([D[2], E[2]])
        end_sc._offsets3d = ([E[0]], [E[1]], [E[2]])
        
        info_ax.clear()
        info_ax.text(0.05, 0.5, 
            f"α = {format_value(current_alpha)}°\nβ = {format_value(current_beta)}°\nγ = {format_value(current_gamma)}°\n"
            f"h_CB = {format_value(current_h_CB)} sm\nOA = {format_value(current_oa)} sm\n"
            f"Tirsak = {format_value(elbow)}°",
            fontsize=10, va='center', fontweight='bold')
        info_ax.set_xlim(0,1)
        info_ax.set_ylim(0,1)
        info_ax.axis('off')
        
        coord_ax.clear()
        coord_ax.text(0.05, 0.5, 
            f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
            f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
            f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
            f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
            f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
            f"n: ({format_value(nx)}, {format_value(ny)}, {format_value(nz)})",
            fontsize=9, va='center', fontweight='bold')
        coord_ax.set_xlim(0,1)
        coord_ax.set_ylim(0,1)
        coord_ax.axis('off')
        
        fig.canvas.draw()
        fig.canvas.flush_events()
    
    current_alpha, current_beta, current_gamma = ta, tb, tg
    current_h_CB, current_oa = th, to
    update_error("✓ Holat: Tayyor!", 'green')

btn.on_clicked(start_anim)
plt.show()