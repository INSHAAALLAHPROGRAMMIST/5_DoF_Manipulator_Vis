'''
# ===============================================================================
# ИМПОРТ БИБЛИОТЕК / KUTUBXONALARNI IMPORT QILISH
# ===============================================================================
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
import time
from fractions import Fraction

# ===============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНЫХ ВЫЧИСЛЕНИЙ / ANIQ HISOB-KITOBLAR UCHUN FUNKSIYALAR
# ===============================================================================
def to_fraction(value, max_denominator=1000000):
    """Sonni kasr ko'rinishida qaytaradi"""
    return Fraction(value).limit_denominator(max_denominator)

def format_value(value):
    """BARCHA qiymatlarni ANIQ kasr ko'rinishida formatlaydi"""
    frac = to_fraction(float(value), max_denominator=1000000)
    if frac.denominator == 1:
        return str(frac.numerator)
    else:
        return f"{frac.numerator}/{frac.denominator}"

# ===============================================================================
# КОНСТАНТЫ МАНИПУЛЯТОРА / MANIPULYATOR KONSTANTALARI
# ===============================================================================
L = 30.0  # CD va DE bo'g'inlari uzunligi (sm)

# FIZIK CHEKLOVLAR
H_CB_MIN = 29.5  # Minimal C_z balandligi (sm)
H_CB_MAX = 57.0  # Maksimal C_z balandligi (sm)

# Burchak cheklovlari (radianlarda)
import math
ALPHA_MIN = -math.pi      # -180°
ALPHA_MAX = math.pi       # 180°
BETA_MIN = 0.0            # 0° (CD vertikal)
BETA_MAX = math.pi / 2    # 90° (CD gorizontal)
GAMMA_MIN = -math.pi / 4  # -45°
GAMMA_MAX = math.pi / 4   # 45°

# ===============================================================================
# КИНЕМАТИЧЕСКИЕ ФУНКЦИИ / KINEMATIK FUNKSIYALAR
# ===============================================================================

def inverse_kinematics(E_x, E_y, E_z, n_x, n_y, n_z):
    """
    TESKARI KINEMATIKA (HAQIQIY ROBOT UCHUN)
    
    Berilgan:
        E_x, E_y, E_z: End-effektor (E nuqta) koordinatalari
        n_x, n_y, n_z: ED bo'g'ini yo'nalish vektori (D dan E ga)
    
    Shartlar:
        - D_z >= C_z (D nuqta C dan pastda bo'la olmaydi)
        - Gamma (γ) = ED va XY tekislik orasidagi burchak
        - Gamma faqat n vektoriga bog'liq, α va β ga bog'liq emas
    
    Qaytaradi:
        A, B, C, D, E: Barcha nuqtalar koordinatalari
        alpha, beta, gamma: Bo'g'in burchaklari (radian)
        valid: Konfiguratsiya haqiqiyligi (bool)
        message: Xabar (string)
    """
    
    # 1. n vektorini normalizatsiya qilish
    vector_length = np.sqrt(n_x**2 + n_y**2 + n_z**2)
    if vector_length > 0:
        n_x = n_x / vector_length
        n_y = n_y / vector_length
        n_z = n_z / vector_length
    
    # 2. Gamma burchagini hisoblash (tirsak bo'g'ini)
    n_z_clipped = np.clip(n_z, -1.0, 1.0)
    gamma = np.arcsin(n_z_clipped)
    
    # Gamma cheklovlarini tekshirish
    if gamma < GAMMA_MIN or gamma > GAMMA_MAX:
        gamma_deg = np.degrees(gamma)
        return None, None, None, None, None, None, None, None, False, \
               f"Gamma burchagi ({gamma_deg:.1f}°) chegaradan tashqarida [{np.degrees(GAMMA_MIN):.1f}°, {np.degrees(GAMMA_MAX):.1f}°]"
    
    # 3. E nuqtasi
    E = np.array([E_x, E_y, E_z])
    
    # 4. D nuqtasini hisoblash (DE = L, n vektori D dan E ga)
    D = E - L * np.array([n_x, n_y, n_z])
    
    # 5. Alpha burchagini hisoblash (asos aylanishi)
    alpha = np.arctan2(D[1], D[0])
    
    # 6. A va B nuqtalari (asos)
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    
    # 7. C nuqtasini hisoblash (C = (0, 0, C_z), CD = L)
    D_horizontal = np.sqrt(D[0]**2 + D[1]**2)
    
    # Tekshirish: gorizontal masofa L dan katta bo'lishi mumkin emas
    if D_horizontal > L + 1e-6:
        return None, None, None, None, None, None, None, None, False, \
               f"D gorizontal masofa ({D_horizontal:.2f} sm) > {L} sm. Robot yetib borolmaydi."
    
    # Vertikal farq (Pifagor teoremasi)
    vertical_diff = np.sqrt(max(0, L**2 - D_horizontal**2))
    
    # Ikkita yechim
    C_z1 = D[2] + vertical_diff  # C D dan pastda
    C_z2 = D[2] - vertical_diff  # C D dan yuqorida
    
    # Shart: D_z >= C_z (D nuqta C dan pastda bo'la olmaydi)
    if D[2] >= C_z1:
        C_z = C_z1
    elif D[2] >= C_z2:
        C_z = C_z2
    else:
        return None, None, None, None, None, None, None, None, False, \
               f"D_z ({D[2]:.2f}) >= C_z sharti qanoatlantirilmadi. Robot bu pozitsiyaga yetib borolmaydi."
    
    # 8. Fizik cheklovlarni tekshirish (C_z balandligi)
    if C_z < H_CB_MIN:
        return None, None, None, None, None, None, None, None, False, \
               f"C_z ({C_z:.2f} sm) < H_CB_MIN ({H_CB_MIN} sm)"
    elif C_z > H_CB_MAX:
        return None, None, None, None, None, None, None, None, False, \
               f"C_z ({C_z:.2f} sm) > H_CB_MAX ({H_CB_MAX} sm)"
    
    C = np.array([0.0, 0.0, C_z])
    
    # 9. Beta burchagini hisoblash (yelka bo'g'ini)
    cd_vector = D - C
    cd_horizontal = np.sqrt(cd_vector[0]**2 + cd_vector[1]**2)
    cd_vertical = cd_vector[2]
    beta = np.arctan2(cd_horizontal, cd_vertical)
    
    # Beta cheklovlarini tekshirish
    if beta < BETA_MIN or beta > BETA_MAX:
        beta_deg = np.degrees(beta)
        return None, None, None, None, None, None, None, None, False, \
               f"Beta burchagi ({beta_deg:.1f}°) chegaradan tashqarida [{np.degrees(BETA_MIN):.1f}°, {np.degrees(BETA_MAX):.1f}°]"
    
    return A, B, C, D, E, alpha, beta, gamma, True, "OK"


def forward_kinematics(alpha, beta, gamma):
    """
    TO'G'RI KINEMATIKA
    
    Args:
        alpha: Asos aylanish burchagi (radian)
        beta: Yelka burchagi - CD va vertikal orasidagi burchak (radian)
        gamma: Tirsak burchagi - ED va XY tekislik orasidagi burchak (radian)
    
    Returns:
        E: End-effektor pozitsiyasi
        n: Yo'nalish vektori
        A, B, C, D: Barcha nuqtalar
        valid, message
    """
    
    # Burchak cheklovlarini tekshirish
    if alpha < ALPHA_MIN or alpha > ALPHA_MAX:
        return None, None, None, None, None, None, False, f"Alpha chegaradan tashqarida"
    if beta < BETA_MIN or beta > BETA_MAX:
        return None, None, None, None, None, None, False, f"Beta chegaradan tashqarida"
    if gamma < GAMMA_MIN or gamma > GAMMA_MAX:
        return None, None, None, None, None, None, False, f"Gamma chegaradan tashqarida"
    
    # 1. A va B nuqtalari
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    
    # 2. C nuqtasi
    C = np.array([0.0, 0.0, 0.0])
    
    # 3. D nuqtasi (CD = L)
    cd_horizontal = L * np.sin(beta)
    cd_vertical = L * np.cos(beta)
    
    D_x = cd_horizontal * np.cos(alpha)
    D_y = cd_horizontal * np.sin(alpha)
    D_z = C[2] + cd_vertical
    D = np.array([D_x, D_y, D_z])
    
    # C_z ni tekshirish
    if D_z < H_CB_MIN or D_z > H_CB_MAX:
        return None, None, None, None, None, None, False, f"D_z ({D_z:.2f}) chegaradan tashqarida"
    
    # 4. n vektori (gamma burchagidan)
    n_z = np.sin(gamma)
    n_xy = np.cos(gamma)
    n_x = n_xy * np.cos(alpha)
    n_y = n_xy * np.sin(alpha)
    n = np.array([n_x, n_y, n_z])
    
    # 5. E nuqtasi (DE = L)
    E = D + L * n
    
    return E, n, A, B, C, D, True, "OK"


# ===============================================================================
# ФУНКЦИИ ДЛЯ СОЗДАНИЯ 3D ОБЪЕКТОВ / 3D OBYEKTLARNI YARATISH FUNKSIYALARI
# ===============================================================================
def create_platform_vertices(center_x, platform_width=40, platform_length=25, platform_height=15):
    """Parallelepiped platformaning uchlarini yaratish"""
    hw = platform_width / 2
    hl = platform_length / 2
    
    vertices = np.array([
        [center_x - hw, -hl, 0],
        [center_x + hw, -hl, 0],
        [center_x + hw,  hl, 0],
        [center_x - hw,  hl, 0],
        [center_x - hw, -hl, platform_height],
        [center_x + hw, -hl, platform_height],
        [center_x + hw,  hl, platform_height],
        [center_x - hw,  hl, platform_height]
    ])
    return vertices


def create_fence_vertices(fence_width=120, fence_length=40, fence_height=60, x_offset=70, y_offset=35):
    """Pomidor egatlari parallelepiped uchlarini yaratish"""
    hw = fence_width / 2
    hl = fence_length / 2
    
    vertices = np.array([
        [x_offset - hw, y_offset - hl, 0],
        [x_offset + hw, y_offset - hl, 0],
        [x_offset + hw, y_offset + hl, 0],
        [x_offset - hw, y_offset + hl, 0],
        [x_offset - hw, y_offset - hl, fence_height],
        [x_offset + hw, y_offset - hl, fence_height],
        [x_offset + hw, y_offset + hl, fence_height],
        [x_offset - hw, y_offset + hl, fence_height]
    ])
    return vertices


# ===============================================================================
# НАЧАЛЬНЫЕ ПАРАМЕТРЫ / BOSHLANG'ICH PARAMETRLAR
# ===============================================================================
# Boshlang'ich konfiguratsiya (Test-1)
E_x0 = 3786028/794123
E_y0 = 0.0
E_z0 = 19875327/548842
n_x0 = 489061/564719
n_y0 = 0.0
n_z0 = -0.5

# Joriy qiymatlar
current_E_x = E_x0
current_E_y = E_y0
current_E_z = E_z0
current_n_x = n_x0
current_n_y = n_y0
current_n_z = n_z0

# ===============================================================================
# СОЗДАНИЕ ГЛАВНОГО ОКНА / ASOSIY OYNA YARATISH
# ===============================================================================
plt.close('all')
fig = plt.figure(figsize=(14, 9))
ax = fig.add_axes([0.22, 0.05, 0.76, 0.90], projection='3d')
ax.set_facecolor('#f7f7f7')

# Koordinata o'qlari
axis_len = 100
ax.plot([0, axis_len], [0, 0], [0, 0], color='red', linewidth=1)
ax.plot([0, 0], [0, axis_len], [0, 0], color='green', linewidth=1)
ax.plot([0, 0], [0, 0], [0, axis_len], color='blue', linewidth=1)

# ===============================================================================
# ВЫЧИСЛЕНИЕ НАЧАЛЬНЫХ КООРДИНАТ / BOSHLANG'ICH KOORDINATALARNI HISOBLASH
# ===============================================================================
A, B, C, D, E, alpha, beta, gamma, valid, msg = inverse_kinematics(E_x0, E_y0, E_z0, n_x0, n_y0, n_z0)

if not valid:
    print(f"XATO: {msg}")
    # Default qiymatlarni qo'lda o'rnatish
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    C = np.array([0.0, 0.0, 45.0])
    D = np.array([25.0, 0.0, 55.0])
    E = np.array([50.0, 0.0, 40.0])

# ===============================================================================
# СОЗДАНИЕ ПЛАТФОРМЫ / PLATFORMA YARATISH
# ===============================================================================
platform_vertices = create_platform_vertices(A[0])
platform_lines = []

bottom_face = [0, 1, 2, 3, 0]
line, = ax.plot(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1], 
                platform_vertices[bottom_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

top_face = [4, 5, 6, 7, 4]
line, = ax.plot(platform_vertices[top_face, 0], platform_vertices[top_face, 1], 
                platform_vertices[top_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

for i in range(4):
    line, = ax.plot([platform_vertices[i, 0], platform_vertices[i+4, 0]], 
                    [platform_vertices[i, 1], platform_vertices[i+4, 1]], 
                    [platform_vertices[i, 2], platform_vertices[i+4, 2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# ===============================================================================
# СОЗДАНИЕ ГРЯДОК ПОМИДОРОВ / POMIDOR EGATLARI
# ===============================================================================
fence_lines = []

# 1-egat
fence1_vertices = create_fence_vertices(x_offset=70, y_offset=-50)
for face_indices in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:
    line, = ax.plot(fence1_vertices[face_indices, 0], fence1_vertices[face_indices, 1], 
                    fence1_vertices[face_indices, 2], color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

for i in range(4):
    line, = ax.plot([fence1_vertices[i, 0], fence1_vertices[i+4, 0]], 
                    [fence1_vertices[i, 1], fence1_vertices[i+4, 1]], 
                    [fence1_vertices[i, 2], fence1_vertices[i+4, 2]], 
                    color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

# 2-egat
fence2_vertices = create_fence_vertices(x_offset=70, y_offset=50)
for face_indices in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:
    line, = ax.plot(fence2_vertices[face_indices, 0], fence2_vertices[face_indices, 1], 
                    fence2_vertices[face_indices, 2], color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

for i in range(4):
    line, = ax.plot([fence2_vertices[i, 0], fence2_vertices[i+4, 0]], 
                    [fence2_vertices[i, 1], fence2_vertices[i+4, 1]], 
                    [fence2_vertices[i, 2], fence2_vertices[i+4, 2]], 
                    color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

# ===============================================================================
# ЗВЕНЬЯ МАНИПУЛЯТОРА / MANIPULYATOR BO'G'INLARI
# ===============================================================================
sb_line, = ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], 
                   color='black', linewidth=3, label='BC')

cd_line, = ax.plot([C[0], D[0]], [C[1], D[1]], [C[2], D[2]], 
                   color='tab:orange', linewidth=3, marker='o', label='CD')

de_line, = ax.plot([D[0], E[0]], [D[1], E[1]], [D[2], E[2]], 
                   color='tab:blue', linewidth=3, marker='o', label='DE')

end_scatter = ax.scatter(E[0], E[1], E[2], color='purple', s=80, marker='*')

# ===============================================================================
# НАСТРОЙКА 3D СЦЕНЫ / 3D SAHNA SOZLAMALARI
# ===============================================================================
ax.set_xlim(-25, 175)
ax.set_ylim(-axis_len, axis_len)
ax.set_zlim(-10, 190)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.view_init(elev=20, azim=-75)
ax.legend(loc='upper left')
ax.set_title(f"3D Vizualizatsiya - Haqiqiy Robot Kinematikasi")

# ===============================================================================
# ИНФОРМАЦИОННЫЕ ПАНЕЛИ / MA'LUMOT PANELLARI
# ===============================================================================
# Yuqori panel - E va n parametrlari
info_text = f"E_x = {format_value(E_x0)}\nE_y = {format_value(E_y0)}\nE_z = {format_value(E_z0)}\n"
info_text += f"n_x = {format_value(n_x0)}\nn_y = {format_value(n_y0)}\nn_z = {format_value(n_z0)}"
info_ax = fig.add_axes([0.02, 0.45, 0.18, 0.15])
info_ax.text(0.05, 0.5, info_text, fontsize=13, verticalalignment='center', fontweight='bold')
info_ax.set_xlim(0, 1)
info_ax.set_ylim(0, 1)
info_ax.axis('off')

# Pastki panel - Barcha nuqtalar va burchaklar
coord_text = f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
coord_text += f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
coord_text += f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
coord_text += f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
coord_text += f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
coord_text += f"α: {format_value(np.degrees(alpha))}°\n"
coord_text += f"β: {format_value(np.degrees(beta))}°\n"
coord_text += f"γ: {format_value(np.degrees(gamma))}°"

coord_ax = fig.add_axes([0.02, 0.05, 0.18, 0.35])
coord_ax.text(0.05, 0.5, coord_text, fontsize=11, verticalalignment='center', fontweight='bold')
coord_ax.set_xlim(0, 1)
coord_ax.set_ylim(0, 1)
coord_ax.axis('off')

# ===============================================================================
# ЭЛЕМЕНТЫ УПРАВЛЕНИЯ / BOSHQARUV ELEMENTLARI
# ===============================================================================
ex_ax = plt.axes([0.06, 0.87, 0.08, 0.03])
ey_ax = plt.axes([0.06, 0.83, 0.08, 0.03])
ez_ax = plt.axes([0.06, 0.79, 0.08, 0.03])
nx_ax = plt.axes([0.06, 0.75, 0.08, 0.03])
ny_ax = plt.axes([0.06, 0.71, 0.08, 0.03])
nz_ax = plt.axes([0.06, 0.67, 0.08, 0.03])

button_ax = plt.axes([0.08, 0.61, 0.12, 0.04])
animate_button = Button(button_ax, 'ЗАПУСК', color='lightgreen', hovercolor='green')

fig.text(0.02, 0.88, 'E_x:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.84, 'E_y:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.80, 'E_z:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.76, 'n_x:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.72, 'n_y:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.68, 'n_z:', fontsize=12, fontweight='bold')

ex_textbox = TextBox(ex_ax, '', initial=str(E_x0))
ey_textbox = TextBox(ey_ax, '', initial=str(E_y0))
ez_textbox = TextBox(ez_ax, '', initial=str(E_z0))
nx_textbox = TextBox(nx_ax, '', initial=str(n_x0))
ny_textbox = TextBox(ny_ax, '', initial=str(n_y0))
nz_textbox = TextBox(nz_ax, '', initial=str(n_z0))

# Xatolik paneli
error_ax = fig.add_axes([0.02, 0.92, 0.18, 0.05])
error_ax.text(0.5, 0.5, "Holat: OK", fontsize=10, verticalalignment='center', 
              horizontalalignment='center', fontweight='bold', color='green',
              transform=error_ax.transAxes)
error_ax.set_xlim(0, 1)
error_ax.set_ylim(0, 1)
error_ax.axis('off')

# ===============================================================================
# ФУНКЦИИ АНИМАЦИИ / ANIMATSIYA FUNKSIYALARI
# ===============================================================================
def start_animation(event):
    """Animatsiyani boshlash"""
    try:
        target_E_x = float(ex_textbox.text)
        target_E_y = float(ey_textbox.text)
        target_E_z = float(ez_textbox.text)
        target_n_x = float(nx_textbox.text)
        target_n_y = float(ny_textbox.text)
        target_n_z = float(nz_textbox.text)
        
        animate_to_values(target_E_x, target_E_y, target_E_z, 
                         target_n_x, target_n_y, target_n_z)
    except ValueError:
        error_ax.clear()
        error_ax.text(0.5, 0.5, "XATO: Noto'g'ri son formati!", fontsize=10, 
                     verticalalignment='center', horizontalalignment='center', 
                     fontweight='bold', color='red', transform=error_ax.transAxes)
        error_ax.set_xlim(0, 1)
        error_ax.set_ylim(0, 1)
        error_ax.axis('off')
        fig.canvas.draw()


def animate_to_values(target_E_x, target_E_y, target_E_z, target_n_x, target_n_y, target_n_z):
    """Silliq animatsiya"""
    global current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
    
    # Maqsadli konfiguratsiyani tekshirish
    A_test, B_test, C_test, D_test, E_test, alpha_test, beta_test, gamma_test, valid, msg = \
        inverse_kinematics(target_E_x, target_E_y, target_E_z, target_n_x, target_n_y, target_n_z)
    
    if not valid:
        error_ax.clear()
        error_ax.text(0.5, 0.5, f"XATO: {msg}", fontsize=8, 
                     verticalalignment='center', horizontalalignment='center', 
                     fontweight='bold', color='red', transform=error_ax.transAxes)
        error_ax.set_xlim(0, 1)
        error_ax.set_ylim(0, 1)
        error_ax.axis('off')
        fig.canvas.draw()
        return
    
    error_ax.clear()
    error_ax.text(0.5, 0.5, "Holat: Animatsiya...", fontsize=10, 
                 verticalalignment='center', horizontalalignment='center', 
                 fontweight='bold', color='blue', transform=error_ax.transAxes)
    error_ax.set_xlim(0, 1)
    error_ax.set_ylim(0, 1)
    error_ax.axis('off')
    fig.canvas.draw()
    
    steps = 40
    
    E_x_step = (target_E_x - current_E_x) / steps
    E_y_step = (target_E_y - current_E_y) / steps
    E_z_step = (target_E_z - current_E_z) / steps
    n_x_step = (target_n_x - current_n_x) / steps
    n_y_step = (target_n_y - current_n_y) / steps
    n_z_step = (target_n_z - current_n_z) / steps
    
    for i in range(steps):
        current_E_x += E_x_step
        current_E_y += E_y_step
        current_E_z += E_z_step
        current_n_x += n_x_step
        current_n_y += n_y_step
        current_n_z += n_z_step
        
        A, B, C, D, E, alpha, beta, gamma, valid, _ = inverse_kinematics(
            current_E_x, current_E_y, current_E_z, 
            current_n_x, current_n_y, current_n_z
        )
        
        if not valid:
            continue
        
        # Platformani yangilash
        platform_vertices = create_platform_vertices(A[0])
        bottom_face = [0, 1, 2, 3, 0]
        platform_lines[0].set_data(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1])
        platform_lines[0].set_3d_properties(platform_vertices[bottom_face, 2])
        
        top_face = [4, 5, 6, 7, 4]
        platform_lines[1].set_data(platform_vertices[top_face, 0], platform_vertices[top_face, 1])
        platform_lines[1].set_3d_properties(platform_vertices[top_face, 2])
        
        for j in range(4):
            platform_lines[j+2].set_data([platform_vertices[j, 0], platform_vertices[j+4, 0]], 
                                        [platform_vertices[j, 1], platform_vertices[j+4, 1]])
            platform_lines[j+2].set_3d_properties([platform_vertices[j, 2], platform_vertices[j+4, 2]])
        
        # Bo'g'inlarni yangilash
        sb_line.set_data([A[0], C[0]], [A[1], C[1]])
        sb_line.set_3d_properties([A[2], C[2]])
        
        cd_line.set_data([C[0], D[0]], [C[1], D[1]])
        cd_line.set_3d_properties([C[2], D[2]])
        
        de_line.set_data([D[0], E[0]], [D[1], E[1]])
        de_line.set_3d_properties([D[2], E[2]])
        
        end_scatter._offsets3d = ([E[0]], [E[1]], [E[2]])
        
        # Panellarni yangilash
        info_text = f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
        info_text += f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}"
        info_ax.clear()
        info_ax.text(0.05, 0.5, info_text, fontsize=13, verticalalignment='center', fontweight='bold')
        info_ax.set_xlim(0, 1)
        info_ax.set_ylim(0, 1)
        info_ax.axis('off')
        
        coord_text = f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
        coord_text += f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
        coord_text += f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
        coord_text += f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
        coord_text += f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
        coord_text += f"α: {format_value(np.degrees(alpha))}°\n"
        coord_text += f"β: {format_value(np.degrees(beta))}°\n"
        coord_text += f"γ: {format_value(np.degrees(gamma))}°"
        coord_ax.clear()
        coord_ax.text(0.05, 0.5, coord_text, fontsize=11, verticalalignment='center', fontweight='bold')
        coord_ax.set_xlim(0, 1)
        coord_ax.set_ylim(0, 1)
        coord_ax.axis('off')
        
        fig.canvas.draw()
        fig.canvas.flush_events()
    
    current_E_x = target_E_x
    current_E_y = target_E_y
    current_E_z = target_E_z
    current_n_x = target_n_x
    current_n_y = target_n_y
    current_n_z = target_n_z
    
    error_ax.clear()
    error_ax.text(0.5, 0.5, "Holat: OK", fontsize=10, 
                 verticalalignment='center', horizontalalignment='center', 
                 fontweight='bold', color='green', transform=error_ax.transAxes)
    error_ax.set_xlim(0, 1)
    error_ax.set_ylim(0, 1)
    error_ax.axis('off')
    fig.canvas.draw()


# ===============================================================================
# СОБЫТИЕ ОБРАБОТЧИКИ / HODISA ISHLOVCHILARI
# ===============================================================================
animate_button.on_clicked(start_animation)

# ===============================================================================
# ПРОГРАММА ЗАПУСК / DASTURNI ISHGA TUSHIRISH
# ===============================================================================
plt.show()
'''









































'''
# ===============================================================================
# ИМПОРТ БИБЛИОТЕК / KUTUBXONALARNI IMPORT QILISH
# ===============================================================================
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
from fractions import Fraction
import math

# ===============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНЫХ ВЫЧИСЛЕНИЙ / ANIQ HISOB-KITOBLAR UCHUN FUNKSIYALAR
# ===============================================================================
def to_fraction(value, max_denominator=1000000):
    return Fraction(value).limit_denominator(max_denominator)

def format_value(value):
    frac = to_fraction(float(value), max_denominator=1000000)
    if frac.denominator == 1:
        return str(frac.numerator)
    else:
        return f"{frac.numerator}/{frac.denominator}"

# ===============================================================================
# КОНСТАНТЫ МАНИПУЛЯТОРА / MANIPULYATOR KONSTANTALARI
# ===============================================================================
L = 30.0  # CD va DE bo'g'inlari uzunligi (sm)

# ===============================================================================
# HAQIQIY ROBOTNING BURCHAK CHEKLOVLARI (gradus va radian)
# ===============================================================================
# Alfa (asos) - to'liq aylana oladi
ALPHA_MIN_DEG = -180.0
ALPHA_MAX_DEG = 180.0
ALPHA_MIN = np.radians(ALPHA_MIN_DEG)
ALPHA_MAX = np.radians(ALPHA_MAX_DEG)

# Beta (yelka) - faqat 70° dan 150° gacha (soat yo'nalishi bo'yicha)
BETA_MIN_DEG = 70.0
BETA_MAX_DEG = 150.0
BETA_MIN = np.radians(BETA_MIN_DEG)
BETA_MAX = np.radians(BETA_MAX_DEG)

# Gamma (tirsak) - -60° dan 120° gacha (soat miliga teskari)
GAMMA_MIN_DEG = -60.0
GAMMA_MAX_DEG = 120.0
GAMMA_MIN = np.radians(GAMMA_MIN_DEG)
GAMMA_MAX = np.radians(GAMMA_MAX_DEG)

# ===============================================================================
# TESKARI KINEMATIKA (HAQIQIY ROBOT UCHUN)
# ===============================================================================
def inverse_kinematics(E_x, E_y, E_z, n_x, n_y, n_z):
    """
    TESKARI KINEMATIKA - HAQIQIY ROBOT CHEKLOVLARI BILAN
    
    Berilgan:
        E_x, E_y, E_z: End-effektor koordinatalari (sm)
        n_x, n_y, n_z: ED bo'g'ini yo'nalish vektori
    
    Qaytaradi:
        A, B, C, D, E: Barcha nuqtalar
        alpha, beta, gamma: Bo'g'in burchaklari (radian)
        valid: Konfiguratsiya haqiqiyligi
        message: Xabar
    """
    
    # 1. n vektorini normalizatsiya qilish
    vec_len = np.sqrt(n_x**2 + n_y**2 + n_z**2)
    if vec_len < 1e-10:
        return None, None, None, None, None, None, None, None, False, "n vektori nolga teng"
    
    n_x = n_x / vec_len
    n_y = n_y / vec_len
    n_z = n_z / vec_len
    
    # 2. E nuqtasi
    E = np.array([E_x, E_y, E_z])
    
    # 3. D nuqtasini hisoblash (DE = L)
    D = E - L * np.array([n_x, n_y, n_z])
    
    # 4. Alfa burchagini hisoblash (asos aylanishi)
    alpha = np.arctan2(D[1], D[0])
    
    # Alfa cheklovlarini tekshirish
    if alpha < ALPHA_MIN or alpha > ALPHA_MAX:
        return None, None, None, None, None, None, None, None, False, \
               f"Alfa ({np.degrees(alpha):.1f}°) chegaradan tashqarida"
    
    # 5. A va B nuqtalari (asos)
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    
    # 6. C nuqtasini hisoblash
    D_horizontal = np.sqrt(D[0]**2 + D[1]**2)
    
    # Gorizontal masofa L dan katta bo'lishi mumkin emas
    if D_horizontal > L + 1e-6:
        return None, None, None, None, None, None, None, None, False, \
               f"D gorizontal masofa ({D_horizontal:.2f} sm) > {L} sm"
    
    # Vertikal farq (Pifagor)
    vertical_diff = np.sqrt(max(0, L**2 - D_horizontal**2))
    
    # Ikkita mumkin bo'lgan C_z
    C_z1 = D[2] + vertical_diff  # C D dan pastda
    C_z2 = D[2] - vertical_diff  # C D dan yuqorida
    
    # Beta burchagini hisoblaymiz va tekshiramiz
    valid_config = False
    best_C = None
    best_beta = None
    best_gamma = None
    best_msg = ""
    
    for C_z in [C_z1, C_z2]:
        if C_z < 0:  # C nuqta yerdan pastda bo'la olmaydi
            continue
            
        C = np.array([0.0, 0.0, C_z])
        
        # Beta burchagini hisoblash (CD va vertikal orasidagi burchak)
        cd_vector = D - C
        cd_horizontal = np.sqrt(cd_vector[0]**2 + cd_vector[1]**2)
        cd_vertical = cd_vector[2]
        
        # Beta: CD va Z o'qi orasidagi burchak
        # cd_horizontal = L * sin(beta)
        # cd_vertical = L * cos(beta)
        beta = np.arctan2(cd_horizontal, cd_vertical)
        
        # Beta ni [0, π] oralig'ida saqlash
        if beta < 0:
            beta += np.pi
        
        # Beta cheklovlarini tekshirish
        if beta < BETA_MIN or beta > BETA_MAX:
            continue
        
        # Gamma burchagini hisoblash (n va XY tekislik orasidagi burchak)
        gamma = np.arcsin(np.clip(n_z, -1.0, 1.0))
        
        # n vektorining XY proyeksiyasi alfa bilan mos kelishini tekshirish
        n_xy = np.cos(gamma)
        n_x_expected = n_xy * np.cos(alpha)
        n_y_expected = n_xy * np.sin(alpha)
        
        # n vektori yo'nalishini tekshirish
        dot_product = n_x * n_x_expected + n_y * n_y_expected
        if dot_product < 0.9:  # Yo'nalishlar mos emas
            # Gamma ni teskari ishora bilan ham tekshirib ko'ramiz
            gamma_alt = -gamma
            if GAMMA_MIN <= gamma_alt <= GAMMA_MAX:
                n_xy_alt = np.cos(gamma_alt)
                n_x_expected_alt = n_xy_alt * np.cos(alpha)
                n_y_expected_alt = n_xy_alt * np.sin(alpha)
                dot_product_alt = n_x * n_x_expected_alt + n_y * n_y_expected_alt
                if dot_product_alt >= 0.9:
                    gamma = gamma_alt
        
        # Gamma cheklovlarini tekshirish
        if gamma < GAMMA_MIN or gamma > GAMMA_MAX:
            continue
        
        # D nuqta C dan pastda bo'lmasligi kerak
        if D[2] < C_z - 1e-6:
            continue
        
        # Barcha tekshiruvlardan o'tdi
        valid_config = True
        best_C = C
        best_beta = beta
        best_gamma = gamma
        break
    
    if not valid_config:
        return None, None, None, None, None, None, None, None, False, \
               f"Konfiguratsiya mumkin emas. Beta: [{BETA_MIN_DEG}°, {BETA_MAX_DEG}°], Gamma: [{GAMMA_MIN_DEG}°, {GAMMA_MAX_DEG}°]"
    
    return A, B, best_C, D, E, alpha, best_beta, best_gamma, True, "OK"


# ===============================================================================
# TO'G'RI KINEMATIKA
# ===============================================================================
def forward_kinematics(alpha, beta, gamma):
    """
    TO'G'RI KINEMATIKA - Burchaklardan pozitsiyani hisoblash
    
    Args:
        alpha: Asos aylanishi (radian)
        beta: Yelka burchagi (radian) - CD va Z o'qi orasidagi burchak
        gamma: Tirsak burchagi (radian) - ED va XY tekislik orasidagi burchak
    
    Returns:
        E, n, A, B, C, D, valid, message
    """
    
    # Burchak cheklovlarini tekshirish
    if alpha < ALPHA_MIN or alpha > ALPHA_MAX:
        return None, None, None, None, None, None, False, f"Alfa chegaradan tashqarida"
    if beta < BETA_MIN or beta > BETA_MAX:
        return None, None, None, None, None, None, False, f"Beta chegaradan tashqarida"
    if gamma < GAMMA_MIN or gamma > GAMMA_MAX:
        return None, None, None, None, None, None, False, f"Gamma chegaradan tashqarida"
    
    # A va B nuqtalari
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    
    # C nuqtasi
    C = np.array([0.0, 0.0, 0.0])
    
    # D nuqtasi (CD = L)
    cd_horizontal = L * np.sin(beta)
    cd_vertical = L * np.cos(beta)
    
    D_x = cd_horizontal * np.cos(alpha)
    D_y = cd_horizontal * np.sin(alpha)
    D_z = C[2] + cd_vertical
    D = np.array([D_x, D_y, D_z])
    
    # n vektori
    n_z = np.sin(gamma)
    n_xy = np.cos(gamma)
    n_x = n_xy * np.cos(alpha)
    n_y = n_xy * np.sin(alpha)
    n = np.array([n_x, n_y, n_z])
    
    # E nuqtasi (DE = L)
    E = D + L * n
    
    return E, n, A, B, C, D, True, "OK"


# ===============================================================================
# 3D OBYEKTLARNI YARATISH
# ===============================================================================
def create_platform_vertices(center_x, platform_width=40, platform_length=25, platform_height=15):
    hw = platform_width / 2
    hl = platform_length / 2
    
    vertices = np.array([
        [center_x - hw, -hl, 0],
        [center_x + hw, -hl, 0],
        [center_x + hw,  hl, 0],
        [center_x - hw,  hl, 0],
        [center_x - hw, -hl, platform_height],
        [center_x + hw, -hl, platform_height],
        [center_x + hw,  hl, platform_height],
        [center_x - hw,  hl, platform_height]
    ])
    return vertices


def create_fence_vertices(fence_width=120, fence_length=40, fence_height=60, x_offset=70, y_offset=35):
    hw = fence_width / 2
    hl = fence_length / 2
    
    vertices = np.array([
        [x_offset - hw, y_offset - hl, 0],
        [x_offset + hw, y_offset - hl, 0],
        [x_offset + hw, y_offset + hl, 0],
        [x_offset - hw, y_offset + hl, 0],
        [x_offset - hw, y_offset - hl, fence_height],
        [x_offset + hw, y_offset - hl, fence_height],
        [x_offset + hw, y_offset + hl, fence_height],
        [x_offset - hw, y_offset + hl, fence_height]
    ])
    return vertices


# ===============================================================================
# BOSHLANG'ICH PARAMETRLAR (ISHCHI ZONA ICHIDA)
# ===============================================================================
# Beta = 90°, Gamma = 0° holatiga mos keladigan qiymatlar
alpha0 = np.radians(0.0)
beta0 = np.radians(90.0)   # CD gorizontal
gamma0 = np.radians(0.0)   # ED gorizontal

E0, n0, A0, B0, C0, D0, valid0, msg0 = forward_kinematics(alpha0, beta0, gamma0)

if not valid0:
    # Agar xato bo'lsa, qo'lda qiymatlar
    E_x0, E_y0, E_z0 = 30.0, 0.0, 30.0
    n_x0, n_y0, n_z0 = 1.0, 0.0, 0.0
else:
    E_x0, E_y0, E_z0 = E0[0], E0[1], E0[2]
    n_x0, n_y0, n_z0 = n0[0], n0[1], n0[2]

current_E_x = E_x0
current_E_y = E_y0
current_E_z = E_z0
current_n_x = n_x0
current_n_y = n_y0
current_n_z = n_z0

# ===============================================================================
# GRAFIK OYNA YARATISH
# ===============================================================================
plt.close('all')
fig = plt.figure(figsize=(14, 9))
ax = fig.add_axes([0.22, 0.05, 0.76, 0.90], projection='3d')
ax.set_facecolor('#f7f7f7')

# Koordinata o'qlari
axis_len = 100
ax.plot([0, axis_len], [0, 0], [0, 0], color='red', linewidth=1, alpha=0.5)
ax.plot([0, 0], [0, axis_len], [0, 0], color='green', linewidth=1, alpha=0.5)
ax.plot([0, 0], [0, 0], [0, axis_len], color='blue', linewidth=1, alpha=0.5)

# Boshlang'ich holatni hisoblash
A, B, C, D, E, alpha, beta, gamma, valid, msg = inverse_kinematics(
    current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
)

if not valid:
    print(f"XATO: {msg}")
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    C = np.array([0.0, 0.0, h_CB])
    D = np.array([30.0, 0.0, 0.0])
    E = np.array([60.0, 0.0, 0.0])
    alpha, beta, gamma = 0.0, np.pi/2, 0.0

# ===============================================================================
# PLATFORMA
# ===============================================================================
platform_vertices = create_platform_vertices(A[0])
platform_lines = []

bottom_face = [0, 1, 2, 3, 0]
line, = ax.plot(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1], 
                platform_vertices[bottom_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

top_face = [4, 5, 6, 7, 4]
line, = ax.plot(platform_vertices[top_face, 0], platform_vertices[top_face, 1], 
                platform_vertices[top_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

for i in range(4):
    line, = ax.plot([platform_vertices[i, 0], platform_vertices[i+4, 0]], 
                    [platform_vertices[i, 1], platform_vertices[i+4, 1]], 
                    [platform_vertices[i, 2], platform_vertices[i+4, 2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# ===============================================================================
# POMIDOR EGATLARI
# ===============================================================================
fence_lines = []

for y_off in [-50, 50]:
    fence_vertices = create_fence_vertices(x_offset=70, y_offset=y_off)
    for face in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:
        line, = ax.plot(fence_vertices[face, 0], fence_vertices[face, 1], 
                        fence_vertices[face, 2], color='brown', linewidth=2, alpha=0.7)
        fence_lines.append(line)
    for i in range(4):
        line, = ax.plot([fence_vertices[i, 0], fence_vertices[i+4, 0]], 
                        [fence_vertices[i, 1], fence_vertices[i+4, 1]], 
                        [fence_vertices[i, 2], fence_vertices[i+4, 2]], 
                        color='brown', linewidth=2, alpha=0.7)
        fence_lines.append(line)

# ===============================================================================
# MANIPULYATOR BO'G'INLARI
# ===============================================================================
sb_line, = ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], 
                   color='black', linewidth=3, label='BC (asos)')

cd_line, = ax.plot([C[0], D[0]], [C[1], D[1]], [C[2], D[2]], 
                   color='tab:orange', linewidth=3, marker='o', label='CD (yelka)')

de_line, = ax.plot([D[0], E[0]], [D[1], E[1]], [D[2], E[2]], 
                   color='tab:blue', linewidth=3, marker='o', label='DE (tirsak)')

end_scatter = ax.scatter(E[0], E[1], E[2], color='purple', s=80, marker='*', label='E (oxirgi)')

# Ishchi zonani ko'rsatish (cheklovlar)
def draw_workspace_boundary():
    """Robotning ishchi zonasi chegaralarini ko'rsatish"""
    # Beta va gamma cheklovlarini ko'rsatadigan yoylar chizish
    pass  # Kerak bo'lsa qo'shish mumkin

# ===============================================================================
# SAHNA SOZLAMALARI
# ===============================================================================
ax.set_xlim(-50, 150)
ax.set_ylim(-100, 100)
ax.set_zlim(-10, 120)
ax.set_xlabel('X (sm)')
ax.set_ylabel('Y (sm)')
ax.set_zlabel('Z (sm)')
ax.view_init(elev=20, azim=-60)
ax.legend(loc='upper left')

title_text = f"3D Manipulyator - Haqiqiy Robot\n"
title_text += f"β: [{BETA_MIN_DEG}°, {BETA_MAX_DEG}°], γ: [{GAMMA_MIN_DEG}°, {GAMMA_MAX_DEG}°]"
ax.set_title(title_text)

# ===============================================================================
# MA'LUMOT PANELLARI
# ===============================================================================
# Yuqori panel - E va n
info_text = f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
info_text += f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}"
info_ax = fig.add_axes([0.02, 0.45, 0.18, 0.15])
info_ax.text(0.05, 0.5, info_text, fontsize=12, verticalalignment='center', fontweight='bold')
info_ax.set_xlim(0, 1)
info_ax.set_ylim(0, 1)
info_ax.axis('off')

# Pastki panel - Barcha nuqtalar va burchaklar
coord_text = f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
coord_text += f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
coord_text += f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
coord_text += f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
coord_text += f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
coord_text += f"α: {format_value(np.degrees(alpha))}°\n"
coord_text += f"β: {format_value(np.degrees(beta))}°  [{BETA_MIN_DEG}°, {BETA_MAX_DEG}°]\n"
coord_text += f"γ: {format_value(np.degrees(gamma))}°  [{GAMMA_MIN_DEG}°, {GAMMA_MAX_DEG}°]"

coord_ax = fig.add_axes([0.02, 0.05, 0.18, 0.35])
coord_ax.text(0.05, 0.5, coord_text, fontsize=10, verticalalignment='center', fontweight='bold')
coord_ax.set_xlim(0, 1)
coord_ax.set_ylim(0, 1)
coord_ax.axis('off')

# ===============================================================================
# BOSHQARUV ELEMENTLARI
# ===============================================================================
ex_ax = plt.axes([0.06, 0.87, 0.08, 0.03])
ey_ax = plt.axes([0.06, 0.83, 0.08, 0.03])
ez_ax = plt.axes([0.06, 0.79, 0.08, 0.03])
nx_ax = plt.axes([0.06, 0.75, 0.08, 0.03])
ny_ax = plt.axes([0.06, 0.71, 0.08, 0.03])
nz_ax = plt.axes([0.06, 0.67, 0.08, 0.03])

button_ax = plt.axes([0.08, 0.61, 0.12, 0.04])
animate_button = Button(button_ax, 'ЗАПУСК', color='lightgreen', hovercolor='green')

fig.text(0.02, 0.88, 'E_x:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.84, 'E_y:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.80, 'E_z:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.76, 'n_x:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.72, 'n_y:', fontsize=12, fontweight='bold')
fig.text(0.02, 0.68, 'n_z:', fontsize=12, fontweight='bold')

ex_textbox = TextBox(ex_ax, '', initial=f"{current_E_x:.6f}")
ey_textbox = TextBox(ey_ax, '', initial=f"{current_E_y:.6f}")
ez_textbox = TextBox(ez_ax, '', initial=f"{current_E_z:.6f}")
nx_textbox = TextBox(nx_ax, '', initial=f"{current_n_x:.6f}")
ny_textbox = TextBox(ny_ax, '', initial=f"{current_n_y:.6f}")
nz_textbox = TextBox(nz_ax, '', initial=f"{current_n_z:.6f}")

# Xatolik paneli
error_ax = fig.add_axes([0.02, 0.92, 0.18, 0.05])
error_ax.text(0.5, 0.5, "Holat: OK", fontsize=10, verticalalignment='center', 
              horizontalalignment='center', fontweight='bold', color='green',
              transform=error_ax.transAxes)
error_ax.set_xlim(0, 1)
error_ax.set_ylim(0, 1)
error_ax.axis('off')

# ===============================================================================
# ANIMATSIYA FUNKSIYALARI
# ===============================================================================
def start_animation(event):
    try:
        target_E_x = float(ex_textbox.text)
        target_E_y = float(ey_textbox.text)
        target_E_z = float(ez_textbox.text)
        target_n_x = float(nx_textbox.text)
        target_n_y = float(ny_textbox.text)
        target_n_z = float(nz_textbox.text)
        
        animate_to_values(target_E_x, target_E_y, target_E_z, 
                         target_n_x, target_n_y, target_n_z)
    except ValueError as e:
        update_error(f"XATO: Noto'g'ri son formati!", 'red')


def animate_to_values(target_E_x, target_E_y, target_E_z, target_n_x, target_n_y, target_n_z):
    global current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
    
    # Maqsadli konfiguratsiyani tekshirish
    A_test, B_test, C_test, D_test, E_test, alpha_test, beta_test, gamma_test, valid, msg = \
        inverse_kinematics(target_E_x, target_E_y, target_E_z, target_n_x, target_n_y, target_n_z)
    
    if not valid:
        update_error(f"XATO: {msg}", 'red')
        return
    
    update_error("Holat: Animatsiya...", 'blue')
    
    steps = 40
    
    E_x_step = (target_E_x - current_E_x) / steps
    E_y_step = (target_E_y - current_E_y) / steps
    E_z_step = (target_E_z - current_E_z) / steps
    n_x_step = (target_n_x - current_n_x) / steps
    n_y_step = (target_n_y - current_n_y) / steps
    n_z_step = (target_n_z - current_n_z) / steps
    
    for i in range(steps):
        current_E_x += E_x_step
        current_E_y += E_y_step
        current_E_z += E_z_step
        current_n_x += n_x_step
        current_n_y += n_y_step
        current_n_z += n_z_step
        
        A, B, C, D, E, alpha, beta, gamma, valid, _ = inverse_kinematics(
            current_E_x, current_E_y, current_E_z, 
            current_n_x, current_n_y, current_n_z
        )
        
        if not valid:
            continue
        
        update_plot(A, B, C, D, E, alpha, beta, gamma)
        fig.canvas.draw()
        fig.canvas.flush_events()
    
    current_E_x = target_E_x
    current_E_y = target_E_y
    current_E_z = target_E_z
    current_n_x = target_n_x
    current_n_y = target_n_y
    current_n_z = target_n_z
    
    update_error("Holat: OK", 'green')


def update_plot(A, B, C, D, E, alpha, beta, gamma):
    # Platforma
    platform_vertices = create_platform_vertices(A[0])
    
    bottom_face = [0, 1, 2, 3, 0]
    platform_lines[0].set_data(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1])
    platform_lines[0].set_3d_properties(platform_vertices[bottom_face, 2])
    
    top_face = [4, 5, 6, 7, 4]
    platform_lines[1].set_data(platform_vertices[top_face, 0], platform_vertices[top_face, 1])
    platform_lines[1].set_3d_properties(platform_vertices[top_face, 2])
    
    for j in range(4):
        platform_lines[j+2].set_data([platform_vertices[j, 0], platform_vertices[j+4, 0]], 
                                     [platform_vertices[j, 1], platform_vertices[j+4, 1]])
        platform_lines[j+2].set_3d_properties([platform_vertices[j, 2], platform_vertices[j+4, 2]])
    
    # Bo'g'inlar
    sb_line.set_data([A[0], C[0]], [A[1], C[1]])
    sb_line.set_3d_properties([A[2], C[2]])
    
    cd_line.set_data([C[0], D[0]], [C[1], D[1]])
    cd_line.set_3d_properties([C[2], D[2]])
    
    de_line.set_data([D[0], E[0]], [D[1], E[1]])
    de_line.set_3d_properties([D[2], E[2]])
    
    end_scatter._offsets3d = ([E[0]], [E[1]], [E[2]])
    
    # Ma'lumot paneli
    info_text = f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
    info_text += f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}"
    info_ax.clear()
    info_ax.text(0.05, 0.5, info_text, fontsize=12, verticalalignment='center', fontweight='bold')
    info_ax.set_xlim(0, 1)
    info_ax.set_ylim(0, 1)
    info_ax.axis('off')
    
    # Koordinatalar paneli
    alpha_deg = np.degrees(alpha)
    beta_deg = np.degrees(beta)
    gamma_deg = np.degrees(gamma)
    
    coord_text = f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
    coord_text += f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
    coord_text += f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
    coord_text += f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
    coord_text += f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
    
    # Beta holatini rang bilan ko'rsatish
    if BETA_MIN_DEG <= beta_deg <= BETA_MAX_DEG:
        coord_text += f"α: {format_value(alpha_deg)}°\n"
        coord_text += f"β: {format_value(beta_deg)}° ✓\n"
    else:
        coord_text += f"β: {format_value(beta_deg)}° ✗\n"
    
    if GAMMA_MIN_DEG <= gamma_deg <= GAMMA_MAX_DEG:
        coord_text += f"γ: {format_value(gamma_deg)}° ✓"
    else:
        coord_text += f"γ: {format_value(gamma_deg)}° ✗"
    
    coord_ax.clear()
    coord_ax.text(0.05, 0.5, coord_text, fontsize=10, verticalalignment='center', fontweight='bold')
    coord_ax.set_xlim(0, 1)
    coord_ax.set_ylim(0, 1)
    coord_ax.axis('off')


def update_error(message, color):
    error_ax.clear()
    error_ax.text(0.5, 0.5, message, fontsize=9, verticalalignment='center', 
                  horizontalalignment='center', fontweight='bold', color=color,
                  transform=error_ax.transAxes)
    error_ax.set_xlim(0, 1)
    error_ax.set_ylim(0, 1)
    error_ax.axis('off')
    fig.canvas.draw()


# ===============================================================================
# HODISA ISHLOVCHILARI
# ===============================================================================
animate_button.on_clicked(start_animation)

# ===============================================================================
# DASTURNI ISHGA TUSHIRISH
# ===============================================================================
plt.show()
'''






















'''
# ===============================================================================
# ИМПОРТ БИБЛИОТЕК / KUTUBXONALARNI IMPORT QILISH
# ===============================================================================
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
from fractions import Fraction
import math

# ===============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНЫХ ВЫЧИСЛЕНИЙ / ANIQ HISOB-KITOBLAR UCHUN FUNKSIYALAR
# ===============================================================================
def to_fraction(value, max_denominator=1000000):
    return Fraction(value).limit_denominator(max_denominator)

def format_value(value):
    frac = to_fraction(float(value), max_denominator=1000000)
    if frac.denominator == 1:
        return str(frac.numerator)
    else:
        return f"{frac.numerator}/{frac.denominator}"

# ===============================================================================
# ROBOT O'LCHAMLARI VA CHEKLOVLARI
# ===============================================================================
L = 30.0       # CD va DE bo'g'inlari uzunligi (sm)
h_CB = 20.0    # C nuqtaning asosdan balandligi (sm)

# Burchak cheklovlari (gradus va radian)
ALPHA_MIN_DEG = -180.0
ALPHA_MAX_DEG = 180.0
ALPHA_MIN = np.radians(ALPHA_MIN_DEG)
ALPHA_MAX = np.radians(ALPHA_MAX_DEG)

BETA_MIN_DEG = 70.0
BETA_MAX_DEG = 150.0
BETA_MIN = np.radians(BETA_MIN_DEG)
BETA_MAX = np.radians(BETA_MAX_DEG)

GAMMA_MIN_DEG = -60.0
GAMMA_MAX_DEG = 120.0
GAMMA_MIN = np.radians(GAMMA_MIN_DEG)
GAMMA_MAX = np.radians(GAMMA_MAX_DEG)

# ===============================================================================
# TESKARI KINEMATIKA
# ===============================================================================
def inverse_kinematics(E_x, E_y, E_z, n_x, n_y, n_z):
    """Haqiqiy robot uchun teskari kinematika"""
    
    # 1. n vektorini normalizatsiya
    vec_len = np.sqrt(n_x**2 + n_y**2 + n_z**2)
    if vec_len < 1e-10:
        return None, None, None, None, None, None, None, None, False, "n vektori nolga teng"
    
    n_x, n_y, n_z = n_x/vec_len, n_y/vec_len, n_z/vec_len
    
    # 2. Gamma burchagi
    gamma = np.arcsin(np.clip(n_z, -1.0, 1.0))
    if gamma < GAMMA_MIN or gamma > GAMMA_MAX:
        return None, None, None, None, None, None, None, None, False, \
               f"Gamma: {np.degrees(gamma):.1f}° [{GAMMA_MIN_DEG}°, {GAMMA_MAX_DEG}°]"
    
    # 3. E va D nuqtalari
    E = np.array([E_x, E_y, E_z])
    D = E - L * np.array([n_x, n_y, n_z])
    
    # 4. Alfa burchagi
    alpha = np.arctan2(D[1], D[0])
    
    # 5. A, B, C nuqtalari
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    C = np.array([0.0, 0.0, h_CB])
    
    # 6. Beta burchagini hisoblash
    D_horizontal = np.sqrt(D[0]**2 + D[1]**2)
    D_vertical = D[2] - h_CB
    
    CD_length = np.sqrt(D_horizontal**2 + D_vertical**2)
    if abs(CD_length - L) > 0.5:
        return None, None, None, None, None, None, None, None, False, \
               f"CD uzunligi {CD_length:.1f} sm ≠ {L} sm"
    
    beta = np.arctan2(D_horizontal, D_vertical)
    if beta < 0:
        beta += np.pi
    
    if beta < BETA_MIN or beta > BETA_MAX:
        return None, None, None, None, None, None, None, None, False, \
               f"Beta: {np.degrees(beta):.1f}° [{BETA_MIN_DEG}°, {BETA_MAX_DEG}°]"
    
    # 7. n vektori yo'nalishini tekshirish
    n_xy = np.cos(gamma)
    n_x_exp = n_xy * np.cos(alpha)
    n_y_exp = n_xy * np.sin(alpha)
    
    dot = n_x * n_x_exp + n_y * n_y_exp
    if dot < 0.9:
        gamma_alt = -gamma
        if GAMMA_MIN <= gamma_alt <= GAMMA_MAX:
            n_xy_alt = np.cos(gamma_alt)
            n_x_exp_alt = n_xy_alt * np.cos(alpha)
            n_y_exp_alt = n_xy_alt * np.sin(alpha)
            if n_x * n_x_exp_alt + n_y * n_y_exp_alt >= 0.9:
                gamma = gamma_alt
            else:
                return None, None, None, None, None, None, None, None, False, \
                       "n vektori yo'nalishi noto'g'ri"
        else:
            return None, None, None, None, None, None, None, None, False, \
                   "n vektori yo'nalishi noto'g'ri"
    
    return A, B, C, D, E, alpha, beta, gamma, True, "OK"


# ===============================================================================
# TO'G'RI KINEMATIKA
# ===============================================================================
def forward_kinematics(alpha, beta, gamma):
    """Burchaklardan pozitsiyani hisoblash"""
    
    if not (ALPHA_MIN <= alpha <= ALPHA_MAX):
        return None, None, None, None, None, None, False, "Alfa chegaradan tashqarida"
    if not (BETA_MIN <= beta <= BETA_MAX):
        return None, None, None, None, None, None, False, "Beta chegaradan tashqarida"
    if not (GAMMA_MIN <= gamma <= GAMMA_MAX):
        return None, None, None, None, None, None, False, "Gamma chegaradan tashqarida"
    
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    C = np.array([0.0, 0.0, h_CB])
    
    cd_h = L * np.sin(beta)
    cd_v = L * np.cos(beta)
    
    D_x = cd_h * np.cos(alpha)
    D_y = cd_h * np.sin(alpha)
    D_z = h_CB + cd_v
    D = np.array([D_x, D_y, D_z])
    
    n_z = np.sin(gamma)
    n_xy = np.cos(gamma)
    n_x = n_xy * np.cos(alpha)
    n_y = n_xy * np.sin(alpha)
    n = np.array([n_x, n_y, n_z])
    
    E = D + L * n
    
    return E, n, A, B, C, D, True, "OK"


# ===============================================================================
# 3D OBYEKTLARNI YARATISH
# ===============================================================================
def create_platform_vertices(center_x, platform_width=40, platform_length=25, platform_height=15):
    hw, hl = platform_width/2, platform_length/2
    return np.array([
        [center_x - hw, -hl, 0], [center_x + hw, -hl, 0],
        [center_x + hw,  hl, 0], [center_x - hw,  hl, 0],
        [center_x - hw, -hl, platform_height], [center_x + hw, -hl, platform_height],
        [center_x + hw,  hl, platform_height], [center_x - hw,  hl, platform_height]
    ])

def create_fence_vertices(fence_width=120, fence_length=40, fence_height=60, x_offset=70, y_offset=35):
    hw, hl = fence_width/2, fence_length/2
    return np.array([
        [x_offset - hw, y_offset - hl, 0], [x_offset + hw, y_offset - hl, 0],
        [x_offset + hw, y_offset + hl, 0], [x_offset - hw, y_offset + hl, 0],
        [x_offset - hw, y_offset - hl, fence_height], [x_offset + hw, y_offset - hl, fence_height],
        [x_offset + hw, y_offset + hl, fence_height], [x_offset - hw, y_offset + hl, fence_height]
    ])


# ===============================================================================
# BOSHLANG'ICH PARAMETRLAR
# ===============================================================================
alpha0 = np.radians(0.0)
beta0 = np.radians(90.0)
gamma0 = np.radians(0.0)

E0, n0, A0, B0, C0, D0, valid0, _ = forward_kinematics(alpha0, beta0, gamma0)

E_x0, E_y0, E_z0 = E0[0], E0[1], E0[2]
n_x0, n_y0, n_z0 = n0[0], n0[1], n0[2]

current_E_x, current_E_y, current_E_z = E_x0, E_y0, E_z0
current_n_x, current_n_y, current_n_z = n_x0, n_y0, n_z0

# ===============================================================================
# GRAFIK OYNA YARATISH
# ===============================================================================
plt.close('all')
fig = plt.figure(figsize=(15, 10))
ax = fig.add_axes([0.20, 0.05, 0.78, 0.90], projection='3d')
ax.set_facecolor('#f5f5f5')

axis_len = 100
ax.plot([0, axis_len], [0, 0], [0, 0], color='red', linewidth=1, alpha=0.5)
ax.plot([0, 0], [0, axis_len], [0, 0], color='green', linewidth=1, alpha=0.5)
ax.plot([0, 0], [0, 0], [0, axis_len], color='blue', linewidth=1, alpha=0.5)

A, B, C, D, E, alpha, beta, gamma, valid, msg = inverse_kinematics(
    current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
)

if not valid:
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    C = np.array([0.0, 0.0, h_CB])
    D = np.array([30.0, 0.0, h_CB])
    E = np.array([60.0, 0.0, h_CB])
    alpha, beta, gamma = 0.0, np.pi/2, 0.0

# ===============================================================================
# PLATFORMA
# ===============================================================================
platform_vertices = create_platform_vertices(A[0])
platform_lines = []

for face in [[0,1,2,3,0], [4,5,6,7,4]]:
    line, = ax.plot(platform_vertices[face, 0], platform_vertices[face, 1], 
                    platform_vertices[face, 2], color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

for i in range(4):
    line, = ax.plot([platform_vertices[i,0], platform_vertices[i+4,0]], 
                    [platform_vertices[i,1], platform_vertices[i+4,1]], 
                    [platform_vertices[i,2], platform_vertices[i+4,2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# ===============================================================================
# POMIDOR EGATLARI
# ===============================================================================
fence_lines = []
for y_off in [-50, 50]:
    fence_v = create_fence_vertices(x_offset=70, y_offset=y_off)
    for face in [[0,1,2,3,0], [4,5,6,7,4]]:
        line, = ax.plot(fence_v[face,0], fence_v[face,1], fence_v[face,2], 
                        color='brown', linewidth=2, alpha=0.7)
        fence_lines.append(line)
    for i in range(4):
        line, = ax.plot([fence_v[i,0], fence_v[i+4,0]], 
                        [fence_v[i,1], fence_v[i+4,1]], 
                        [fence_v[i,2], fence_v[i+4,2]], 
                        color='brown', linewidth=2, alpha=0.7)
        fence_lines.append(line)

# ===============================================================================
# MANIPULYATOR BO'G'INLARI
# ===============================================================================
bc_line, = ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], 
                   color='black', linewidth=3, label='BC (ustun)')
cd_line, = ax.plot([C[0], D[0]], [C[1], D[1]], [C[2], D[2]], 
                   color='orange', linewidth=3, marker='o', label='CD (yelka)')
de_line, = ax.plot([D[0], E[0]], [D[1], E[1]], [D[2], E[2]], 
                   color='blue', linewidth=3, marker='o', label='DE (tirsak)')
end_scatter = ax.scatter(E[0], E[1], E[2], color='purple', s=100, marker='*', label='E')

# ===============================================================================
# SAHNA SOZLAMALARI
# ===============================================================================
ax.set_xlim(-40, 140)
ax.set_ylim(-100, 100)
ax.set_zlim(-5, 120)
ax.set_xlabel('X (sm)', fontsize=11)
ax.set_ylabel('Y (sm)', fontsize=11)
ax.set_zlabel('Z (sm)', fontsize=11)
ax.view_init(elev=20, azim=-60)
ax.legend(loc='upper left', fontsize=9)
ax.set_title(f"3D Manipulyator | β:[{BETA_MIN_DEG}°,{BETA_MAX_DEG}°] γ:[{GAMMA_MIN_DEG}°,{GAMMA_MAX_DEG}°] h_CB={h_CB}sm", fontsize=12)

# ===============================================================================
# MA'LUMOT PANELLARI
# ===============================================================================
# Parametrlar paneli
info_ax = fig.add_axes([0.02, 0.48, 0.16, 0.15])
info_ax.text(0.05, 0.5, f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
              f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}",
              fontsize=10, va='center', fontweight='bold')
info_ax.set_xlim(0, 1)
info_ax.set_ylim(0, 1)
info_ax.axis('off')

# Koordinatalar paneli
coord_ax = fig.add_axes([0.02, 0.05, 0.16, 0.38])
coord_ax.text(0.05, 0.5, 
    f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
    f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
    f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
    f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
    f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
    f"α: {format_value(np.degrees(alpha))}°\n"
    f"β: {format_value(np.degrees(beta))}°\n"
    f"γ: {format_value(np.degrees(gamma))}°",
    fontsize=9, va='center', fontweight='bold')
coord_ax.set_xlim(0, 1)
coord_ax.set_ylim(0, 1)
coord_ax.axis('off')

# ===============================================================================
# BOSHQARUV ELEMENTLARI
# ===============================================================================
ex_ax = plt.axes([0.06, 0.87, 0.07, 0.03])
ey_ax = plt.axes([0.06, 0.83, 0.07, 0.03])
ez_ax = plt.axes([0.06, 0.79, 0.07, 0.03])
nx_ax = plt.axes([0.06, 0.75, 0.07, 0.03])
ny_ax = plt.axes([0.06, 0.71, 0.07, 0.03])
nz_ax = plt.axes([0.06, 0.67, 0.07, 0.03])

button_ax = plt.axes([0.07, 0.61, 0.10, 0.04])
animate_button = Button(button_ax, '▶ YURISH', color='lightgreen', hovercolor='lime')

for i, label in enumerate(['E_x:', 'E_y:', 'E_z:', 'n_x:', 'n_y:', 'n_z:']):
    fig.text(0.02, 0.88 - i*0.04, label, fontsize=11, fontweight='bold')

ex_textbox = TextBox(ex_ax, '', initial=f"{current_E_x:.4f}")
ey_textbox = TextBox(ey_ax, '', initial=f"{current_E_y:.4f}")
ez_textbox = TextBox(ez_ax, '', initial=f"{current_E_z:.4f}")
nx_textbox = TextBox(nx_ax, '', initial=f"{current_n_x:.4f}")
ny_textbox = TextBox(ny_ax, '', initial=f"{current_n_y:.4f}")
nz_textbox = TextBox(nz_ax, '', initial=f"{current_n_z:.4f}")

# Xatolik paneli (yaxshilangan)
error_ax = fig.add_axes([0.02, 0.92, 0.16, 0.06])
error_ax.set_facecolor('#fafafa')
for spine in error_ax.spines.values():
    spine.set_visible(True)
    spine.set_color('#cccccc')
    spine.set_linewidth(1)
error_ax.text(0.5, 0.5, "✓ Holat: Tayyor", fontsize=9, va='center', ha='center', 
              fontweight='bold', color='green', transform=error_ax.transAxes)
error_ax.set_xlim(0, 1)
error_ax.set_ylim(0, 1)
error_ax.set_xticks([])
error_ax.set_yticks([])

# ===============================================================================
# ANIMATSIYA FUNKSIYALARI
# ===============================================================================
def update_error(message, color):
    """Xatolik xabarini qisqa va tushunarli ko'rsatish"""
    error_ax.clear()
    for spine in error_ax.spines.values():
        spine.set_visible(True)
        spine.set_color('#cccccc')
        spine.set_linewidth(1)
    
    # Uzun xabarni qisqartirish
    if len(message) > 45:
        short_msg = message[:42] + "..."
    else:
        short_msg = message
    
    error_ax.text(0.5, 0.5, short_msg, fontsize=8, va='center', ha='center', 
                  fontweight='bold', color=color, transform=error_ax.transAxes,
                  wrap=True)
    error_ax.set_xlim(0, 1)
    error_ax.set_ylim(0, 1)
    error_ax.set_xticks([])
    error_ax.set_yticks([])
    fig.canvas.draw()

def start_animation(event):
    try:
        target_E_x = float(ex_textbox.text)
        target_E_y = float(ey_textbox.text)
        target_E_z = float(ez_textbox.text)
        target_n_x = float(nx_textbox.text)
        target_n_y = float(ny_textbox.text)
        target_n_z = float(nz_textbox.text)
        animate_to_values(target_E_x, target_E_y, target_E_z, 
                         target_n_x, target_n_y, target_n_z)
    except ValueError:
        update_error("✗ Noto'g'ri son formati!", 'red')

def animate_to_values(tgt_Ex, tgt_Ey, tgt_Ez, tgt_nx, tgt_ny, tgt_nz):
    global current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
    
    A_t, B_t, C_t, D_t, E_t, a_t, b_t, g_t, valid, msg = inverse_kinematics(
        tgt_Ex, tgt_Ey, tgt_Ez, tgt_nx, tgt_ny, tgt_nz
    )
    
    if not valid:
        update_error(f"✗ {msg}", 'red')
        return
    
    update_error("⟳ Animatsiya...", 'blue')
    
    steps = 40
    dxE, dyE, dzE = (tgt_Ex - current_E_x)/steps, (tgt_Ey - current_E_y)/steps, (tgt_Ez - current_E_z)/steps
    dxn, dyn, dzn = (tgt_nx - current_n_x)/steps, (tgt_ny - current_n_y)/steps, (tgt_nz - current_n_z)/steps
    
    for _ in range(steps):
        current_E_x += dxE
        current_E_y += dyE
        current_E_z += dzE
        current_n_x += dxn
        current_n_y += dyn
        current_n_z += dzn
        
        A, B, C, D, E, alpha, beta, gamma, vld, _ = inverse_kinematics(
            current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
        )
        if not vld:
            continue
        
        update_plot(A, B, C, D, E, alpha, beta, gamma)
        fig.canvas.draw()
        fig.canvas.flush_events()
    
    current_E_x, current_E_y, current_E_z = tgt_Ex, tgt_Ey, tgt_Ez
    current_n_x, current_n_y, current_n_z = tgt_nx, tgt_ny, tgt_nz
    update_error("✓ Holat: Tayyor", 'green')

def update_plot(A, B, C, D, E, alpha, beta, gamma):
    # Platforma
    pv = create_platform_vertices(A[0])
    platform_lines[0].set_data(pv[[0,1,2,3,0],0], pv[[0,1,2,3,0],1])
    platform_lines[0].set_3d_properties(pv[[0,1,2,3,0],2])
    platform_lines[1].set_data(pv[[4,5,6,7,4],0], pv[[4,5,6,7,4],1])
    platform_lines[1].set_3d_properties(pv[[4,5,6,7,4],2])
    for j in range(4):
        platform_lines[j+2].set_data([pv[j,0], pv[j+4,0]], [pv[j,1], pv[j+4,1]])
        platform_lines[j+2].set_3d_properties([pv[j,2], pv[j+4,2]])
    
    # Bo'g'inlar
    bc_line.set_data([A[0], C[0]], [A[1], C[1]])
    bc_line.set_3d_properties([A[2], C[2]])
    cd_line.set_data([C[0], D[0]], [C[1], D[1]])
    cd_line.set_3d_properties([C[2], D[2]])
    de_line.set_data([D[0], E[0]], [D[1], E[1]])
    de_line.set_3d_properties([D[2], E[2]])
    end_scatter._offsets3d = ([E[0]], [E[1]], [E[2]])
    
    # Panellar
    info_ax.clear()
    info_ax.text(0.05, 0.5, f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
                 f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}",
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
        f"α: {format_value(np.degrees(alpha))}°\n"
        f"β: {format_value(np.degrees(beta))}°\n"
        f"γ: {format_value(np.degrees(gamma))}°",
        fontsize=9, va='center', fontweight='bold')
    coord_ax.set_xlim(0,1)
    coord_ax.set_ylim(0,1)
    coord_ax.axis('off')

# ===============================================================================
# HODISA ISHLOVCHILARI VA DASTURNI ISHGA TUSHIRISH
# ===============================================================================
animate_button.on_clicked(start_animation)
plt.show()
'''















# ===============================================================================
# ИМПОРТ БИБЛИОТЕК / KUTUBXONALARNI IMPORT QILISH
# ===============================================================================
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
from fractions import Fraction
import math

# ===============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНЫХ ВЫЧИСЛЕНИЙ / ANIQ HISOB-KITOBLAR UCHUN FUNKSIYALAR
# ===============================================================================
def to_fraction(value, max_denominator=1000000):
    return Fraction(value).limit_denominator(max_denominator)

def format_value(value):
    frac = to_fraction(float(value), max_denominator=1000000)
    if frac.denominator == 1:
        return str(frac.numerator)
    else:
        return f"{frac.numerator}/{frac.denominator}"

# ===============================================================================
# ROBOT O'LCHAMLARI VA CHEKLOVLARI
# ===============================================================================
L = 30.0       # CD va DE bo'g'inlari uzunligi (sm)
h_CB = 20.0    # C nuqtaning asosdan balandligi (sm)

# Burchak cheklovlari (gradus va radian)
ALPHA_MIN_DEG = -180.0
ALPHA_MAX_DEG = 180.0
ALPHA_MIN = np.radians(ALPHA_MIN_DEG)
ALPHA_MAX = np.radians(ALPHA_MAX_DEG)

BETA_MIN_DEG = 70.0
BETA_MAX_DEG = 150.0
BETA_MIN = np.radians(BETA_MIN_DEG)
BETA_MAX = np.radians(BETA_MAX_DEG)

GAMMA_MIN_DEG = -60.0
GAMMA_MAX_DEG = 120.0
GAMMA_MIN = np.radians(GAMMA_MIN_DEG)
GAMMA_MAX = np.radians(GAMMA_MAX_DEG)

# ===============================================================================
# TESKARI KINEMATIKA
# ===============================================================================
def inverse_kinematics(E_x, E_y, E_z, n_x, n_y, n_z):
    """Haqiqiy robot uchun teskari kinematika"""
    
    # 1. n vektorini normalizatsiya
    vec_len = np.sqrt(n_x**2 + n_y**2 + n_z**2)
    if vec_len < 1e-10:
        return None, None, None, None, None, None, None, None, False, "n vektori nolga teng"
    
    n_x, n_y, n_z = n_x/vec_len, n_y/vec_len, n_z/vec_len
    
    # 2. Gamma burchagi
    gamma = np.arcsin(np.clip(n_z, -1.0, 1.0))
    if gamma < GAMMA_MIN or gamma > GAMMA_MAX:
        return None, None, None, None, None, None, None, None, False, \
               f"Gamma: {np.degrees(gamma):.1f}° [{GAMMA_MIN_DEG}°, {GAMMA_MAX_DEG}°]"
    
    # 3. E va D nuqtalari
    E = np.array([E_x, E_y, E_z])
    D = E - L * np.array([n_x, n_y, n_z])
    
    # 4. Alfa burchagi
    alpha = np.arctan2(D[1], D[0])
    
    # 5. A, B, C nuqtalari
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    C = np.array([0.0, 0.0, h_CB])
    
    # 6. Beta burchagini hisoblash
    D_horizontal = np.sqrt(D[0]**2 + D[1]**2)
    D_vertical = D[2] - h_CB
    
    CD_length = np.sqrt(D_horizontal**2 + D_vertical**2)
    if abs(CD_length - L) > 0.5:
        return None, None, None, None, None, None, None, None, False, \
               f"CD uzunligi {CD_length:.1f} sm ≠ {L} sm"
    
    beta = np.arctan2(D_horizontal, D_vertical)
    if beta < 0:
        beta += np.pi
    
    if beta < BETA_MIN or beta > BETA_MAX:
        return None, None, None, None, None, None, None, None, False, \
               f"Beta: {np.degrees(beta):.1f}° [{BETA_MIN_DEG}°, {BETA_MAX_DEG}°]"
    
    # 7. n vektori yo'nalishini tekshirish
    n_xy = np.cos(gamma)
    n_x_exp = n_xy * np.cos(alpha)
    n_y_exp = n_xy * np.sin(alpha)
    
    dot = n_x * n_x_exp + n_y * n_y_exp
    if dot < 0.9:
        gamma_alt = -gamma
        if GAMMA_MIN <= gamma_alt <= GAMMA_MAX:
            n_xy_alt = np.cos(gamma_alt)
            n_x_exp_alt = n_xy_alt * np.cos(alpha)
            n_y_exp_alt = n_xy_alt * np.sin(alpha)
            if n_x * n_x_exp_alt + n_y * n_y_exp_alt >= 0.9:
                gamma = gamma_alt
            else:
                return None, None, None, None, None, None, None, None, False, \
                       "n vektori yo'nalishi noto'g'ri"
        else:
            return None, None, None, None, None, None, None, None, False, \
                   "n vektori yo'nalishi noto'g'ri"
    
    return A, B, C, D, E, alpha, beta, gamma, True, "OK"


# ===============================================================================
# TO'G'RI KINEMATIKA
# ===============================================================================
def forward_kinematics(alpha, beta, gamma):
    """Burchaklardan pozitsiyani hisoblash"""
    
    if not (ALPHA_MIN <= alpha <= ALPHA_MAX):
        return None, None, None, None, None, None, False, "Alfa chegaradan tashqarida"
    if not (BETA_MIN <= beta <= BETA_MAX):
        return None, None, None, None, None, None, False, "Beta chegaradan tashqarida"
    if not (GAMMA_MIN <= gamma <= GAMMA_MAX):
        return None, None, None, None, None, None, False, "Gamma chegaradan tashqarida"
    
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    C = np.array([0.0, 0.0, h_CB])
    
    cd_h = L * np.sin(beta)
    cd_v = L * np.cos(beta)
    
    D_x = cd_h * np.cos(alpha)
    D_y = cd_h * np.sin(alpha)
    D_z = h_CB + cd_v
    D = np.array([D_x, D_y, D_z])
    
    n_z = np.sin(gamma)
    n_xy = np.cos(gamma)
    n_x = n_xy * np.cos(alpha)
    n_y = n_xy * np.sin(alpha)
    n = np.array([n_x, n_y, n_z])
    
    E = D + L * n
    
    return E, n, A, B, C, D, True, "OK"


# ===============================================================================
# 3D OBYEKTLARNI YARATISH
# ===============================================================================
def create_platform_vertices(center_x, platform_width=40, platform_length=25, platform_height=15):
    hw, hl = platform_width/2, platform_length/2
    return np.array([
        [center_x - hw, -hl, 0], [center_x + hw, -hl, 0],
        [center_x + hw,  hl, 0], [center_x - hw,  hl, 0],
        [center_x - hw, -hl, platform_height], [center_x + hw, -hl, platform_height],
        [center_x + hw,  hl, platform_height], [center_x - hw,  hl, platform_height]
    ])

def create_fence_vertices(fence_width=120, fence_length=40, fence_height=60, x_offset=70, y_offset=35):
    hw, hl = fence_width/2, fence_length/2
    return np.array([
        [x_offset - hw, y_offset - hl, 0], [x_offset + hw, y_offset - hl, 0],
        [x_offset + hw, y_offset + hl, 0], [x_offset - hw, y_offset + hl, 0],
        [x_offset - hw, y_offset - hl, fence_height], [x_offset + hw, y_offset - hl, fence_height],
        [x_offset + hw, y_offset + hl, fence_height], [x_offset - hw, y_offset + hl, fence_height]
    ])


# ===============================================================================
# BOSHLANG'ICH PARAMETRLAR
# ===============================================================================
alpha0 = np.radians(0.0)
beta0 = np.radians(90.0)
gamma0 = np.radians(0.0)

E0, n0, A0, B0, C0, D0, valid0, _ = forward_kinematics(alpha0, beta0, gamma0)

E_x0, E_y0, E_z0 = E0[0], E0[1], E0[2]
n_x0, n_y0, n_z0 = n0[0], n0[1], n0[2]

current_E_x, current_E_y, current_E_z = E_x0, E_y0, E_z0
current_n_x, current_n_y, current_n_z = n_x0, n_y0, n_z0

# ===============================================================================
# GRAFIK OYNA YARATISH
# ===============================================================================
plt.close('all')
fig = plt.figure(figsize=(15, 10))
ax = fig.add_axes([0.20, 0.05, 0.78, 0.90], projection='3d')
ax.set_facecolor('#f5f5f5')

axis_len = 100
ax.plot([0, axis_len], [0, 0], [0, 0], color='red', linewidth=1, alpha=0.5)
ax.plot([0, 0], [0, axis_len], [0, 0], color='green', linewidth=1, alpha=0.5)
ax.plot([0, 0], [0, 0], [0, axis_len], color='blue', linewidth=1, alpha=0.5)

A, B, C, D, E, alpha, beta, gamma, valid, msg = inverse_kinematics(
    current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
)

if not valid:
    A = np.array([0.0, 0.0, 0.0])
    B = A.copy()
    C = np.array([0.0, 0.0, h_CB])
    D = np.array([30.0, 0.0, h_CB])
    E = np.array([60.0, 0.0, h_CB])
    alpha, beta, gamma = 0.0, np.pi/2, 0.0

# ===============================================================================
# PLATFORMA
# ===============================================================================
platform_vertices = create_platform_vertices(A[0])
platform_lines = []

for face in [[0,1,2,3,0], [4,5,6,7,4]]:
    line, = ax.plot(platform_vertices[face, 0], platform_vertices[face, 1], 
                    platform_vertices[face, 2], color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

for i in range(4):
    line, = ax.plot([platform_vertices[i,0], platform_vertices[i+4,0]], 
                    [platform_vertices[i,1], platform_vertices[i+4,1]], 
                    [platform_vertices[i,2], platform_vertices[i+4,2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# ===============================================================================
# POMIDOR EGATLARI
# ===============================================================================
fence_lines = []
for y_off in [-50, 50]:
    fence_v = create_fence_vertices(x_offset=70, y_offset=y_off)
    for face in [[0,1,2,3,0], [4,5,6,7,4]]:
        line, = ax.plot(fence_v[face,0], fence_v[face,1], fence_v[face,2], 
                        color='brown', linewidth=2, alpha=0.7)
        fence_lines.append(line)
    for i in range(4):
        line, = ax.plot([fence_v[i,0], fence_v[i+4,0]], 
                        [fence_v[i,1], fence_v[i+4,1]], 
                        [fence_v[i,2], fence_v[i+4,2]], 
                        color='brown', linewidth=2, alpha=0.7)
        fence_lines.append(line)

# ===============================================================================
# MANIPULYATOR BO'G'INLARI
# ===============================================================================
bc_line, = ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], 
                   color='black', linewidth=3, label='BC (ustun)')
cd_line, = ax.plot([C[0], D[0]], [C[1], D[1]], [C[2], D[2]], 
                   color='orange', linewidth=3, marker='o', label='CD (yelka)')
de_line, = ax.plot([D[0], E[0]], [D[1], E[1]], [D[2], E[2]], 
                   color='blue', linewidth=3, marker='o', label='DE (tirsak)')
end_scatter = ax.scatter(E[0], E[1], E[2], color='purple', s=100, marker='*', label='E')

# ===============================================================================
# SAHNA SOZLAMALARI
# ===============================================================================
ax.set_xlim(-40, 140)
ax.set_ylim(-100, 100)
ax.set_zlim(-5, 120)
ax.set_xlabel('X (sm)', fontsize=11)
ax.set_ylabel('Y (sm)', fontsize=11)
ax.set_zlabel('Z (sm)', fontsize=11)
ax.view_init(elev=20, azim=-60)
ax.legend(loc='upper left', fontsize=9)
ax.set_title(f"3D Manipulyator | β:[{BETA_MIN_DEG}°,{BETA_MAX_DEG}°] γ:[{GAMMA_MIN_DEG}°,{GAMMA_MAX_DEG}°] h_CB={h_CB}sm", fontsize=12)

# ===============================================================================
# MA'LUMOT PANELLARI
# ===============================================================================
# Parametrlar paneli
info_ax = fig.add_axes([0.02, 0.48, 0.16, 0.15])
info_ax.text(0.05, 0.5, f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
              f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}",
              fontsize=10, va='center', fontweight='bold')
info_ax.set_xlim(0, 1)
info_ax.set_ylim(0, 1)
info_ax.axis('off')

# Koordinatalar paneli
coord_ax = fig.add_axes([0.02, 0.05, 0.16, 0.38])
coord_ax.text(0.05, 0.5, 
    f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
    f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
    f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
    f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
    f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
    f"α: {format_value(np.degrees(alpha))}°\n"
    f"β: {format_value(np.degrees(beta))}°\n"
    f"γ: {format_value(np.degrees(gamma))}°",
    fontsize=9, va='center', fontweight='bold')
coord_ax.set_xlim(0, 1)
coord_ax.set_ylim(0, 1)
coord_ax.axis('off')

# ===============================================================================
# BOSHQARUV ELEMENTLARI
# ===============================================================================
ex_ax = plt.axes([0.06, 0.87, 0.07, 0.03])
ey_ax = plt.axes([0.06, 0.83, 0.07, 0.03])
ez_ax = plt.axes([0.06, 0.79, 0.07, 0.03])
nx_ax = plt.axes([0.06, 0.75, 0.07, 0.03])
ny_ax = plt.axes([0.06, 0.71, 0.07, 0.03])
nz_ax = plt.axes([0.06, 0.67, 0.07, 0.03])

button_ax = plt.axes([0.07, 0.61, 0.10, 0.04])
animate_button = Button(button_ax, '▶ YURISH', color='lightgreen', hovercolor='lime')

for i, label in enumerate(['E_x:', 'E_y:', 'E_z:', 'n_x:', 'n_y:', 'n_z:']):
    fig.text(0.02, 0.88 - i*0.04, label, fontsize=11, fontweight='bold')

ex_textbox = TextBox(ex_ax, '', initial=f"{current_E_x:.4f}")
ey_textbox = TextBox(ey_ax, '', initial=f"{current_E_y:.4f}")
ez_textbox = TextBox(ez_ax, '', initial=f"{current_E_z:.4f}")
nx_textbox = TextBox(nx_ax, '', initial=f"{current_n_x:.4f}")
ny_textbox = TextBox(ny_ax, '', initial=f"{current_n_y:.4f}")
nz_textbox = TextBox(nz_ax, '', initial=f"{current_n_z:.4f}")

# Xatolik paneli (yaxshilangan)
error_ax = fig.add_axes([0.02, 0.92, 0.16, 0.06])
error_ax.set_facecolor('#fafafa')
for spine in error_ax.spines.values():
    spine.set_visible(True)
    spine.set_color('#cccccc')
    spine.set_linewidth(1)
error_ax.text(0.5, 0.5, "✓ Holat: Tayyor", fontsize=9, va='center', ha='center', 
              fontweight='bold', color='green', transform=error_ax.transAxes)
error_ax.set_xlim(0, 1)
error_ax.set_ylim(0, 1)
error_ax.set_xticks([])
error_ax.set_yticks([])

# ===============================================================================
# ANIMATSIYA FUNKSIYALARI
# ===============================================================================
def update_error(message, color):
    """Xatolik xabarini qisqa va tushunarli ko'rsatish"""
    error_ax.clear()
    for spine in error_ax.spines.values():
        spine.set_visible(True)
        spine.set_color('#cccccc')
        spine.set_linewidth(1)
    
    # Uzun xabarni qisqartirish
    if len(message) > 45:
        short_msg = message[:42] + "..."
    else:
        short_msg = message
    
    error_ax.text(0.5, 0.5, short_msg, fontsize=8, va='center', ha='center', 
                  fontweight='bold', color=color, transform=error_ax.transAxes,
                  wrap=True)
    error_ax.set_xlim(0, 1)
    error_ax.set_ylim(0, 1)
    error_ax.set_xticks([])
    error_ax.set_yticks([])
    fig.canvas.draw()

def start_animation(event):
    try:
        target_E_x = float(ex_textbox.text)
        target_E_y = float(ey_textbox.text)
        target_E_z = float(ez_textbox.text)
        target_n_x = float(nx_textbox.text)
        target_n_y = float(ny_textbox.text)
        target_n_z = float(nz_textbox.text)
        animate_to_values(target_E_x, target_E_y, target_E_z, 
                         target_n_x, target_n_y, target_n_z)
    except ValueError:
        update_error("✗ Noto'g'ri son formati!", 'red')

def animate_to_values(tgt_Ex, tgt_Ey, tgt_Ez, tgt_nx, tgt_ny, tgt_nz):
    global current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
    
    A_t, B_t, C_t, D_t, E_t, a_t, b_t, g_t, valid, msg = inverse_kinematics(
        tgt_Ex, tgt_Ey, tgt_Ez, tgt_nx, tgt_ny, tgt_nz
    )
    
    if not valid:
        update_error(f"✗ {msg}", 'red')
        return
    
    update_error("⟳ Animatsiya...", 'blue')
    
    steps = 40
    dxE, dyE, dzE = (tgt_Ex - current_E_x)/steps, (tgt_Ey - current_E_y)/steps, (tgt_Ez - current_E_z)/steps
    dxn, dyn, dzn = (tgt_nx - current_n_x)/steps, (tgt_ny - current_n_y)/steps, (tgt_nz - current_n_z)/steps
    
    for _ in range(steps):
        current_E_x += dxE
        current_E_y += dyE
        current_E_z += dzE
        current_n_x += dxn
        current_n_y += dyn
        current_n_z += dzn
        
        A, B, C, D, E, alpha, beta, gamma, vld, _ = inverse_kinematics(
            current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
        )
        if not vld:
            continue
        
        update_plot(A, B, C, D, E, alpha, beta, gamma)
        fig.canvas.draw()
        fig.canvas.flush_events()
    
    current_E_x, current_E_y, current_E_z = tgt_Ex, tgt_Ey, tgt_Ez
    current_n_x, current_n_y, current_n_z = tgt_nx, tgt_ny, tgt_nz
    update_error("✓ Holat: Tayyor", 'green')

def update_plot(A, B, C, D, E, alpha, beta, gamma):
    # Platforma
    pv = create_platform_vertices(A[0])
    platform_lines[0].set_data(pv[[0,1,2,3,0],0], pv[[0,1,2,3,0],1])
    platform_lines[0].set_3d_properties(pv[[0,1,2,3,0],2])
    platform_lines[1].set_data(pv[[4,5,6,7,4],0], pv[[4,5,6,7,4],1])
    platform_lines[1].set_3d_properties(pv[[4,5,6,7,4],2])
    for j in range(4):
        platform_lines[j+2].set_data([pv[j,0], pv[j+4,0]], [pv[j,1], pv[j+4,1]])
        platform_lines[j+2].set_3d_properties([pv[j,2], pv[j+4,2]])
    
    # Bo'g'inlar
    bc_line.set_data([A[0], C[0]], [A[1], C[1]])
    bc_line.set_3d_properties([A[2], C[2]])
    cd_line.set_data([C[0], D[0]], [C[1], D[1]])
    cd_line.set_3d_properties([C[2], D[2]])
    de_line.set_data([D[0], E[0]], [D[1], E[1]])
    de_line.set_3d_properties([D[2], E[2]])
    end_scatter._offsets3d = ([E[0]], [E[1]], [E[2]])
    
    # Panellar
    info_ax.clear()
    info_ax.text(0.05, 0.5, f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
                 f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}",
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
        f"α: {format_value(np.degrees(alpha))}°\n"
        f"β: {format_value(np.degrees(beta))}°\n"
        f"γ: {format_value(np.degrees(gamma))}°",
        fontsize=9, va='center', fontweight='bold')
    coord_ax.set_xlim(0,1)
    coord_ax.set_ylim(0,1)
    coord_ax.axis('off')

# ===============================================================================
# HODISA ISHLOVCHILARI VA DASTURNI ISHGA TUSHIRISH
# ===============================================================================
animate_button.on_clicked(start_animation)
plt.show()