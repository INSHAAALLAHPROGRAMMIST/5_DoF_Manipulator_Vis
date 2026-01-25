# ===============================================================================
# ИМПОРТ БИБЛИОТЕК / KUTUBXONALARNI IMPORT QILISH
# ===============================================================================
# numpy - математические вычисления / matematik hisob-kitoblar
# matplotlib - 2D/3D графики и визуализация / 2D/3D grafiklar va vizualizatsiya
# mpl_toolkits.mplot3d - 3D координатная система / 3D koordinata tizimi
# matplotlib.widgets - интерактивные элементы UI / interaktiv UI elementlari
# time - временные задержки / vaqt kechikishlari
# fractions - точные дробные вычисления / aniq kasr hisob-kitoblari
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
import time
from fractions import Fraction

# ===============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНЫХ ВЫЧИСЛЕНИЙ / ANIQ HISOB-KITOBLAR UCHUN FUNKSIYALAR
# ===============================================================================

# Yuqori aniqlik uchun fractions ishlatamiz
def to_fraction(value, max_denominator=1000000):
    """Sonni kasr ko'rinishida qaytaradi / Преобразует число в дробный вид
    
    Args:
        value: Преобразуемое число / O'zgartiriladigan son
        max_denominator: Максимальный знаменатель / Maksimal maxraj
    """
    return Fraction(value).limit_denominator(max_denominator)

def format_value(value):
    """BARCHA qiymatlarni ANIQ kasr ko'rinishida formatlaydi
    Форматирует ВСЕ значения в ТОЧНОМ дробном виде
    
    Args:
        value: Форматируемое значение / Formatlanadigan qiymat
    """
    frac = to_fraction(float(value), max_denominator=1000000)
    # HAMMA SONNI kasr ko'rinishida chiqaramiz
    if frac.denominator == 1:
        return str(frac.numerator)
    else:
        return f"{frac.numerator}/{frac.denominator}"

# ===============================================================================
# КОНСТАНТЫ МАНИПУЛЯТОРА / MANIPULYATOR KONSTANTALARI
# ===============================================================================
# L - длина звена манипулятора (см) / manipulyator bo'g'inining uzunligi (sm)
# h_DC - высота звена DC (см) / DC bo'g'inining balandligi (sm)
L = 30.0
h_DC = 30.0

# ===============================================================================
# КИНЕМАТИЧЕСКИЕ ФУНКЦИИ / KINEMATIK FUNKSIYALAR
# ===============================================================================

def fk_cpp(alpha_deg, beta_deg, gamma_deg, h_CB, oa_dist):
    """Forward Kinematics - прямая кинематика / to'g'ri kinematika
    Вычисляет координаты конечной точки E / E nuqtasining koordinatalarini hisoblaydi
    
    Args:
        alpha_deg: Угол поворота вокруг Z (градусы) / Z o'qi atrofida aylanish burchagi (gradus)
        beta_deg: Угол наклона в вертикальной плоскости / vertikal tekislikdagi egilish burchagi
        gamma_deg: Угол ориентации конечного звена / oxirgi bo'g'inning orientatsiya burchagi
        h_CB: Высота подъема BC (см) / BC ko'tarilish balandligi (sm)
        oa_dist: Горизонтальное смещение OA (см) / OA gorizontal siljish (sm)
    """
    alpha = np.radians(alpha_deg)
    beta  = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)

    Ex = oa_dist + L * (np.cos(alpha) * np.cos(beta) - np.cos(alpha) * np.cos(gamma + beta))
    Ey = L * (np.sin(alpha) * np.cos(beta) - np.sin(alpha) * np.cos(gamma + beta))
    Ez = L * (np.sin(beta) - np.sin(gamma + beta)) + h_CB
    return np.array([Ex, Ey, Ez])

def compute_chain(alpha_deg, beta_deg, gamma_deg, h_CB, oa_dist):
    """Вычисляет все точки кинематической цепи манипулятора
    Manipulyatorning barcha kinematik zanjir nuqtalarini hisoblaydi
    
    Args:
        alpha_deg: Угол α (градусы) / α burchagi (gradus)
        beta_deg: Угол β (градусы) / β burchagi (gradus) 
        gamma_deg: Угол γ (градусы) / γ burchagi (gradus)
        h_CB: Высота h_CB (см) / h_CB balandligi (sm)
        oa_dist: Расстояние OA (см) / OA masofasi (sm)
        
    Returns:
        A, B, C, D, E: Координаты точек / Nuqtalar koordinatalari
        h_ED: Длина звена ED / ED bo'g'in uzunligi
        n_x, n_y, n_z: Компоненты единичного вектора / Birlik vektor komponentlari
    """
    alpha = np.radians(alpha_deg)
    beta  = np.radians(beta_deg)

    # Manipulyator platformaning ustida, y o'qiga yaqinroq joyda turadi
    # Манипулятор находится на платформе, ближе к оси Y
    platform_height = 15      # высота платформы / platforma balandligi
    manipulator_y_offset = 6.25  # смещение по Y / Y bo'yicha siljish
    
    A = np.array([oa_dist, manipulator_y_offset, platform_height])
    B = A.copy()  # B точка совпадает с A / B nuqta A bilan bir xil
    C = np.array([oa_dist, manipulator_y_offset, platform_height + h_CB])

    # Координаты точки D / D nuqtasining koordinatalari
    Dx = oa_dist + np.cos(beta) * h_DC * np.cos(alpha)
    Dy = manipulator_y_offset + np.cos(beta) * h_DC * np.sin(alpha)
    Dz = platform_height + h_CB + np.sin(beta) * h_DC
    D = np.array([Dx, Dy, Dz])

    # E nuqtasini ham yangi A nuqtasiga nisbatan hisoblash
    # Вычисление точки E относительно новой точки A
    Ex = oa_dist + L * (np.cos(alpha) * np.cos(beta) - np.cos(alpha) * np.cos(np.radians(gamma_deg) + beta))
    Ey = manipulator_y_offset + L * (np.sin(alpha) * np.cos(beta) - np.sin(alpha) * np.cos(np.radians(gamma_deg) + beta))
    Ez = platform_height + L * (np.sin(beta) - np.sin(np.radians(gamma_deg) + beta)) + h_CB
    E = np.array([Ex, Ey, Ez])
    
    # DE vektorining uzunligi va birlik vektori
    # Длина вектора DE и единичный вектор
    h_ED = np.sqrt((E[0] - D[0])**2 + (E[1] - D[1])**2 + (E[2] - D[2])**2)
    n_x = (E[0] - D[0]) / h_ED if h_ED != 0 else 0
    n_y = (E[1] - D[1]) / h_ED if h_ED != 0 else 0
    n_z = (E[2] - D[2]) / h_ED if h_ED != 0 else 0
    
    return A, B, C, D, E, h_ED, n_x, n_y, n_z

# ===============================================================================
# ФУНКЦИИ ДЛЯ СОЗДАНИЯ 3D ОБЪЕКТОВ / 3D OBYEKTLARNI YARATISH FUNKSIYALARI
# ===============================================================================

def create_platform_vertices(center_x, platform_width=40, platform_length=25, platform_height=15):
    """Parallelepiped platformaning uchlarini yaratish
    Создает вершины параллелепипеда платформы
    
    Args:
        center_x: Центр по оси X / X o'qi bo'yicha markaz
        platform_width: Ширина платформы / Platforma kengligi (40)
        platform_length: Длина платформы / Platforma uzunligi (25)
        platform_height: Высота платформы / Platforma balandligi (15)
    """
    hw = platform_width / 2   # yarim kenglik (x o'qi bo'yicha)
    hl = platform_length / 2  # yarim uzunlik (y o'qi bo'yicha) - qisqaroq
    
    # Platformaning pastki qismi (z = 0 dan boshlanadi)
    # Нижняя часть платформы (начинается с z = 0)
    vertices = np.array([
        [center_x - hw, -hl, 0],                    # 0: chap-orqa-past / лево-зад-низ
        [center_x + hw, -hl, 0],                    # 1: o'ng-orqa-past / право-зад-низ
        [center_x + hw,  hl, 0],                    # 2: o'ng-old-past / право-перед-низ
        [center_x - hw,  hl, 0],                    # 3: chap-old-past / лево-перед-низ
        [center_x - hw, -hl, platform_height],     # 4: chap-orqa-yuqori / лево-зад-верх
        [center_x + hw, -hl, platform_height],     # 5: o'ng-orqa-yuqori / право-зад-верх
        [center_x + hw,  hl, platform_height],     # 6: o'ng-old-yuqori / право-перед-верх
        [center_x - hw,  hl, platform_height]      # 7: chap-old-yuqori / лево-перед-верх
    ])
    return vertices

def create_fence_vertices(fence_width=120, fence_length=40, fence_height=60, x_offset=70, y_offset=35):
    """Pomidor egatlari parallelepiped uchlarini yaratish - bir joyda turadi
    Создает вершины параллелепипеда грядок помидоров - остается на месте
    
    Args:
        fence_width: Ширина грядки / Egat kengligi (120)
        fence_length: Длина грядки / Egat uzunligi (40)
        fence_height: Высота грядки / Egat balandligi (60)
        x_offset: Смещение по X / X bo'yicha siljish (70)
        y_offset: Смещение по Y / Y bo'yicha siljish (35)
    """
    hw = fence_width / 2   # yarim kenglik (x o'qi bo'yicha - uzunroq)
    hl = fence_length / 2  # yarim uzunlik (y o'qi bo'yicha)
    
    # Pomidor egatining pastki qismi (z = 0 dan boshlanadi)
    # Нижняя часть грядки помидоров (начинается с z = 0)
    # x_offset - X o'qining musbat tomonida, manipulyatordan uzunroq intervalda
    # x_offset - на положительной стороне оси X, на большем расстоянии от манипулятора
    vertices = np.array([
        [x_offset - hw, y_offset - hl, 0],                    # 0: chap-orqa-past
        [x_offset + hw, y_offset - hl, 0],                    # 1: o'ng-orqa-past
        [x_offset + hw, y_offset + hl, 0],                    # 2: o'ng-old-past
        [x_offset - hw, y_offset + hl, 0],                    # 3: chap-old-past
        [x_offset - hw, y_offset - hl, fence_height],        # 4: chap-orqa-yuqori
        [x_offset + hw, y_offset - hl, fence_height],        # 5: o'ng-orqa-yuqori
        [x_offset + hw, y_offset + hl, fence_height],        # 6: o'ng-old-yuqori
        [x_offset - hw, y_offset + hl, fence_height]         # 7: chap-old-yuqori
    ])
    return vertices

def draw_platform_edges(ax, vertices):
    """Platformaning qirralarini chizish
    Рисует ребра платформы
    
    Args:
        ax: 3D axes объект / 3D o'qlar obyekti
        vertices: Вершины платформы / Platforma uchlari
    """
    # Pastki yuz (0-1-2-3-0) / Нижняя грань
    bottom_face = [0, 1, 2, 3, 0]
    ax.plot(vertices[bottom_face, 0], vertices[bottom_face, 1], vertices[bottom_face, 2], 
            color='gray', linewidth=2, alpha=0.8)
    
    # Yuqori yuz (4-5-6-7-4) / Верхняя грань
    top_face = [4, 5, 6, 7, 4]
    ax.plot(vertices[top_face, 0], vertices[top_face, 1], vertices[top_face, 2], 
            color='gray', linewidth=2, alpha=0.8)
    
    # Vertikal qirralar / Вертикальные ребра
    for i in range(4):
        ax.plot([vertices[i, 0], vertices[i+4, 0]], 
                [vertices[i, 1], vertices[i+4, 1]], 
                [vertices[i, 2], vertices[i+4, 2]], 
                color='gray', linewidth=2, alpha=0.8)
    
    return ax

# ===============================================================================
# НАЧАЛЬНЫЕ ПАРАМЕТРЫ МАНИПУЛЯТОРА / MANIPULYATORNING BOSHLANG'ICH PARAMETRLARI
# ===============================================================================
# Углы в градусах / Burchaklar gradusda
alpha0 = 0.0    # Поворот вокруг Z оси / Z o'qi atrofida aylanish
beta0  = 135.0  # Наклон в вертикальной плоскости / Vertikal tekislikda egilish
gamma0 = 15.0   # Ориентация конечного звена / Oxirgi bo'g'in orientatsiyasi
h0     = 30.0   # Высота подъема BC (см) / BC ko'tarilish balandligi (sm)
oa0    = 0.0    # Горизонтальное смещение OA (см) / OA gorizontal siljish (sm)

# Hozirgi qiymatlar (animatsiya uchun)
# Текущие значения (для анимации)
current_alpha = alpha0
current_beta = beta0
current_gamma = gamma0
current_h = h0
current_oa = oa0

# ===============================================================================
# СОЗДАНИЕ ГЛАВНОГО ОКНА И 3D ГРАФИКА / ASOSIY OYNA VA 3D GRAFIK YARATISH
# ===============================================================================
plt.close('all')  # Закрыть все предыдущие окна / Barcha oldingi oynalarni yopish

# Создание фигуры размером 14x9 дюймов / 14x9 dyuymli figura yaratish
fig = plt.figure(figsize=(14, 9))

# Создание 3D осей: [left, bottom, width, height] в долях от размера окна
# 3D o'qlarni yaratish: [chap, past, kenglik, balandlik] oyna o'lchamidan ulush sifatida
ax = fig.add_axes([0.22, 0.05, 0.76, 0.90], projection='3d')
ax.set_facecolor('#f7f7f7')  # Светло-серый фон / Och kulrang fon

# ===============================================================================
# КООРДИНАТНЫЕ ОСИ / KOORDINATA O'QLARI
# ===============================================================================
# Рисуем координатные оси X, Y, Z разными цветами
# X, Y, Z koordinata o'qlarini turli ranglarda chizamiz
axis_len = 100  # Длина осей / O'qlar uzunligi
ax.plot([0, axis_len], [0, 0], [0, 0], color='red',   linewidth=1)  # X o'qi - qizil
ax.plot([0, 0], [0, axis_len], [0, 0], color='green', linewidth=1)  # Y o'qi - yashil  
ax.plot([0, 0], [0, 0], [0, axis_len], color='blue',  linewidth=1)  # Z o'qi - ko'k

# ===============================================================================
# ВЫЧИСЛЕНИЕ НАЧАЛЬНЫХ КООРДИНАТ / BOSHLANG'ICH KOORDINATALARNI HISOBLASH
# ===============================================================================
# Вычисляем все точки манипулятора с начальными параметрами
# Boshlang'ich parametrlar bilan manipulyatorning barcha nuqtalarini hisoblaymiz
A, B, C, D, E, h_ED, n_x, n_y, n_z = compute_chain(alpha0, beta0, gamma0, h0, oa0)

# ===============================================================================
# СОЗДАНИЕ ПОДВИЖНОЙ ПЛАТФОРМЫ / HARAKATLANUVCHI PLATFORMANI YARATISH
# ===============================================================================
# Platformani chizish / Рисуем платформу
platform_vertices = create_platform_vertices(oa0)  # Начальная позиция / Boshlang'ich pozitsiya
platform_lines = []  # Список линий платформы / Platforma chiziqlarining ro'yxati

# Pastki yuz / Нижняя грань (индексы вершин: 0-1-2-3-0)
bottom_face = [0, 1, 2, 3, 0]
line, = ax.plot(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1], 
                platform_vertices[bottom_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

# Yuqori yuz / Верхняя грань (индексы вершин: 4-5-6-7-4)
top_face = [4, 5, 6, 7, 4]
line, = ax.plot(platform_vertices[top_face, 0], platform_vertices[top_face, 1], 
                platform_vertices[top_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

# Vertikal qirralar / Вертикальные ребра (соединяют нижние и верхние вершины)
for i in range(4):
    line, = ax.plot([platform_vertices[i, 0], platform_vertices[i+4, 0]], 
                    [platform_vertices[i, 1], platform_vertices[i+4, 1]], 
                    [platform_vertices[i, 2], platform_vertices[i+4, 2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# ===============================================================================
# СОЗДАНИЕ НЕПОДВИЖНЫХ ГРЯДОК ПОМИДОРОВ / QO'ZG'ALMAS POMIDOR EGATLARI
# ===============================================================================
# Pomidor egatlarini chizish (ikki yonda, bir joyda turadi)
# Рисуем грядки помидоров (с двух сторон, остаются на месте)
fence_lines = []  # Список линий грядок / Egat chiziqlarining ro'yxati

# Y o'qining manfiy tomonidagi pomidor egati (bir joyda turadi)
# Грядка помидоров на отрицательной стороне оси Y (остается на месте)
fence1_vertices = create_fence_vertices(x_offset=70, y_offset=-50)
for face_indices in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:  # pastki va yuqori yuzlar
    line, = ax.plot(fence1_vertices[face_indices, 0], fence1_vertices[face_indices, 1], 
                    fence1_vertices[face_indices, 2], color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

for i in range(4):  # vertikal qirralar / вертикальные ребра
    line, = ax.plot([fence1_vertices[i, 0], fence1_vertices[i+4, 0]], 
                    [fence1_vertices[i, 1], fence1_vertices[i+4, 1]], 
                    [fence1_vertices[i, 2], fence1_vertices[i+4, 2]], 
                    color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

# Y o'qining musbat tomonidagi pomidor egati (bir joyda turadi)
# Грядка помидоров на положительной стороне оси Y (остается на месте)
fence2_vertices = create_fence_vertices(x_offset=70, y_offset=50)
for face_indices in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:  # pastki va yuqori yuzlar
    line, = ax.plot(fence2_vertices[face_indices, 0], fence2_vertices[face_indices, 1], 
                    fence2_vertices[face_indices, 2], color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

for i in range(4):  # vertikal qirralar / вертикальные ребра
    line, = ax.plot([fence2_vertices[i, 0], fence2_vertices[i+4, 0]], 
                    [fence2_vertices[i, 1], fence2_vertices[i+4, 1]], 
                    [fence2_vertices[i, 2], fence2_vertices[i+4, 2]], 
                    color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

# ===============================================================================
# ЗВЕНЬЯ МАНИПУЛЯТОРА / MANIPULYATOR BO'G'INLARI
# ===============================================================================
# BC звено (от точки A до C) - черный цвет / BC bo'g'ini (A dan C gacha) - qora rang
sb_line, = ax.plot(
    [A[0], C[0]],  # X координаты / X koordinatalari
    [A[1], C[1]],  # Y координаты / Y koordinatalari  
    [A[2], C[2]],  # Z координаты / Z koordinatalari
    color='black', linewidth=3, label='BC'
)

# CD звено (от точки C до D) - оранжевый цвет с маркерами / CD bo'g'ini - to'q sariq rang
cd_line, = ax.plot(
    [C[0], D[0]],
    [C[1], D[1]],
    [C[2], D[2]],
    color='tab:orange', linewidth=3, marker='o', label='CD'
)

# DE звено (от точки D до E) - синий цвет с маркерами / DE bo'g'ini - ko'k rang
de_line, = ax.plot(
    [D[0], E[0]],
    [D[1], E[1]],
    [D[2], E[2]],
    color='tab:blue', linewidth=3, marker='o', label='DE'
)

# Конечная точка E (end effector) - фиолетовая звездочка / Oxirgi nuqta E - binafsha yulduzcha
end_scatter = ax.scatter(E[0], E[1], E[2], color='purple', s=80, marker='*')

# ===============================================================================
# НАСТРОЙКА 3D СЦЕНЫ / 3D SAHNA SOZLAMALARI
# ===============================================================================
# X o'qini -25 dan boshlatish, musbat tomonni ko'proq ko'rsatish
# Начинаем ось X с -25, показываем больше положительную сторону
ax.set_xlim(-25, 175)  # X o'qi: -25 dan 175 gacha (musbat tomoni kattaroq)
ax.set_ylim(-axis_len, axis_len)  # Y o'qi: -100 dan 100 gacha
ax.set_zlim(-10, 190)  # Z o'qini ham biroz yuqoriroq boshlatish (-10 dan 190 gacha)

# Подписи осей / O'q nomlari
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Угол обзора камеры / Kamera ko'rish burchagi
ax.view_init(elev=20, azim=-75)  # elev: вертикальный угол / vertikal burchak, azim: горизонтальный / gorizontal

# Легенда и заголовок / Legend va sarlavha
ax.legend(loc='upper left')  # Легенда в верхнем левом углу / Chap yuqori burchakda legend
ax.set_title(f"3D Визуализация Манипулятора")  # Заголовок окна / Oyna sarlavhasi

# ===============================================================================
# ИНФОРМАЦИОННЫЕ ПАНЕЛИ / MA'LUMOT PANELLARI
# ===============================================================================
# Параметры манипулятора - левая верхняя панель / Manipulyator parametrlari - chap yuqori panel
info_text = f"α = {format_value(alpha0)}°\nβ = {format_value(beta0)}°\nγ = {format_value(gamma0)}°\nh_CB = {format_value(h0)}см\nOA = {format_value(oa0)}см"
info_ax = fig.add_axes([0.02, 0.45, 0.18, 0.15])  # [left, bottom, width, height] в долях
info_ax.text(0.05, 0.5, info_text, fontsize=13, verticalalignment='center', fontweight='bold')
info_ax.set_xlim(0, 1)
info_ax.set_ylim(0, 1)
info_ax.axis('off')  # Скрыть оси панели / Panel o'qlarini yashirish

# Координатalar paneli - har bir nuqta alohida qatorda / Панель координат - каждая точка на отдельной строке
coord_text = f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\nB: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\nC: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\nD: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\nE: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\nn: ({format_value(n_x)}, {format_value(n_y)}, {format_value(n_z)})"
coord_ax = fig.add_axes([0.02, 0.05, 0.18, 0.35])  # Левая нижняя панель / Chap pastki panel
coord_ax.text(0.05, 0.5, coord_text, fontsize=11, verticalalignment='center', fontweight='bold')
coord_ax.set_xlim(0, 1)
coord_ax.set_ylim(0, 1)
coord_ax.axis('off')

# ===============================================================================
# ЭЛЕМЕНТЫ УПРАВЛЕНИЯ / BOSHQARUV ELEMENTLARI
# ===============================================================================
# Текстовые поля для ввода параметров / Parametrlarni kiritish uchun matn maydonlari
# [left, bottom, width, height] координаты в долях от размера окна
alpha_ax = plt.axes([0.06, 0.87, 0.08, 0.03])  # Поле для угла α / α burchagi uchun maydon
beta_ax  = plt.axes([0.06, 0.83, 0.08, 0.03])  # Поле для угла β / β burchagi uchun maydon
gamma_ax = plt.axes([0.06, 0.79, 0.08, 0.03])  # Поле для угла γ / γ burchagi uchun maydon
h_ax     = plt.axes([0.1, 0.75, 0.08, 0.03])   # Поле для высоты h_CB / h_CB balandligi uchun maydon
oa_ax    = plt.axes([0.09, 0.71, 0.08, 0.03])  # Поле для смещения OA / OA siljishi uchun maydon

# Кнопка запуска анимации / Animatsiyani ishga tushirish tugmasi
button_ax = plt.axes([0.08, 0.65, 0.12, 0.04])  # Позиция кнопки / Tugma pozitsiyasi
animate_button = Button(button_ax, 'ЗАПУСК', color='lightgreen', hovercolor='green')

# Подписи к полям ввода / Kiritish maydonlari uchun yorliqlar
fig.text(0.02, 0.88, 'α (°):', fontsize=12, fontweight='bold')      # Угол поворота / Aylanish burchagi
fig.text(0.02, 0.84, 'β (°):', fontsize=12, fontweight='bold')      # Угол наклона / Egilish burchagi
fig.text(0.02, 0.80, 'γ (°):', fontsize=12, fontweight='bold')      # Угол ориентации / Orientatsiya burchagi
fig.text(0.02, 0.76, 'h_CB (см):', fontsize=12, fontweight='bold')  # Высота подъема / Ko'tarilish balandligi
fig.text(0.02, 0.72, 'OA (см):', fontsize=12, fontweight='bold')    # Горизонтальное смещение / Gorizontal siljish

# Создание текстовых полей с начальными значениями / Boshlang'ich qiymatlar bilan matn maydonlarini yaratish
alpha_textbox = TextBox(alpha_ax, '', initial=str(alpha0))  # α параметри uchun TextBox
beta_textbox  = TextBox(beta_ax,  '', initial=str(beta0))   # β параметри uchun TextBox
gamma_textbox = TextBox(gamma_ax, '', initial=str(gamma0))  # γ параметри uchun TextBox
h_textbox     = TextBox(h_ax,     '', initial=str(h0))      # h_CB параметри uchun TextBox
oa_textbox    = TextBox(oa_ax,    '', initial=str(oa0))     # OA параметри uchun TextBox

# ===============================================================================
# ФУНКЦИИ АНИМАЦИИ / ANIMATSIYA FUNKSIYALARI
# ===============================================================================

def start_animation(event):
    """Анимацию запуск функции / Animatsiyani boshlash funksiyasi
    Пользователь кнопку нажимает, параметры читает и анимацию запускает
    Foydalanuvchi tugmani bosganida parametrlarni o'qiydi va animatsiyani boshlaydi
    
    Args:
        event: Событие нажатия кнопки / Tugma bosilish hodisasi
    """
    try:
        # Текстовых полей из значения читаем / Matn maydonlaridan qiymatlarni o'qiymiz
        target_alpha = float(alpha_textbox.text)   # Целевой угол α / Maqsadli α burchagi
        target_beta = float(beta_textbox.text)     # Целевой угол β / Maqsadli β burchagi
        target_gamma = float(gamma_textbox.text)   # Целевой угол γ / Maqsadli γ burchagi
        target_h = float(h_textbox.text)       # Целевая высота h_CB / Maqsadli h_CB balandligi
        target_oa = float(oa_textbox.text)     # Целевое смещение OA / Maqsadli OA siljishi
        
        # Анимацию к целевым значениям запускаем / Maqsadli qiymatlarga animatsiyani ishga tushiramiz
        animate_to_values(target_alpha, target_beta, target_gamma, target_h, target_oa)
    except ValueError:
        # Ошибка при неправильном вводе / Noto'g'ri kiritishda xatolik
        print("Ошибка: Проверьте правильность введенных значений!")

def animate_to_values(target_alpha, target_beta, target_gamma, target_h, target_oa):
    """Плавная анимация к целевым значениям / Maqsadli qiymatlarga silliq animatsiya
    Текущие значения от целевых значений до плавно переходит
    Joriy qiymatlardan maqsadli qiymatlargacha silliq o'tish
    
    Args:
        target_alpha: Целевой угол α / Maqsadli α burchagi
        target_beta: Целевой угол β / Maqsadli β burchagi  
        target_gamma: Целевой угол γ / Maqsadli γ burchagi
        target_h: Целевая высота h_CB / Maqsadli h_CB balandligi
        target_oa: Целевое смещение OA / Maqsadli OA siljishi
    """
    global current_alpha, current_beta, current_gamma, current_h, current_oa
    
    steps = 40  # Animatsiya qadamlari soni / Количество шагов анимации
    
    # Har bir qadam uchun o'zgarish miqdori / Величина изменения для каждого шага
    alpha_step = (target_alpha - current_alpha) / steps  # α uchun qadam / Шаг для α
    beta_step = (target_beta - current_beta) / steps     # β uchun qadam / Шаг для β
    gamma_step = (target_gamma - current_gamma) / steps  # γ uchun qadam / Шаг для γ
    h_step = (target_h - current_h) / steps              # h_CB uchun qadam / Шаг для h_CB
    oa_step = (target_oa - current_oa) / steps           # OA uchun qadam / Шаг для OA
    
    # Анимация цикли / Цикл анимации
    for i in range(steps):
        # Текущие значения обновляем / Joriy qiymatlarni yangilaymiz
        current_alpha += alpha_step
        current_beta += beta_step
        current_gamma += gamma_step
        current_h += h_step
        current_oa += oa_step
        
        # Grafik yangilash / Обновление графики
        # Новые координаты всех точек вычисляем / Barcha nuqtalarning yangi koordinatalarini hisoblaymiz
        A, B, C, D, E, h_ED, n_x, n_y, n_z = compute_chain(current_alpha, current_beta, current_gamma, current_h, current_oa)
        
        # Platformani yangilash (manipulyator bilan birga harakat qiladi)
        # Обновление платформы (движется вместе с манипулятором)
        platform_vertices = create_platform_vertices(current_oa)
        
        # Pastki yuz yangilash / Обновление нижней грани
        bottom_face = [0, 1, 2, 3, 0]
        platform_lines[0].set_data(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1])
        platform_lines[0].set_3d_properties(platform_vertices[bottom_face, 2])
        
        # Yuqori yuz yangilash / Обновление верхней грани
        top_face = [4, 5, 6, 7, 4]
        platform_lines[1].set_data(platform_vertices[top_face, 0], platform_vertices[top_face, 1])
        platform_lines[1].set_3d_properties(platform_vertices[top_face, 2])
        
        # Vertikal qirralarni yangilash / Обновление вертикальных ребер
        for i in range(4):
            platform_lines[i+2].set_data([platform_vertices[i, 0], platform_vertices[i+4, 0]], 
                                        [platform_vertices[i, 1], platform_vertices[i+4, 1]])
            platform_lines[i+2].set_3d_properties([platform_vertices[i, 2], platform_vertices[i+4, 2]])
        
        # POMIDOR EGATLARI YANGILANMAYDI - ular bir joyda turadi!
        # ГРЯДКИ ПОМИДОРОВ НЕ ОБНОВЛЯЮТСЯ - они остаются на месте!
        # (fence_lines o'zgartirilmaydi / fence_lines не изменяются)
        
        # Manipulyator bo'g'inlarini yangilash / Обновление звеньев манипулятора
        sb_line.set_data([A[0], C[0]], [A[1], C[1]])      # BC bo'g'ini / Звено BC
        sb_line.set_3d_properties([A[2], C[2]])
        
        cd_line.set_data([C[0], D[0]], [C[1], D[1]])      # CD bo'g'ini / Звено CD
        cd_line.set_3d_properties([C[2], D[2]])
        
        de_line.set_data([D[0], E[0]], [D[1], E[1]])      # DE bo'g'ini / Звено DE
        de_line.set_3d_properties([D[2], E[2]])
        
        # End effector pozitsiyasini yangilash / Обновление позиции концевого эффектора
        end_scatter._offsets3d = ([E[0]], [E[1]], [E[2]])
        
        # Ma'lumot panelini yangilash / Обновление информационной панели
        info_text = f"α = {format_value(current_alpha)}°\nβ = {format_value(current_beta)}°\nγ = {format_value(current_gamma)}°\nh_CB = {format_value(current_h)}см\nOA = {format_value(current_oa)}см"
        info_ax.clear()
        info_ax.text(0.05, 0.5, info_text, fontsize=13, verticalalignment='center', fontweight='bold')
        info_ax.set_xlim(0, 1)
        info_ax.set_ylim(0, 1)
        info_ax.axis('off')
        
        # Koordinatalar paneli yangilash - har bir nuqta alohida qatorda
        # Обновление панели координат - каждая точка на отдельной строке
        coord_text = f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\nB: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\nC: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\nD: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\nE: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\nn: ({format_value(n_x)}, {format_value(n_y)}, {format_value(n_z)})"
        coord_ax.clear()
        coord_ax.text(0.05, 0.5, coord_text, fontsize=11, verticalalignment='center', fontweight='bold')
        coord_ax.set_xlim(0, 1)
        coord_ax.set_ylim(0, 1)
        coord_ax.axis('off')
        
        # Экран обновляем / Ekranni yangilaymiz
        fig.canvas.draw()          # Графику перерисовываем / Grafikni qayta chizamiz
        fig.canvas.flush_events()  # События обрабатываем / Hodisalarni qayta ishlaymiz
    
    # Aniq qiymatlarni o'rnatish / Установка точных значений
    current_alpha = target_alpha
    current_beta = target_beta
    current_gamma = target_gamma
    current_h = target_h
    current_oa = target_oa

# ===============================================================================
# СОБЫТИЕ ОБРАБОТЧИКИ / HODISA ISHLOVCHILARI
# ===============================================================================
# TextBox widgetlari avtomatik ravishda qiymatlarni saqlaydi
# TextBox виджеты автоматически сохраняют значения

# Tugma event handler / Обработчик события кнопки
animate_button.on_clicked(start_animation)  # Кнопку нажатие событие функции связываем

# ===============================================================================
# ПРОГРАММА ЗАПУСК / DASTURNI ISHGA TUSHIRISH
# ===============================================================================
plt.show()  # Matplotlib окно показываем / Matplotlib oynasini ko'rsatamiz