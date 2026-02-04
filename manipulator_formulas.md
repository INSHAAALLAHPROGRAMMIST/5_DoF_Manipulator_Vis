# Manipulyator Kinematik Formulalari

> **MUHIM OGOHLANTIRISH:** 
> 
> Ushbu formulalar va chegaralar **FAQAT JORIY MANIPULYATOR** uchun mo'ljallangan va uning aniq xarakteristikalariga moslashtirilgan. 
> 
> **Foydalanuvchilar o'z manipulyatorlari uchun quyidagi parametrlarni o'z qurilmalarining texnik xususiyatlariga qarab o'zgartirishi SHART:**
> - Bo'g'inlar uzunligi (L, h_DC)
> - Harakat chegaralari (H_CB_MIN, H_CB_MAX)
> - Burchak chegaralari (α, β, γ limitlari)
> - Koordinata tizimi yo'nalishi
> - Fizik cheklovlar va xavfsizlik zonalari
>
> **Bu formulalarni boshqa manipulyatorlarda ishlatishdan oldin, albatta o'z qurilmangizning texnik hujjatlarini tekshiring!**

## Konstantalar (Joriy manipulyator uchun)

- **L** = 30 sm (DE bo'g'in uzunligi)
- **h_DC** = 30 sm (CD bo'g'in uzunligi)
- **H_CB_MIN** = 29.5 sm (minimal h_CB balandligi)
- **H_CB_MAX** = 57.0 sm (maksimal h_CB balandligi)

---

## 1. FORWARD KINEMATICS (To'g'ri kinematika)

### Berilgan parametrlar:
- α (gradus) - Z o'qi atrofida aylanish burchagi
- β (gradus) - vertikal tekislikda egilish burchagi
- γ (gradus) - oxirgi bo'g'inning orientatsiya burchagi
- h_CB (sm) - BC bo'g'inining balandligi
- OA (sm) - gorizontal siljish

### Cheklovlar:
```
H_CB_MIN ≤ h_CB ≤ H_CB_MAX
29.5 ≤ h_CB ≤ 57.0
```

### Formulalar:

#### 1.1 Burchaklarni radianga o'tkazish:
```
α_rad = α × π/180
β_rad = β × π/180
γ_rad = γ × π/180
```

#### 1.2 Nuqtalar koordinatalari:

**A va B nuqtalari:**
```
A = [OA, 0, 0]
B = [OA, 0, 0]    (B = A)
```

**C nuqtasi:**
```
C = [OA, 0, h_CB]
```

**D nuqtasi:**
```
D_x = OA + cos(β) × h_DC × cos(α)
D_y = cos(β) × h_DC × sin(α)
D_z = h_CB + sin(β) × h_DC

D = [D_x, D_y, D_z]
```

**E nuqtasi:**
```
E_x = OA + L × (cos(α)×cos(β) - cos(α)×cos(γ+β))
E_y = L × (sin(α)×cos(β) - sin(α)×cos(γ+β))
E_z = L × (sin(β) - sin(γ+β)) + h_CB

E = [E_x, E_y, E_z]
```

#### 1.3 Birlik vektor n:
```
DE_vector = E - D
h_ED = √((E_x - D_x)² + (E_y - D_y)² + (E_z - D_z)²)

n_x = (E_x - D_x) / h_ED
n_y = (E_y - D_y) / h_ED
n_z = (E_z - D_z) / h_ED
```

---

## 2. INVERSE KINEMATICS (Teskari kinematika)

### Berilgan parametrlar:
- E_x, E_y, E_z - E nuqtasining koordinatalari
- n_x, n_y, n_z - birlik vektorning komponentlari

### Formulalar:

#### 2.1 Birlik vektorni normalizatsiya qilish:
```
|n| = √(n_x² + n_y² + n_z²)

Agar |n| ≠ 1:
    n_x = n_x / |n|
    n_y = n_y / |n|
    n_z = n_z / |n|
```

#### 2.2 Nuqtalar koordinatalari:

**E nuqtasi:**
```
E = [E_x, E_y, E_z]
```

**D nuqtasini hisoblash:**
```
D = E - L × [n_x, n_y, n_z]

D_x = E_x - L × n_x
D_y = E_y - L × n_y
D_z = E_z - L × n_z
```

**A va B nuqtalari:**
```
A = [0, 0, 0]
B = [0, 0, 0]    (B = A)
```

**C nuqtasini hisoblash:**
```
CD_horizontal = √(D_x² + D_y²)

Agar CD_horizontal² ≤ h_DC²:
    vertical_distance = √(h_DC² - CD_horizontal²)
    C_z = D_z - vertical_distance
    
    Agar C_z < 0:
        C_z = D_z + vertical_distance
Aks holda:
    C_z = D_z

C = [0, 0, C_z]
```

#### 2.3 Fizik cheklovlarni tekshirish:
```
h_CB = C_z

Agar h_CB < H_CB_MIN:
    C_z = H_CB_MIN
    Ogohlantirish: "Pozitsiya fizik jihatdan mumkin emas!"

Agar h_CB > H_CB_MAX:
    C_z = H_CB_MAX
    Ogohlantirish: "Pozitsiya fizik jihatdan mumkin emas!"
```

---

## 3. UZUNLIKLARNI TEKSHIRISH

### CD bo'g'in uzunligi:
```
CD_length = √((D_x - C_x)² + (D_y - C_y)² + (D_z - C_z)²)
CD_length = h_DC = 30 sm
```

### DE bo'g'in uzunligi:
```
DE_length = √((E_x - D_x)² + (E_y - D_y)² + (E_z - D_z)²)
DE_length = L = 30 sm
```

---

## 4. MATEMATIK MUNOSABATLAR

### Forward → Inverse bog'lanish:
```
Forward: (α, β, γ, h_CB, OA) → (A, B, C, D, E, n)
Inverse: (E_x, E_y, E_z, n_x, n_y, n_z) → (A, B, C, D, E)
```

### Asosiy tenglamalar:
```
1. |DE| = L = 30 sm
2. |CD| = h_DC = 30 sm
3. A = B = [0, 0, 0] (soddalashtirilgan holat)
4. C = [0, 0, h_CB]
5. H_CB_MIN ≤ h_CB ≤ H_CB_MAX
```

### Geometrik cheklovlar:
```
1. CD² = CD_horizontal² + (D_z - C_z)² = 900
2. DE = L × n (birlik vektor yo'nalishida)
3. C nuqtasi A ning ustida (vertikal)
4. A va B nuqtalari koordinata boshida
```

---

## FOYDALANUVCHILAR UCHUN SOZLASH YO'RIQNOMASI

### O'z manipulyatoringiz uchun sozlash:

#### 1. Bo'g'inlar uzunligini o'zgartirish:
```python
# Kodda quyidagi qiymatlarni o'zgartiring:
L = 30.0        # → O'z DE bo'g'in uzunligingiz (sm)
h_DC = 30.0     # → O'z CD bo'g'in uzunligingiz (sm)
```

#### 2. Harakat chegaralarini sozlash:
```python
# Kodda quyidagi qiymatlarni o'zgartiring:
H_CB_MIN = 29.5  # → O'z manipulyatoringizning minimal balandligi
H_CB_MAX = 57.0  # → O'z manipulyatoringizning maksimal balandligi
```

#### 3. Qo'shimcha cheklovlar qo'shish:
- **Burchak chegaralari:** α, β, γ uchun min/max qiymatlar
- **Tezlik chegaralari:** Harakat tezligi limitlari  
- **Kuch chegaralari:** Maksimal yuk ko'tarish qobiliyati
- **Xavfsizlik zonalari:** Taqiqlangan hududlar

#### 4. Koordinata tizimini moslash:
- **Baza nuqtasi:** A nuqtasining joylashuvi
- **O'qlar yo'nalishi:** X, Y, Z o'qlarining musbat yo'nalishlari
- **Birliklar:** sm, mm, m - kerakli o'lchov birligini tanlash

### Tekshirish bosqichlari:
1. **Fizik o'lchash:** Manipulyatorning haqiqiy o'lchamlarini o'lchang
2. **Harakat testlari:** Min/max pozitsiyalarni sinab ko'ring
3. **Xavfsizlik tekshiruvi:** Barcha chegaralar xavfsiz ekanligini tasdiqlang
4. **Kalibratsiya:** Haqiqiy va hisoblangan qiymatlarni solishtiring

---

## 5. MISOL HISOBLASH (Joriy manipulyator uchun)

### Forward kinematics misoli:
```
Berilgan: α=0°, β=135°, γ=15°, h_CB=30sm, OA=0sm

Natija:
A = [0, 0, 0]
B = [0, 0, 0]
C = [0, 0, 30]
D = [-21.21, 0, 51.21]
E = [4.77, 0, 36.21]
n = [0.866, 0, -0.5]
```

### Inverse kinematics misoli:
```
Berilgan: E=[4.77, 0, 36.21], n=[0.866, 0, -0.5]

Natija:
A = [0, 0, 0]
B = [0, 0, 0]
C = [0, 0, 30]
D = [-21.21, 0, 51.21]
E = [4.77, 0, 36.21]
```

Bu ikkala natija bir xil, demak formulalar to'g'ri!

---

## XAVFSIZLIK VA MAS'ULIYAT

⚠️ **DIQQAT:** 
- Ushbu formulalar faqat hisoblash maqsadida berilgan
- Real manipulyatorni boshqarishdan oldin xavfsizlik choralarini ko'ring
- Noto'g'ri parametrlar qurilmaga zarar yetkazishi mumkin
- Har doim texnik hujjatlarga amal qiling
- Shubhali hollarda mutaxassisga murojaat qiling

📋 **TAVSIYALAR:**
- Dastlab kichik qiymatlar bilan test qiling
- Har bir o'zgarishdan keyin ehtiyotkorlik bilan tekshiring  
- Backup va qayta tiklash imkoniyatlarini tayyorlang
- Xavfsizlik sensorlari va to'xtash tugmalarini o'rnating

---

*Bu hujjat joriy loyiha uchun yaratilgan va boshqa manipulyatorlar uchun moslashtirilishi kerak.*