import cv2
import serial
import time
import numpy as np
from inference_sdk import InferenceHTTPClient
#(x=215, y=172, w=69, h=91)
# === CONFIGURAÇÕES  GLOBAIS===
PORTA_SERIAL = 'COM12'  # Porta serial
BAUD_RATE = 9600
AREA_RECORTE = (213, 94, 143, 178) # (x, y, largura, altura)
IMAGE_PATH = "recorte.jpg"

# Inicializa o modelo do Roboflow
MODEL = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="ejAvalkdOP2sWCZkux3L"
)
# Mapeamento de classes para a garra
MAPEAMENTO_CLASSES = {
    # METAL
    "Aluminum can": "metal",
    "Aluminum caps": "metal",
    "Iron utensils": "metal",
    "Metal shavings": "metal",
    "Scrap metal": "metal",
    "Tin": "metal",
    "Printing industry": "metal",

    # VIDRO
    "Glass bottle": "vidro",
    "Ceramic": "vidro",
    "Container for household chemicals": "vidro",

    # PLÁSTICO
    "Combined plastic": "plástico",
    "Disposable tableware": "plástico",
    "Foil": "plástico",
    "Milk bottle": "plástico",
    "Plastic bag": "plástico",
    "Plastic bottle": "plástico",
    "Plastic can": "plástico",
    "Plastic canister": "plástico",
    "Plastic caps": "plástico",
    "Plastic cup": "plástico",
    "Plastic shaker": "plástico",
    "Plastic shavings": "plástico",
    "Plastic toys": "plástico",
    "Postal packaging": "plástico",
    "Stretch film": "plástico",
    "Tetra pack": "plástico",
    "Unknown plastic": "plástico",
    "Zip plastic bag": "plástico",

    # ORGÂNICO
    "Organic": "organico",
    "Food": "organico",
    "Cellulose": "organico",
    "Wood": "organico",

    # PAPEL
    "Cardboard": "organico",
    "Paper": "organico",
    "Paper bag": "organico",
    "Paper cups": "organico",
    "Paper shavings": "organico",
    "Papier mache": "organico",

    # OUTROS 
    "Electronics": "outros",
    "Furniture": "outros",
    "Textile": "outros",
    "Liquid": "outros", 
}


def detectar_cor(imagem_hsv):
    
    faixas = {
        "vidro": [
            (np.array([35,10,10]), np.array([70,250,250])),
            (np.array([0,10,20]), np.array([150,240,115]))
        ],

        "organico": [  # substitui papel
            (np.array([0,85,15]), np.array([19,182,116])),   # maçã
            (np.array([10,65,4]), np.array([177,255,242])),   # banana
            (np.array([0,40,10]), np.array([20,120,180]))     # pizza
        ]
    }
    areas_por_tipo = {}
    for tipo, ranges in faixas.items():
        area_total = 0
        for lower, upper in ranges:
            mask = cv2.inRange(imagem_hsv, lower, upper)
            area_total += cv2.countNonZero(mask)  
        areas_por_tipo[tipo] = area_total
            
        tipo_detectado = max(areas_por_tipo, key=areas_por_tipo.get)
        if areas_por_tipo[tipo_detectado] > 500:
            print(f"Detecção por cor: {tipo_detectado} com área total {areas_por_tipo[tipo_detectado]:.2f}")
            return tipo_detectado
        else:
            print("Nenhum material identificado por cor.")
            return "nenhum" 
# === FUNÇÃO DE CAPTURA E RECORTE ===
def capturar_e_recortar(cap):
    for _ in range(5): cap.read(); time.sleep(0.05) 
    ret, frame = cap.read()
    if not ret:
        print("Erro ao capturar imagem.")
        return False

    x, y, w, h = AREA_RECORTE
    recorte = frame[y:y+h, x:x+w]

    cv2.imwrite("imagem_completa.jpg", frame)
    cv2.imwrite(IMAGE_PATH, recorte)
    print("[INFO] Imagens salvas.")
    return True

# === INICIALIZA SERIAL E CÂMERA ===
ser = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=1)
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Erro ao acessar a webcam.")
    exit()

print("Aguardando comandos...")
modo_ativo = True  # habilita comandos

# === LOOP PRINCIPAL ===
while True:
    if ser.in_waiting > 0:
        comando = ser.readline().decode().strip()
        print(f"[ARDUINO] {comando}")

        if comando == "foto" and modo_ativo:
            modo_ativo = False  # bloqueia comandos
            time.sleep(0.3)
            if capturar_e_recortar(cap):
                class_name = "nenhum"
                result = MODEL.infer(IMAGE_PATH, model_id="yolo-waste-detection/1")
                
                if result['predictions']:
                    classe_detectada = result['predictions'][0]['class']
                    print(f"Classe YOLO: {classe_detectada}")
                    class_name = MAPEAMENTO_CLASSES.get(classe_detectada, "nenhum")
                else:
                    print("Nenhuma predição.")
                  # Se YOLO não deu encontrou, faz PDI
                if class_name in ("nenhum", "outros", "organico", "plastico", "vidro"):
                    img = cv2.imread(IMAGE_PATH)
                    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                    class_name = detectar_cor(hsv)           
                             
                if class_name != "nenhum":
                    ser.write(f"{class_name}\n".encode())
                    print(f"Enviado: {class_name}")
                else:
                    print("Nenhuma classe reconhecida. Ignorando.")
                    modo_ativo = True

        elif comando == "zero" and not modo_ativo:
            modo_ativo = True
            print("[INFO] Sistema desbloqueado. Pronto para nova captura.")

    time.sleep(0.1)

