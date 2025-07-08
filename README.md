# POLI-Garra
![Garra](https://github.com/user-attachments/assets/679e970a-3752-485f-bd0b-28b5819ce4d1)
## 0. Introdução ao Projeto
A POLI-Garra é um robô criado a fim de demonstrar a separação automática de lixo, utilizando técnicas de Processamento de Imagem (PDI) e Machine Learning (ML).

## 1. Montagem
  ### 1.1 Eletrônicos:
  Os eletrônicos utilizados no projeto foram:
  * 4 Servos SG90: para controlar todas as articulações da garra;
  * 1 Sensor de proximidade: quando um objeto está próximo suficiente da garra, o sensor aciona a câmera para tirar uma foto 
  * 1 Arduíno: para controlar a movimentação da garra, o sensor de proximidade e fazer a comunicação com código externo (PDI e ML);
  * 1 Protoboard: para facilitar as conexões entre Arduino, servos, sensor e fonte;
  * 1 Fonte 5V: para alimentar os servos e sensor;
  * 1 Câmera: para obter a imagem dos objetos posicionados na frente da garra.

  ### 1.2 Carcaça:
  Para a carcaça/corpo do robô, foi utilizado impressão 3D. Foram impressas diversas peças, que após polidas foram anexadas com parafusos.

  ### 1.3 Miniaturas:
  Para os testes confeccionamos algumas miniaturas representando os diferentes tipos de lixo.
  
![462840147-13d94890-0e48-4db9-969a-ca378475deeb](https://github.com/user-attachments/assets/9ffd3952-bdee-43c1-ba46-9cfa948f8357)
//foto miniaturas


## 2. Simulações no Coppeliasim
Inicialmente utilizamos o Coppeliasim para validar o protótipo, simulando os movimentos da garra e o modelo classificador.

https://github.com/user-attachments/assets/74c5f524-cc54-417d-bd72-2f93921bb70f


https://github.com/user-attachments/assets/5be12b5e-d645-49fb-9847-7bf1b857ea93

### Comparação com a Garra:
https://github.com/user-attachments/assets/d4cafcb3-beaa-4d3d-8039-004f4e5429fe


## 3. Fluxo do projeto
![Fluxo](https://github.com/user-attachments/assets/c4004daf-345d-4a4c-8691-60adf487c9b8)
  ### 1 Detecção de Objeto:
  * O sensor detecta a presença de um objeto.
  * O Arduino envia o comando "foto" via porta serial para o computador.
  
  ### 2 Captura e Recorte da Imagem:
  * O Python ativa a webcam.
  * Captura a imagem da região de interesse.
  * Salva a imagem e a usa para detectar o tipo de lixo.
  
  ### 3.1 Classificação com YOLO:
  A imagem é enviada para o modelo YOLOv8 (via Roboflow).
  Se algum objeto for detectado:
  * Converte a classe YOLO para um tipo de resíduo (metal, plastico, vidro, etc.) com base em um mapeamento.
  * Envia esse tipo para o Arduino.
  
  ### 3.2 Classificação por Cor (HSV):
  Usamos biscuit para modelar alguns lixos, impossibilitando que o modelo consiga detectar a maioria.
  Então, se nada for detectado pelo YOLO:
  * A imagem é convertida para HSV.
  * Verifica-se a presença de faixas de cor pré-definidas.
  * O material é detectado e é enviado para o Arduino.
  
  ### 4 Movimentação da Garra:
  * O Arduino recebe o tipo do material.
  * Move a garra até o centro (posição inicial).
  * Abaixa e pega o objeto.
  * Move para a posição da lixeira correspondente.
  * Solta o objeto e retorna para a posição inicial.

## 4. Algoritmo de Machine Learning
A detecção com Machine Learning foi feita com YOLOv8 via Roboflow. O modelo YOLO é utilizado para identificar visualmente um objeto prototipado e classificá-lo entre dezenas de categorias (ex: plastic bottle, tin, glass bottle, paper cup, etc.).
Em seguida, essas classes são convertidas para tipos de resíduos compreendidos pelo sistema (ex: tin → metal, glass bottle → vidro), por meio de um dicionário de mapeamento.

```
codigo
```
//foto da classificacao

## 5. Algoritimo de PDI
Caso o modelo YOLO não consiga identificar nenhum dos objetos prototipados, o sistema utiliza análise por cor no espaço HSV para tentar identificar o material com base na coloração predominante.
São consideradas faixas de HSV para objetos orgânicos e vidro.
```
import cv2
import serial
import time
import numpy as np
from inference_sdk import InferenceHTTPClient
#(x=215, y=172, w=69, h=91)
# === CONFIGURAÇÕES  GLOBAIS===
PORTA_SERIAL = 'COM12'  # Porta serial
BAUD_RATE = 9600
AREA_RECORTE = (329, 221, 42, 53)  # (x, y, largura, altura)
IMAGE_PATH = "recorte.jpg"

# Inicializa o modelo do Roboflow
MODEL = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="ejAvalkdOP2sWCZkux3L"
)
# Mapeamento de classes para a garra
MAPEAMENTO_CLASSES = {
    "Aluminum can": "metal",
    "Aluminum caps": "metal",
    "Iron utensils": "metal",
    "Metal shavings": "metal",
    "Scrap metal": "metal",
    "Tin": "metal",
    "paper": "papel",
    "glass": "vidro",
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
            area_total += cv2.countNonZero(mask)  # soma pixels detectados
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
    for _ in range(5): cap.read(); time.sleep(0.05) # isso aqui botei pra ver se ele para de bugar ,mas ve se funciona
    ret, frame = cap.read()
    if not ret:
        print("Erro ao capturar imagem.")
        return False

    # recorte
    x, y, w, h = AREA_RECORTE
    recorte = frame[y:y+h, x:x+w]

    # salvar imagens
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
                if class_name == "nenhum":
                    img = cv2.imread(IMAGE_PATH)
                    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                    class_name = detectar_cor(hsv)           
                             
                if class_name != "nenhum":
                    ser.write(f"{class_name}\n".encode())
                    print(f"Enviado: {class_name}")
                else:
                    print("Nenhuma classe reconhecida. Ignorando.")
                    modo_ativo = True # tira nova foto

        elif comando == "zero" and not modo_ativo:
            modo_ativo = True
            print("[INFO] Sistema desbloqueado. Pronto para nova captura.")

    time.sleep(0.1)


```
![pdi_banana](https://github.com/user-attachments/assets/49298a16-9648-47dd-9ba6-bc8245cee7dd)

## Resultado
// fotos de garra e video funcionando
