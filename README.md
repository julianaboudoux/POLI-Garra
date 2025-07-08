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
  ![462840147-13d94890-0e48-4db9-969a-ca378475deeb](https://github.com/user-attachments/assets/9ffd3952-bdee-43c1-ba46-9cfa948f8357)
  
  ![20250610_012948(2)(1)](https://github.com/user-attachments/assets/8df4076a-4dbf-40bd-b7e0-a4ae64b159b0)


  ### 1.3 Miniaturas:
  Para os testes confeccionamos algumas miniaturas representando os diferentes tipos de lixo.
![Screenshot_20250708_072509_Samsung Notes](https://github.com/user-attachments/assets/fa3687bd-e3b7-4207-87e2-38da38acaaf6)



## 2. Simulações no Coppeliasim
Inicialmente, utilizamos o Coppeliasim para validar o protótipo, simulando os movimentos da garra e o modelo classificador.

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
A utilização de biscuit para modelar as miniaturas impossibilitou que algumas delas fossem reconhecidas pelo modelo.
  Nesse caso, se nada for detectado pelo YOLO:
  * A imagem é convertida para HSV.
  * Verifica-se a presença de faixas de cor pré-definidas.
  * O material é detectado e é enviado para o Arduino.
  
  ### 4 Movimentação da Garra:
  * O Arduino recebe o tipo do material.
  * Move a garra até o centro (posição inicial).
  * Se desloca e pega o objeto.
  * Move para a posição da lixeira correspondente.
  * Solta o objeto e retorna para a posição inicial.

## 4. Código do Arduino:
```
#include <Servo.h>
#include <String.h>
// Servos
Servo baseServo;   
Servo bracoServo;  
Servo pulsoServo;  
Servo garraServo;

#define FIM_DE_CURSO_PIN 7

String comando = "";
bool aguardandoResposta = false;

// Posições de referência
const int BASE_ZERO = 90;
const int BASE_ORGANICO = 0;
const int BASE_PLASTICO = 45;
const int BASE_METAL = 135;
const int BASE_VIDRO = 180;

const int BRACO = 80;

const int GARRA_ABERTA = 40;
const int GARRA_FECHADA = 0;

const int PULSO_BAIXO = 0;
const int PULSO_ALTO = 90;
int state;


//Funções de Movimento
// Movements
void moverServo(Servo& servo, int alvo, int passo = 1, int delay_ms = 60) {
  int atual = servo.read();  // Lê a posição atual real do servo

  alvo = constrain(alvo, 0, 180);


  if (atual < alvo) {
    for (int pos = atual; pos <= alvo; pos += passo) {
      servo.write(pos);
      delay(delay_ms);
    }
  } else {
    for (int pos = atual; pos >= alvo; pos -= passo) {
      servo.write(pos);
      delay(delay_ms);
    }
  }
}

void zero() {
  moverServo(garraServo, GARRA_ABERTA);
  moverServo(pulsoServo, PULSO_ALTO);
  moverServo(bracoServo, BRACO);
  moverServo(baseServo, BASE_ZERO);
  delay(200);
}

void fetch() {
  moverServo(pulsoServo, PULSO_BAIXO);
  moverServo(garraServo, GARRA_FECHADA);
  moverServo(pulsoServo, PULSO_ALTO);
}

void drop(String thrash) {
  int posicao;

  if (thrash == "metal") {
    posicao = BASE_METAL;
  } else if (thrash == "vidro") {
    posicao = BASE_VIDRO;
  } else if (thrash == "organico") {
    posicao = BASE_ORGANICO;
  } else if (thrash == "plastico") {
    posicao = BASE_PLASTICO;
  } else {
    Serial.println("Comando desconhecido, tente novamente");
    return;
  }

  moverServo(baseServo, posicao);
  moverServo(pulsoServo, PULSO_BAIXO);
  moverServo(garraServo, GARRA_ABERTA);
  moverServo(pulsoServo, PULSO_ALTO);
}


void executarComando(String material) {
  if (material == "lixo" || material == "") return;

  Serial.print(">> Detectado: ");
  Serial.println(material);
  zero();
  fetch();
  drop(material);
  zero();
}


void setup() {
  Serial.begin(9600);
  pinMode(FIM_DE_CURSO_PIN, INPUT_PULLUP);

  baseServo.attach(2);
  bracoServo.attach(5);
  pulsoServo.attach(4);
  garraServo.attach(3);
  state = 0;
  zero();
  Serial.println("Sistema iniciado. Monitorando sensor fim de curso...");
}

void loop() {
  if (!aguardandoResposta && digitalRead(FIM_DE_CURSO_PIN) == LOW) {
    Serial.println("foto");  // pede imagem ao Python
    aguardandoResposta = true;
    comando = "";
    delay(500);
  }

  if (aguardandoResposta && Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      comando.trim();
      executarComando(comando);
      aguardandoResposta = false;
      comando = "";
      Serial.println("zero"); // sinaliza ao Python que pode continuar
    } else {
      comando += c;
    }
  }
}
```

## 5. Algoritmo de Machine Learning e PDI
A detecção com Machine Learning foi feita com YOLOv8 via Roboflow. O modelo YOLO é utilizado para identificar visualmente um objeto prototipado e classificá-lo entre dezenas de categorias (ex: plastic bottle, tin, glass bottle, paper cup, etc.).
Em seguida, essas classes são convertidas para tipos de resíduos compreendidos pelo sistema (ex: tin → metal, glass bottle → vidro), por meio de um dicionário de mapeamento.

Caso o modelo YOLO não consiga identificar nenhum dos objetos prototipados, o sistema utiliza análise por cor no espaço HSV para tentar identificar o material com base na coloração predominante.
São consideradas faixas de HSV para objetos orgânicos e vidro.

### Código para o Recorte da Imagem:
```
import cv2

# === VARIÁVEIS GLOBAIS ===
drawing = False
ix, iy = -1, -1
fx, fy = -1, -1
recorte = None
frame_atual = None

def desenhar_retangulo(event, x, y, flags, param):
    global ix, iy, fx, fy, drawing, frame_atual, recorte

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            fx, fy = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        fx, fy = x, y
        x0, y0 = min(ix, fx), min(iy, fy)
        x1, y1 = max(ix, fx), max(iy, fy)
        recorte = frame_atual[y0:y1, x0:x1]
        print(f"[INFO] Coordenadas do recorte: ({x0}, {y0}, {x1 - x0}, {y1 - y0})")

# === INICIA WEBCAM ===
cap = cv2.VideoCapture(1)
cv2.namedWindow("Webcam")
cv2.setMouseCallback("Webcam", desenhar_retangulo)

print("[INFO] Pressione 's' para salvar o recorte. Pressione 'q' para sair.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erro ao acessar webcam.")
        break

    frame_atual = frame.copy()

    # Desenha retângulo durante movimentação
    if drawing or recorte is not None:
        x0, y0 = min(ix, fx), min(iy, fy)
        x1, y1 = max(ix, fx), max(iy, fy)
        cv2.rectangle(frame_atual, (x0, y0), (x1, y1), (0, 255, 0), 2)

    cv2.imshow("Webcam", frame_atual)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s') and recorte is not None:
        cv2.imwrite("recorte_calibrado.jpg", recorte)
        print("[INFO] Recorte salvo como 'recorte_calibrado.jpg'.")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

```

### Código para o Classificador:
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
```
![pdi_banana](https://github.com/user-attachments/assets/49298a16-9648-47dd-9ba6-bc8245cee7dd)

## Resultado
![Foto de Igor](https://github.com/user-attachments/assets/b840669f-4232-4c86-a9ca-af3f0448b4c4)


https://github.com/user-attachments/assets/aa308237-e751-4629-9061-b9ea88760b09


https://github.com/user-attachments/assets/f5e09f35-8620-4692-a2b9-b6fa4daa856b



https://github.com/user-attachments/assets/d9fa307f-c654-4f9a-9d2b-aced07bf1218


https://github.com/user-attachments/assets/c3fe43d8-73a9-4003-9019-c505cf67c479




