# POLI-Garra
![Garra](https://github.com/user-attachments/assets/679e970a-3752-485f-bd0b-28b5819ce4d1)
## 0. Introdução ao Projeto
A POLI-Garra é um robô criado a fim de demonstrar a separação automática de lixo, utilizando técnicas de Processamento de Imagem (PDI) e Machine Learning (ML).

## 1. Montagem
  1.1 Eletrônicos:
  Os eletrônicos utilizados no projeto foram:
  * 4 Servos SG90: para controlar todas as articulações da garra;
  * 1 Sensor de proximidade: quando um objeto está próximo suficiente da garra, o sensor aciona a câmera para tirar uma foto 
  * 1 Arduíno: para controlar a movimentação da garra, o sensor de proximidade e fazer a comunicação com código externo (PDI e ML);
  * 1 Protoboard: para facilitar as conexões entre Arduino, servos, sensor e fonte;
  * 1 Fonte 5V: para alimentar os servos e sensor;
  * 1 Câmera: para obter a imagem dos objetos posicionados na frente da garra.

  1.2 Carcaça:
  Para a carcaça/corpo do robô, foi utilizado impressão 3D. Foram impressas diversas peças, que após polidas foram anexadas com parafusos.
  
  

![WhatsApp Image 2025-07-05 at 18 47 27](https://github.com/user-attachments/assets/13d94890-0e48-4db9-969a-ca378475deeb)

## 2. 🔁Fluxo do projeto
1️⃣ Detecção de Objeto
O sensor detecta a presença de um objeto.

O Arduino envia o comando "foto" via porta serial para o computador.

2️⃣ Captura e Recorte da Imagem (Python)
O Python ativa a webcam.

Captura a imagem da região de interesse (ROI).

Salva a imagem e a usa para detectar o tipo de lixo.

3️⃣ Classificação com YOLO 
A imagem é enviada para o modelo YOLOv8 (via Roboflow).

Se algum objeto for detectado:

Converte a classe YOLO para um tipo de resíduo (metal, plastico, vidro, etc.) com base em um mapeamento.

Envia esse tipo para o Arduino.

4️⃣ Classificação por Cor (HSV)
 usamos biscuit para modelar alguns lixos, impossibilitando que o modelo consiga detectar a maioria.
Então, se nada for detectado pelo YOLO:

A imagem é convertida para HSV.

Verifica-se a presença de faixas de cor pré-definidas.

O material é detectado e é enviado para o Arduino.

5️⃣ Movimentação da Garra Robótica (Arduino)
O Arduino recebe o tipo do material.

Move a garra até o centro (posição zero).

Abaixa e pega o objeto.

Move para a posição da lixeira correspondente.

Solta o objeto e retorna para a posição inicial.

## 3. Algoritmo de Machine Learning


## 4. Algoritimo de PDI


## Resultado

