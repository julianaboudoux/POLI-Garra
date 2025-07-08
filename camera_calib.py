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
