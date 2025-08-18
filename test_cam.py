import cv2
from datetime import datetime

# Cambia el índice si es necesario (0 suele ser /dev/video0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    exit()

print("Presiona 'c' para capturar imagen. Presiona 'q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo leer el frame.")
        break

    cv2.imshow("Vista del Microscopio USB", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        # Genera nombre de archivo con la fecha y hora actual
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"captura_{timestamp}.png"
        cv2.imwrite(filename, frame)
        print(f"Imagen guardada como: {filename}")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
