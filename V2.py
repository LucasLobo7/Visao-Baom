from cv2 import cv2
import numpy as np
import imutils
import time 
from threading import Timer
vid = cv2.VideoCapture(0)
cv2.namedWindow("Controle")
pular = 0
vx = 0.0
vy = 0.0




while True:
    boll, frame = vid.read()
    tempo = time.time()
    
    minimo = np.array([6,145,184])
    maximo = np.array([23,255,255])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    filtrado = cv2.inRange(hsv, minimo, maximo)
    filtrado = cv2.erode(filtrado, None, iterations=2) 
    filtrado = cv2.dilate(filtrado, None, iterations=2)
    #contornar 
    contornos = cv2.findContours(filtrado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos = imutils.grab_contours(contornos)
    centro = None
    if len(contornos) > 0:
        c = max(contornos, key=cv2.contourArea)
        ((x,y), raio) = cv2.minEnclosingCircle(c)
        m = cv2.moments(c)
        centroX = int(m['m10']/m['m00'])
        centroY = int(m['m01']/m['m00'])
        nx = int(x)
        ny = int(y)
        nr = int(raio)
        cv2.circle(frame, (nx,ny), nr, (0,0,255), 2)
        cv2.circle(frame, (centroX,centroY), 5,(255,0,0),-1)

        ## Calculo velocidade (Precisa converter pixels -> m/s) tem que manter dentro do if?
        if(pular > 0):
            vx = (centroX - cxAntigo) / (tempo - tempoAntigo)
            vy = (centroY - cyAntigo) / (tempo - tempoAntigo)
        tempoAntigo = tempo             
        cxAntigo = centroX
        cyAntigo = centroY
        pular += 1  
    
    ## Mostrar imagens
    filtrado = cv2.cvtColor(filtrado, cv2.COLOR_GRAY2BGR)
    imagens = np.hstack((frame,filtrado))
    cv2.imshow('Controle', imagens)
    print("velocidade x", vx)
    print("velocidade y", vy)


    if cv2.waitKey(1) & 0xFF == ord('s'):
        break
