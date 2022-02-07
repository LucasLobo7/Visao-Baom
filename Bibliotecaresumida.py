from io import TextIOBase
import cv2
import numpy as np
import imutils
import time 
import os
import csv
from math import *
from tkinter import *
import glob
import pickle


robotName = "moabRAS_"
boolc1 = False 
lastBallX = 255
lastBallY = 255

lastTime = 0

# with open('datacsv.txt', 'r') as arquivo:
#     leitor = csv.reader(arquivo)
#     dados = list(leitor)

infile = open('cameraconfig','rb')
propriedades = pickle.load(infile)
infile.close

def dataAngles(fileName):
    with open(fileName, 'r') as arquivo:
        leitor = csv.reader(arquivo)
        dados = list(leitor)
        
    return dados

def getTrackBarHSV(hue,saturation,value):

    minimoHSV = np.array([hue[0].get(), saturation[0].get(), value[0].get()])
    maximoHSV = np.array([hue[1].get(), saturation[1].get(), value[1].get()])

    return minimoHSV,maximoHSV

def getTrackBarPID(oKp,oKi,oKd):
    kp = oKp.get()
    ki = oKi.get()
    kd = oKd.get()
    
    return kp,ki,kd

def CalcAlphaBeta(Ix, Iy):

    resSqrt = sqrt(pow(Ix, 2) + pow(Iy, 2))
    if(abs(resSqrt) > 1):
        resSqrt = 1
        
    alpha = degrees(asin(resSqrt))
    if((Ix == 0) or (Iy == 0)):
        y = 180
    else:
        y = round(degrees(atan(Iy / Ix)), 2)

  # Normalização de Quadrante
    if Ix > 0 and Iy >= 0:
        beta = 180 - abs(y)
    elif (Ix > 0 and Iy <= 0):
        beta = 180 + abs(y)
    elif Ix < 0 and Iy >= 0:
        beta = abs(y)
    else:
        beta = 360 - abs(y)
 # Arredondamento par de alpha
    if (round(round(alpha%1,2)-round(alpha%0.1,2),2)%0.2 != 0.0):
        if round(alpha % 1, 1) <= round(alpha % 1, 2):

            alpha = round(alpha, 1) + 0.1
        else:
            alpha = round(alpha, 1)
    else:

        alpha = round(alpha - round(alpha % 0.1, 2), 1)

 #Arredondamento par de beta

    if (round(round(beta%1,2)-round(beta%0.1,2),2)%0.2 != 0.0):

        if round(beta%1,1)<= round(beta%1,2):

            beta = round(beta, 1)+0.1
        else:
            beta = round(beta, 1)
    else:

        beta = round(beta-round(beta%0.1,2), 1)

    if(alpha > 35):
        alpha = 35
    if(beta > 360):
        beta = 360    

    print("Alpha: ", alpha)
    print("Beta: ", beta)
    
    return (beta, alpha)

def calcVelocity(point, lastPoint, currentTime, lastTime):

    dt = (currentTime - lastTime)
    vel = (point - lastPoint) / dt
    
    lastPoint = point
    lastTime = currentTime

    return vel, lastTime,lastPoint


def FilterVision(frame,minimoHSV,maximoHSV):

    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    filtrado = cv2.inRange(hsv, minimoHSV, maximoHSV)

    #comandos para retirar ruido, testar se vale a pena usar
    filtrado = cv2.erode(filtrado, None, iterations=2)
    filtrado = cv2.dilate(filtrado, None, iterations=2)
    
    return filtrado,frame

def contourBall(frame,filtrado):

    contornos = cv2.findContours(filtrado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos = imutils.grab_contours(contornos)

    centro = None
    if len(contornos) > 0:
        c = max(contornos, key = cv2.contourArea ) 
        ((x, y), raio) = cv2.minEnclosingCircle(c)

        m = cv2.moments(c)
        ballX = int(m['m10']/m['m00'])
        ballY = int(m['m01']/m['m00'])
        print("Posição: (", ballX,",",ballY,")")

        nx = int(x)
        ny = int(y)
        nr = int(raio)
        cv2.circle(frame, (nx, ny), nr, (0, 0, 255), 2)
        cv2.circle(frame, (ballX, ballY), 5, (255, 0, 0), -1)
        
        currentTime = time.time()
        lastTime = 0
        lastBallX = 0
        lastBallY = 0
        ball_Vx,lastTime,lastBallX = calcVelocity(ballX,lastBallX,currentTime,lastTime)
        ball_Vy,lastTime,lastBallY = calcVelocity(ballY,lastBallY,currentTime,lastTime)
        print("velocidade: (", ball_Vx,",",ball_Vy,")")
    
    return frame

def NumericalPID(Kp,Ki,Kd, setPoint, point, vel, dt):
    erro = setPoint - point
    Serro = 0
    Serro += erro

    erroVel = erro + 0.2*vel

    I = ( ( Kp*erroVel ) + (Ki*Serro) + (Kd*erroVel/dt))/10000

    return I

def searchAngles(alpha, beta):
    n = int( ((alpha/0.2)+1) + ( ((beta/0.2)+1)*(175)) )
    Angles = dados[n][2:4]  

def takePictures():
    cam = cv2.VideoCapture(0)
    cv2.namedWindow("fotos")
    contador = 0
    while True:
        ret, frame = cam.read()
        if ret == False:
            print("problema na camera")
            break
        cv2.imshow("fotos", frame)

        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            break
        if k%256 == 32:
            # SPACE pressed
            NomeImagem = "Calibra{}.png".format(contador)
            cv2.imwrite(NomeImagem, frame)
            print("{} salvo!".format(NomeImagem))
            contador += 1
    cv2.destroyWindow('fotos')
    
def calibrateCam():
    cam = cv2.VideoCapture(0)
    tamanhoTabuleiro = (12,7) # number of squares
    ret, frame = cam.read()
    tamanho = (frame.shape[0],frame.shape[1])

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((tamanhoTabuleiro[0] * tamanhoTabuleiro[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:tamanhoTabuleiro[0],0:tamanhoTabuleiro[1]].T.reshape(-1,2)


    objetoReal = [] #points in real world space
    objetoImagem = [] #points in image plane.

    imagens = glob.glob('*.png')
    for imagem in imagens:

        img = cv2.imread(imagem)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, vertice = cv2.findChessboardCorners(gray, tamanhoTabuleiro, None) # Find the chess board corners

        if ret == True:

            objetoReal.append(objp)
            verticeNovo = cv2.cornerSubPix(gray, vertice, (11,11), (-1,-1), criteria)
            objetoImagem.append(vertice)

            cv2.drawChessboardCorners(img, tamanhoTabuleiro, verticeNovo, ret) # Draw corners
            cv2.imshow('img', img)
            while(True):
                k = cv2.waitKey(1)
                if k%256 == 32:
                    break
    cv2.destroyAllWindows()
    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objetoReal, objetoImagem, tamanho, None, None)
    propriedades = [ret, cameraMatrix, dist, rvecs, tvecs]

    outfile = open('cameraconfig','wb')
    pickle.dump(propriedades,outfile)
    outfile.close
    return propriedades

def undistort(imagem,propriedades):
    h, w = imagem.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(propriedades[1], propriedades[2], (w,h), 1, (w,h))

    imagem = cv2.undistort(imagem, propriedades[1], propriedades[2], None, newCameraMatrix)

    x, y, w, h = roi
    imagem = imagem[y:y+h, x:x+w]
