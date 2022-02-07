from tkinter import *
import cv2
from cv2 import destroyWindow
import Bibliotecaresumida as zxc
import pickle
import time
import numpy as np
import imutils
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
from PIL import Image, ImageTk 


cor1 = '#404040' # cinza de fundo
cor2 = '#ff7f2a' #
cor3 = '#757575'
tamanhoTabuleiro = (12,7)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
cam = cv2.VideoCapture(0)
ret, frame = cam.read()
boolfoto = False
boolconfirmo = False
boolrecuso = False
def boolfoto():
    global boolfoto
    boolfoto = not boolfoto

def boolconfirmo():
    global boolconfirmo
    boolconfirmo = not boolconfirmo

def boolrecuso():
    global boolrecuso
    boolrecuso = not boolrecuso

def cali():
    cam = cv2.VideoCapture(0)
    cv2.namedWindow("fotos")
    contador = 0
    global boolfoto
    global boolrecuso
    global boolconfirmo
    boolfoto = False
    boolrecuso = False
    boolconfirmo = False
    while True:
        ret, frame = cam.read()
        salvo = frame
        if ret == False:
            print("problema na camera")
            break
        cv2.imshow("fotos", frame)
        tamanho = (frame.shape[0],frame.shape[1])
        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            break
        menu.update()
        if boolfoto  == True:
            # SPACE pressed
            NomeImagem = "calibra{}.png".format(contador)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, vertice = cv2.findChessboardCorners(gray, tamanhoTabuleiro, None) # Find the chess board corners
            if ret==0:
                print('erro no tabuleiro')
                boolfoto  = False
            else:
                verticeNovo = cv2.cornerSubPix(gray, vertice, (11,11), (-1,-1), criteria)
                desenhos = frame
                cv2.imwrite(NomeImagem, salvo)
                cv2.drawChessboardCorners(desenhos, tamanhoTabuleiro, verticeNovo, ret) # Draw corners
                imagem = cv2.cvtColor(desenhos, cv2.COLOR_BGR2RGB) 
                imagem = Image.fromarray(imagem)
                tkimage = ImageTk.PhotoImage(imagem)
                l1.config(image=tkimage)
                menu.update()
                while(True):
                    if boolconfirmo == True:
                        print("{} salvo!".format(NomeImagem))
                        contador += 1
                        boolconfirmo = False
                        numerofotos.config(text="Numero de imagens: {0}".format(contador))
                        break
                    if boolrecuso == True:
                        os.remove(NomeImagem)
                        boolrecuso = False
                        break
                    boolfoto = 0 
                    menu.update()
        if boolconfirmo == True:
            break
    cv2.destroyWindow('fotos')
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
    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objetoReal, objetoImagem, tamanho, None, None)
    propriedades = [ret, cameraMatrix, dist, rvecs, tvecs]
    outfile = open('cameraconfig','wb')
    pickle.dump(propriedades,outfile)
    outfile.close
    return propriedades
    
def creattreco(frame):
    menu = Tk()
    geometria = "{0}x{1}".format(frame.shape[1]+150,frame.shape[0]+400)
    menu.geometry(geometria)
    menu.option_add("*font", "lucida 10 bold italic")
    menu.title('Menu')
    menu.config(bg = cor1)
    menu.iconbitmap('iconrasie.ico')
    
    
    botao1 = Button(menu,text='Foto',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command=boolfoto)
    botao1.place(x=375,y=10)

    botao2 = Button(menu,text='Confirmar',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command=boolconfirmo)
    botao2.place(x=375,y=50)

    botao3 = Button(menu,text='Apagar',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command=boolrecuso)
    botao3.place(x=375,y=90)

    numero = Label(bg = cor1,highlightbackground = cor1,activebackground = cor1, fg = cor2,text = "Numero de imagens: 0")
    numero.place(x=340,y=frame.shape[1]-100)
    imagem = Image.fromarray(frame)
    tkimage = ImageTk.PhotoImage(imagem)
    l1 = Label(menu, image=tkimage)
    l1.place(x = 70, y = 150)
    return menu, l1,numero

menu,l1,numerofotos = creattreco(frame)     
print(cali())

