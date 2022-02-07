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


boolc1 = False
boolc2 = False
boolc3 = False
pular = 0
vx = 0.0
vy = 0.0
infile = open('cameraconfig','rb')
propriedades = pickle.load(infile)
#print(propriedades)
infile.close
tamanhoTabuleiro = (12,7)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
cam = cv2.VideoCapture(0)
ret, frameinicial = cam.read()
boolfoto = False
boolconfirmo = False
boolrecuso = False

def boolCamera1():
    global boolc1
    boolc1 = not boolc1

def boolCamera2():
    global boolc2
    boolc2 = not boolc2

def boolfotoc():
    global boolfoto
    boolfoto = not boolfoto

def boolconfirmoc():
    global boolconfirmo
    boolconfirmo = not boolconfirmo

def boolrecusoc():
    global boolrecuso
    boolrecuso = not boolrecuso

def cali(menu2, l1,numero):
    global cam 
    cv2.namedWindow("fotos")
    contador = 0
    global boolfoto
    global boolrecuso
    global boolconfirmo

    while True:
        menu2.update()
        ret, frame = cam.read()
        salvo = frame
        if ret == False:
            print("problema na camera")
            break
        cv2.imshow("fotos", frame)
        print(boolfoto)
        tamanho = (frame.shape[0],frame.shape[1])
        #k = cv2.waitKey(1)
        #if k%256 == 27:
            # ESC pressed
            #break
        if boolfoto  == True:
            # SPACE pressed
            NomeImagem = "calibra{}.png".format(contador)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, vertice = cv2.findChessboardCorners(gray, tamanhoTabuleiro, None) # Find the chess board corners
            if ret==0:
                print('erro no tabuleiro')
            else:
                verticeNovo = cv2.cornerSubPix(gray, vertice, (11,11), (-1,-1), criteria)
                desenhos = frame
                cv2.imwrite(NomeImagem, salvo)
                cv2.drawChessboardCorners(desenhos, tamanhoTabuleiro, verticeNovo, ret) # Draw corners
                imagem = cv2.cvtColor(desenhos, cv2.COLOR_BGR2RGB) 
                imagem = Image.fromarray(imagem)
                tkimage = ImageTk.PhotoImage(imagem)
                l1.config(image=tkimage)
                menu2.update()
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
                    boolfoto = False
                    menu2.update()
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
    
def creattreco():
    global frameinicial
    global boolfoto
    global boolconfirmo
    global boolrecuso
    frame = frameinicial
    cor1 = '#404040' # cinza de fundo
    cor2 = '#ff7f2a' #
    cor3 = '#757575'
    menu2 = Toplevel(menu)
    geometria = "{0}x{1}".format(frame.shape[1]+150,frame.shape[0]+400)
    menu2.geometry(geometria)
    menu2.option_add("*font", "lucida 10 bold italic")
    menu2.title('c')
    menu2.config(bg = cor1)
    menu2.iconbitmap('iconrasie.ico')
    
    
    botao1 = Button(menu2,text='Foto',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command=boolfotoc)
    botao1.place(x=375,y=10)

    botao2 = Button(menu2,text='Confirmar',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command=boolconfirmoc)
    botao2.place(x=375,y=50)

    botao3 = Button(menu2,text='Apagar',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command=boolrecusoc)
    botao3.place(x=375,y=90)

    numero = Label(menu2,bg = cor1,highlightbackground = cor1,activebackground = cor1, fg = cor2,text = "Numero de imagens: 0")
    numero.place(x=340,y=frame.shape[1]-100)
    imagem = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    tkimage = ImageTk.PhotoImage(image=imagem, master=menu2)
    l1 = Label(menu2, image=tkimage)
    l1.place(x = 70, y = 150)
    menu2.update()
    
    cv2.namedWindow("fotos")
    contador = 0

    while True:
        menu2.update()
        ret, frame = cam.read()
        salvo = frame
        if ret == False:
            print("problema na camera")
            break
        cv2.imshow("fotos", frame)
        #print(boolfoto)
        tamanho = (frame.shape[0],frame.shape[1])
        #k = cv2.waitKey(1)
        #if k%256 == 27:
            # ESC pressed
            #break
        if boolfoto  == True:
            # SPACE pressed
            NomeImagem = "calibra{}.png".format(contador)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, vertice = cv2.findChessboardCorners(gray, tamanhoTabuleiro, None) # Find the chess board corners
            if ret==0:
                print('erro no tabuleiro')
                boolfoto = False
            else:
                verticeNovo = cv2.cornerSubPix(gray, vertice, (11,11), (-1,-1), criteria)
                desenhos = frame
                cv2.imwrite(NomeImagem, salvo)
                cv2.drawChessboardCorners(desenhos, tamanhoTabuleiro, verticeNovo, ret) # Draw corners
                imagem = cv2.cvtColor(desenhos, cv2.COLOR_BGR2RGB) 
                imagem = Image.fromarray(imagem)
                tkimage = ImageTk.PhotoImage(imagem, master= menu2)
                l1.config(image=tkimage)
                menu2.update()
                while(True):
                    if boolconfirmo == True:
                        print("{} salvo!".format(NomeImagem))
                        contador += 1
                        boolconfirmo = False
                        numero.config(text="Numero de imagens: {0}".format(contador))
                        break
                    if boolrecuso == True:
                        os.remove(NomeImagem)
                        boolrecuso = False
                        break
                    boolfoto = False
                    menu2.update()
        if boolconfirmo == True:
            break
    cv2.destroyWindow('fotos')
    menu2.destroy()
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

    return menu2, l1,numero

        
def boolCamera3():
    global boolc3
    boolc3 = not boolc3

def creatMenu():

    global boolC1
    global boolfoto
    cor1 = '#404040' # cinza de fundo
    cor2 = '#ff7f2a' #
    cor3 = '#757575'
    
    menu = Tk("menu")
    menu.option_add("*font", "lucida 10 bold italic")
    menu.title('Menu')
    menu.geometry('860x440')
    menu.config(bg = cor1)
    menu.iconbitmap('iconrasie.ico')


    moldura1 = LabelFrame(menu, text='Filtro De Cor',padx=1,pady=1,bg =cor1,fg = cor2, highlightbackground=cor2, highlightcolor=cor2)
    moldura1.place(x=20,y=20,height=400,width=400)

    moldura2 = LabelFrame(menu, text='PID',padx=1,pady=1,bg =cor1,fg = cor2, highlightbackground=cor2, highlightcolor=cor2)
    moldura2.place(x=440,y=20,height=220,width=400)

    moldura3 = LabelFrame(menu, text='botões',padx=1,pady=1,bg =cor1,fg = cor2, highlightbackground=cor2, highlightcolor=cor2)
    moldura3.place(x=440,y=260,height= 75,width=400)

    moldura4 = LabelFrame(menu, text='Dados',padx=1,pady=1,bg =cor1,fg = cor2, highlightbackground=cor2, highlightcolor=cor2)
    moldura4.place(x=440,y=355,height= 65,width=400)

    #MOLDURA 1: FILTRO COR
    l1 = Label(moldura1,text= 'Hue',bg =cor1,fg = cor2)
    l1.pack()
    hMin = Scale(moldura1,from_=0,to=179,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    hMin.pack()
    hMax = Scale(moldura1,from_=0,to=179,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    hMax.pack()
    hMax.set(179)
    l2 = Label(moldura1,text= '',bg =cor1,fg = cor2)
    l2.pack()

    l3 = Label(moldura1,text= 'Saturation',bg =cor1,fg = cor2)
    l3.pack()
    sMin = Scale(moldura1,from_=0,to=255,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    sMin.pack()
    sMax = Scale(moldura1,from_=0,to=255,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    sMax.pack()
    sMax.set(255)
    l3 = Label(moldura1,text= '',bg =cor1,fg = cor2)
    l3.pack()

    l4 = Label(moldura1,text= 'Value',bg =cor1,fg = cor2)
    l4.pack()
    vMin = Scale(moldura1,from_=0,to=255,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    vMin.pack()
    vMax = Scale(moldura1,from_=0,to=255,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    vMax.pack()
    vMax.set(255)
    l5 = Label(moldura1,text= '',bg =cor1,fg = cor2)
    l5.pack()
    
    hue = [hMin,hMax]
    saturation = [sMin,sMax]
    value = [vMin,vMax]

    #MOLDURA 2: PID
    l6 = Label(moldura2,text= 'Proportional',bg =cor1,fg = cor2)
    l6.pack()
    p = Scale(moldura2,from_=0,to=179,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    p.pack()

    l7 = Label(moldura2,text= 'Integrated',bg =cor1,fg = cor2)
    l7.pack()
    i = Scale(moldura2,from_=0,to=179,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    i.pack()

    l8 = Label(moldura2,text= 'Derivative',bg =cor1,fg = cor2)
    l8.pack()
    d = Scale(moldura2,from_=0,to=179,length=400,orient=HORIZONTAL,bg = cor1,highlightbackground = cor1,troughcolor = cor3,	
    activebackground = cor1, fg = cor2)
    d.pack()


    #MOLDURA 3: BOTOES
    botao1 = Button(moldura3,text='Camera',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command = boolCamera1)
    botao1.place(x=10,y=10)

    botao2 = Button(moldura3,text='Mascara',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command = boolCamera2)
    botao2.place(x=90,y=10)

    botao3 = Button(moldura3,text='Calibragem',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command = creattreco)
    botao3.place(x=175,y=10)
    

    #MOLDURA 4: DADOS
    Px = Label(moldura4,bg = cor1,highlightbackground = cor1,	
    activebackground = cor1, fg = cor2, text=0)
    Px.place(x=10,y=0)
    Py = Label(moldura4,bg = cor1,highlightbackground = cor1,	
    activebackground = cor1, fg = cor2, text=0)
    Py.place(x=100,y=0)
    vx = Label(moldura4,bg = cor1,highlightbackground = cor1,	
    activebackground = cor1, fg = cor2, text=0)
    vx.place(x=10,y=29)
    vy = Label(moldura4,bg = cor1,highlightbackground = cor1,	
    activebackground = cor1, fg = cor2, text=0)
    vy.place(x=100,y=29)
    dados = [Px,Py,vx,vy]

    #FIM

    return menu,hue,saturation,value,dados



menu, hue,saturation,value,dados = creatMenu()
while(True):

    ret, frame = cam.read() 
    tempo = time.time()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    
    minimo, maximo = zxc.getTrackBarHSV(hue,saturation,value)

    filtrado = cv2.inRange(hsv, minimo, maximo)
    filtrado = cv2.erode(filtrado, None, iterations=2) 
    filtrado = cv2.dilate(filtrado, None, iterations=2)


    #converter função
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
    #print([vx,vy])
    #ate aki
    if(boolc1):
        cv2.imshow('camera',frame)
    else:
        try:
            cv2.destroyWindow('camera')
        except:
            pass

    if(boolc2):
        cv2.imshow('mascara',filtrado)
    else:
        try:
            cv2.destroyWindow('mascara')
        except:
            pass
    dados[0].config(text="Px = {0}".format(centroX))
    dados[1].config(text="Py = {0}".format(centroY))
    dados[2].config(text="Vx = {0}".format(round(vx,1)))
    dados[3].config(text="Vy = {0}".format(round(vy,1)))
    menu.update()
    
