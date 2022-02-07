from tkinter import *
import cv2
import Bibliotecaresumida as zxc
import pickle
import time
import numpy as np
import imutils
def boolCamera1():
    global boolc1
    if(boolc1):
        boolc1 = False
    else:
        boolc1 = True

def boolCamera2():
    global boolc2
    if(boolc2):
        boolc2 = False
    else:
        boolc2 = True

# def boolCamera2():
#     menu2 = creattreco()
#     while(True):
#         menu2.update()
        
def boolCamera3():
    #global prop
    zxc.takePictures()
    propriedades = zxc.calibrateCam()
def creattreco():

    menu2 = Tk()
    w = Label(menu2, text=vx)
    w.pack()
    return menu2
def creatMenu():

    global boolC1
    cor1 = '#404040' # cinza de fundo
    cor2 = '#ff7f2a' #
    cor3 = '#757575'
    
    menu = Tk()
    menu.option_add("*font", "lucida 10 bold italic")
    menu.title('Menu')
    menu.geometry('860x440')
    menu.config(bg = cor1)
    menu.iconbitmap('iconrasie.ico')

    w = Label(menu, text=vx)
    w.pack()

    moldura1 = LabelFrame(menu, text='Filtro De Cor',padx=1,pady=1,bg =cor1,fg = cor2, highlightbackground=cor2, highlightcolor=cor2)
    moldura1.place(x=20,y=20,height=400,width=400)

    moldura2 = LabelFrame(menu, text='PID',padx=1,pady=1,bg =cor1,fg = cor2, highlightbackground=cor2, highlightcolor=cor2)
    moldura2.place(x=440,y=20,height=220,width=400)

    moldura3 = LabelFrame(menu, text='botões',padx=1,pady=1,bg =cor1,fg = cor2, highlightbackground=cor2, highlightcolor=cor2)
    moldura3.place(x=440,y=260,height= 160,width=400)

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
    botao1.place(x=10,y=50)

    botao2 = Button(moldura3,text='Mascara',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command = boolCamera2)
    botao2.place(x=90,y=50)

    botao3 = Button(moldura3,text='Calibragem',bg =cor1,fg = cor2,activebackground = cor1,activeforeground = cor2,command = boolCamera3)
    botao3.place(x=175,y=50)
    
    #FIM

    return menu,hue,saturation,value

boolc1 = False
boolc2 = False
boolc3 = False
pular = 0
vx = 0.0
vy = 0.0
infile = open('cameraconfig','rb')
propriedades = pickle.load(infile)
print(propriedades)
infile.close


menu, hue,saturation,value  = creatMenu()
img = cv2.VideoCapture(0)
while(True):
    ret, frame = img.read()   
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
    print([vx,vy])
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
    #if(boolc1 == False and boolc2 == False):
        #cv2.destroyAllWindows()
    
    menu.update()
    
