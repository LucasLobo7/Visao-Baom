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
import threading

cam = cv2.VideoCapture(0)
ret, frame = cam.read()

def main():
    global frame
    menu = Tk()
    menu.config(bg = '#404040')
    menu.update()
    imagem = Image.fromarray(frame)
    tkimage = ImageTk.PhotoImage(imagem)
    l1 = Label(menu, image=tkimage)
    l1.place(x = 70, y = 150)
    botao1 = Button(menu,text='Foto',command=boolfoto)
    botao1.place(x=375,y=10)
    return menu,l1
def boolfoto():
    pass

def thread_fun(menu,l1):
    while(True):
        ret, frameteste = cam.read() 
        imagem = cv2.cvtColor(frameteste, cv2.COLOR_BGR2RGB)  
        imagem = Image.fromarray(imagem)
        tkimage = ImageTk.PhotoImage(imagem)
        l1.config(image=tkimage)
        menu.update()
        


menu,l1 = main()
threading.Thread(target=thread_fun(menu,l1)).start()
while(True):
    
    menu.update()
