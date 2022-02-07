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
import multiprocessing

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
    canva = Canvas()
    canva.place(x = 70, y = 600)
    canva.create_image(70,600,anchor='n', image=tkimage)
    canva.update()
    return menu,l1

def boolfoto():
    global boolc1
    boolc1 = False

def thread_fun(menu,l1,boolc1):
    while(boolc1):
        ret, frameteste = cam.read() 
        imagem = cv2.cvtColor(frameteste, cv2.COLOR_BGR2RGB)  
        imagem = Image.fromarray(imagem)
        tkimage = ImageTk.PhotoImage(imagem)
        l1.config(image=tkimage)
        menu.update()

def thread_fun2(menu,l2):
    while(True):
        ret, frameteste = cam.read() 
        imagem = Image.fromarray(frameteste)
        tkimage = ImageTk.PhotoImage(imagem)
        l2.config(image=tkimage)
        menu.update()        


menu,l1 = main()

while(True):
    ret, frame = cam.read()
    img_update = ImageTk.PhotoImage(Image.fromarray(frame))
    l1.configure(image=img_update)
    #l1.image=img_update
    #l1.update()
    menu.update()
