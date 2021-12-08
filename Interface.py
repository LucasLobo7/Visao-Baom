from tkinter import *
import cv2
boolc1 = False 

def boolCamera1():
    global boolc1
    if(boolc1):
        boolc1 = False
    else:
        boolc1 = True



def creatMenu():

    global boolC1
    cor1 = '#404040' # cinza de fundo
    cor2 = '#ff7f2a' #
    cor3 = '#757575'
    
    menu = Tk()
    menu.option_add("*font", "lucida 10 bold italic")
    menu.title('Home')
    menu.geometry('860x440')
    menu.config(bg = cor1)
    #menu.iconbitmap('iconrasie.ico')
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
    
    #FIM

    return menu,hue,saturation,value,



menu, x,y,v  = creatMenu()
img = cv2.VideoCapture(0)
while(True):
    
    if(boolc1):
        ret, frame = img.read()
        cv2.imshow('teste',frame)
    else:
        cv2.destroyAllWindows()
    
    menu.update()
