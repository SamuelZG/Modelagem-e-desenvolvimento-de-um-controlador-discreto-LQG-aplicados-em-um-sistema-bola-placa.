#bibliotecas

from ast import While
from lib2to3.pytree import convert
from pyexpat import model
from tokenize import Double
from turtle import delay
from imutils.video import VideoStream
import cv2
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
import threading
import math
from math import *
import argparse
import imutils
import os
#import tensorflow as tf
import multiprocessing
from multiprocessing import Process
from simple_pid import PID
import pandas as pd

cv2.cuda.getCudaEnabledDeviceCount()

ixx=0
iyy=0
#não linearidade eixo x
dataDictx = {}
lines = open("servoX1.txt").read().splitlines()  # Read the file data.txt and split it by lines and put it in a list
max_alphaX, max_thetaX = lines[1].split("|")
max_alphaX = - float(max_alphaX)
max_thetaX = float(max_thetaX)

for i in range(1, len(lines)):  # for loop to go over the lines list and feed it to the dictionary
    alphaX, thetaX = lines[i].split("|")
    dataDictx[float(alphaX)] = float(thetaX)



#não linearidade eixo y
dataDicty = {}
lines = open("servoY1.txt").read().splitlines()  # Read the file data.txt and split it by lines and put it in a list
max_alphaY, max_thetaY = lines[1].split("|")
max_alphaY = - float(max_alphaY)
max_thetaY = float(max_thetaY)

for i in range(1, len(lines)):  # for loop to go over the lines list and feed it to the dictionary
    alphaY, thetaY = lines[i].split("|")
    dataDicty[float(alphaY)] = float(thetaY)


#cor Bola

BLower = (10, 10, 10)   
BUpper = (75, 115, 115)
BLower = np.array([10, 10, 10], dtype="uint8")
BUpper = np.array([75, 115, 115], dtype="uint8")
#BLower = (50, 40, 40)   
#BUpper = (75, 115, 115)

#faixa de cor da mesa
#GLower = (65, 70, 150)
#GUpper = (75, 110, 180)

#GLower = (70, 110, 110)
#GUpper = (85, 180, 225)

GLower = (60, 130, 160)
GUpper = (70, 180, 230)

BLower = np.array([5, 5, 5], dtype="uint8")
BUpper = np.array([75, 115, 115], dtype="uint8")

GLower = np.array([50, 100, 160], dtype="uint8")
GUpper = np.array([65, 160, 210], dtype="uint8")

#dados do video
cv2.useOptimized()
cv2.setUseOptimized(True) 
cap=cv2.VideoCapture(2) #, cv2.CAP_DSHOW
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

#print de vetores
anguloteste=[]
posicaoteste=[]
angulotestey=[]
posicaotestey=[]
tempoo=[]
setramp=[]
ramp=0
setx = []
sety = []
pos = 0
posy = 0

#serial
USB = serial.Serial("com4",38400)

#variaveis geral
fps = 0.066666666666667
angulomax=math.radians(6)
angulomin=math.radians(1)
conv=(angulomax/0.25)
servo11 = 90 #X
servo21 = 90 #Y
center=None
mesa=25
spx=0
spy=0
posicaox=0.0
posicaoy=0.0
frame= None
x= 0
y=0
x1=0
y1=0
radius= 0
cnts= 0
i=0
cpi=0
cx=[]
ix=0
ix1=0
angulomotorx=0
angulomotory=0
posicaox1=0
posicaoy1=0



#variaveis grafico
vetorposicaox=[]
vetorposicaoy=[]
vetorsetpointx=[]
vetorsetpointy=[]
tempo=[]
k=0
xmin1=0
xmax1=10
marcador= 0.5
tempoatrasado=time.time()
k1=0
k=0
k2=20

## Variaveis controle

#xb = center
xbx = 0
vbx = 0
amx = 0
vax = 0
disx = 0 
u0x = 0
y0x = 0
ux = 0

xby = 0    
vby = 0
amy = 0
vay = 0
disy = 0 
u0y = 0
y0y = 0
uy = 0

posicoteste1 = []
posicotestey1 = []
    
    
#################### Espaço estado
def espaco_estado():
    global G1, G2, G3, G4, B1, B2, B3, B4, B5, xbx, vbx, amx, vax, disx, u0x, y0x, xby, vby, amy, vay, disy, u0y, y0y, zO1x, zO2x, zO3x, zO4x, zux, zy0x ,zO1y, zO2y, zO3y, zO4y, zuy, zy0y
    global A11, A12, A13, A14, A15, A21, A22, A23, A24, A25, A31, A32, A33, A34, A35, A41, A42, A43, A44, A45, A51, A52, A53, A54, A55, A, B, C, G, C1, C2, C3, C4, Ox, Oy, K1, K2, K3, K4, K, posicoteste1, posicotestey1

    A11 = 1
    A12 = 0.065076645935133
    A13 = -0.013896261599352
    A14 = -1.297509799283182e-04
    
    A21 = 0
    A22 = 0.952681651800801
    A23 = -0.384378813804298
    A24 = -0.004323234299715

    A31 = 0
    A32 = 0
    A33 = 0.651997488224296
    A34 = 0.009783457615355

    A41 = 0
    A42 = 0
    A43 = -5.400468604162340
    A44 = -0.069826014676204

    B1 = -0.00142658112917340
    B2 = -0.0716225409268854
    B3 = 0.348002511775704
    B4 = 5.40046860416234

    #C1 = [1, 0, 0, 0, 0]
    
    C1 = 1
    C2 = 0
    C3 = 0
    C4 = 0

    D1 = 0

    G1 = 0.549027284179266
    G2 = 3.21427901071943
    G3 = -0.974372624675991
    G4 = 7.05536842452666
    
    K1 = -0.765517875921543
    K2 = -0.441647691436181
    K3 = 0.265815947632561
    K4 = 0.004574912328557
    
    A = np.array([[A11, A12, A13, A14], [A21, A22, A23, A24], [A31, A32, A33, A34], [A41, A42, A43, A44]])
    B = np.array([[B1], [B2], [B3], [B4]])
    C = np.array([C1, C2, C3, C4])
    #print(A)
    #print(B)
    #print(C)
    G = np.array([[G1], [G2], [G3], [G4]])
    #print(G)
    K = np.array([K1, K2, K3, K4])
    #print(K)
    
    Ox = np.array([[xbx], [vax], [amx], [vax]])
    Oy = np.array([[xby], [vay], [amy], [vay]])
    
    
    zO1x = []
    zO2x = []
    zO3x = []
    zO4x = []
    zux = []
    zy0x = []
    
    zO1y = []
    zO2y = []
    zO3y = []
    zO4y = []
    zuy = []
    zy0y = []
    

    #print('###############################################################################################################################')


df = pd.DataFrame()
    
espaco_estado()

#Vetor Circulo
aj=0
ajy=0
velocidade=4
raio=6
pointsListCirclex = []
pointsListCircley = []
for angle in range(0, 360):
        angle = angle - 90
        pointsListCirclex.append(raio * cos(radians(angle)) + 0)
        pointsListCircley.append(raio * sin(radians(angle)) + 0)

#Vetor Quadrado
tamanho=12
pointsListQuadradox = []
pointsListQuadradoy = []
for qx1 in range (0,tamanho*5):
        pointsListQuadradox.append((0-tamanho/2)+(qx1/5))
        pointsListQuadradoy.append(0-tamanho/2)
for q1 in range (0, tamanho*5):
    pointsListQuadradox.append((0+tamanho/2))
    pointsListQuadradoy.append(0-tamanho/2)


for qy1 in range (0,tamanho*5):
        pointsListQuadradoy.append((0-tamanho/2)+(qy1/5))
        pointsListQuadradox.append(0+tamanho/2)
for qx0 in range (0, tamanho*5):
    pointsListQuadradox.append((0+tamanho/2))
    pointsListQuadradoy.append(0+tamanho/2)

    
for qx2 in range (0,tamanho*5):
        pointsListQuadradox.append(0+tamanho/2-(qx2/5))
        pointsListQuadradoy.append(0+tamanho/2)
for q3 in range (0, tamanho*5):
    pointsListQuadradox.append((0-tamanho/2))
    pointsListQuadradoy.append(0+tamanho/2)


for qy2 in range (0,tamanho*5):
        pointsListQuadradoy.append(0+tamanho/2-(qy2/5))
        pointsListQuadradox.append(0-tamanho/2)
for q4 in range (0, tamanho*5):
    pointsListQuadradox.append((0-tamanho/2))
    pointsListQuadradoy.append(0-tamanho/2)


    
#Ajuste de posição da camera
while 1:
     
    ret, frame = cap.read()
    #frame= cv2.GaussianBlur(cap2, (1, 1), 0)
        
    #frame = cv2.erode(frame, None, iterations=2)
    #frame = cv2.dilate(frame, None, iterations=2)
    cap2=frame
    cv2.line(frame, (130,0),(130,720), (0, 0, 255), 2)
    cv2.putText(frame,"Ajuste a lateral da mesa a linha vermelha",
        (135,55), cv2.FONT_ITALIC,0.6,(255,255,255), 1)
    cv2.putText(frame,"em seguida precione 'ESC'",
        (135,75), cv2.FONT_ITALIC,0.6,(255,255,255), 1)
    cv2.imshow("Ajuste de posicao da camera", frame)

    if cv2.waitKey(1) & 0xff == 27 :
        break
cv2.destroyAllWindows()
cv2.waitKey (99)


print(1)
#detecta mesa
while 1:
    #global w, h
    print(2)
    ret, frame = cap.read()
    #frame = frame[128 : 588, 398 : 862]
    frame = frame[150 : 620, 395 : 865]
     
    #upscaled = sr.upsample(cap3)
    #frame = cv2.resize(cap3,	(upscaled.shape[1], upscaled.shape[0]),	interpolation=cv2.INTER_CUBIC)
    
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GLower, GUpper)
    
    mask= cv2.GaussianBlur(mask, (15, 15), 0)
    mask = cv2.medianBlur(mask,9)
    
    
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    
    result = cv2.bitwise_and(frame, frame, mask = mask)
    mask1 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    _,mask1 = cv2.threshold(mask1, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
  
    cnts,hierarchy = cv2.findContours(mask1.copy(),cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)

    #time.sleep(1)
    
    if len(cnts) > 0:
        d = max(cnts, key=cv2.contourArea)
        x1, y1, w, h = cv2.boundingRect(d)
        
        cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (237, 28, 36), 2)
        cv2.putText(frame,"PRONTO!",
            (180,250), cv2.FONT_ITALIC,2,(255,255,255), 2)
        cv2.imshow('detecção da mesa', frame)
        print(2.5)
        
    if cv2.waitKey(1) & 0xff == 27 :   
    
        break
    
cv2.destroyAllWindows()
cv2.waitKey (99)

print(3)

#janela principal do app
app=tk.Tk()
app.title("Inicio")
app.w, app.h = app.winfo_screenwidth(), app.winfo_screenheight()
#app.geometry("%dx%d" % (app.w, app.h))
app.geometry("500x500")
app.configure(bg='white')

#configurações iniciais da janela
MostrarCamera = False
b1Texto = tk.StringVar()
b1Texto.set("Abrir Camera")

Plot = False
b2Texto = tk.StringVar()
b2Texto.set("Plotar Grafico")

Degrau = False
b4Texto = tk.StringVar()
b4Texto.set("Sem controle")

salvar = False
b8Texto = tk.StringVar()
b8Texto.set("Salvar")

Circulo = False
b6Texto = tk.StringVar()
b6Texto.set("Circulo")

avanco = False
b7Texto = tk.StringVar()
b7Texto.set("Controle")

Quadrado = False
b10Texto = tk.StringVar()
b10Texto.set("Quadrado")

#configura janelas
def cam():
    global MostrarCamera

    if MostrarCamera == False:
        MostrarCamera = True
        b1Texto.set("Fechar Camera")
        
    else:
        MostrarCamera = False
        b1Texto.set("Abrir Camera")



#configura setpoint
def SetPoint():
    global spx,spy
    global x00, x01, xresposta, y00,y01,yresposta,x02,y02
    global fx00, fx01, fxresposta, fy00,fy01,fyresposta
    spx=xtext.get("1.0","end-1c")
    spy=ytext.get("1.0","end-1c")
    x00=0
    x01=0
    xresposta=0
    y00=0
    y01=0
    yresposta=0
    x02=0
    y02=0
    fx00=0
    fx01=0
    fxresposta=0
    fy00=0
    fy01=0
    fyresposta=0
    spx=float(spx)
    spy=float(spy)
    '''dipx=PID.setpoint(spx/100)
    dipy=PID.setpoint = (spy/100)'''
    SPT.configure(text=("SetPoint: ("+ str (spx)+","+ str(spy)+")"),  fg="#4682B4")

#Degrau sem controlador liga/desliga
def IniciaDegrau():
    global Degrau, Plot
    if Degrau == False and avanco == False and salvar == False:
        Degrau = True
        b4Texto.set("Parar")
        
        
    else:
        Degrau = False
        b4Texto.set("Sem controle")

def Salvar():
    global salvar
    if avanco == False and Degrau == False and salvar == False:
        salvar = True
        b8Texto.set("Salvando")
        
        
    else:
        salvar = False
        b8Texto.set("Salvar")

def IniciaCirculo():
    global Circulo
    if Circulo==False:
        ix=0
        iy=52
        Circulo = True
        b6Texto.set("Parar")
        
    else:
        ix=0
        iy=52
        Circulo = False
        b6Texto.set("Circulo")

def IniciaQuadrado():
    global Quadrado
    if Quadrado==False:
        ix1=0
        Quadrado = True
        b10Texto.set("Parar")
        
    else:
        ix1=0
        Quadrado = False
        b10Texto.set("Circulo")

#grafico Liga/deliga
def PlotGrafico():

    global Plot,temporeal

    if Plot == False:
        k1=0
        k=0
        k2=10
        b2Texto.set("Fechar Grafico")
        tempoatrasado=time.time()
        Plot = True
        
        
    else:
        b2Texto.set("Plotar Grafico")
        Plot = False
 
def Avanco():
    global avanco,zeradox, zeradoy
    if avanco == False and Degrau == False and salvar == False:
        avanco = True
        b7Texto.set("Parar")
        zeradox=posicaox
        zeradoy=posicaoy
        
    else:
        avanco = False
        b7Texto.set("Controle")
   

    


def controle():
    global posicaox,posicaoy, i, angulomax, conv,spx,spy
    global servo11,servo21
    global aj,ajy,ix,ix1
    global anguloteste, posicaoteste, ramp, salvar ,setramp, avanco
    global x00, x01, xresposta, y00,y01,yresposta,x02,y02
    global fx00, fx01, fxresposta, fy00,fy01,fyresposta,zeradox,zeradoy
    global ixx,iyy
    global ux, uy, df,t0, posicaoteste, posicaotestey, y0x,y0y, Ox, Oy

    while 1:
        #t0=time.time()          
        t0 = time.perf_counter()
        #################### camera
        (ret, cap2) = cap.read()

        cap2 = cap2[150 : 620, 395 : 865]
            
        #cap3 = imutils.resize(cap2, width=800)
            
        frame = cv2.GaussianBlur(cap2, (9, 9), 0)
        frame = cv2.medianBlur(frame,7)
            
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
        mask = cv2.inRange(hsv, BLower, BUpper)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
                
        result = cv2.bitwise_and(frame, frame, mask = mask)
        mask1 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        _,mask1 = cv2.threshold(mask1, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        cnts = cv2.findContours(mask1.copy(),cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)[-2]
        #print('3')         
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            
            M = cv2.moments(c)
            
            if M["m00"]>0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                i=1

                #print('4')

                if radius > 10:
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                        (237, 28, 36), 2)
                
                    #posicao pixel p/ centimetro
                
                    posicaox = round((center[0]- w/2)/(w/mesa),6)
                    posicaoy = round(-(center[1]+((-y1-h)/2))/(h/mesa),6)
                
            pos=posicaox/100
            posy=posicaoy/100
            
            y0x = pos
            y0y = posy
            #print(pos,posy)
            
            posicao.configure(text=("Posição da bola: "+ str (round(posicaox,3))+ ", "+ str (round(posicaoy,3))), fg="#36db8b")

            posicoteste1.append(pos)
            posicotestey1.append(posy)

            ################################################################ controle
            if Circulo==True:
                if ix<360:
                        spx=pointsListCirclex[ix]
                        spy=pointsListCircley[ix]
                        ix=ix+1+velocidade*2
                        aj=0
                        ajy=3
                        

                else:
                        ix=0

            if Quadrado==True:
                if ix1<((tamanho*5*4)+ (tamanho*4*5)):
                        spx=pointsListQuadradox[ix1]
                        spy=pointsListQuadradoy[ix1]
                        ix1=ix1+1+velocidade
                        aj=-1
                        ajy=4

                
                else:
                    ix1=0

            if Quadrado==False and Circulo==False:
                aj=0;
                ajy=0

                
            if avanco == True and i==1:
                #print('controle')
                Ox = np.dot(A, Ox) + ((B - (G * np.dot(C, B))) * ux) + ((G * y0x) - (G * np.dot(C , np.dot(A, Ox))))
                ux = (-spx/100) + ((-K1 * ( Ox[0])) + (-K2 * Ox[1]) + (-K3 * Ox[2]) + (-K4 * Ox[3]))
                
                Oy = np.dot(A, Oy) + ((B - (G * np.dot(C, B))) * uy) + ((G * y0y) - (G * np.dot(C , np.dot(A, Oy))))
                uy = (-spy/100) + ((-K1 * ( Oy[0])) + (-K2 * Oy[1]) + (-K3 * Oy[2]) + (-K4 * Oy[3]))

                if (ux)>=angulomax:
                    ux=angulomax
                if (ux)<=-angulomax:
                    ux=-angulomax
                    
                if (uy)>=angulomax:
                    uy=angulomax
                if (uy)<=-angulomax:
                    uy=-angulomax

                zy0x.append(y0x)
                zO1x.append(Ox[0])
                zO2x.append(Ox[1])
                zO3x.append(Ox[2])
                zO4x.append(Ox[3])
                zux.append(ux)
                
                zy0y.append(y0y)
                zO1y.append(Oy[0])
                zO2y.append(Oy[1])
                zO3y.append(Oy[2])
                zO4y.append(Oy[3])
                zuy.append(uy)
                
                """####################minimo
                if ukx > 0:
                    if (ukx)<=angulomin:
                        ukx=angulomin
                        
                    elif (ukx)>=angulomax:
                        ukx=angulomax
                ####################maximo
                elif ukx < 0:
                    if (ukx)>=-angulomin:
                        ukx=-angulomin
                    
                    elif (ukx)<=-angulomax:
                        ukx=-angulomax    
                        
                ####################minimo
                if uky > 0:
                    if (uky)<=angulomin:
                        uky=angulomin
                        
                    elif (uky)>=angulomax:
                        uky=angulomax
                ####################maximo
                elif uky < 0:
                    if (uky)>=-angulomin:
                        uky=-angulomin
                    
                    elif (uky)<=-angulomax:
                        uky=-angulomax"""                            
                
                uxx=round((math.degrees(ux)),1)
                uyy=round((math.degrees(uy)),1)

                """angulomesax=(ux)
                angulomotorx=(math.asin((10/1.74)*math.sin(angulomesax)))
                angulomotorx=(math.degrees(angulomotorx))
                angulomotorx=round((servo11 + angulomotorx),1)
                #print('angulo motor x:', uxx)
                
                angulomesay=(uy)
                angulomotory=(math.asin((10/1.74)*math.sin(angulomesay)))
                angulomotory=(math.degrees(angulomotory))
                angulomotory=round((servo21 + angulomotory),1)"""
                #angulomotory = 90'''
                #print('angulo motor y:', uyy)


                #testes
                #teste de saidas
                anguloteste.append(ux)
                posicaoteste.append(pos)
                angulotestey.append(uy)
                posicaotestey.append(posy)
                tempoo.append((time.perf_counter_ns()))
                setx.append(spx)
                sety.append(spy)
                
                USB.write(((str(dataDictx[uxx]) + ":" + str(dataDicty[-uyy]) + "$").encode()))
                
            if avanco == False:
                x00=0
                x01=0
                xresposta=0
                y00=0
                y01=0
                yresposta=0
                x02=0
                y02=0
                fx00=0
                fx01=0
                fxresposta=0
                fy00=0
                fy01=0
                fyresposta=0

            #Degrau sem controlador
            if Degrau == True and i==1:
                #################################################################testeX
                angulomesax=(pos)
                angulomotor=(math.asin((10/1.74)*math.sin(angulomesax)))
                angulomotor=(math.degrees(angulomotor))
                angulomotor=round((servo11 + angulomotor),1)
                
                angulomesay=(posy)
                angulomotor=(math.asin((10/1.74)*math.sin(angulomesay)))
                angulomotor=(math.degrees(angulomotor))
                angulomotor=round((servo21 + angulomotor),1)
                
                print("angulo do motor:",angulomotor)
                
                y0x = pos
                y0y = posy
                
                zy0x.append(y0x)
                zy0y.append(y0y)
                
                tempoo.append((time.perf_counter_ns()))
                
                setx.append(spx)
                sety.append(spy)
                
                ####################minimo
                #if angulomesax > 0:
                    #if (angulomesax)<=angulomin:
                        #angulomesax=angulomin
                        
                    #elif (angulomesax)>=angulomax:
                        #angulomesax=angulomax
                
                
                ####################maximo
                #elif angulomesax < 0:
                    #if (angulomesax)>=-angulomin:
                        #angulomesax=-angulomin
                    
                    #elif (angulomesax)<=-angulomax:
                        #angulomesax=-angulomax
                
                #print("angulo do mesa:",angulomesax)
                ixx=round((math.degrees(angulomesax*conv)),1)
                #print("ixx:",ixx)
                #print("angulo do mesa:",angulomesax)
                iyy=round((math.degrees(angulomesay*conv)),1)
                #print("ixx:",iyy)
                angulomax = 6
                if (ixx)>=angulomax:
                    ixx=angulomax
                    
                if (ixx)<=-angulomax:
                    ixx=-angulomax
                    
                if (iyy)>=angulomax:
                    iyy=angulomax
                    
                if (iyy)<=-angulomax:
                    iyy=-angulomax
                    
                #teste de saidas
                anguloteste.append(angulomesax)
                posicaoteste.append(pos)

                USB.write(((str(dataDictx[ixx]) + ":" + str(dataDicty[-iyy]) + "$").encode()))

                #################################################################testeY

                ####################minimo
                #if angulomesax > 0:
                    #if (angulomesax)<=angulomin:
                        #angulomesax=angulomin
                        
                    #elif (angulomesax)>=angulomax:
                        #angulomesax=angulomax
                
                
                ####################maximo
                #elif angulomesax < 0:
                    #if (angulomesax)>=-angulomin:
                        #angulomesax=-angulomin
                    
                    #elif (angulomesax)<=-angulomax:
                        #angulomesax=-angulomax
                
                print("angulo do mesa:",angulomesay)
                iyy=round((math.degrees(angulomesay)),1)
                print("iyy:",iyy)
                
                #teste de saidas
                anguloteste.append(angulomesay)
                posicaoteste.append(pos)
                tempoo.append(round(time.perf_counter_ns(),5))



            if salvar==True and i==1:

                dados(df)
                
                print('finalizado')
                
                
                return
                
        else:
            posicao.configure(text="Posição da bola: (0,0)", fg="red")
            i=0
            #print('oi')
            
        #setpont centimetro p/ pixel
        xx= ((w/mesa)* float(spx)) + w/2
        yy= -((h/mesa)* float(spy)) + (y1+h)/2

        cv2.circle(frame, (int(xx), int(yy)), int(1),
            (0, 0, 255), 8)
        if MostrarCamera == True:
            cv2.imshow('video',frame)
            
        else:
            cv2.destroyAllWindows()

        if cv2.waitKey(1) & 0xff == 27:
            
            break 
        
        """while((time.time()-t0) <= (fps)):
            print(time.time()-t0)""" 
            
        while((time.perf_counter()-t0) <= (fps)):
            print(time.perf_counter()-t0)     
            
            
        """if (((fps-time.perf_counter())/1e9) <= fps):
            time.sleep(((fps-(time.perf_counter()/1e9)) - (t0/1e9)))"""  
                              



                
  
def dados(df):
    global zO1x, zO2x, zO3x, zO4x,t0,posicoteste1, posicotestey1
    dict = {'xbx': zO1x, 'vx': zO2x, 'ax': zO3x, 'vax': zO4x, 'ux': zux, 'posx': zy0x, 'setx': setx, 'xby': zO1y, 'vy': zO2y, 'ay': zO3y, 'vay': zO4y, 'uy': zuy, 'posy': zy0y, 'sety': sety, 'tr':tempoo}
    print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
    print('PosX:', posicoteste1)
    
    print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
    print('PosY:', posicotestey1)
    
    df = pd.DataFrame(dict)

    df.to_csv('C:/Users/samue/Desktop/TCC/Codigos/dados/dados.csv', sep=';', encoding ='utf-8', header=True)
    print(df)
    return df
    
    
    
            

def Grafico():
    global k, k2, i, Plot,temporeal,tempoatrasado,marcador,pocicaox,spx,tempo
    global vetorposicaox,xmin1,xmax1,vetorsetpointx,k1

    if Plot == True:
        b2Texto.set("Fechar Grafico") 
        if i==1:
            '''temporeal=time.time()'''
            plt.ion()
            

            '''plt.subplot(2,1,1)
            plt.plot(tempo ,vetorposicaox,color='blue',label='Posição em x',linewidth=1.0)
            plt.plot(tempo, vetorsetpointx, color='red',label='Set point em x',linewidth=1.0,linestyle='--')
            plt.hlines(y=35, xmin=xmin1, xmax=xmax1,linewidth=0.005)
            plt.ylabel('Posição (cm)')
            plt.title("Posição X Tempo")
            
            
            plt.subplot(2,1,2)
            plt.plot(tempo ,vetorposicaoy,color='blue',label='Posição em y',linewidth=1.0)
            plt.plot(tempo, vetorsetpointy, color='red',label='Set point em y',linewidth=1.0,linestyle='--')
            plt.hlines(y=35, xmin=xmin1, xmax=xmax1,linewidth=0.005)
            plt.xlabel('Tempo (s)')
            plt.ylabel('Posição (cm)')'''

            plt.plot
            plt.plot(vetorposicaox ,vetorposicaoy,color='blue',label='Posição em x',linewidth=1.0)
            plt.plot(vetorsetpointx, vetorsetpointy, color='red',label='Set point em x',linewidth=1.0,linestyle='--')
            plt.hlines(y=0, xmin=-10, xmax=10,linewidth=0.005)
            plt.ylabel('Posição (cm)')
            plt.title("Posição X Tempo")
            
  


            
            
            plt.pause(0.001)
            '''f k==0:
                k=k+0.033
            else:
                k=k + temporeal-tempoatrasado
            tempoatrasado=temporeal'''
            b2Texto.set("Fechar Grafico")       
        
    else:
        plt.close()
        vetorposicaox.clear()
        vetorsetpointx.clear()
        vetorposicaoy.clear()
        vetorsetpointy.clear()
        tempo.clear()
        k=0
        k1=0
        b2Texto.set("Plotar Grafico")
        xmin1=0
        xmax1=20
        k2=20
        
        
    if k>=k2:

        xmin1=k
        xmax1=20+xmin1
        vetorposicaox.clear()
        vetorsetpointx.clear()
        vetorposicaoy.clear()
        vetorsetpointy.clear()
        tempo.clear()
        plt.clf()
    
    
        k2=k2+20
    app.after(1500,Grafico)

       
#botões
tag = tk.Label(app,text="Sistema Mesa Bola"  ,font="bold 25", fg="#4682B4")
tag.pack(fill="both")
b1=tk.Button(app,textvariable=b1Texto,command=cam).place(x=20, y=60)
b2=tk.Button(app,textvariable=b2Texto,command=PlotGrafico).place(x=20, y=120)
b4=tk.Button(app,textvariable=b4Texto,command=IniciaDegrau).place(x=20, y=180)
b8=tk.Button(app,textvariable=b8Texto,command=Salvar).place(x=20, y=230)
b9=tk.Button(app,textvariable=b7Texto,command=Avanco).place(x=320, y=180)

SC = tk.Label(app, text="Circulo: raio 7",  fg="#4682B4",bg="white")
SC.place(x=20,y=360)
b6=tk.Button(app,textvariable=b6Texto,command=IniciaCirculo).place(x=20, y=400)
SC = tk.Label(app, text="Quadrado: tamanho 10",  fg="#4682B4",bg="white")
SC.place(x=120,y=360)
b10=tk.Button(app,textvariable=b10Texto,command=IniciaQuadrado).place(x=120, y=400)


angulo = tk.Label(app,text="Angulo do Servo 1: 90º\nAngulo do Servo 2: 90º", fg="red",bg="white")
angulo.place(x=250, y=100)
posicao = tk.Label(app, text="Posição da bola: 0,0", fg="#4682B4",bg="white")
posicao.place(x=240, y=80)

x0 = tk.Label(app, text="Valor de x", fg="#4682B4",bg="white").place(x=20, y=320)
xtext=tk.Text(app,height=1, width=5,highlightbackground = "#B0C4DE", 
                         highlightthickness = 2)
xtext.place(x=95,y=320)
y0 = tk.Label(app, text="Valor de y", fg="#4682B4",bg="white").place(x=165, y=320)
ytext=tk.Text(app,height=1, width=5,highlightbackground = "#B0C4DE", 
                         highlightthickness = 2)
ytext.place(x=240,y=320)
b5=tk.Button(app,text="Aplicar",command=lambda: SetPoint()).place(x=320,y=320)
SPT = tk.Label(app, text="SetPoint(Degrau): (12.5,12.5)",  fg="#4682B4",bg="white")
SPT.place(x=20,y=280)

if __name__ == "__main__":

    
    t1 = threading.Thread(target=controle)
    t1.start()
    
    #controle()

    app.mainloop()
    
    
    cap.release()
    cv2.destroyAllWindows()





