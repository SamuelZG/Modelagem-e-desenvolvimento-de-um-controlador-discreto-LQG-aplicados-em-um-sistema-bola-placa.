import numpy as np
import cv2

ESC_KEY = 27


"""cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)"""

cap =cv2.VideoCapture(2, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#cap.set(cv2.CAP_PROP_FPS, 30)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
def setLimitsOfTrackbar():
    hue = {}
    hue["min"] = cv2.getTrackbarPos("Min Hue", trackbarWindow)
    hue["max"] = cv2.getTrackbarPos("Max Hue", trackbarWindow)
    
    if hue["min"] > hue["max"]:
        cv2.setTrackbarPos("Max Hue", trackbarWindow, hue["min"])
        hue["max"] = cv2.getTrackbarPos("Max Hue", trackbarWindow)
    
    sat = {}
    sat["min"] = cv2.getTrackbarPos("Min Saturation", trackbarWindow)
    sat["max"] = cv2.getTrackbarPos("Max Saturation", trackbarWindow)
    
    if sat["min"] > sat["max"]:
        cv2.setTrackbarPos("Max Saturation", trackbarWindow, sat["min"])
        sat["max"] = cv2.getTrackbarPos("Max Saturation", trackbarWindow)

    val = {}
    val["min"] = cv2.getTrackbarPos("Min Value", trackbarWindow)
    val["max"] = cv2.getTrackbarPos("Max Value", trackbarWindow)
    
    if val["min"] > val["max"]:
        cv2.setTrackbarPos("Max Value", trackbarWindow, val["min"])
        val["max"] = cv2.getTrackbarPos("Max Value", trackbarWindow)
        
    return hue, sat, val
def computeTracking(frame, hue, sat, val):
    
    #transforma a imagem de RGB para HSV
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #definir os intervalos de cores que vão aparecer na imagem final
    lowerColor = np.array([hue['min'], sat["min"], val["min"]])
    upperColor = np.array([hue['max'], sat["max"], val["max"]])
    
    #marcador pra saber se o pixel pertence ao intervalo ou não
    mask = cv2.inRange(hsvImage, lowerColor, upperColor)
    
    #aplica máscara que "deixa passar" pixels pertencentes ao intervalo, como filtro
    result = cv2.bitwise_and(frame, frame, mask = mask)
    
    #aplica limiarização
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    _,gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    
    #encontra pontos que circundam regiões conexas (contour)
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    #se existir contornos    
    if contours:
        #retornando a área do primeiro grupo de pixels brancos
        maxArea = cv2.contourArea(contours[0])
        contourMaxAreaId = 0
        i = 0
        
        #para cada grupo de pixels branco
        for cnt in contours:
            #procura o grupo com a maior área
            if maxArea < cv2.contourArea(cnt):
                maxArea = cv2.contourArea(cnt)
                contourMaxAreaId = i
            i += 1
            
        #achei o contorno com maior área em pixels
        cntMaxArea = contours[contourMaxAreaId]
        
        #retorna um retângulo que envolve o contorno em questão
        xRect, yRect, wRect, hRect = cv2.boundingRect(cntMaxArea)
        
        #desenha caixa envolvente com espessura 3
        cv2.rectangle(frame, (xRect, yRect), (xRect + wRect, yRect + hRect), (0, 0, 255), 2)
    
    return frame, mask
trackbarWindow = "trackbar window"
cv2.namedWindow(trackbarWindow)

def onChange(val):
    return

cv2.createTrackbar("Min Hue", trackbarWindow, 0, 255, onChange)
cv2.createTrackbar("Max Hue", trackbarWindow, 255, 255, onChange)

cv2.createTrackbar("Min Saturation", trackbarWindow, 0, 255, onChange)
cv2.createTrackbar("Max Saturation", trackbarWindow, 255, 255, onChange)

cv2.createTrackbar("Min Value", trackbarWindow, 0, 255, onChange)
cv2.createTrackbar("Max Value", trackbarWindow, 255, 255, onChange)

min_hue = cv2.getTrackbarPos("Min Hue", trackbarWindow)
max_hue = cv2.getTrackbarPos("Max Hue", trackbarWindow)

min_sat = cv2.getTrackbarPos("Min Saturation", trackbarWindow)
max_sat = cv2.getTrackbarPos("Max Saturation", trackbarWindow)

min_val = cv2.getTrackbarPos("Min Value", trackbarWindow)
max_val = cv2.getTrackbarPos("Max Value", trackbarWindow)

camHeight = 720
camWidth = 1280

while True:
    
    #frame = cv2.imread(cap)
    
    ret, frame = cap.read()
    
    frame = frame[205 : 575, 445 : 820]
    
    hue, sat, val = setLimitsOfTrackbar()
    
    frame= cv2.GaussianBlur(frame, (9, 9), 0)
    frame = cv2.medianBlur(frame,7)

    frame = cv2.erode(frame, None, iterations=2)
    frame = cv2.dilate(frame, None, iterations=2)
    
    
    frame, mask = computeTracking(frame, hue, sat, val)
    
    
    
    cv2.imshow("mascara", mask)
    cv2.imshow("webcam", frame)

    if cv2.waitKey(1) & 0xFF == ord('q') or 0xFF == ESC_KEY:
        break
        
cap.release()
cv2.destroyAllWindows()
