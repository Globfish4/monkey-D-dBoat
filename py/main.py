import time
import numpy as np

from imu9_driver_v2 import *
from tc74_driver_v2 import *
from encoders_driver_v2 import *
from arduino_driver_v2 import *
from gps_driver_v2 import *







class Bateau:
    def __init__(self):
        self.IMU = Imu9IO()

        self.A = np.array([[ 68700193.11172161,   6330530.48572302  , 3539631.62464239],
 [ 12450116.55030809 , 58861304.81990033 , 13376801.18223511],
 [ -5602878.6345501 ,  -5059121.08352694 , 68824529.17179936]])
        self.b = np.array([[-542. ],
 [ 359. ],
 [ 174.5]])

        self.stateVar = np.array([[0], [0], [0], [0]])
        self.x0 = 0
        self.Gamma0 = np.array([[100, 0], [0, 100]])

        self.temp = TempTC74IO()

        dt = 0.1
        self.enc = EncoderIO()
        self.enc.set_older_value_delay_v2(1/dt)

        self.ard = ArduinoIO()

        self.lastAcc = np.array(self.IMU.read_accel_raw()).reshape(3, 1)
        self.lastMagn = np.array(self.IMU.read_mag_raw()).reshape(3, 1)
        self.lastPsi = self.get_angle()
        self.slidingHeading = []

        #Initialisation du GPS
        
        self.gps = GpsIO()
        self.gps.set_filter_speed("0.4")
        self.gps.get_filter_speed()
        self.gps.set_filter_speed("0")
        self.gps.get_filter_speed()
        
        print("I am Monkey D DiBoat, the future king of pirates")




    def calibrate_MAG(self) :
        """
        fonction à utiliser une fois pour calibrer le capteur GPS
        :return: nothing
        """
        R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        xN, xS, xW, xu = 0, 0, 0, 0
        input("get measure? tap enter")
        xN = R@np.array(self.IMU.read_mag_raw()).reshape(3, 1)
        input("get measure? tap enter")
        xS = R@np.array(self.IMU.read_mag_raw()).reshape(3, 1)
        input("get measure? tap enter")
        xW = R@np.array(self.IMU.read_mag_raw()).reshape(3, 1)
        input("get measure? tap enter")
        xu = R@np.array(self.IMU.read_mag_raw()).reshape(3, 1)

        I = 64*np.pi/180
        beta = 46*10**-6
        yN = np.array([[beta*np.cos(I)], [0], [-beta*np.sin(I)]])
        yS = np.array([[-beta*np.cos(I)], [0], [beta*np.sin(I)]])
        yW = np.array([[0], [-beta*np.cos(I)], [-beta*np.sin(I)]])
        yu = np.array([[-beta*np.sin(I)], [0], [beta*np.cos(I)]])

        self.b = (-xN -xS)/2

        Y = np.hstack((yN, yW, yu))
        X = np.hstack((xN + self.b, xW + self.b, xu + self.b))
        self.A = X@np.linalg.inv(Y)


        print(self.A, self.b)





    def get_angle(self):
        R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        x1 = self.lastMagn*0. + R@np.array(self.IMU.read_mag_raw()).reshape(3, 1)
        self.lastMagn = x1

        y1 = np.linalg.inv(self.A)@(x1 + self.b)
        anciengetangle = -180/np.pi*np.arctan2(y1[1, 0], y1[0, 0]) #retourne le cap actuel du bateau
        #print("ancien get angle : ", anciengetangle)

        y1 = y1 / np.linalg.norm(y1)

        a1 = R@np.array(self.IMU.read_accel_raw()).reshape(3, 1)*0.1 + self.lastAcc*0.9
        self.lastAcc = a1
        a1 = a1 / np.linalg.norm(a1) #on normalise les vecteurs
        a0 = np.array([[0], [0], [1]])
        
        Rh = rotuv(a1, a0)
        yh = Rh@y1
        yh1, yh2, yh3 = yh.flatten()
        psihat = -np.arctan2(yh2, yh1)
        #print("nouveau get angle :", psihat*180/np.pi)
        #print("x : ", np.array(self.IMU.read_mag_raw()).reshape(3, 1))

        #calcul des autres angles d'euler:
        #phihat = np.arcsin(a1.T@np.array([[0], [1], [0]]))
        #thetahat = np.arcsin(a1.T@np.array([[1], [0], [0]]))
        #print('phihat : ', phihat[0, 0]*180/np.pi, 'thetahat : ', thetahat[0, 0]*180/np.pi)
        return 180*psihat/np.pi #retourne le cap en degré




    def test(self): #to test calibration
        for k in range(50000):
            self.ard.send_arduino_cmd_motor(20*np.cos(0.1*k) + 40, 20*np.sin(0.1*k) + 40)
            print(self.get_angle())
            time.sleep(0.1)



    def try_Accelerometer(self):
        lastAcc = np.array(self.IMU.read_accel_raw()).reshape(3, 1)
        #self.ard.send_arduino_cmd_motor(40, 40)
        for k in range(50000):
            #self.ard.send_arduino_cmd_motor(20*np.cos(0.1*k) + 40, 20*np.sin(0.1*k) + 40)
            vect = np.array(self.IMU.read_accel_raw()).reshape(3, 1)*0.1 + lastAcc*0.9
            lastAcc = vect
            print("acceleration : ", vect/np.linalg.norm(vect))
            time.sleep(0.1)




    def get_rpm(self):
    #time.sleep(5.0)

        st1, st0 = self.enc.get_last_and_older_values_v2()
        data_encoders0 = np.array(st0.split(',')).astype(np.float)
        data_encoders1 = np.array(st1.split(',')).astype(np.float)

        timeAcq0 = data_encoders0[0]/10.0
        sensLeft0 = data_encoders0[2]
        sensRight0 = data_encoders0[1]
        posLeft0 = data_encoders0[4]
        posRight0 = data_encoders0[3]

        timeAcq1 = data_encoders1[0]/10.0
        sensLeft1 = data_encoders1[2]
        sensRight1 = data_encoders1[1]
        posLeft1 = data_encoders1[4]
        posRight1 = data_encoders1[3]

        delta_t = timeAcq1 - timeAcq0
        rpmL = abs(delta_odo(posLeft1, posLeft0)/8.0/delta_t*60.0)
        rpmR = abs(delta_odo(posRight1, posRight0)/8.0/delta_t*60.0)

        #print("RPM Left", rpmL, "RPM Right", rpmR)

        return abs(rpmL), abs(rpmR)





    def choosew(self, wbarbar, deltawbar):
        wbar1, wbar2 = wbarbar, wbarbar + abs(deltawbar)
        if deltawbar < 0:
            #print("go right")
            return wbar2, wbar1
        elif deltawbar >= 0:
            #print("go left")
            return wbar1, wbar2



    def go_straight(self,wbar,T):
        dt = 0.1
        z1, z2 = 0, 0
        KD1, KD2 = 0.04, 0.04 
        KG1, KG2 = 0.06, 0.02

        self.ard.send_arduino_cmd_motor(0, 0)

        T0 = time.time()
        while time.time() - T0 < T:
            t0 =time.time()
            w1, w2 = self.get_rpm()
            print('rpmL : ', w1, 'rpmR', w2)
            e1, e2 = wbar - w1, wbar - w2
            z1 += e1*dt
            z2 += e2*dt
            u1, u2 = KG1*z1+KG2*e1, KD1*z2+KD2*e2
            u1, u2 = check_u(u1), check_u(u2)
            print(u1, u2)
            self.ard.send_arduino_cmd_motor(u1, u2)
            t = time.time()
            if t - t0 < dt:
                time.sleep(dt-(t-t0))
        self.ard.send_arduino_cmd_motor(0, 0)






    def MAJHeadingList(self, psi):
        if len(self.slidingHeading) < 4:
            self.slidingHeading.append(psi)
            return 0
        elif len(self.slidingHeading) == 4:
            self.slidingHeading = self.slidingHeading[1:] + [psi]
            return sawtooth(self.slidingHeading[-1] - self.slidingHeading[0])
        



    def wait(self, T):
        self.ard.send_arduino_cmd_motor(0, 0)
        dt = 0.15
        self.lastPsi = self.get_angle()
        print("I'm waiting...")
        T0 = time.time()
        while time.time() - T0 < T:
            t0 = time.time()

            psi = 0.9*self.lastPsi + 0.1*self.get_angle()
            self.lastPsi = psi
            sigma = self.MAJHeadingList(psi)
            print("ecart-type : ", sigma)
            print("psi : ", psi)

            t1 = time.time()
            if t1 - t0 < dt:
                time.sleep(dt - (t1 - t0))
            else:
                print("temps de boucle pas assez long")

        print('maintenant je vais démarrer')





    def follow_one_cap(self, psibar, T):
        """
        fonction to follow a direction psi
        :param psibar: is an angle
        :return: nothing
        """
        self.wait(5)
        dt = 0.1
        KD1, KD2 = 0.03, 0.02
        KG1, KG2 = 0.03, 0.015
        K3 = 900
        z1, z2 = 0, 0
        wbarbar = 3000 #Consigne pivot 
        self.lastPsi = self.get_angle()
        #fichier.write("################ Nouvelle Run ###################\n\n")

        T0 = time.time()
        while time.time() - T0 < T:
            t0 = time.time()
            print("###################################")

            psi = 0.9*self.lastPsi + 0.1*self.get_angle()
            self.lastPsi = psi

            deltawbar = K3*sawtooth(np.pi*psibar/180 - np.pi*psi/180) #sawtooth prend des radians
            wbar1, wbar2 = self.choosew(wbarbar, deltawbar) #détermine si on va à gauche ou à droite
            w1, w2 = self.get_rpm() #retourne les mesures des rpm
            e1, e2 = wbar1 - w1, wbar2 - w2

            z1 += e1*dt #termes intégrateurs
            z2 += e2*dt
            u1, u2 = KG1*z1+KG2*e1, KD1*z2+KD2*e2
            u1, u2 = check_u(u1),check_u(u2)
            
            
            print("rpmL : ", w1, "rpmR : ", w2)
            print("omega bar : ", wbar1, wbar2)
            print("deltawbar : ", deltawbar)
            #print("z : ", z1, z2)
            #print("u : ", u1, u2)
            print("psi : ", psi)

            self.ard.send_arduino_cmd_motor(u1, u2)

            t1 = time.time()
            if t1 - t0 < dt:
                time.sleep(dt - (t1 - t0))
            else:
                print("temps de boucle pas assez long")

        self.ard.send_arduino_cmd_motor(0, 0) #on coupe les moteurs 





    def follow_several_caps(self, psibarL, TL):
        """
        fonction to follow a direction psi
        :param psibarL: is a list with all the caps
        :param TL : is a list with the direction times
        :return: nothing
        """
        for cap in range(len(psibarL)):
            self.follow_one_cap(psibarL[cap], TL[cap])
            time.sleep(1) #petite pause entre deux caps
            print("ok bb je passe au cap suivant ;) ;)")




    def try_GPS(self):
        for k in range(5000):
            lat, lon = self.get_lat_lon()
            print("lat : ", lat, 'lon : ', lon)
            p = convert_coordinates(lat, lon)
            print("p : ", p[0:2, 0])
            #print("position : ", p, "balise : ", p_bouee_3)




    def get_lat_lon(self):
        msg, gps_data_string = self.gps.read_gll_non_blocking()
        while not msg:
            msg, gps_data_string = self.gps.read_gll_non_blocking()
        lat, lon = cvt_gll_ddmm_2_dd(gps_data_string)
        #print(lat, lon)
        #print("GPS:", gps_data_string)
        return lat, lon




    def get_psibar(self, p, phat, vhat, nhat):
        K4 = 10
        dhat = K4*vhat - 2*nhat @ np.transpose(nhat)@(p - phat)
        psibar = np.arctan2(dhat[1, 0], dhat[0, 0])
        return sawtooth(psibar - np.pi/2)*180/np.pi





    def follow_line(self, phat, ptilde): #suit juste une ligne
        """
        phat et ptilde sont des coordonnées en cartésien (en vecteurs colonnes)
        """
        self.wait(5)
        dt, dt3 = 0.1, 0.7
        t1 = t3 = time.time()
        K3 = 800
        KD1, KD2 = 0.02, 0.01
        KG1, KG2 = 0.03, 0.025
        z1, z2 = 0, 0
        psibar = 0
        psi = 0
        wbar1, wbar2 = 0, 0
        wbarbar = 2300 #consigne pivot
        nhat, vhat = get_nhat(phat, ptilde)

        lat, lon = self.get_lat_lon()
        p = convert_coordinates(lat, lon)
        psibar = self.get_psibar(p, phat, vhat, nhat)
        #print("psibar recalculé : ", psibar)

        self.lastPsi = self.get_angle()

        while (ptilde-phat).T@(ptilde-p) and np.linalg.norm(p - ptilde) > 7:
        #while np.linalg.norm(p - ptilde) > 30: #tant qu'éloigné de la balise de plus de 6 mètres
            t0 = time.time()

            if time.time() - t3 > dt3:
                lat, lon = self.get_lat_lon()
                p = convert_coordinates(lat, lon)
                psibar = self.get_psibar(p, phat, vhat, nhat)
                t3 = time.time()
                #print("psibar recalculé : ", psibar)

            psi = 0.9*self.lastPsi + 0.1*self.get_angle()
            self.lastPsi = psi
            sigma = self.MAJHeadingList(psi)
            if sigma > 15:
                print("Ma boussole part en couilles")
                self.wait(2)

            deltawbar = K3*sawtooth(np.pi*psibar/180 - np.pi*psi/180) #sawtooth prend des radians
            wbar1, wbar2 = self.choosew(wbarbar, deltawbar) #détermine si on va à gauche ou à droite
            w1, w2 = self.get_rpm() #retourne les mesures des rpm
            e1, e2 = wbar1 - w1, wbar2 - w2

            z1 += e1*dt #termes intégrateurs
            z2 += e2*dt
            u1, u2 = KG1*z1+KG2*e1,KD1*z2+KD2*e2
            u1, u2 = check_u(u1),check_u(u2)
            
            print("###################################")
            print("rpmL : ", w1, "rpmR : ", w2)
            print("omega bar : ", wbar1, wbar2)
            print("deltaomegabar : ", deltawbar)
            #print("z : ", z1, z2)
            #print("u : ", u1, u2)
            print("psi : ", psi)
            print("psibar : ", psibar)
            print("position : ", p)
            print("distance à la bouee : ", np.linalg.norm(ptilde - p))


            self.ard.send_arduino_cmd_motor(u1, u2)

            t1 = time.time()
            if t1 - t0 < dt:
                time.sleep(dt - (t1 - t0))
            else:
                print("le dt est trop court")

        self.ard.send_arduino_cmd_motor(0, 0) #on coupe les moteurs
        print("c'est bon, je suis au point GPS demandé")




    def f(u):
        u1, u2 = u.flatten()
        x, y, v, theta = self.stateVar.flatten()
        return np.array([[v*np.cos(theta)], [v*np.sin(theta)], [u1], [u2]])




    def computeKalman(self):
        x, y, v, theta = self.stateVar.flatten()
        dt = 0.5

        Ak = np.array([[1, 0, dt*np.cos(theta)], [0, 1, dt*np.sin(theta)], [0, 0, 1]])
        ak = 0
        uc = np.array([[0], [0], [dt*ak]])
        Ck = np.array([[1, 0, 0], [0, 1, 0]])
        Gbeta = np.array([[25, 0], [0, 25]])
        Galpha = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 10]])
        lat, lon = self.get_lat_lon()
        p = convert_coordinates(lat, lon)
        yk = p #coordonnées GPS traduites

        kalman(self.x0, self.Gamma0, uc, yk, Galpha, Gbeta, Ak, Ck)




    def computeU(self, psi):
        a, b, c = 1, 2, 1 #paramètres du régulateur

        xy = self.x0[0:2, 0]
        v = self.x0[2, 0]
        dxy = np.array([[v*np.cos(psi)], [v*np.sin(psi)]])

        dP = np.array([[r*np.cos(omega*t + phi) + xc], [r*np.sin(omega*t - phi) + yc]])
        ddP = np.array([[-omega*r*np.sin(ometa*t + phi)], [omega*r*np.cos(omega*t - phi)]])
        dddP = omega**2*r*np.array([[-np.cos(omega*t + phi)], [-np.sin(omega*t - phi)]])

        A = array([[np.cos(psi), -v*np.sin(psi)], [np.sin(psi), v*np.cos(psi)]])
        u = np.linalg.inv(A) @ (a*(dP - xy) + b*(ddP - dxy) + c*dddP )
        return u


    


    def follow_circle(self):
        dt, dt3 = 0.15, 1 # pas de temps
        T0 = time.time() # temps machine initial
        t1 = t3 = time.time()
        K1, K2, K3 = 0.06, 0.01, 750 # constantes de régulation
        z1, z2 = 0, 0 # intégrateurs de la commande moteur
        psibar = 0 #cap voulu
        wbar1, wbar2 = 0, 0 # consignes des moteurs
        wbarbar = 3000 # consigne pivot

        self.computeKalman()
        lastPsi = self.get_angle()

        u = self.computeU(lastPsi)
        self.stateVar += dt*self.f(u)
        psibar, wbarbar = self.stateVar[2, 0], self.stateVar[3, 0]
        t3 = time.time()
        print("psibar recalculé : ", psibar, "v recalculé : ", wbarbar)

        

        while time.time() - T0 < T:
            t0 = time.time()

            self.computeKalman()
            psi = 0.9*lastPsi + 0.1*self.get_angle()
            lastPsi = psi

            if time.time() - t3 > dt3: #on remet à jour les commandes
                u = self.computeU()
                self.stateVar += dt*self.f(u)
                psibar, wbarbar = self.stateVar[2, 0], self.stateVar[3, 0]
                t3 = time.time()
                print("psibar recalculé : ", psibar, "v recalculé : ", wbarbar)

            deltawbar = K3*sawtooth(np.pi*psibar/180 - np.pi*psi/180) #sawtooth prend des radians
            wbar1, wbar2 = self.choosew(wbarbar, deltawbar) #détermine si on va à gauche ou à droite
            w1, w2 = self.get_rpm() #retourne les mesures des rpm
            e1, e2 = wbar1 - w1, wbar2 - w2

            z1 += e1*dt #termes intégrateurs
            z2 += e2*dt
            u1, u2 = K1*z1+K2*e1,K1*z2+K2*e2
            u1, u2 = check_u(u1),check_u(u2)
            
            print("###################################")
            print("rpmL : ", w1, "rpmR : ", w2)
            print("omega bar : ", wbar1, wbar2)
            print("psi : ", psi)


            self.ard.send_arduino_cmd_motor(u1, u2)

            t1 = time.time()
            if t1 - t0 < dt:
                time.sleep(dt - (t1 - t0))
            else:
                print("le dt est trop court")

        self.ard.send_arduino_cmd_motor(0, 0) #on coupe les moteurs
        print("c'est bon, je suis au point GPS demandé")






def delta_odo(odo1, odo0):
    '''
    computes the difference between 2 encoder values
    '''
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


def check_u(u):
    if u<0:
        return 0
    elif u>255:
        return 255
    else:
        return int(u)


##########################################################################
## Mathematical functions


def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))


def scalarprod(u,v): # scalar product
    u,v=u.flatten(),v.flatten()
    return np.sum(u[:]*v[:])



def adjoint(w):
    if isinstance(w, (float, int)): return np.array([[0,-w] , [w,0]])
    #print('w=',w)
    w=np.ndarray.tolist(w)
    #print('tolist(w)=',w)
    return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])


def rotuv(u,v): #returns rotation with minimal angle  such that  v=R*u
            # see https://en.wikipedia.org/wiki/Rotation_matrix#Vector_to_vector_formulation
    u=np.array(u).reshape(3,1)
    v=np.array(v).reshape(3,1)
    u=(1/np.linalg.norm(u))*u
    v=(1/np.linalg.norm(v))*v
    c=scalarprod(u,v)
    A=v@u.T-u@v.T
    return np.eye(3,3)+A+(1/(1+c))*A@A





##########################################################################
## Fonctions de conversions GPS


def cvt_gll_ddmm_2_dd(st): #sert à convertire les données NMEA en coord lat lon
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat / 100))
    olon = float(int(ilon / 100))
    olat_mm = (ilat % 100) / 60
    olon_mm = (ilon % 100) / 60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat, olon



def convert_coordinates(lat, lon):
        roh = 6371000
        lat = lat*np.pi/180
        lon = lon*np.pi/180
        lat_ponton = np.pi*48.198943/180
        lon_ponton = -np.pi*3.014750/180
        xtilde = roh * np.cos(lat) * (lon - lon_ponton)
        ytilde = roh * (lat - lat_ponton)
        return np.array([[xtilde], [ytilde]]) #retourne un vecteur colonne



def get_nhat(phat, ptilde): #phat point de départ, ptilde point d'arrivée
        """
        get_line renvoie nhat, le vecteur orthogonal à la ligne
        """
        u = ptilde - phat
        unorm = np.linalg.norm(u)

        vhat = u/unorm
        nhat= np.array([[-u[1, 0]], [u[0, 0]]]) / unorm
        return nhat, vhat





##########################################################################
##Fonctions pour le cercle


def kalman_predict(xup,Gup,u,Γα,A):
    Γ1 = A @ Gup @ A.T + Γα
    x1 = A @ xup + u    
    return(x1,Γ1)    

def kalman_correc(x0,Γ0,y,Γβ,C):
    S = C @ Γ0 @ C.T + Γβ        
    K = Γ0 @ C.T @ np.linalg.inv(S)           
    ytilde = y - C @ x0        
    Gup = (np.eye(len(x0))-K @ C) @ Γ0 
    xup = x0 + K@ytilde
    return(xup,Gup) 
    
def kalman(x0,Γ0,u,y,Γα,Γβ,A,C):
    xup,Gup = kalman_correc(x0,Γ0,y,Γβ,C)
    x1,Γ1=kalman_predict(xup,Gup,u,Γα,A)
    return(x1,Γ1)












if __name__ == "__main__":

    
    p_bouee_ponton = convert_coordinates(48.198943, -3.014750)
    p_bouee_1 = convert_coordinates(48.199202, -3.015000)
    p_bouee_3 = convert_coordinates(48.1991840, -3.0152830)
    p_bouee_2 = convert_coordinates(48.199508, -3.0152950)
    p_bfond= convert_coordinates(48.1994, -3.0166)

    print("ponton : ", p_bouee_ponton)
    print("1 : ", p_bouee_1)
    print("2 : ", p_bouee_2)
    print("3 : ", p_bouee_3)


    bat = Bateau()

    #"psibarL = [0, 90, 180, 270] #valeurs en degrés
    #TL = [40, 30, 30, 30]

    
    #bat.try_Accelerometer()
    #bat.test()
    #bat.wait(60)
    #bat.try_GPS()
    #bat.calibrate_MAG()
    #bat.go_straight(1500, 60)
    #bat.follow_one_cap(0, 60)
    #bat.follow_several_caps(psibarL, TL)
    bat.follow_line(p_bouee_ponton, p_bfond)



    


"""
p_bouee_ponton = convert_coordinates(48.1989428, -3.0147503)
p_bouee_1 = convert_coordinates(48.19917, -3.01485)
p_bouee_3 = convert_coordinates(48.1991840, -3.0152830)
p_bouee_2 = convert_coordinates(48.1995076, -3.0152830)

bat.follow_line(p_bouee_3, p_bouee_ponton)

t0=time.time()
bat.follow_line(p_bouee_ponton, p_bouee_1)
bat.follow_line(p_bouee_1, p_bouee_2)
bat.follow_line(p_bouee_2, p_bouee_3)
while time.time()-t0<5*60:
    pass
bat.follow_line(p_bouee_3, p_bouee_1)
bat.follow_line(p_bouee_1, p_bouee_2)
bat.follow_line(p_bouee_2, p_bouee_3)
bat.follow_line(p_bouee_3, p_bouee_ponton)
"""



"""
U = [np.random.randn(3, 1) for k in range(100)]
V = [np.random.randn(3, 1) for k in range(100)]
for k in range(len(U)):
    print('rot du prof : ', rotuvp(U[k], V[k]))
    print('rot de nous : ', rotuv(U[k], V[k]))

def rotuv(u, v):
    k = 1000
    #uvectv = np.linalg.det(np.hstack((u, v)))
    uvectv = np.cross(u.reshape(3,), v.reshape(3,))
    X = np.arccos(u.T@v)[0, 0]*uvectv/np.linalg.norm(uvectv)
    rotation = (np.eye(3) + adjoint(X)/k)**k
    return rotation
"""
